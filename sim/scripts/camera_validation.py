#!/usr/bin/env python3
"""Capture and validate the two RGB views of the Isaac FR3 baseline."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import subprocess
import sys
import traceback
import uuid
from pathlib import Path

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


args = parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import numpy as np
from PIL import Image
from isaacsim.core.version import get_version

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "sim" / "src"))
sys.path.insert(0, str(REPO_ROOT / "third_party/openpi/packages/openpi-client/src"))

from isaac_fr3.cameras import camera_metadata, capture_rgb, create_cameras
from isaac_fr3.grasp_poses import (
    approach_targets,
    make_solver,
    move_to_tcp_pose,
    tip_center,
)
from isaac_fr3.scene import (
    CUBE_SETTLE_STEPS,
    HOME_SETTLE_STEPS,
    command_home,
    create_scene,
    get_tcp_pose,
    load_config,
    settle,
)
from openpi_client.image_tools import resize_with_pad
from scipy.spatial.transform import Rotation


def git_value(*command: str) -> str:
    return subprocess.check_output(
        ["git", *command], cwd=REPO_ROOT, text=True
    ).strip()


def pose_dict(position: np.ndarray, orientation: np.ndarray) -> dict:
    return {
        "position_xyz_m": np.asarray(position).reshape(-1).tolist(),
        "orientation_wxyz": np.asarray(orientation).reshape(-1).tolist(),
    }


def project_points(camera_info: dict, points: dict) -> dict:
    """Project world points into the USD camera's top-left-origin image."""
    position = np.asarray(camera_info["world_position_xyz_m"])
    w, x, y, z = camera_info["world_orientation_wxyz"]
    rotation = Rotation.from_quat([x, y, z, w])
    intrinsics = np.asarray(camera_info["intrinsics_matrix_pixels"])
    width, height = camera_info["resolution_wh"]
    result = {}
    for label, world_xyz in points.items():
        point = np.asarray(world_xyz)
        camera_xyz = rotation.inv().apply(point - position)
        depth = -camera_xyz[2]
        if depth <= 0:
            result[label] = {"world_xyz_m": point.tolist(), "visible": False}
            continue
        u = float(intrinsics[0, 0] * camera_xyz[0] / depth + intrinsics[0, 2])
        v = float(intrinsics[1, 2] - intrinsics[1, 1] * camera_xyz[1] / depth)
        result[label] = {
            "world_xyz_m": point.tolist(),
            "pixel_uv": [u, v],
            "inside_image": bool(0 <= u < width and 0 <= v < height),
        }
    return result


def workspace_points(cube_xyz: np.ndarray) -> dict:
    """Five nearby test locations, including the unchanged target center."""
    offsets = {
        "center": [0.0, 0.0, 0.0],
        "x_minus_5cm": [-0.05, 0.0, 0.0],
        "x_plus_5cm": [0.05, 0.0, 0.0],
        "y_minus_5cm": [0.0, -0.05, 0.0],
        "y_plus_5cm": [0.0, 0.05, 0.0],
    }
    return {
        label: cube_xyz + offset for label, offset in offsets.items()
    }


def verify_wrist_relative_pose(camera, spec: dict) -> dict:
    """Check that Isaac authored the supplied native USD camera pose unchanged."""
    configured_position = np.asarray(spec["position_xyz_m"], dtype=np.float64)
    configured_quaternion = np.asarray(
        spec["orientation_wxyz"], dtype=np.float64
    )
    measured_position, measured_quaternion = camera.get_local_pose(
        camera_axes="usd"
    )
    measured_position = np.asarray(measured_position, dtype=np.float64)
    measured_quaternion = np.asarray(measured_quaternion, dtype=np.float64)
    position_error = float(np.linalg.norm(
        measured_position - configured_position
    ))
    configured_rotation = Rotation.from_quat(configured_quaternion[[1, 2, 3, 0]])
    measured_rotation = Rotation.from_quat(measured_quaternion[[1, 2, 3, 0]])
    angular_error = float((
        configured_rotation.inv() * measured_rotation
    ).magnitude())
    result = {
        "configured_position_xyz_m": configured_position.tolist(),
        "measured_position_xyz_m": measured_position.tolist(),
        "position_error_m": position_error,
        "configured_orientation_wxyz": configured_quaternion.tolist(),
        "measured_orientation_wxyz": measured_quaternion.tolist(),
        "angular_error_rad": angular_error,
    }
    print("=== WRIST RELATIVE USD POSE CHECK ===")
    for key, value in result.items():
        print(f"{key}: {value}")
    if position_error > 1e-6 or angular_error > 1e-5:
        raise RuntimeError("Wrist camera does not match configured USD pose")
    return result


def capture_stage(
    label: str, handles, config: dict, cameras: dict, output_dir: Path
) -> dict:
    """Save raw and OpenPI-ready images and measured state for one arm stage."""
    specs = config["cameras"]
    camera_info = {
        name: camera_metadata(camera, specs[name])
        for name, camera in cameras.items()
    }
    images = {}
    for name, camera in cameras.items():
        raw = capture_rgb(camera, specs[name]["resolution_wh"])
        processed = resize_with_pad(raw, 224, 224)
        if processed.shape != (224, 224, 3) or processed.dtype != np.uint8:
            raise RuntimeError(f"Unexpected OpenPI image for {name}")
        raw_path = output_dir / f"{name}_raw_{label}.png"
        processed_path = output_dir / f"{name}_openpi_224_{label}.png"
        Image.fromarray(raw, mode="RGB").save(raw_path)
        Image.fromarray(processed, mode="RGB").save(processed_path)
        images[name] = {
            "raw": raw_path.name,
            "openpi_224": processed_path.name,
            "raw_shape_hwc": list(raw.shape),
            "openpi_shape_hwc": list(processed.shape),
            "dtype": str(raw.dtype),
            "color_order": "RGB",
        }
    joints = np.asarray(handles.fr3.get_dof_positions()).reshape(-1)
    fingers = joints[handles.finger_indices]
    tcp_position, tcp_orientation = get_tcp_pose(config)
    cube_position, cube_orientation = handles.target.get_world_pose()
    cube_xyz = np.asarray(cube_position).reshape(3)
    robot_cfg = config["robot"]
    tip_centers = {
        "left": tip_center(handles.world.stage, robot_cfg["left_finger_prim"]),
        "right": tip_center(handles.world.stage, robot_cfg["right_finger_prim"]),
    }
    stage = {
        "captured_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "arm_joint_positions_rad": joints[handles.arm_indices].tolist(),
        "finger_joint_positions_m": fingers.tolist(),
        "gripper_width_m": float(np.sum(fingers)),
        "tcp_pose_world": pose_dict(tcp_position, tcp_orientation),
        "cube_pose_world": pose_dict(cube_position, cube_orientation),
        "cameras": camera_info,
        "images": images,
        "wrist_tip_center_projection": project_points(
            camera_info["wrist"], tip_centers
        ),
        "workspace_projection": project_points(
            camera_info["wrist"], workspace_points(cube_xyz)
        ),
    }
    print(f"=== {label.upper()} ===")
    print(f"q[7] [rad]: {stage['arm_joint_positions_rad']}")
    print(f"TCP: {stage['tcp_pose_world']}")
    print(f"cube: {stage['cube_pose_world']}")
    for name in cameras:
        info = camera_info[name]
        print(f"=== {name.upper()} CAMERA ===")
        print(f"parent: {info['parent_prim']}")
        print(f"USD frame: {info['frame']}; optical frame: "
              f"{info['optical_frame']}; quaternion: wxyz")
        print(f"relative: {info['parent_relative_position_xyz_m']} "
              f"{info['parent_relative_orientation_wxyz']}")
        print(f"world: {info['world_position_xyz_m']} "
              f"{info['world_orientation_wxyz']}")
        print(f"resolution WH: {info['resolution_wh']}")
        print(f"intrinsics: {info['intrinsics_matrix_pixels']}; "
              f"FOV: {info['horizontal_fov_deg']} deg")
        print(f"raw: {images[name]['raw']}; OpenPI: "
              f"{images[name]['openpi_224']}")
    return stage


def main() -> None:
    config = load_config(args.config)
    handles = create_scene(config)
    command_home(handles, config)
    settle(handles, HOME_SETTLE_STEPS, render=False)
    settle(handles, CUBE_SETTLE_STEPS, render=False)
    pregrasp, grasp, orientation, geometry = approach_targets(handles, config)
    solver = make_solver(config)
    rig = create_cameras(config)
    cameras = {"wrist": rig.wrist, "external": rig.external}
    wrist_pose_check = verify_wrist_relative_pose(
        rig.wrist, config["cameras"]["wrist"]
    )
    # RGB annotators need rendered frames before their buffers are valid.
    settle(handles, 16, render=True)
    run_id = (dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
              + "_" + uuid.uuid4().hex[:8])
    output_dir = REPO_ROOT / "outputs" / "isaac_camera_validation" / run_id
    output_dir.mkdir(parents=True, exist_ok=False)
    stages = {"home": capture_stage("home", handles, config, cameras, output_dir)}
    move_to_tcp_pose(handles, config, solver, pregrasp, orientation)
    stages["pregrasp"] = capture_stage(
        "pregrasp", handles, config, cameras, output_dir
    )
    move_to_tcp_pose(handles, config, solver, grasp, orientation)
    stages["grasp_open"] = capture_stage(
        "grasp_open", handles, config, cameras, output_dir
    )

    first = stages["home"]["cameras"]
    last = stages["grasp_open"]["cameras"]
    wrist_movement = float(np.linalg.norm(
        np.asarray(last["wrist"]["world_position_xyz_m"])
        - np.asarray(first["wrist"]["world_position_xyz_m"])
    ))
    external_drift = float(np.linalg.norm(
        np.asarray(last["external"]["world_position_xyz_m"])
        - np.asarray(first["external"]["world_position_xyz_m"])
    ))
    for stage in stages.values():
        wrist = stage["cameras"]["wrist"]
        external = stage["cameras"]["external"]
        if (np.linalg.norm(
                np.asarray(wrist["parent_relative_position_xyz_m"])
                - np.asarray(first["wrist"]["parent_relative_position_xyz_m"])
            ) > 1e-5 or np.linalg.norm(
                np.asarray(wrist["parent_relative_orientation_wxyz"])
                - np.asarray(first["wrist"]["parent_relative_orientation_wxyz"])
            ) > 1e-5 or np.linalg.norm(
                np.asarray(external["world_position_xyz_m"])
                - np.asarray(first["external"]["world_position_xyz_m"])
            ) > 1e-5 or np.linalg.norm(
                np.asarray(external["world_orientation_wxyz"])
                - np.asarray(first["external"]["world_orientation_wxyz"])
            ) > 1e-5):
            raise RuntimeError("Camera rigid-attachment check failed")
        if stage["gripper_width_m"] < 0.075:
            raise RuntimeError("Gripper did not remain open")
    if wrist_movement < 0.001 or external_drift > 1e-5:
        raise RuntimeError("Wrist did not move or external camera drifted")

    metadata = {
        "run_id": run_id,
        "repository": {
            "commit": git_value("rev-parse", "HEAD"),
            "dirty": bool(git_value("status", "--porcelain")),
        },
        "isaac_version": get_version()[0],
        "headless": args.headless,
        "config": str(args.config.resolve()),
        "wrist_transform_provenance": {
            "type": "cad_derived_nominal_optical_center",
            "source": config["cameras"]["wrist"]["calibration"],
            "viewpoint": "left RGB optical center, not ZED Mini body center",
            "nominal_stereo_baseline_m": 0.063,
            "nominal_camera_center_to_left_optical_center_m": 0.0315,
        },
        "wrist_relative_pose_check": wrist_pose_check,
        "grasp_geometry": geometry,
        "pregrasp_target_world_m": pregrasp.tolist(),
        "grasp_target_world_m": grasp.tolist(),
        "stages": stages,
        "attachment_check": {
            "wrist_world_movement_m": wrist_movement,
            "external_world_drift_m": external_drift,
            "wrist_parent_relative_pose_constant": True,
        },
        "preprocessing": (
            "Pinned openpi_client.image_tools.resize_with_pad(224, 224): "
            "PIL bilinear, centered zero padding; RGB, no flip or mirror"
        ),
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    print("=== CAPTURE ===")
    print(f"wrist moved {wrist_movement:.6f} m; "
          f"external drift {external_drift:.6f} m")
    print(f"workspace projection: "
          f"{stages['grasp_open']['workspace_projection']}")
    print(f"finger-tip projection: "
          f"{stages['grasp_open']['wrist_tip_center_projection']}")
    print(f"output directory: {output_dir}")


try:
    main()
except Exception:
    traceback.print_exc()
    raise
finally:
    simulation_app.close()
