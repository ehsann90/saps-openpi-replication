#!/usr/bin/env python3
"""Validate the separate FR3–Robotiq assembly without policy inference."""

from __future__ import annotations

import argparse
import datetime as dt
import json
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
from pxr import PhysxSchema, PhysicsSchemaTools, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation
from omni.physics.core import get_physics_simulation_interface

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "sim/src"))
sys.path.insert(0, str(REPO_ROOT / "third_party/openpi/packages/openpi-client/src"))

from isaac_fr3.cameras import camera_metadata, capture_rgb, create_cameras
from isaac_fr3.grasp_poses import make_solver
from isaac_fr3.robotiq_gripper import (
    RobotiqGripperController, measured_opening,
)
from isaac_fr3.scene import (
    CUBE_SETTLE_STEPS, HOME_SETTLE_STEPS, command_home, create_scene,
    get_tcp_pose, load_config, settle,
)
from openpi_client.image_tools import resize_with_pad


def matrix(position: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    value = np.eye(4)
    value[:3, :3] = rotation
    value[:3, 3] = position
    return value


def prim_world_matrix(stage, path: str) -> np.ndarray:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid() or not prim.IsActive():
        raise RuntimeError(f"Missing active prim: {path}")
    # Gf uses row vectors; the CAD package uses column vectors.
    return np.asarray(
        UsdGeom.XformCache().GetLocalToWorldTransform(prim), dtype=np.float64
    ).T


def transform_error(actual: np.ndarray, expected: np.ndarray) -> dict:
    difference = np.linalg.inv(expected) @ actual
    return {
        "translation_m": float(np.linalg.norm(difference[:3, 3])),
        "rotation_rad": float(Rotation.from_matrix(
            difference[:3, :3]
        ).magnitude()),
    }


class ContactEvidence:
    """Record loaded pad/cube contacts from PhysX event data."""

    def __init__(self, stage, spec: dict):
        self.phase = "HOME"
        self.events: list[dict] = []
        self.root = spec["body_prim"].rsplit("/base_link", 1)[0]
        for path in (spec["left_pad_prim"], spec["right_pad_prim"],
                     "/World/TargetCube"):
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"Contact body is missing: {path}")
            PhysxSchema.PhysxContactReportAPI.Apply(
                prim
            ).CreateThresholdAttr().Set(0.0)
        self.subscription = (
            get_physics_simulation_interface()
            .subscribe_physics_contact_report_events(self._on_events)
        )
        if self.subscription is None:
            raise RuntimeError("PhysX contact subscription failed")

    def _on_events(self, headers, contacts, _anchors) -> None:
        for header in headers:
            pair = [str(PhysicsSchemaTools.intToSdfPath(value))
                    for value in (header.collider0, header.collider1)]
            if not any(path.startswith("/World/TargetCube") for path in pair):
                continue
            for side in ("left", "right"):
                if not any(path.startswith(self.root + "/" + side)
                           for path in pair):
                    continue
                data = contacts[
                    header.contact_data_offset:
                    header.contact_data_offset + header.num_contact_data
                ]
                impulse = sum(float(np.linalg.norm([
                    item.impulse.x, item.impulse.y, item.impulse.z
                ])) for item in data)
                self.events.append({
                    "phase": self.phase, "side": side, "colliders": pair,
                    "point_count": len(data), "impulse_ns": impulse,
                    "min_separation_m": min(
                        (float(item.separation) for item in data),
                        default=None,
                    ),
                })

    def summary(self, phase: str, side: str) -> dict:
        items = [event for event in self.events
                 if event["phase"] == phase and event["side"] == side
                 and event["impulse_ns"] > 1e-6]
        return {
            "loaded_events": len(items),
            "total_impulse_ns": sum(item["impulse_ns"] for item in items),
            "minimum_separation_m": min(
                (item["min_separation_m"] for item in items
                 if item["min_separation_m"] is not None), default=None
            ),
            "first_colliders": items[0]["colliders"] if items else None,
        }


def run() -> None:
    config = load_config(args.config)
    if config.get("gripper", {}).get("kind") != "robotiq_2f85":
        raise ValueError("This validator requires the Robotiq embodiment")
    source = json.loads((REPO_ROOT / "sim/assets/FR3_2F85_ZED_transforms/"
                         "fr3_2f85_zed_transforms.json").read_text())
    handles = create_scene(config)
    stage = handles.world.stage
    spec = config["gripper"]
    root = spec["body_prim"].rsplit("/base_link", 1)[0]
    flange_path = "/World/fr3/fr3_link8"
    link7_path = "/World/fr3/fr3_link7"
    camera_path = config["cameras"]["wrist"]["prim_path"]
    contact = ContactEvidence(stage, spec)
    command_home(handles, config)
    settle(handles, HOME_SETTLE_STEPS + CUBE_SETTLE_STEPS, render=False)
    rig = create_cameras(config)
    settle(handles, 16, render=True)
    solver = make_solver(config)

    run_id = (dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
              + "_" + uuid.uuid4().hex[:8])
    output = REPO_ROOT / "outputs/isaac_robotiq_validation" / run_id
    output.mkdir(parents=True, exist_ok=False)
    print("output directory:", output, flush=True)

    cad_hand = np.asarray(source["matrices"]["T_flange_from_hand_cad"])
    visual_path = root + "/base_link/visuals"
    visual = stage.GetPrimAtPath(visual_path)
    if not visual.IsValid():
        raise RuntimeError("Missing upstream base CAD visual registration")
    usd_base_from_cad = np.asarray(
        UsdGeom.Xformable(visual).GetLocalTransformation(), dtype=np.float64
    ).T
    flange_from_base = cad_hand @ np.linalg.inv(usd_base_from_cad)
    flange_from_camera = np.asarray(
        source["matrices"]["T_flange_from_usd_camera_nominal"]
    )
    articulation_roots = [str(prim.GetPath()) for prim in stage.Traverse()
                          if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    joint_names = list(handles.fr3._dof_names)
    if (articulation_roots != ["/World/fr3"] or len(joint_names) != 13
            or joint_names[:7] != config["robot"]["arm_dof_names"]
            or joint_names[7] != spec["driver_joint"]):
        raise RuntimeError("Combined articulation does not match FR3 + 2F-85")
    if stage.GetPrimAtPath(root + "/root_joint").IsActive():
        raise RuntimeError("Standalone Robotiq root joint remains active")
    if stage.GetPrimAtPath("/World/fr3/fr3_hand").IsActive():
        raise RuntimeError("Stock Franka Hand remains active")
    drive_path = root + "/Joints/finger_joint"
    drive_force = stage.GetPrimAtPath(drive_path).GetAttribute(
        "drive:angular:physics:maxForce"
    ).Get()
    if drive_force != 10.0:
        raise RuntimeError(f"Unexpected Robotiq drive force: {drive_force}")

    stages = {}
    cameras = {"wrist": rig.wrist, "external": rig.external}

    def capture(label: str) -> dict:
        handles.world.render()
        handles.world.render()
        images = {}
        for role, camera in cameras.items():
            rgb = capture_rgb(camera, config["cameras"][role]["resolution_wh"])
            processed = resize_with_pad(rgb, 224, 224)
            if processed.shape != (224, 224, 3) or processed.dtype != np.uint8:
                raise RuntimeError("OpenPI camera preprocessing changed")
            raw_name = f"{role}_raw_{label}.png"
            processed_name = f"{role}_openpi_224_{label}.png"
            Image.fromarray(rgb, mode="RGB").save(output / raw_name)
            Image.fromarray(processed, mode="RGB").save(output / processed_name)
            images[role] = {
                "raw": raw_name, "openpi_224": processed_name,
                "raw_shape_hwc": list(rgb.shape),
                "openpi_shape_hwc": list(processed.shape),
                "dtype": str(rgb.dtype), "color_order": "RGB",
                "metadata": camera_metadata(
                    camera, config["cameras"][role]
                ),
            }
        flange = prim_world_matrix(stage, flange_path)
        link7 = prim_world_matrix(stage, link7_path)
        body = prim_world_matrix(stage, spec["body_prim"])
        camera = prim_world_matrix(stage, camera_path)
        width, closure = measured_opening(handles, config)
        tcp_pos, tcp_quat = get_tcp_pose(config)
        cube_pos, cube_quat = handles.target.get_world_pose()
        row = {
            "arm_q_rad": np.asarray(handles.fr3.get_dof_positions()).reshape(-1)[
                handles.arm_indices
            ].tolist(),
            "driver_q_rad": float(np.asarray(
                handles.fr3.get_dof_positions()
            ).reshape(-1)[np.asarray(handles.finger_indices).reshape(-1)[0]]),
            "width_m": width, "droid_gripper_scalar": closure.tolist(),
            "tcp_position_xyz_m": np.asarray(tcp_pos).reshape(3).tolist(),
            "tcp_orientation_wxyz": np.asarray(tcp_quat).reshape(4).tolist(),
            "cube_position_xyz_m": np.asarray(cube_pos).reshape(3).tolist(),
            "cube_orientation_wxyz": np.asarray(cube_quat).reshape(4).tolist(),
            "flange_from_base": (np.linalg.inv(flange) @ body).tolist(),
            "flange_from_camera": (np.linalg.inv(flange) @ camera).tolist(),
            "link7_from_flange": (np.linalg.inv(link7) @ flange).tolist(),
            "base_transform_error": transform_error(
                np.linalg.inv(flange) @ body, flange_from_base
            ),
            "camera_transform_error": transform_error(
                np.linalg.inv(flange) @ camera, flange_from_camera
            ),
            "images": images,
        }
        stages[label] = row
        print(label, "width", width, "cube", row["cube_position_xyz_m"],
              "base_error", row["base_transform_error"],
              "camera_error", row["camera_transform_error"], flush=True)
        return row

    home = capture("home")
    left_path = spec["left_pad_prim"]
    right_path = spec["right_pad_prim"]
    sweep = []
    for target in (0.0, 0.2, 0.4, 0.6, spec["closed_joint_rad"], 0.0):
        handles.fr3.set_dof_position_targets(
            np.asarray([[target]], dtype=np.float32),
            dof_indices=handles.finger_indices,
        )
        settle(handles, 180, render=False)
        width, scalar = measured_opening(handles, config)
        bounds = UsdGeom.BBoxCache(
            0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
        )
        left = bounds.ComputeWorldBound(
            stage.GetPrimAtPath(left_path)
        ).ComputeAlignedBox()
        right = bounds.ComputeWorldBound(
            stage.GetPrimAtPath(right_path)
        ).ComputeAlignedBox()
        visual_gap = float(right.GetMin()[1] - left.GetMax()[1])
        measured = np.asarray(handles.fr3.get_dof_positions()).reshape(-1)
        sweep.append({
            "target_joint_rad": target,
            "measured_joint_rad": float(measured[
                np.asarray(handles.finger_indices).reshape(-1)[0]
            ]),
            "measured_width_m": width,
            "visual_pad_gap_world_y_m": visual_gap,
            "width_minus_visual_gap_m": width - max(0.0, visual_gap),
            "droid_gripper_scalar": scalar.tolist(),
        })
    if max(abs(row["width_minus_visual_gap_m"]) for row in sweep) > 0.001:
        raise RuntimeError("Robotiq pad-opening measurement disagrees with mesh")

    q = np.asarray(handles.fr3.get_dof_positions()).reshape(-1)[:7]
    fk_position, fk_rotation = solver.compute_forward_kinematics(
        "fr3_hand_tcp", q
    )
    w, x, y, z = home["tcp_orientation_wxyz"]
    home_grip = matrix(
        np.asarray(home["tcp_position_xyz_m"]),
        Rotation.from_quat([x, y, z, w]).as_matrix(),
    )
    virtual_from_grip = np.linalg.inv(
        matrix(fk_position, fk_rotation)
    ) @ home_grip

    def move(label: str, target_xyz: list[float]) -> dict:
        nonlocal q
        contact.phase = label
        target = matrix(np.asarray(target_xyz), home_grip[:3, :3])
        virtual = target @ np.linalg.inv(virtual_from_grip)
        xyzw = Rotation.from_matrix(virtual[:3, :3]).as_quat()
        wxyz = np.asarray([xyzw[3], *xyzw[:3]])
        q, solved = solver.compute_inverse_kinematics(
            "fr3_hand_tcp", virtual[:3, 3], wxyz, q
        )
        if not solved or not np.isfinite(q).all():
            raise RuntimeError(f"FR3 IK failed for {label}")
        handles.fr3.set_dof_position_targets(
            np.asarray([q], dtype=np.float32),
            dof_indices=handles.arm_indices,
        )
        settle(handles, 180, render=False)
        row = capture(label)
        row["tcp_target_xyz_m"] = target_xyz
        row["tcp_position_error_m"] = float(np.linalg.norm(
            np.asarray(row["tcp_position_xyz_m"]) - target_xyz
        ))
        return row

    move("pregrasp", [0.55, 0.0, 0.535])
    move("grasp_open", [0.55, 0.0, 0.435])
    controller = RobotiqGripperController()
    contact.phase = "CLOSE"
    if not controller.request(handles, config, 0.9):
        raise RuntimeError("Robotiq CLOSE did not command the driver")
    settle(handles, 240, render=False)
    closed = capture("closed")
    move("lift", [0.55, 0.0, 0.505])
    contact.phase = "RELEASE"
    if not controller.request(handles, config, 0.0):
        raise RuntimeError("Robotiq OPEN did not command the driver")
    settle(handles, 180, render=False)
    released = capture("released")

    contact_summary = {
        phase: {side: contact.summary(phase, side)
                for side in ("left", "right")}
        for phase in ("CLOSE", "lift", "RELEASE")
    }
    table_top = (config["table"]["position_m"][2]
                 + config["table"]["size_m"][2] / 2)
    cube_half_height = config["target_object"]["size_m"][2] / 2
    max_contact_penetration = min(
        contact_summary[phase][side]["minimum_separation_m"]
        for phase in ("CLOSE", "lift") for side in ("left", "right")
        if contact_summary[phase][side]["minimum_separation_m"] is not None
    )
    max_base_translation = max(
        row["base_transform_error"]["translation_m"] for row in stages.values()
    )
    max_base_rotation = max(
        row["base_transform_error"]["rotation_rad"] for row in stages.values()
    )
    max_camera_translation = max(
        row["camera_transform_error"]["translation_m"]
        for row in stages.values()
    )
    max_camera_rotation = max(
        row["camera_transform_error"]["rotation_rad"]
        for row in stages.values()
    )
    link7_from_flange = np.asarray(home["link7_from_flange"])
    flange_datum_m = float(np.linalg.norm(link7_from_flange[:3, 3]))
    flags = {
        "single_articulation": articulation_roots == ["/World/fr3"],
        "flange_datum_107mm": abs(flange_datum_m - 0.107) < 0.001,
        "mount_transform_stable": max_base_translation < 1e-4
                                  and max_base_rotation < 1e-4,
        "camera_transform_stable": max_camera_translation < 1e-4
                                   and max_camera_rotation < 1e-4,
        "camera_intrinsics": (
            home["images"]["wrist"]["metadata"]["horizontal_fov_deg"]
            == 82.19068145751953
            and home["images"]["external"]["metadata"]
            ["horizontal_fov_deg"] == 101.5525131225586
        ),
        "open_close": sweep[0]["measured_width_m"] > 0.08
                      and sweep[4]["measured_width_m"] < 0.002,
        "bilateral_loaded_close": all(
            contact_summary["CLOSE"][side]["loaded_events"] > 0
            for side in ("left", "right")
        ),
        "bilateral_loaded_lift": all(
            contact_summary["lift"][side]["loaded_events"] > 0
            for side in ("left", "right")
        ),
        "cube_lifted": stages["lift"]["cube_position_xyz_m"][2]
                       > home["cube_position_xyz_m"][2] + 0.04,
        "contact_penetration_under_2mm": max_contact_penetration >= -0.002,
        "released": released["width_m"] > 0.08
                    and released["cube_position_xyz_m"][2]
                    <= table_top + cube_half_height + 0.01,
    }
    results = {
        "run_id": run_id, "config": str(args.config.resolve()),
        "source_sha256": source["source_sha256"],
        "robotiq_source_revision": spec["source_revision"],
        "variant": "Physx_parallel_grip / Standard",
        "fr3_articulation_roots": articulation_roots,
        "joint_names": joint_names,
        "drive_max_force_nm": drive_force,
        "measured_flange_datum_m": flange_datum_m,
        "maximum_transform_errors": {
            "base_translation_m": max_base_translation,
            "base_rotation_rad": max_base_rotation,
            "camera_translation_m": max_camera_translation,
            "camera_rotation_rad": max_camera_rotation,
        },
        "flange_from_usd_base_expected": flange_from_base.tolist(),
        "flange_from_camera_expected": flange_from_camera.tolist(),
        "virtual_ik_tcp_from_grip_tcp": virtual_from_grip.tolist(),
        "grip_tcp_from_usd_base_z_m": 0.13371706760793903,
        "source_flange_datum_offset_mm": source["provenance"]["flange_datum_offset_mm"],
        "width_sweep": sweep, "stages": stages,
        "contact_summary": contact_summary,
        "maximum_contact_penetration_m": max_contact_penetration,
        "flags": flags,
    }
    with (output / "metadata.json").open("w") as file:
        json.dump(results, file, indent=2, allow_nan=False)
        file.write("\n")
    with (output / "contact_events.json").open("w") as file:
        json.dump(contact.events, file, indent=2, allow_nan=False)
        file.write("\n")
    print("validation flags:", flags, "output:", output, flush=True)
    if not all(flags.values()):
        raise RuntimeError("Robotiq mechanical validation did not pass")


try:
    run()
except Exception:
    traceback.print_exc()
    sys.exit(1)
finally:
    simulation_app.close()
