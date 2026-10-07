#!/usr/bin/env python3
"""Send one settled Isaac FR3 observation to pi05_droid without actuation."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import subprocess
import sys
import time
import traceback
import uuid
from pathlib import Path

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--prompt", default="Pick up the red object")
    parser.add_argument("--policy-episode-seed", type=int, default=20260827)
    parser.add_argument("--replan-index", type=int, default=0)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


args = parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import numpy as np
from PIL import Image
from isaacsim.core.version import get_version

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "sim" / "src"))
sys.path.insert(0, str(REPO_ROOT / "third_party/openpi/packages/openpi-client/src"))

from isaac_fr3.cameras import camera_metadata, create_cameras
from isaac_fr3.droid_observation import capture_observation
from isaac_fr3.scene import (
    CUBE_SETTLE_STEPS, HOME_SETTLE_STEPS, command_home, create_scene,
    load_config, settle,
)
from saps.physical.shadow_audit import save_model_audit
from saps.physical.shadow_config import OPENPI_COMMIT, POLICY_CHECKPOINT, POLICY_CONFIG
from saps.policies.model_input_audit import array_evidence
from saps.policies.openpi_droid import (
    DROID_POLICY_INPUT_KEYS, OpenPiDroidPolicy, json_compatible,
    summarize_action_chunk, validate_droid_action_response,
)
from openpi_client.image_tools import resize_with_pad


def git_value(directory: Path, *command: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(directory), *command], text=True
    ).strip()


def write_json(path: Path, value: object) -> None:
    with path.open("x", encoding="utf-8") as file:
        json.dump(json_compatible(value), file, indent=2, allow_nan=False)
        file.write("\n")


def array_stats(value: np.ndarray) -> dict:
    array = np.asarray(value)
    numeric = array.astype(np.float64)
    return {
        **array_evidence(array),
        "minimum": float(np.min(numeric)),
        "maximum": float(np.max(numeric)),
        "mean": float(np.mean(numeric)),
        "standard_deviation": float(np.std(numeric)),
    }


def gpu_snapshot() -> str | None:
    result = subprocess.run(
        ["nvidia-smi", "--query-gpu=memory.used,memory.total,utilization.gpu",
         "--format=csv,noheader,nounits"],
        capture_output=True, text=True, check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def ram_snapshot() -> dict:
    values = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        name, _, value = line.partition(":")
        if name in ("MemTotal", "MemAvailable"):
            values[name] = int(value.strip().split()[0])
    return values


def main() -> None:
    config = load_config(args.config)
    if args.host == "0.0.0.0":
        raise ValueError("Use a reachable policy-server address, not 0.0.0.0")
    policy = OpenPiDroidPolicy(host=args.host, port=args.port)
    metadata = policy.server_metadata
    policy.validate_policy_identity(
        config_name=POLICY_CONFIG, checkpoint=POLICY_CHECKPOINT
    )
    audit_identity = metadata.get("saps_model_input_audit", {})
    if audit_identity.get("openpi_commit") != OPENPI_COMMIT:
        raise RuntimeError("Server OpenPI commit differs from the pinned client")
    print("server metadata:", json.dumps(json_compatible(metadata), indent=2))
    print("policy config:", POLICY_CONFIG)
    print("checkpoint:", POLICY_CHECKPOINT)
    print("OpenPI commit:", audit_identity["openpi_commit"])
    print("prompt:", repr(args.prompt))

    handles = create_scene(config)
    command_home(handles, config)
    settle(handles, HOME_SETTLE_STEPS, render=False)
    settle(handles, CUBE_SETTLE_STEPS, render=False)
    rig = create_cameras(config)
    settle(handles, 16, render=True)

    run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_id += "_" + uuid.uuid4().hex[:8]
    output = REPO_ROOT / "outputs" / "isaac_droid_shadow" / run_id
    output.mkdir(parents=True, exist_ok=False)
    gpu_before, ram_before = gpu_snapshot(), ram_snapshot()
    cycle_start = time.perf_counter_ns()
    request, fingers, width, stamps, capture_seconds, prep_seconds = (
        capture_observation(handles, rig, config, args.prompt)
    )
    if tuple(request) != DROID_POLICY_INPUT_KEYS:
        raise RuntimeError("Noncanonical DROID request")
    image_keys = {
        "exterior": "observation/exterior_image_1_left",
        "wrist": "observation/wrist_image_left",
    }
    image_records = {}
    for role, key in image_keys.items():
        raw = request[key]
        raw_path = output / f"{role}_raw.png"
        request_path = output / f"{role}_request.png"
        Image.fromarray(raw, mode="RGB").save(raw_path)
        Image.fromarray(request[key], mode="RGB").save(request_path)
        image_records[role] = {
            "raw": {"path": raw_path.name, **array_stats(raw)},
            "transmitted": {"path": request_path.name,
                            **array_stats(request[key])},
            "exact_raw_request_bytes": bool(np.array_equal(raw, request[key])),
            "raw_png_sha256": hashlib.sha256(raw_path.read_bytes()).hexdigest(),
            "request_png_sha256": hashlib.sha256(
                request_path.read_bytes()).hexdigest(),
        }
    write_json(output / "request.json", {
        "keys": list(request), "prompt": request["prompt"],
        "joint_position": request["observation/joint_position"].tolist(),
        "gripper_position": request["observation/gripper_position"].tolist(),
        "arrays": {
            key: array_stats(value) for key, value in request.items()
            if isinstance(value, np.ndarray)
        },
        "images": image_records,
        "policy_episode_seed": args.policy_episode_seed,
        "replan_index": args.replan_index,
        "audit_model_input": True,
    })
    object_states = []
    for index, cube in enumerate(handles.cubes):
        position, orientation = cube.get_world_pose()
        object_states.append({
            "index": index, "position_xyz_m": np.asarray(position).reshape(-1).tolist(),
            "orientation_wxyz": np.asarray(orientation).reshape(-1).tolist(),
        })
    write_json(output / "provenance.json", {
        "repository_commit": git_value(REPO_ROOT, "rev-parse", "HEAD"),
        "repository_dirty": bool(git_value(REPO_ROOT, "status", "--porcelain")),
        "openpi_submodule_commit": git_value(REPO_ROOT / "third_party/openpi", "rev-parse", "HEAD"),
        "isaac_version": get_version()[0], "scene_config": str(args.config.resolve()),
        "camera_specs": config["cameras"],
        "camera_measured": {
            "external": camera_metadata(rig.external, config["cameras"]["external"]),
            "wrist": camera_metadata(rig.wrist, config["cameras"]["wrist"]),
        },
        "policy_host": args.host, "policy_port": args.port,
        "server_metadata": metadata, "policy_config": POLICY_CONFIG,
        "checkpoint": POLICY_CHECKPOINT, "prompt": args.prompt,
        "policy_episode_seed": args.policy_episode_seed,
        "replan_index": args.replan_index,
        "arm_joint_names": config["robot"]["arm_dof_names"],
        "q_measured_rad": request["observation/joint_position"].tolist(),
        "finger_joint_names": config["robot"]["finger_dof_names"],
        "finger_positions_m": fingers.tolist(), "finger_width_m": width,
        "droid_gripper_scalar": request["observation/gripper_position"].tolist(),
        "objects": object_states, "policy_actions_executed": 0,
    })

    stamps["request_send_ns"] = time.time_ns()
    response = policy.infer(
        request, policy_episode_seed=args.policy_episode_seed,
        replan_index=args.replan_index, audit_model_input=True,
    )
    stamps["response_receive_ns"] = time.time_ns()
    total_seconds = (time.perf_counter_ns() - cycle_start) / 1e9
    actions = validate_droid_action_response({"actions": response.actions})
    np.save(output / "actions.npy", actions, allow_pickle=False)
    write_json(output / "actions.json", actions.tolist())
    audit_summary = None
    if response.model_input_audit is not None:
        audit_summary = save_model_audit(
            response.model_input_audit, request, output / "model_input"
        )
        audit_summary["statistics"] = {
            group: {
                name: array_stats(value) for name, value in
                response.model_input_audit[group].items()
            }
            for group in ("transformed_images", "model_images")
        }
        audit_summary["state"] = array_stats(response.model_input_audit["state"])
        audit_summary["request_image_evidence"] = image_records
        audit_summary["resize_verification"] = {}
        for role, key, model_name in (
            ("exterior", image_keys["exterior"], "base_0_rgb"),
            ("wrist", image_keys["wrist"], "left_wrist_0_rgb"),
        ):
            expected = resize_with_pad(request[key], 224, 224)
            actual = response.model_input_audit["transformed_images"][model_name]
            matches = bool(np.array_equal(expected, actual))
            audit_summary["resize_verification"][role] = {
                "pinned_client_resize_matches_server": matches,
                "expected": array_evidence(expected),
            }
            if not matches:
                raise RuntimeError(f"Server {role} resize differs from pinned client")
        write_json(output / "model_input_audit.json", audit_summary)
    write_json(output / "response.json", {
        "response_keys": response.response_keys,
        "actions": actions.tolist(), "shape": list(actions.shape),
        "dtype": str(actions.dtype), "first_action": actions[0].tolist(),
        "last_action": actions[-1].tolist(),
        "per_dimension_summary": summarize_action_chunk(actions),
        "sampling_metadata": response.sampling_metadata,
        "policy_timing": response.policy_timing,
        "server_timing": response.server_timing,
        "model_input_audit_available": audit_summary is not None,
    })
    skew_ns = max(stamps[key] for key in (
        "exterior_image_capture_ns", "wrist_image_capture_ns",
        "joint_state_read_ns", "gripper_state_read_ns",
    )) - min(stamps[key] for key in (
        "exterior_image_capture_ns", "wrist_image_capture_ns",
        "joint_state_read_ns", "gripper_state_read_ns",
    ))
    write_json(output / "timing.json", {
        "timestamps_unix_ns": stamps,
        "observation_skew_seconds": skew_ns / 1e9,
        "observation_capture_seconds": capture_seconds,
        "request_preparation_seconds": prep_seconds,
        "client_round_trip_seconds": response.client_round_trip_seconds,
        "server_policy_timing": response.policy_timing,
        "server_reported_timing": response.server_timing,
        "server_transport_timing": None,
        "total_shadow_cycle_seconds": total_seconds,
        "gpu_before": gpu_before, "gpu_after": gpu_snapshot(),
        "ram_before_kib": ram_before, "ram_after_kib": ram_snapshot(),
    })
    print("request keys:", list(request))
    print("measured q:", request["observation/joint_position"].tolist())
    print("finger positions:", fingers.tolist(), "width:", width,
          "DROID gripper:", request["observation/gripper_position"].tolist())
    print("actions:", actions.shape, actions.dtype)
    print("first:", actions[0].tolist(), "last:", actions[-1].tolist())
    print("per-dimension:", json.dumps(summarize_action_chunk(actions)))
    print("round trip seconds:", response.client_round_trip_seconds)
    print("server policy timing:", response.policy_timing)
    print("server timing:", response.server_timing)
    print("observation skew seconds:", skew_ns / 1e9)
    print("output directory:", output)


try:
    main()
except Exception:
    traceback.print_exc()
    raise
finally:
    simulation_app.close()
