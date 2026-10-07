#!/usr/bin/env python3
"""Infer once at HOME, then execute eight DROID actions at 15 Hz in Isaac."""

from __future__ import annotations

import argparse
import datetime as dt
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
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.version import get_version

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "sim" / "src"))
sys.path.insert(0, str(REPO_ROOT / "third_party/openpi/packages/openpi-client/src"))

from isaac_fr3.cameras import camera_metadata, create_cameras
from isaac_fr3.droid_observation import capture_observation
from isaac_fr3.policy_execution import execute_chunk, select_actions
from isaac_fr3.scene import (
    CUBE_SETTLE_STEPS, HOME_SETTLE_STEPS, command_home, create_scene,
    load_config, settle,
)
from saps.physical.shadow_config import OPENPI_COMMIT, POLICY_CHECKPOINT, POLICY_CONFIG
from saps.policies.model_input_audit import array_evidence
from saps.policies.openpi_droid import (
    DROID_POLICY_INPUT_KEYS, OpenPiDroidPolicy, json_compatible,
    summarize_action_chunk,
)


def git_value(directory: Path, *command: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(directory), *command], text=True
    ).strip()


def write_json(path: Path, value: object) -> None:
    with path.open("x", encoding="utf-8") as file:
        json.dump(json_compatible(value), file, indent=2, allow_nan=False)
        file.write("\n")


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
    if args.host == "0.0.0.0":
        raise ValueError("Use a reachable policy-server address, not 0.0.0.0")
    config = load_config(args.config)
    if config["droid"]["control_hz"] != 15.0:
        raise ValueError("SIM-P5 requires the 15 Hz DROID baseline")
    policy = OpenPiDroidPolicy(host=args.host, port=args.port)
    metadata = policy.server_metadata
    policy.validate_policy_identity(
        config_name=POLICY_CONFIG, checkpoint=POLICY_CHECKPOINT
    )
    if metadata.get("saps_model_input_audit", {}).get("openpi_commit") != OPENPI_COMMIT:
        raise RuntimeError("Server OpenPI commit differs from pinned client")
    print("server:", metadata["saps_seeded_sampling"])
    print("OpenPI commit:", OPENPI_COMMIT)

    handles = create_scene(config)
    command_home(handles, config)
    settle(handles, HOME_SETTLE_STEPS, render=False)
    settle(handles, CUBE_SETTLE_STEPS, render=False)
    rig = create_cameras(config)
    settle(handles, 16, render=True)

    run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_id += "_" + uuid.uuid4().hex[:8]
    output = REPO_ROOT / "outputs" / "isaac_droid_one_chunk" / run_id
    output.mkdir(parents=True, exist_ok=False)
    gpu_before, ram_before = gpu_snapshot(), ram_snapshot()
    request, fingers, width, stamps, capture_seconds, prep_seconds = (
        capture_observation(handles, rig, config, args.prompt)
    )
    if tuple(request) != DROID_POLICY_INPUT_KEYS:
        raise RuntimeError("Noncanonical DROID request")
    for role, key in (
        ("exterior", "observation/exterior_image_1_left"),
        ("wrist", "observation/wrist_image_left"),
    ):
        Image.fromarray(request[key], mode="RGB").save(output / f"{role}_raw.png")
    observed_q = request["observation/joint_position"].copy()
    write_json(output / "observation.json", {
        "request_keys": list(request), "prompt": request["prompt"],
        "arrays": {
            key: array_evidence(value) for key, value in request.items()
            if isinstance(value, np.ndarray)
        },
        "joint_position_rad": observed_q.tolist(),
        "finger_positions_m": fingers.tolist(), "finger_width_m": width,
        "gripper_position": request["observation/gripper_position"].tolist(),
        "capture_timestamps_unix_ns": stamps,
        "exterior_image_path": "exterior_raw.png", "wrist_image_path": "wrist_raw.png",
    })
    cube_position, cube_orientation = handles.target.get_world_pose()
    write_json(output / "provenance.json", {
        "repository_commit": git_value(REPO_ROOT, "rev-parse", "HEAD"),
        "repository_dirty": bool(git_value(REPO_ROOT, "status", "--porcelain")),
        "openpi_submodule_commit": git_value(REPO_ROOT / "third_party/openpi", "rev-parse", "HEAD"),
        "isaac_version": get_version()[0],
        "scene_config": str(args.config.resolve()),
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
        "initial_cube_pose": {
            "position_xyz_m": np.asarray(cube_position).reshape(-1).tolist(),
            "orientation_wxyz": np.asarray(cube_orientation).reshape(-1).tolist(),
        },
    })

    infer_q_before = np.asarray(handles.fr3.get_dof_positions()).reshape(-1).copy()
    infer_start = time.perf_counter()
    response = policy.infer(
        request, policy_episode_seed=args.policy_episode_seed,
        replan_index=args.replan_index,
    )
    infer_elapsed = time.perf_counter() - infer_start
    infer_q_after = np.asarray(handles.fr3.get_dof_positions()).reshape(-1).copy()
    if not np.array_equal(infer_q_before, infer_q_after):
        raise RuntimeError("Isaac articulation changed during held inference")
    selected, suffix_count = select_actions(response.actions)
    np.save(output / "actions.npy", response.actions, allow_pickle=False)
    np.save(output / "selected_actions.npy", selected, allow_pickle=False)
    write_json(output / "policy_response.json", {
        "response_keys": list(response.response_keys),
        "native_horizon": int(response.actions.shape[0]),
        "native_shape": list(response.actions.shape),
        "native_dtype": str(response.actions.dtype),
        "actions": response.actions.tolist(),
        "selected_action_count": len(selected),
        "discarded_suffix_count": suffix_count,
        "selected_action_indices": list(range(len(selected))),
        "per_dimension_summary": summarize_action_chunk(response.actions),
        "sampling_metadata": response.sampling_metadata,
        "policy_timing": response.policy_timing,
        "server_timing": response.server_timing,
        "inference_hold_q_before": infer_q_before.tolist(),
        "inference_hold_q_after": infer_q_after.tolist(),
    })

    execution = execute_chunk(
        handles, config, selected,
        physics_dt=SimulationManager.get_physics_dt(), render=False,
    )
    execution["native_horizon"] = int(response.actions.shape[0])
    execution["selected_action_count"] = len(selected)
    execution["discarded_suffix_count"] = suffix_count
    write_json(output / "execution.json", execution)
    write_json(output / "timing.json", {
        "capture_seconds": capture_seconds,
        "request_preparation_seconds": prep_seconds,
        "inference_wall_seconds": infer_elapsed,
        "client_round_trip_seconds": response.client_round_trip_seconds,
        "server_policy_timing": response.policy_timing,
        "server_reported_timing": response.server_timing,
        "simulated_execution_seconds": execution["simulated_execution_seconds"],
        "wall_execution_seconds": execution["wall_execution_seconds"],
        "gpu_before": gpu_before, "gpu_after": gpu_snapshot(),
        "ram_before_kib": ram_before, "ram_after_kib": ram_snapshot(),
    })
    print("native horizon:", response.actions.shape[0],
          "selected:", len(selected), "suffix:", suffix_count)
    print("execution status:", execution["status"],
          "actions executed:", execution["actions_executed"])
    print("final q:", execution["final_q_rad"])
    print("terminal hold:", execution["terminal_hold"])
    print("output directory:", output)
    if execution["status"] != "complete":
        raise RuntimeError("One-chunk execution aborted; inspect execution.json")


try:
    main()
except Exception:
    traceback.print_exc()
    raise
finally:
    simulation_app.close()
