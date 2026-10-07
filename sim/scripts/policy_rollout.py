#!/usr/bin/env python3
"""Run an unlimited seeded DROID rollout with Isaac held during inference."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import signal
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
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-replans", type=int, default=None,
                        help="Explicit diagnostic cap; default runs indefinitely")
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
from isaac_fr3.policy_execution import (
    GripperRuntime, InvalidPolicyResponse, command_fresh_hold,
    execute_chunk, run_rollout_loop,
    select_actions,
)
from isaac_fr3.scene import (
    CUBE_SETTLE_STEPS, HOME_SETTLE_STEPS, command_home, create_scene,
    get_tcp_pose, load_config, settle,
)
from saps.physical.shadow_config import OPENPI_COMMIT, POLICY_CHECKPOINT, POLICY_CONFIG
from saps.policies.model_input_audit import array_evidence
from saps.policies.openpi_droid import (
    DROID_POLICY_INPUT_KEYS, OpenPiDroidPolicy, json_compatible,
    summarize_action_chunk,
)


def _raise_keyboard_interrupt(_signum, _frame) -> None:
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, _raise_keyboard_interrupt)


def git_value(directory: Path, *command: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(directory), *command], text=True
    ).strip()


def write_json(path: Path, value: object) -> None:
    temporary = path.with_name(path.name + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8") as file:
            json.dump(json_compatible(value), file, indent=2, allow_nan=False)
            file.write("\n")
        os.replace(temporary, path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def pose(position, orientation) -> dict:
    return {
        "position_xyz_m": np.asarray(position).reshape(-1).tolist(),
        "orientation_wxyz": np.asarray(orientation).reshape(-1).tolist(),
    }


def object_poses(handles) -> list[dict]:
    return [
        {"index": index, **pose(*cube.get_world_pose())}
        for index, cube in enumerate(handles.cubes)
    ]


def measured_dofs(handles, indices) -> list[float]:
    selected = np.asarray(indices).astype(int).reshape(-1)
    positions = np.asarray(handles.fr3.get_dof_positions()).reshape(-1)
    return positions[selected].tolist()


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
        raise ValueError("Use a reachable server address, not 0.0.0.0")
    if args.max_replans is not None and args.max_replans <= 0:
        raise ValueError("max_replans must be positive")
    config = load_config(args.config)
    if config["droid"]["control_hz"] != 15.0:
        raise ValueError("Rollout requires the validated 15 Hz DROID baseline")
    policy = OpenPiDroidPolicy(host=args.host, port=args.port)
    metadata = policy.server_metadata
    policy.validate_policy_identity(
        config_name=POLICY_CONFIG, checkpoint=POLICY_CHECKPOINT
    )
    if metadata.get("saps_model_input_audit", {}).get("openpi_commit") != OPENPI_COMMIT:
        raise RuntimeError("Server OpenPI commit differs from pinned client")

    handles = create_scene(config)
    command_home(handles, config)
    settle(handles, HOME_SETTLE_STEPS, render=False)
    settle(handles, CUBE_SETTLE_STEPS, render=False)
    rig = create_cameras(config)
    settle(handles, 16, render=True)
    physics_dt = SimulationManager.get_physics_dt()
    run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_id += "_" + uuid.uuid4().hex[:8]
    output = REPO_ROOT / "outputs" / "isaac_droid_rollout" / run_id
    (output / "replans").mkdir(parents=True, exist_ok=False)
    wall_start = time.perf_counter()
    gpu_before, ram_before = gpu_snapshot(), ram_snapshot()
    gripper = GripperRuntime()
    hold_records = {}
    terminal_step_before_next_observation = None
    pause_starts = {}
    per_replan = []
    inference_wall_total = 0.0
    action_wall_total = 0.0
    paused_wall_total = 0.0
    simulation_control_total = 0.0
    transitions = []
    last_summary = {
        "inference_requests": 0, "completed_replans": 0,
        "policy_actions_executed": 0,
    }
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
        "max_replans": args.max_replans,
        "initial_arm_q_rad": measured_dofs(handles, handles.arm_indices),
        "initial_finger_positions_m": measured_dofs(handles, handles.finger_indices),
        "initial_tcp_pose": pose(*get_tcp_pose(config)),
        "initial_objects": object_poses(handles),
    })

    def current_dir(index: int) -> Path:
        return output / "replans" / f"{index:04d}"

    def persist_progress(summary: dict) -> None:
        write_json(output / "episode.json", {
            **summary, "task_outcome": "not_evaluated",
            "replans": per_replan,
            "simulation_control_seconds": simulation_control_total,
            "wall_inference_seconds": inference_wall_total,
            "wall_action_execution_seconds": action_wall_total,
            "wall_paused_for_hold_and_inference_seconds": paused_wall_total,
            "wall_total_seconds": time.perf_counter() - wall_start,
            "gripper_transitions": transitions,
            "gripper_intent": gripper.intent,
            "final_arm_q_rad": measured_dofs(handles, handles.arm_indices),
            "final_finger_positions_m": measured_dofs(handles, handles.finger_indices),
            "final_tcp_pose": pose(*get_tcp_pose(config)),
            "final_objects": object_poses(handles),
        })
        write_json(output / "timing.json", {
            "per_replan": [{key: row.get(key) for key in (
                "replan_index", "capture_seconds", "preparation_seconds",
                "inference_wall_seconds", "client_round_trip_seconds",
                "server_policy_timing", "execution_wall_seconds",
                "execution_simulated_seconds", "paused_wall_seconds",
            )} for row in per_replan],
            "wall_inference_seconds": inference_wall_total,
            "wall_action_execution_seconds": action_wall_total,
            "wall_paused_for_hold_and_inference_seconds": paused_wall_total,
            "simulation_control_seconds": simulation_control_total,
            "wall_total_seconds": time.perf_counter() - wall_start,
            "gpu_before": gpu_before, "gpu_current": gpu_snapshot(),
            "ram_before_kib": ram_before, "ram_current_kib": ram_snapshot(),
        })

    persist_progress(last_summary)
    print("output directory:", output, flush=True)
    print("max replans:", args.max_replans, "prompt:", repr(args.prompt), flush=True)

    def hold(index: int) -> None:
        if (terminal_step_before_next_observation is not None
                and handles.world.current_time_step_index
                != terminal_step_before_next_observation):
            raise RuntimeError("Physics advanced after terminal hold")
        pause_starts[index] = time.perf_counter()
        hold_records[index] = command_fresh_hold(handles)

    def capture(index: int) -> dict:
        path = current_dir(index)
        path.mkdir(parents=True, exist_ok=False)
        # Render the held state without advancing physics or restarting the
        # gripper ramp. This refreshes both RGB buffers after the last chunk.
        step_before = handles.world.current_time_step_index
        if step_before != hold_records[index]["simulation_step_index"]:
            raise RuntimeError("Physics advanced before held observation")
        handles.world.render()
        handles.world.render()
        if handles.world.current_time_step_index != step_before:
            raise RuntimeError("Rendering advanced physics during held capture")
        request, fingers, width, stamps, capture_s, prep_s = (
            capture_observation(handles, rig, config, args.prompt)
        )
        if tuple(request) != DROID_POLICY_INPUT_KEYS:
            raise RuntimeError("Noncanonical DROID observation")
        for role, key in (
            ("exterior", "observation/exterior_image_1_left"),
            ("wrist", "observation/wrist_image_left"),
        ):
            Image.fromarray(request[key], mode="RGB").save(path / f"{role}_raw.png")
        record = {
            "replan_index": index, "prompt": request["prompt"],
            "request_keys": list(request),
            "array_evidence": {
                key: array_evidence(value) for key, value in request.items()
                if isinstance(value, np.ndarray)
            },
            "measured_arm_q_rad": request[
                "observation/joint_position"
            ].tolist(),
            "finger_positions_m": fingers.tolist(), "finger_width_m": width,
            "droid_gripper_scalar": request["observation/gripper_position"].tolist(),
            "tcp_pose": pose(*get_tcp_pose(config)),
            "objects": object_poses(handles),
            "hold_before_observation": hold_records[index],
            "capture_timestamps_unix_ns": stamps,
            "capture_seconds": capture_s,
            "preparation_seconds": prep_s,
            "simulation_time_seconds": float(handles.world.current_time),
            "simulation_step_index": int(handles.world.current_time_step_index),
        }
        write_json(path / "observation.json", record)
        return request

    def infer(index: int, request: dict) -> dict:
        nonlocal inference_wall_total
        path = current_dir(index)
        step_before = int(handles.world.current_time_step_index)
        sim_before = float(handles.world.current_time)
        all_q_before = np.asarray(
            handles.fr3.get_dof_positions()
        ).reshape(-1).copy()
        arm_q_before = np.asarray(measured_dofs(
            handles, handles.arm_indices
        ))
        fingers_before = np.asarray(measured_dofs(
            handles, handles.finger_indices
        ))
        objects_before = object_poses(handles)
        start_unix_ns = time.time_ns()
        start = time.perf_counter()
        try:
            response = policy.infer(
                request, policy_episode_seed=args.policy_episode_seed,
                replan_index=index,
            )
        except (TypeError, ValueError) as error:
            raise InvalidPolicyResponse(str(error)) from error
        duration = time.perf_counter() - start
        end_unix_ns = time.time_ns()
        inference_wall_total += duration
        all_q_after = np.asarray(handles.fr3.get_dof_positions()).reshape(-1)
        arm_q_after = measured_dofs(handles, handles.arm_indices)
        fingers_after = measured_dofs(handles, handles.finger_indices)
        held = (
            handles.world.current_time_step_index == step_before
            and handles.world.current_time == sim_before
            and np.array_equal(all_q_before, all_q_after)
            and objects_before == object_poses(handles)
        )
        response_record = {
            "replan_index": index, "inference_start_unix_ns": start_unix_ns,
            "inference_end_unix_ns": end_unix_ns,
            "inference_wall_seconds": duration,
            "client_round_trip_seconds": response.client_round_trip_seconds,
            "server_policy_timing": response.policy_timing,
            "server_reported_timing": response.server_timing,
            "sampling_metadata": response.sampling_metadata,
            "response_keys": list(response.response_keys),
            "native_horizon": int(response.actions.shape[0]),
            "native_shape": list(response.actions.shape),
            "native_dtype": str(response.actions.dtype),
            "per_dimension_summary": summarize_action_chunk(response.actions),
            "simulation_time_before_seconds": sim_before,
            "simulation_time_after_seconds": float(handles.world.current_time),
            "simulation_step_before": step_before,
            "simulation_step_after": int(handles.world.current_time_step_index),
            "arm_q_before_inference_rad": arm_q_before.tolist(),
            "arm_q_after_inference_rad": arm_q_after,
            "finger_positions_before_inference_m": fingers_before.tolist(),
            "finger_positions_after_inference_m": fingers_after,
            "held_without_evolution": held,
        }
        np.save(path / "actions.npy", response.actions, allow_pickle=False)
        write_json(path / "response.json", response_record)
        if not held:
            raise RuntimeError("Simulator state evolved during inference")
        try:
            selected, suffix = select_actions(response.actions)
        except (TypeError, ValueError) as error:
            raise InvalidPolicyResponse(str(error)) from error
        np.save(path / "selected_actions.npy", selected, allow_pickle=False)
        response_record.update({
            "selected_action_count": len(selected),
            "discarded_suffix_count": suffix,
        })
        write_json(path / "response.json", response_record)
        return {"response": response, "selected": selected,
                "record": response_record}

    def execute(index: int, payload: dict) -> dict:
        nonlocal action_wall_total, paused_wall_total, simulation_control_total
        nonlocal terminal_step_before_next_observation
        path = current_dir(index)
        paused = time.perf_counter() - pause_starts[index]
        paused_wall_total += paused
        rows = []
        write_json(path / "execution.json", {
            "status": "in_progress", "actions_executed": 0, "rows": rows,
        })

        def save_action(row: dict) -> None:
            rows.append(row)
            write_json(path / "execution.json", {
                "status": "in_progress",
                "actions_executed": sum(
                    item.get("safety_result") == "accepted" for item in rows
                ),
                "rows": rows,
            })

        result = execute_chunk(
            handles, config, payload["selected"], physics_dt=physics_dt,
            render=not args.headless, gripper_runtime=gripper,
            on_action=save_action, terminal_evidence_steps=0,
        )
        result["replan_index"] = index
        result["paused_wall_seconds"] = paused
        terminal_step_before_next_observation = result[
            "terminal_hold"
        ].get("simulation_step_index")
        write_json(path / "execution.json", result)
        action_wall_total += result["wall_execution_seconds"]
        simulation_control_total += result["simulated_execution_seconds"]
        return result

    def record(index: int, request: dict, payload: dict, execution: dict) -> None:
        nonlocal last_summary
        obs = json.loads((current_dir(index) / "observation.json").read_text())
        response = payload["record"]
        row = {
            "replan_index": index,
            "observation_hashes": {
                key: value["sha256"] for key, value in obs["array_evidence"].items()
            },
            "arm_q_before_inference_rad": response[
                "arm_q_before_inference_rad"
            ],
            "finger_positions_before_inference_m": response[
                "finger_positions_before_inference_m"
            ],
            "finger_width_m": obs["finger_width_m"],
            "gripper_scalar": obs["droid_gripper_scalar"],
            "tcp_before": obs["tcp_pose"], "objects_before": obs["objects"],
            "inference_start_unix_ns": response["inference_start_unix_ns"],
            "inference_end_unix_ns": response["inference_end_unix_ns"],
            "held_without_evolution": response["held_without_evolution"],
            "native_horizon": response["native_horizon"],
            "selected_action_count": len(payload["selected"]),
            "capture_seconds": obs["capture_seconds"],
            "preparation_seconds": obs["preparation_seconds"],
            "inference_wall_seconds": response["inference_wall_seconds"],
            "client_round_trip_seconds": response["client_round_trip_seconds"],
            "server_policy_timing": response["server_policy_timing"],
            "execution_wall_seconds": execution["wall_execution_seconds"],
            "execution_simulated_seconds": execution["simulated_execution_seconds"],
            "paused_wall_seconds": execution["paused_wall_seconds"],
            "actions_executed": execution["actions_executed"],
            "terminal_hold": execution["terminal_hold"],
            "arm_q_after_rad": execution["final_q_rad"],
            "finger_positions_after_m": measured_dofs(
                handles, handles.finger_indices
            ),
            "tcp_after": execution["final_tcp_pose"],
            "objects_after": object_poses(handles),
            "gripper_transitions": execution["gripper_transitions"],
            "status": execution["status"],
        }
        per_replan.append(row)
        transitions.extend({"replan_index": index, **item}
                           for item in execution["gripper_transitions"])
        last_summary = {
            "inference_requests": index + 1,
            "completed_replans": sum(x["status"] == "complete" for x in per_replan),
            "policy_actions_executed": sum(x["actions_executed"] for x in per_replan),
        }
        persist_progress(last_summary)
        print("replan", index, "actions", execution["actions_executed"],
              "gripper", gripper.intent, "q", execution["final_q_rad"],
              "cube", row["objects_after"][0]["position_xyz_m"], flush=True)

    def emergency_hold() -> None:
        record = command_fresh_hold(handles)
        write_json(output / "interrupt_hold.json", record)

    summary = run_rollout_loop(
        hold=hold, capture=capture, infer=infer, execute=execute,
        record=record, emergency_hold=emergency_hold,
        max_replans=args.max_replans,
    )
    # A failed/interrupting request may have started but not reached record().
    summary["inference_requests"] = max(
        summary["inference_requests"],
        len([p for p in (output / "replans").iterdir()
             if (p / "response.json").exists()]),
    )
    partial_replans = []
    for directory in sorted((output / "replans").iterdir()):
        execution_path = directory / "execution.json"
        if not execution_path.exists():
            continue
        execution_record = json.loads(execution_path.read_text())
        if execution_record.get("status") == "in_progress":
            partial_replans.append({
                "replan_index": int(directory.name),
                "saved_actions_executed": execution_record["actions_executed"],
                "saved_action_rows": len(execution_record["rows"]),
            })
    summary["partial_replans"] = partial_replans
    summary["policy_actions_executed"] += sum(
        item["saved_actions_executed"] for item in partial_replans
    )
    persist_progress(summary)
    write_json(output / "termination.json", {
        "reason": summary["termination_reason"],
        "error": summary["error"],
        "task_outcome": "not_evaluated",
        "wall_timestamp_unix_ns": time.time_ns(),
        "fresh_hold_on_interrupt_or_error": (
            (output / "interrupt_hold.json").exists()
        ),
    })
    print("termination:", summary, "output directory:", output, flush=True)


try:
    main()
except Exception:
    traceback.print_exc()
    raise
finally:
    simulation_app.close()
