#!/usr/bin/env python3
"""Short Isaac camera/video check without a policy server or full rollout."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import sys
import time
import uuid
from pathlib import Path

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--config", type=Path, required=True)
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()
app = SimulationApp({"headless": args.headless})

import numpy as np
from PIL import Image
from isaacsim.core.simulation_manager import SimulationManager

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "sim/src"))

from isaac_fr3.cameras import capture_rgb, create_cameras
from isaac_fr3.policy_video import (
    PolicyVideoRecorder, export_video, write_metadata,
)
from isaac_fr3.scene import (
    CUBE_SETTLE_STEPS, HOME_SETTLE_STEPS, command_home, create_scene,
    load_config, settle,
)


def digest(pixels: np.ndarray) -> str:
    return hashlib.sha256(pixels.tobytes(order="C")).hexdigest()


def file_digest(path: Path) -> str:
    with Image.open(path) as image:
        return digest(np.asarray(image.convert("RGB")))


def main() -> None:
    config = load_config(args.config)
    handles = create_scene(config)
    command_home(handles, config)
    settle(handles, HOME_SETTLE_STEPS, render=False)
    settle(handles, CUBE_SETTLE_STEPS, render=False)
    rig = create_cameras(config)
    settle(handles, 16, render=True)
    physics_dt = float(SimulationManager.get_physics_dt())
    if not np.isclose(physics_dt, 1 / 60, atol=1e-9):
        raise RuntimeError(f"Expected 60 Hz physics, got {physics_dt}")
    run_id = (dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
              + "_" + uuid.uuid4().hex[:8])
    output = ROOT / "outputs/isaac_video_smoke" / run_id
    output.mkdir(parents=True, exist_ok=False)
    video = PolicyVideoRecorder(output / "videos", source_rollout_id=run_id)
    observation_frames = []
    before_pause_step = int(handles.world.current_time_step_index)

    def capture(index: int, action_index: int) -> None:
        step = int(handles.world.current_time_step_index)
        stamp = float(handles.world.current_time)
        start = time.perf_counter()
        exterior = capture_rgb(
            rig.external, config["cameras"]["external"]["resolution_wh"]
        )
        wrist = capture_rgb(
            rig.wrist, config["cameras"]["wrist"]["resolution_wh"]
        )
        if int(handles.world.current_time_step_index) != step:
            raise RuntimeError("Camera capture advanced physics")
        video.append(
            replan_index=index, action_index=action_index,
            simulation_time_seconds=stamp, simulation_step_index=step,
            exterior=exterior, wrist=wrist,
        )
        video.capture_wall_seconds += time.perf_counter() - start
        if action_index in (-1, 7):
            frame = video.frames[-1]
            observation_frames.append({
                **frame, "index": len(observation_frames),
                "replan_index": len(observation_frames),
            })

    interrupted = False
    try:
        capture(0, -1)
        time.sleep(0.25)
        if int(handles.world.current_time_step_index) != before_pause_step:
            raise RuntimeError("Wall pause advanced physics")
        q = np.asarray(handles.fr3.get_dof_positions()).reshape(-1)
        arm = q[np.asarray(handles.arm_indices).reshape(-1).astype(int)].copy()
        arm[0] += 0.20
        handles.fr3.set_dof_position_targets(
            np.asarray([arm], dtype=np.float32),
            dof_indices=handles.arm_indices,
        )
        for action_index in range(8):
            step_before = int(handles.world.current_time_step_index)
            for _ in range(4):
                handles.world.step(render=True)
            if int(handles.world.current_time_step_index) != step_before + 4:
                raise RuntimeError("Unexpected physics-step count")
            capture(0, action_index)
        # Exercise the same manual-interruption finalization path as a rollout.
        raise KeyboardInterrupt
    except KeyboardInterrupt:
        interrupted = True
    finally:
        motion = video.finalize(
            termination_reason="manual_interrupt" if interrupted else "runtime_error"
        )
    if len(observation_frames) != 2:
        raise RuntimeError("Smoke did not complete one observation interval")
    observation = export_video(
        observation_frames, output / "videos", period_s=8 / 15,
        label="smoke camera samples; no policy request was sent",
        video_name="policy_observations", source_rollout_id=run_id,
    )
    first, last = video.frames[0], video.frames[-1]
    changed = {
        role: file_digest(first[role]) != file_digest(last[role])
        for role in ("exterior", "wrist")
    }
    timing_ok = (
        motion["frame_count"] == 9
        and np.isclose(motion["simulated_span_seconds"], 8 / 15, atol=1e-4)
        and observation["frame_count"] == 2
        and np.isclose(observation["simulated_span_seconds"], 8 / 15,
                       atol=1e-4)
        and motion.get("nominal_fps") == "15"
        and observation["nominal_fps"] == "15/8"
    )
    report = {
        "status": "pass" if all(changed.values()) and timing_ok
                  and motion["status"] == "partial" else "fail",
        "camera_frames_changed": changed,
        "timing_ok": timing_ok,
        "wall_pause_seconds": 0.25,
        "wall_pause_physics_steps": 0,
        "action_count": 8,
        "physics_steps_per_action": 4,
        "motion_frames": motion["frame_count"],
        "motion_simulated_span_seconds": motion["simulated_span_seconds"],
        "motion_encoding": motion.get("encoded_stream"),
        "observation_frames": observation["frame_count"],
        "observation_encoding": observation["encoded_stream"],
        "interruption_status": motion["status"],
        "output": str(output),
    }
    write_metadata(output / "smoke.json", report)
    print(json.dumps(report, indent=2), flush=True)
    if report["status"] != "pass":
        raise RuntimeError("Isaac video smoke checks failed")


try:
    main()
finally:
    app.close()
