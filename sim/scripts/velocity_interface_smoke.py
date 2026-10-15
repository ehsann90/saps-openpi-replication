#!/usr/bin/env python3
"""Run a bounded, contact-free FR3 velocity-target and gravity-hold check."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
import uuid
from pathlib import Path

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--config", type=Path, required=True)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--gripper-transition", action="store_true",
                    help="Exercise close then open during arm velocity mode")
args = parser.parse_args()
app = SimulationApp({"headless": args.headless})

import numpy as np
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.version import get_version

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "sim/src"))

from isaac_fr3.policy_execution import ArmVelocityDrive
from isaac_fr3.robotiq_gripper import RobotiqGripperController
from isaac_fr3.scene import (
    CUBE_SETTLE_STEPS, HOME_SETTLE_STEPS, command_home, create_scene,
    load_config, settle,
)


def arm_state(handles) -> tuple[np.ndarray, np.ndarray]:
    indices = np.asarray(handles.arm_indices).reshape(-1).astype(int)
    q = np.asarray(handles.fr3.get_dof_positions()).reshape(-1)[indices].copy()
    dq = np.asarray(handles.fr3.get_dof_velocities()).reshape(-1)[indices].copy()
    return q, dq


def main() -> None:
    config = load_config(args.config)
    handles = create_scene(config)
    solver_iterations = handles.world.stage.GetPrimAtPath(
        config["robot"]["prim_path"]
    ).GetAttribute("physxArticulation:solverPositionIterationCount").Get()
    if solver_iterations != 32:
        raise RuntimeError(
            f"Expected original 32 solver position iterations, got {solver_iterations}"
        )
    command_home(handles, config)
    settle(handles, HOME_SETTLE_STEPS, render=False)
    settle(handles, CUBE_SETTLE_STEPS, render=False)
    drive = ArmVelocityDrive(handles)
    physics_dt = SimulationManager.get_physics_dt()
    if not np.isclose(physics_dt, 1 / 60, atol=1e-9):
        raise RuntimeError(f"Expected 60 Hz physics, got {physics_dt}")
    run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = ROOT / "outputs/isaac_droid_velocity_interface" / (
        run_id + "_" + uuid.uuid4().hex[:8]
    )
    output.mkdir(parents=True, exist_ok=False)
    samples = []
    gripper_result = None

    def sample(phase: str, previous_q: np.ndarray | None = None) -> None:
        q, dq = arm_state(handles)
        samples.append({
            "phase": phase,
            "step_index": int(handles.world.current_time_step_index),
            "time_seconds": float(handles.world.current_time),
            "q_rad": q.tolist(),
            "measured_dq_rad_s": dq.tolist(),
            "finite_difference_dq_rad_s": (
                ((q - previous_q) / physics_dt).tolist()
                if previous_q is not None else None
            ),
        })

    def step(phase: str) -> None:
        previous_q, _ = arm_state(handles)
        handles.world.step(render=False)
        sample(phase, previous_q)

    try:
        drive.hold()
        sample("initial_hold")
        for _ in range(8):
            step("initial_hold")
        q_before, _ = arm_state(handles)
        velocity = np.zeros(7, dtype=np.float64)
        velocity[0] = min(0.03, float(drive.max_velocity[0]) / 10)
        drive.enter_velocity()
        drive.command(velocity)
        for _ in range(8):
            step("velocity")
        q_after, _ = arm_state(handles)
        drive.hold()
        for _ in range(8):
            step("terminal_hold")
        q_final, _ = arm_state(handles)
        if args.gripper_transition:
            gripper = RobotiqGripperController()
            initial_gripper = gripper.report(handles, config)
            drive.enter_velocity()
            drive.command(np.zeros(7, dtype=np.float64))
            transition_rows = []
            try:
                for intent, policy_value in (("CLOSED", 1.0), ("OPEN", 0.0)):
                    before = gripper.report(handles, config)
                    decision = gripper.request_intent(
                        policy_value, before["measured_width_m"]
                    )
                    if not decision["transition"] or decision["to"] != intent:
                        raise RuntimeError("Gripper intent transition rejected")
                    gripper.apply_transition(handles, config)
                    for count in range(120):
                        gripper.advance_step(handles, config, physics_dt)
                        step("gripper_" + intent.lower())
                        state = gripper.report(handles, config)
                        if not state["transition_active"]:
                            break
                    transition_rows.append({
                        "intent": intent,
                        "arm_mode_active": drive.in_velocity_mode,
                        "steps": count + 1,
                        "before": before,
                        "after": state,
                    })
                    if state["transition_active"] or state["faults"]:
                        raise RuntimeError(f"Gripper {intent} did not settle")
            finally:
                drive.hold()
            for _ in range(8):
                step("gripper_terminal_hold")
            gripper_result = {
                "initial": initial_gripper,
                "transitions": transition_rows,
                "final": gripper.report(handles, config),
            }
        displacement = float(q_after[0] - q_before[0])
        hold_drift = float(np.max(np.abs(q_final - q_after)))
        result = {
            "isaac_version": get_version()[0],
            "solver_position_iterations": solver_iterations,
            "config": str(args.config.resolve()),
            "arm_dof_indices": np.asarray(handles.arm_indices).reshape(-1).tolist(),
            "finger_dof_indices": np.asarray(handles.finger_indices).reshape(-1).tolist(),
            "drive": drive.report(),
            "drive_transitions": drive.transitions,
            "arm_control_mode": "joint_velocity",
            "velocity_scale_rad_s_per_action_unit": 1.0,
            "native_arm_action_coordinates": velocity.tolist(),
            "physics_dt_seconds": physics_dt,
            "joint_limits_rad": [
                np.asarray(limit).reshape(-1).tolist()
                for limit in handles.fr3.get_dof_limits(
                    dof_indices=handles.arm_indices
                )
            ],
            "command_saturation_rad_s": [0.0] * 7,
            "commanded_velocity_rad_s": velocity.tolist(),
            "joint1_displacement_rad": displacement,
            "max_terminal_hold_drift_rad": hold_drift,
            "samples": samples,
            "gripper_interface": gripper_result,
            "passed": displacement > 0 and hold_drift < 0.01,
        }
        (output / "smoke.json").write_text(json.dumps(result, indent=2) + "\n")
        print("output:", output, "passed:", result["passed"])
        if not result["passed"]:
            raise RuntimeError("Velocity interface or hold did not track")
    finally:
        drive.hold()


try:
    main()
finally:
    app.close()
