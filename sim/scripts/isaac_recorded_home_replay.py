"""Replay the recorded Isaac actions after a measured HOME state reset."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--config", type=Path, required=True)
parser.add_argument("--actions", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
app = SimulationApp({"headless": True})

import numpy as np
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.version import get_version

ROOT = Path(__file__).resolve().parents[2]
HOME = np.asarray([0.0, -0.4, 0.0, -1.9, 0.0, 1.5, 0.0])
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "sim" / "src")]
from isaac_fr3.policy_execution import arm_target, create_gripper_controller
from isaac_fr3.scene import (CUBE_SETTLE_STEPS, HOME_SETTLE_STEPS,
    command_home, create_scene, get_tcp_pose, load_config, settle)


def array(value):
    return np.asarray(value.numpy() if hasattr(value, "numpy") else value)


def state(handles, config):
    indices = array(handles.arm_indices).astype(int).reshape(-1)
    position, orientation = get_tcp_pose(config)
    return {
        "q_rad": array(handles.fr3.get_dof_positions()).reshape(-1)[indices].tolist(),
        "dq_rad_s": array(handles.fr3.get_dof_velocities()).reshape(-1)[indices].tolist(),
        "tcp_position_xyz_m": array(position).reshape(-1).tolist(),
        "tcp_orientation_wxyz": array(orientation).reshape(-1).tolist(),
        "active_arm_target_rad": array(handles.fr3.get_dof_position_targets(
            dof_indices=handles.arm_indices)).reshape(-1).tolist(),
    }


def optional_dof_property(fr3, name, indices):
    method = getattr(fr3, name, None)
    if method is None:
        return {"available": False}
    try:
        return array(method(dof_indices=indices)).reshape(-1).tolist()
    except (TypeError, RuntimeError, ValueError) as error:
        return {"error": str(error)}


def main():
    config = load_config(args.config)
    response = json.loads(args.actions.read_text())
    native = np.asarray(response["actions"], dtype=np.float64)
    if native.shape != (15, 8) or not np.isfinite(native).all():
        raise ValueError("Expected recorded native [15,8] finite chunk")
    handles = create_scene(config)
    command_home(handles, config)
    settle(handles, HOME_SETTLE_STEPS, render=False)
    settle(handles, CUBE_SETTLE_STEPS, render=False)
    settle(handles, 16, render=False)
    configured_home = np.asarray(config["robot"]["home_arm_rad"])
    if configured_home.shape != (7,) or not np.allclose(
        configured_home, HOME, atol=1e-9, rtol=0
    ):
        raise ValueError("scene HOME differs from established Isaac HOME")
    before_home_reset = state(handles, config)
    finger_position_targets = array(handles.fr3.get_dof_position_targets(
        dof_indices=handles.finger_indices
    ))
    finger_velocity_targets = array(handles.fr3.get_dof_velocity_targets(
        dof_indices=handles.finger_indices
    ))
    handles.fr3.set_dof_positions(
        np.asarray([HOME], dtype=np.float32),
        dof_indices=handles.arm_indices,
    )
    handles.fr3.set_dof_velocities(
        np.zeros((1, 7), dtype=np.float32),
        dof_indices=handles.arm_indices,
    )
    handles.fr3.set_dof_position_targets(
        finger_position_targets, dof_indices=handles.finger_indices
    )
    handles.fr3.set_dof_velocity_targets(
        finger_velocity_targets, dof_indices=handles.finger_indices
    )
    measured_home = state(handles, config)
    finger_targets_preserved = (
        np.array_equal(finger_position_targets, array(
            handles.fr3.get_dof_position_targets(
                dof_indices=handles.finger_indices
            )
        ))
        and np.array_equal(finger_velocity_targets, array(
            handles.fr3.get_dof_velocity_targets(
                dof_indices=handles.finger_indices
            )
        ))
    )
    position_error = float(np.max(np.abs(
        np.asarray(measured_home["q_rad"]) - HOME
    )))
    velocity = float(np.max(np.abs(measured_home["dq_rad_s"])))
    home_check = {
        "requested_home_q_rad": HOME.tolist(),
        "before_reset": before_home_reset,
        "measured_initial": measured_home,
        "max_abs_position_error_rad": position_error,
        "max_abs_velocity_rad_s": velocity,
        "finger_targets_preserved": finger_targets_preserved,
        "passed": (
            position_error <= 0.001
            and velocity <= 0.001
            and finger_targets_preserved
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.with_suffix(".initial_check.json").open("x") as stream:
        json.dump(home_check, stream, indent=2, allow_nan=False)
        stream.write("\n")
    if not home_check["passed"]:
        raise RuntimeError("measured Isaac state did not reach HOME")
    dt = float(SimulationManager.get_physics_dt())
    if not np.isclose(dt, 1 / 60, atol=1e-9, rtol=0):
        raise ValueError(f"Unexpected physics dt {dt}")
    lo, hi = handles.fr3.get_dof_limits(dof_indices=handles.arm_indices)
    lo, hi = array(lo).reshape(-1), array(hi).reshape(-1)
    gripper = create_gripper_controller(config)
    result = {
        "source_actions": str(args.actions.resolve()),
        "source_actions_sha256": hashlib.sha256(args.actions.read_bytes()).hexdigest(),
        "config": str(args.config.resolve()),
        "isaac_version": get_version()[0],
        "physics_dt_seconds": dt,
        "home_settle_steps": [HOME_SETTLE_STEPS, CUBE_SETTLE_STEPS, 16],
        "arm_names": config["robot"]["arm_dof_names"],
        "arm_indices": array(handles.arm_indices).reshape(-1).tolist(),
        "gripper_indices": array(handles.finger_indices).reshape(-1).tolist(),
        "joint_lower_rad": lo.tolist(), "joint_upper_rad": hi.tolist(),
        "drive_properties": {name: optional_dof_property(handles.fr3, name,
            handles.arm_indices) for name in ("get_dof_stiffnesses",
            "get_dof_dampings", "get_dof_max_efforts",
            "get_dof_max_velocities", "get_dof_velocity_limits")},
        "initial": measured_home, "home_check": home_check, "actions": [],
    }
    stiffness, damping = handles.fr3.get_dof_gains(
        dof_indices=handles.arm_indices)
    result["drive_properties"]["stiffness"] = array(stiffness).reshape(-1).tolist()
    result["drive_properties"]["damping"] = array(damping).reshape(-1).tolist()
    result["drive_properties"]["drive_type"] = array(
        handles.fr3.get_dof_drive_types(dof_indices=handles.arm_indices)
    ).reshape(-1).tolist()
    for index, action in enumerate(native[:8]):
        before = state(handles, config)
        q = np.asarray(before["q_rad"])
        mapped = arm_target(action, q, lo, hi)
        handles.fr3.set_dof_position_targets(
            np.asarray([mapped["q_target_rad"]], dtype=np.float32),
            dof_indices=handles.arm_indices)
        active_before_gripper = state(handles, config)["active_arm_target_rad"]
        gripper_state = gripper.read(handles, config)
        transition = gripper.request_intent(float(action[7]), gripper_state["width_m"])
        if transition["transition"]:
            gripper.apply_transition(handles, config)
        active_after_gripper = state(handles, config)["active_arm_target_rad"]
        if not np.allclose(active_after_gripper, mapped["q_target_rad"], atol=1e-6, rtol=0):
            raise RuntimeError("Gripper command changed arm target")
        substeps = []
        for _ in range(4):
            gripper.advance_step(handles, config, dt)
            handles.world.step(render=False)
            substeps.append(state(handles, config))
        after = substeps[-1]
        delta = np.asarray(mapped["delta_q_rad"])
        realized = np.asarray(after["q_rad"]) - q
        error = np.asarray(mapped["q_target_rad"]) - np.asarray(after["q_rad"])
        min_margin = np.minimum(np.asarray(after["q_rad"]) - lo,
                                hi - np.asarray(after["q_rad"]))
        result["actions"].append({
            "index": index, "policy_action": action.tolist(),
            "clipped_action": mapped["clipped_arm_action"].tolist(),
            "desired_delta_rad": delta.tolist(), "issue": before,
            "desired_target_rad": mapped["q_target_rad"].tolist(),
            "active_target_before_gripper_rad": active_before_gripper,
            "active_target_after_gripper_rad": active_after_gripper,
            "gripper_transition": transition, "substeps": substeps,
            "end_tracking_error_rad": error.tolist(),
            "realized_displacement_rad": realized.tolist(),
            "realized_to_commanded_ratio": [float(r / d) if abs(d) > 1e-9
                                             else None for r, d in zip(realized, delta)],
            "joint_limit_margin_rad": min_margin.tolist(),
        })
    q_end = np.asarray(state(handles, config)["q_rad"])
    handles.fr3.set_dof_position_targets(np.asarray([q_end], dtype=np.float32),
                                          dof_indices=handles.arm_indices)
    result["terminal_hold_target_rad"] = q_end.tolist()
    result["terminal_hold_active_rad"] = state(handles, config)["active_arm_target_rad"]
    for _ in range(8):
        gripper.advance_step(handles, config, dt)
        handles.world.step(render=False)
    result["after_terminal_evidence"] = state(handles, config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(args.output)


try:
    main()
finally:
    app.close()
