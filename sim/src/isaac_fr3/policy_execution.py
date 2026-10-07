"""One finite DROID action chunk through Isaac's native position drives."""

from __future__ import annotations

import time
from typing import Any

import numpy as np

from saps.physical.discrete_verifier import REFERENCE_HORIZON
from saps.physical.droid_gripper import droid_gripper_decision
from saps.physical.embodiment import DROID_CONTROL_HZ
from saps.policies.openpi_droid import (
    map_droid_reference_joint_action, validate_droid_action_response,
)

from isaac_fr3.droid_observation import measured_gripper


TOTAL_WIDTH_SPEED_M_S = 0.100
ACTIVE_FINGER_MAX_EFFORT_N = 20.0
PHYSICS_STEPS_PER_ACTION = 4
TERMINAL_EVIDENCE_STEPS = 8


def select_actions(actions: np.ndarray) -> tuple[np.ndarray, int]:
    """Take the physical baseline's first eight actions, preserving dtype."""
    native = validate_droid_action_response({"actions": actions})
    if native.shape[0] < REFERENCE_HORIZON:
        raise ValueError("DROID chunk has fewer than eight actions")
    return native[:REFERENCE_HORIZON].copy(), native.shape[0] - REFERENCE_HORIZON


def arm_target(
    action: np.ndarray, measured_q: np.ndarray,
    lower: np.ndarray, upper: np.ndarray,
) -> dict[str, np.ndarray]:
    """Map one policy action from fresh measured q without target accumulation."""
    raw = np.asarray(action)
    q = np.asarray(measured_q, dtype=np.float64)
    lo = np.asarray(lower, dtype=np.float64)
    hi = np.asarray(upper, dtype=np.float64)
    if (raw.shape != (8,) or q.shape != (7,) or lo.shape != (7,)
            or hi.shape != (7,) or not np.isfinite(raw).all()
            or not np.isfinite([q, lo, hi]).all() or np.any(lo >= hi)):
        raise ValueError("Require finite action[8], measured q[7], and limits")
    if np.any(q < lo) or np.any(q > hi):
        raise ValueError("Measured q is outside FR3 joint limits")
    mapped = map_droid_reference_joint_action(raw)
    target = q + mapped.delta_q_rad
    if not np.isfinite(target).all() or np.any(target < lo) or np.any(target > hi):
        raise ValueError("Policy target is outside FR3 joint limits")
    return {
        "clipped_arm_action": mapped.reference_joint_coordinates.copy(),
        "delta_q_rad": mapped.delta_q_rad.copy(),
        "fresh_q_ref_rad": q.copy(),
        "q_target_rad": target,
    }


def terminal_hold_target(
    measured_q: np.ndarray, lower: np.ndarray, upper: np.ndarray
) -> np.ndarray:
    """Require a finite, in-bounds measured q for the sole terminal hold."""
    q = np.asarray(measured_q, dtype=np.float64)
    lo = np.asarray(lower, dtype=np.float64)
    hi = np.asarray(upper, dtype=np.float64)
    if (q.shape != (7,) or lo.shape != (7,) or hi.shape != (7,)
            or not np.isfinite([q, lo, hi]).all()
            or np.any(q < lo) or np.any(q > hi)):
        raise ValueError("Cannot hold an invalid measured arm state")
    return q.copy()


def _array(value: Any) -> np.ndarray:
    return np.asarray(value.numpy() if hasattr(value, "numpy") else value)


def _arm_state(handles: Any) -> tuple[np.ndarray, np.ndarray]:
    indices = _array(handles.arm_indices).astype(int).reshape(-1)
    q = _array(handles.fr3.get_dof_positions()).reshape(-1)[indices].copy()
    dq = _array(handles.fr3.get_dof_velocities()).reshape(-1)[indices].copy()
    return q, dq


def _finger_state(handles: Any) -> tuple[np.ndarray, float, float]:
    indices = _array(handles.finger_indices).astype(int).reshape(-1)
    fingers = _array(handles.fr3.get_dof_positions()).reshape(-1)[indices].copy()
    width, scalar = measured_gripper(fingers)
    return fingers, width, float(scalar[0])


def _pose(position: Any, orientation: Any) -> dict:
    return {
        "position_xyz_m": _array(position).reshape(-1).tolist(),
        "orientation_wxyz": _array(orientation).reshape(-1).tolist(),
    }


def execute_chunk(
    handles: Any, config: dict, selected: np.ndarray,
    *, physics_dt: float, render: bool = False,
) -> dict:
    """Issue eight fresh-q targets at four-physics-step absolute deadlines."""
    from isaac_fr3.scene import get_tcp_pose

    expected_dt = 1.0 / (DROID_CONTROL_HZ * PHYSICS_STEPS_PER_ACTION)
    if not np.isclose(physics_dt, expected_dt, rtol=0, atol=1e-9):
        raise ValueError("Isaac physics dt does not yield 15 Hz at four steps")
    if selected.shape != (REFERENCE_HORIZON, 8):
        raise ValueError("Selected actions must have shape (8, 8)")
    lower_wp, upper_wp = handles.fr3.get_dof_limits(
        dof_indices=handles.arm_indices
    )
    lower = _array(lower_wp).reshape(-1)
    upper = _array(upper_wp).reshape(-1)
    if lower.shape != (7,) or upper.shape != (7,):
        raise ValueError("FR3 joint limits must have seven elements")

    from isaacsim.core.simulation_manager import SimulationManager
    actual_dt = SimulationManager.get_physics_dt()
    if not np.isclose(actual_dt, physics_dt, rtol=0, atol=1e-9):
        raise ValueError("Runtime physics dt differs from requested dt")

    tcp_before = _pose(*get_tcp_pose(config))
    cube_before = _pose(*handles.target.get_world_pose())
    wall_start = time.perf_counter()
    simulated_steps = 0
    rows = []
    current_intent = "OPEN"  # HOME commands the fingers fully open.
    ramp_origin_width = None
    ramp_start_step = None
    active_effort_set = False
    failed = None

    def advance_step() -> None:
        nonlocal simulated_steps
        if ramp_start_step is not None:
            elapsed = (simulated_steps - ramp_start_step + 1) * physics_dt
            sign = -1.0 if current_intent == "CLOSED" else 1.0
            target_width = float(np.clip(
                ramp_origin_width + sign * TOTAL_WIDTH_SPEED_M_S * elapsed,
                0.0, config["droid"]["gripper_max_width_m"],
            ))
            handles.fr3.set_dof_position_targets(
                np.full((1, 2), target_width / 2, dtype=np.float32),
                dof_indices=handles.finger_indices,
            )
        handles.world.step(render=render)
        simulated_steps += 1

    for index, action in enumerate(selected):
        scheduled_step = index * PHYSICS_STEPS_PER_ACTION
        while simulated_steps < scheduled_step:
            advance_step()
        actual_sim_time = simulated_steps * physics_dt
        scheduled_sim_time = index / DROID_CONTROL_HZ
        wall_now = time.perf_counter()
        row = {
            "action_index": index,
            "raw_policy_action": np.asarray(action).tolist(),
            "scheduled_simulation_seconds": scheduled_sim_time,
            "actual_simulation_seconds": actual_sim_time,
            "simulation_schedule_error_seconds": actual_sim_time - scheduled_sim_time,
            "wall_timestamp_unix_ns": time.time_ns(),
            "wall_elapsed_seconds": wall_now - wall_start,
            "wall_schedule_error_seconds": wall_now - wall_start - scheduled_sim_time,
            "wall_lateness_seconds": max(0.0, wall_now - wall_start - scheduled_sim_time),
        }
        q_before, dq_before = _arm_state(handles)
        fingers, width, gripper_scalar = _finger_state(handles)
        row.update({
            "measured_q_before_rad": q_before.tolist(),
            "measured_dq_before_rad_s": dq_before.tolist(),
            "measured_finger_positions_m": fingers.tolist(),
            "measured_total_width_m": width,
            "normalized_gripper_observation": gripper_scalar,
            "policy_gripper_value": float(action[7]),
        })
        try:
            mapped = arm_target(action, q_before, lower, upper)
            row.update({key: value.tolist() for key, value in mapped.items()})
            decision = droid_gripper_decision(float(action[7]))
            intent = "CLOSED" if decision.binary_closure else "OPEN"
            row["binary_gripper_intent"] = intent
            row["gripper_transition_initiated"] = intent != current_intent
            row["safety_result"] = "accepted"
            handles.fr3.set_dof_position_targets(
                np.asarray([mapped["q_target_rad"]], dtype=np.float32),
                dof_indices=handles.arm_indices,
            )
            if intent != current_intent:
                current_intent = intent
                ramp_origin_width = width
                ramp_start_step = simulated_steps
                if intent == "CLOSED" and not active_effort_set:
                    active_index = int(_array(handles.finger_indices).reshape(-1)[0])
                    handles.fr3.set_dof_max_efforts(
                        np.asarray([[ACTIVE_FINGER_MAX_EFFORT_N]], dtype=np.float32),
                        dof_indices=[active_index],
                    )
                    active_effort_set = True
        except (ValueError, RuntimeError) as error:
            row["safety_result"] = "rejected"
            row["rejection_reason"] = f"{type(error).__name__}: {error}"
            rows.append(row)
            failed = row["rejection_reason"]
            break
        for _ in range(PHYSICS_STEPS_PER_ACTION):
            advance_step()
        q_after, dq_after = _arm_state(handles)
        row["measured_q_after_rad"] = q_after.tolist()
        row["measured_dq_after_rad_s"] = dq_after.tolist()
        row["tcp_pose_after"] = _pose(*get_tcp_pose(config))
        rows.append(row)

    hold = {"scheduled_simulation_seconds": simulated_steps * physics_dt}
    try:
        fresh_q, _ = _arm_state(handles)
        hold_q = terminal_hold_target(fresh_q, lower, upper)
        hold["fresh_q_measured_rad"] = fresh_q.tolist()
        hold["q_hold_rad"] = hold_q.tolist()
        hold["wall_timestamp_unix_ns"] = time.time_ns()
        handles.fr3.set_dof_position_targets(
            np.asarray([hold_q], dtype=np.float32),
            dof_indices=handles.arm_indices,
        )
        hold["applied"] = True
        for _ in range(TERMINAL_EVIDENCE_STEPS):
            advance_step()
    except (ValueError, RuntimeError) as error:
        hold["applied"] = False
        hold["error"] = f"{type(error).__name__}: {error}"

    final_q, final_dq = _arm_state(handles)
    result = {
        "status": "complete" if failed is None and hold.get("applied") else "aborted",
        "failure": failed,
        "control_hz": DROID_CONTROL_HZ,
        "physics_dt_seconds": physics_dt,
        "physics_steps_per_action": PHYSICS_STEPS_PER_ACTION,
        "joint_limits_lower_rad": lower.tolist(),
        "joint_limits_upper_rad": upper.tolist(),
        "actions_executed": sum(r.get("safety_result") == "accepted" for r in rows),
        "rows": rows,
        "terminal_hold": hold,
        "initial_tcp_pose": tcp_before,
        "final_tcp_pose": _pose(*get_tcp_pose(config)),
        "cube_pose_before": cube_before,
        "cube_pose_after": _pose(*handles.target.get_world_pose()),
        "final_q_rad": final_q.tolist(),
        "final_dq_rad_s": final_dq.tolist(),
        "simulated_execution_seconds": simulated_steps * physics_dt,
        "wall_execution_seconds": time.perf_counter() - wall_start,
    }
    return result
