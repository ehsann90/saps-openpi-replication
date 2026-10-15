"""Finite DROID chunks and repeated rollout control for Isaac drives."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np

from saps.physical.discrete_verifier import REFERENCE_HORIZON
from saps.physical.droid_gripper import droid_gripper_decision
from saps.physical.embodiment import DROID_CONTROL_HZ
from saps.policies.openpi_droid import (
    map_droid_reference_joint_action, validate_droid_action_response,
)

from isaac_fr3.droid_observation import measured_gripper
from isaac_fr3.robotiq_gripper import RobotiqGripperController


TOTAL_WIDTH_SPEED_M_S = 0.100
ACTIVE_FINGER_MAX_EFFORT_N = 20.0
PHYSICS_STEPS_PER_ACTION = 4
TERMINAL_EVIDENCE_STEPS = 8
ARM_CONTROL_MODES = ("position_reference", "joint_velocity")
DEFAULT_VELOCITY_SCALE_RAD_S_PER_ACTION_UNIT = 1.0


class InvalidPolicyResponse(ValueError):
    """The server returned an action chunk outside the DROID contract."""


@dataclass
class GripperRuntime:
    """Persist one rate-limited finger transition across action chunks."""

    intent: str = "OPEN"
    ramp_origin_width_m: float | None = None
    ramp_steps: int = 0
    active_effort_set: bool = False

    def request_intent(self, policy_value: float, width_m: float) -> dict:
        """Start a new width ramp only when binary policy intent changes."""
        decision = droid_gripper_decision(policy_value)
        intent = "CLOSED" if decision.binary_closure else "OPEN"
        previous = self.intent
        transition = intent != previous
        if transition:
            self.intent = intent
            self.ramp_origin_width_m = float(width_m)
            self.ramp_steps = 0
        return {"from": previous, "to": intent, "transition": transition}

    def read(self, handles: Any, config: dict) -> dict:
        """Read the original two Franka finger translations without coercion."""
        indices = _array(handles.finger_indices).astype(int).reshape(-1)
        fingers = _array(handles.fr3.get_dof_positions()).reshape(-1)[indices].copy()
        width, scalar = measured_gripper(fingers)
        return {"positions": fingers, "width_m": width, "scalar": scalar}

    def apply_transition(self, handles: Any, config: dict) -> None:
        if self.intent == "CLOSED" and not self.active_effort_set:
            active_index = int(_array(handles.finger_indices).reshape(-1)[0])
            handles.fr3.set_dof_max_efforts(
                np.asarray([[ACTIVE_FINGER_MAX_EFFORT_N]], dtype=np.float32),
                dof_indices=[active_index],
            )
            self.active_effort_set = True

    def advance_step(self, handles: Any, config: dict,
                     physics_dt: float) -> None:
        if self.ramp_origin_width_m is None:
            return
        self.ramp_steps += 1
        elapsed = self.ramp_steps * physics_dt
        sign = -1.0 if self.intent == "CLOSED" else 1.0
        target_width = float(np.clip(
            self.ramp_origin_width_m + sign * TOTAL_WIDTH_SPEED_M_S * elapsed,
            0.0, config["droid"]["gripper_max_width_m"],
        ))
        handles.fr3.set_dof_position_targets(
            np.full((1, 2), target_width / 2, dtype=np.float32),
            dof_indices=handles.finger_indices,
        )

    def freeze(self, handles: Any, config: dict) -> dict:
        fingers = self.read(handles, config)["positions"]
        handles.fr3.set_dof_position_targets(
            np.asarray([fingers], dtype=np.float32),
            dof_indices=handles.finger_indices,
        )
        self.ramp_origin_width_m = None
        return {"gripper_width_frozen_m": float(np.sum(fingers))}

    def report(self, handles: Any, config: dict) -> dict:
        state = self.read(handles, config)
        return {
            "intent": self.intent,
            "finger_positions_m": state["positions"].tolist(),
            "measured_width_m": state["width_m"],
            "normalized_observation": float(state["scalar"][0]),
            "transition_active": self.ramp_origin_width_m is not None,
            "transition_steps": self.ramp_steps,
            "faults": [],
        }


class GripperController(Protocol):
    """Small embodiment boundary used by the unchanged arm action loop."""

    intent: str

    def read(self, handles: Any, config: dict) -> dict: ...
    def request_intent(self, policy_value: float, width_m: float) -> dict: ...
    def apply_transition(self, handles: Any, config: dict) -> None: ...
    def advance_step(self, handles: Any, config: dict,
                     physics_dt: float) -> None: ...
    def freeze(self, handles: Any, config: dict) -> dict: ...
    def report(self, handles: Any, config: dict) -> dict: ...


def create_gripper_controller(config: dict) -> GripperController:
    """Choose the validated gripper implementation from scene configuration."""
    kind = config.get("gripper", {}).get("kind", "franka_hand")
    if kind == "franka_hand":
        return GripperRuntime()
    if kind == "robotiq_2f85":
        return RobotiqGripperController()
    raise ValueError(f"Unsupported gripper embodiment: {kind}")


def run_rollout_loop(
    *, hold: Any, capture: Any, infer: Any, execute: Any,
    record: Any, emergency_hold: Any, max_replans: int | None = None,
) -> dict:
    """Repeat hold/capture/infer/execute with unlimited replans by default."""
    if max_replans is not None and max_replans <= 0:
        raise ValueError("max_replans must be positive when supplied")
    requests = completed = actions = 0
    index = 0
    reason = error = None
    while max_replans is None or index < max_replans:
        try:
            hold(index)
            observation = capture(index)
            requests += 1
            response = infer(index, observation)
            execution = execute(index, response)
            actions += int(execution["actions_executed"])
            record(index, observation, response, execution)
            if (execution["status"] == "complete"
                    and not execution.get("terminal_hold", {}).get("applied")):
                raise RuntimeError("Completed chunk lacks its terminal hold")
            if execution["status"] != "complete":
                reason = "safety_violation"
                error = execution.get("failure") or execution["terminal_hold"].get("error")
                break
            completed += 1
            index += 1
        except KeyboardInterrupt:
            reason = "manual_interrupt"
            error = None
            try:
                emergency_hold()
            except (ValueError, RuntimeError) as hold_error:
                error = f"interrupt hold failed: {hold_error}"
            break
        except Exception as runtime_error:
            reason = (
                "invalid_policy_response"
                if isinstance(runtime_error, InvalidPolicyResponse)
                else "runtime_error"
            )
            error = f"{type(runtime_error).__name__}: {runtime_error}"
            try:
                emergency_hold()
            except (ValueError, RuntimeError) as hold_error:
                error += f"; emergency hold failed: {hold_error}"
            break
    if reason is None:
        reason = "max_replans"
    return {
        "termination_reason": reason,
        "error": error,
        "inference_requests": requests,
        "completed_replans": completed,
        "policy_actions_executed": actions,
        "next_replan_index": index,
    }


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


def velocity_command(
    action: np.ndarray, measured_q: np.ndarray,
    lower: np.ndarray, upper: np.ndarray, max_velocity: np.ndarray,
    scale_rad_s_per_action_unit: float,
) -> dict[str, np.ndarray]:
    """Map normalized DROID coordinates to explicit experimental rad/s."""
    raw = np.asarray(action)
    if (raw.shape != (8,) or not np.issubdtype(raw.dtype, np.floating)
            or not np.isfinite(raw).all()):
        raise ValueError("Require finite floating DROID action[8]")
    q = terminal_hold_target(measured_q, lower, upper)
    limit = np.asarray(max_velocity, dtype=np.float64)
    if (limit.shape != (7,) or not np.isfinite(limit).all()
            or np.any(limit <= 0)):
        raise ValueError("Require finite positive velocity limits[7]")
    if not np.isfinite(scale_rad_s_per_action_unit) or scale_rad_s_per_action_unit <= 0:
        raise ValueError("Velocity scale must be finite and positive")
    clipped = np.clip(raw[:7], -1.0, 1.0)
    requested = scale_rad_s_per_action_unit * clipped
    interval = 1.0 / DROID_CONTROL_HZ
    lo_rate = np.maximum(-limit, (np.asarray(lower) - q) / interval)
    hi_rate = np.minimum(limit, (np.asarray(upper) - q) / interval)
    commanded = np.clip(requested, lo_rate, hi_rate)
    return {
        "clipped_arm_action": clipped,
        "requested_velocity_rad_s": requested,
        "commanded_velocity_rad_s": commanded,
        "velocity_saturation_rad_s": requested - commanded,
    }


def _array(value: Any) -> np.ndarray:
    return np.asarray(value.numpy() if hasattr(value, "numpy") else value)


def _arm_state(handles: Any) -> tuple[np.ndarray, np.ndarray]:
    indices = _array(handles.arm_indices).astype(int).reshape(-1)
    q = _array(handles.fr3.get_dof_positions()).reshape(-1)[indices].copy()
    dq = _array(handles.fr3.get_dof_velocities()).reshape(-1)[indices].copy()
    return q, dq


def _pose(position: Any, orientation: Any) -> dict:
    return {
        "position_xyz_m": _array(position).reshape(-1).tolist(),
        "orientation_wxyz": _array(orientation).reshape(-1).tolist(),
    }


class ArmVelocityDrive:
    """Switch only seven arm drives between velocity motion and position hold."""

    def __init__(self, handles: Any) -> None:
        self.handles = handles
        arm = _array(handles.arm_indices).astype(int).reshape(-1)
        fingers = _array(handles.finger_indices).astype(int).reshape(-1)
        if (arm.shape != (7,) or len(set(arm)) != 7
                or set(arm).intersection(fingers)):
            raise ValueError("Arm indices must be seven unique non-gripper DOFs")
        stiffness, damping = handles.fr3.get_dof_gains(
            dof_indices=handles.arm_indices
        )
        self.stiffness = _array(stiffness).reshape(-1).copy()
        self.damping = _array(damping).reshape(-1).copy()
        self.max_velocity = _array(handles.fr3.get_dof_max_velocities(
            dof_indices=handles.arm_indices
        )).reshape(-1).copy()
        if (self.stiffness.shape != (7,) or self.damping.shape != (7,)
                or self.max_velocity.shape != (7,)
                or not np.isfinite(np.concatenate((
                    self.stiffness, self.damping, self.max_velocity
                ))).all() or np.any(self.stiffness <= 0)
                or np.any(self.damping <= 0)
                or np.any(self.max_velocity <= 0)):
            raise ValueError("Invalid arm gains or articulation velocity limits")
        self.in_velocity_mode = False
        self.transitions: list[dict] = []

    def _record(self, mode: str) -> None:
        self.transitions.append({
            "mode": mode,
            "simulation_step_index": int(
                self.handles.world.current_time_step_index
            ),
            "simulation_time_seconds": float(self.handles.world.current_time),
            "wall_timestamp_unix_ns": time.time_ns(),
        })

    def _verify_gains(self, expected_stiffness: np.ndarray) -> None:
        stiffness, damping = self.handles.fr3.get_dof_gains(
            dof_indices=self.handles.arm_indices
        )
        if (not np.allclose(_array(stiffness).reshape(-1), expected_stiffness,
                            rtol=0, atol=1e-4)
                or not np.allclose(_array(damping).reshape(-1), self.damping,
                                   rtol=0, atol=1e-4)):
            raise RuntimeError("Arm drive gains did not enter requested mode")

    def enter_velocity(self) -> None:
        if self.in_velocity_mode:
            return
        fr3 = self.handles.fr3
        q, _ = _arm_state(self.handles)
        # Remove the old position error before its stiffness goes to zero.
        fr3.set_dof_position_targets(
            q.reshape(1, 7).astype(np.float32),
            dof_indices=self.handles.arm_indices,
        )
        fr3.set_dof_velocity_targets(
            np.zeros((1, 7), dtype=np.float32),
            dof_indices=self.handles.arm_indices,
        )
        # Treat a partially applied gain update as velocity mode so a
        # caller's emergency hold restores the position drive.
        self.in_velocity_mode = True
        fr3.set_dof_gains(
            stiffnesses=np.zeros((1, 7), dtype=np.float32),
            dampings=self.damping.reshape(1, 7),
            dof_indices=self.handles.arm_indices,
            update_default_gains=False,
        )
        self._verify_gains(np.zeros(7))
        self._record("joint_velocity")

    def command(self, velocity: np.ndarray) -> None:
        value = np.asarray(velocity, dtype=np.float64)
        if not self.in_velocity_mode:
            raise RuntimeError("Arm velocity mode is inactive")
        if (value.shape != (7,) or not np.isfinite(value).all()
                or np.any(np.abs(value) > self.max_velocity + 1e-8)):
            raise ValueError("Invalid or over-limit arm velocity command")
        self.handles.fr3.set_dof_velocity_targets(
            value.reshape(1, 7).astype(np.float32),
            dof_indices=self.handles.arm_indices,
        )
        active = _array(self.handles.fr3.get_dof_velocity_targets(
            dof_indices=self.handles.arm_indices
        )).reshape(-1)
        if active.shape != (7,) or not np.allclose(
            active, value, rtol=0, atol=1e-6
        ):
            raise RuntimeError("Arm velocity target was not accepted")

    def hold(self) -> dict:
        fr3 = self.handles.fr3
        if self.in_velocity_mode:
            # Clear velocity and replace the stale position reference before
            # restoring stiffness. The position drive then carries gravity.
            fr3.set_dof_velocity_targets(
                np.zeros((1, 7), dtype=np.float32),
                dof_indices=self.handles.arm_indices,
            )
            q, _ = _arm_state(self.handles)
            fr3.set_dof_position_targets(
                q.reshape(1, 7).astype(np.float32),
                dof_indices=self.handles.arm_indices,
            )
            fr3.set_dof_gains(
                stiffnesses=self.stiffness.reshape(1, 7),
                dampings=self.damping.reshape(1, 7),
                dof_indices=self.handles.arm_indices,
                update_default_gains=False,
            )
            self._verify_gains(self.stiffness)
            self.in_velocity_mode = False
            self._record("position_hold")
        record = command_fresh_hold(self.handles)
        active_velocity = _array(fr3.get_dof_velocity_targets(
            dof_indices=self.handles.arm_indices
        )).reshape(-1)
        if not np.allclose(active_velocity, 0, rtol=0, atol=1e-7):
            raise RuntimeError("Stale arm velocity target remains during hold")
        record["active_arm_velocity_target_rad_s"] = active_velocity.tolist()
        return record

    def report(self) -> dict:
        return {
            "original_stiffness": self.stiffness.tolist(),
            "original_damping": self.damping.tolist(),
            "velocity_stiffness": [0.0] * 7,
            "velocity_damping": self.damping.tolist(),
            "max_velocity_rad_s": self.max_velocity.tolist(),
        }


def command_fresh_hold(handles: Any) -> dict:
    """Command and verify the current measured arm state without stepping."""
    lower_wp, upper_wp = handles.fr3.get_dof_limits(
        dof_indices=handles.arm_indices
    )
    q, _ = _arm_state(handles)
    target = terminal_hold_target(
        q, _array(lower_wp).reshape(-1), _array(upper_wp).reshape(-1)
    )
    handles.fr3.set_dof_position_targets(
        np.asarray([target], dtype=np.float32),
        dof_indices=handles.arm_indices,
    )
    active = _array(handles.fr3.get_dof_position_targets(
        dof_indices=handles.arm_indices
    )).reshape(-1)
    if active.shape != (7,) or not np.allclose(
        active, target, rtol=0, atol=1e-6
    ):
        raise RuntimeError("Fresh measured-state hold is not the active target")
    return {
        "measured_arm_q_rad": q.tolist(),
        "commanded_arm_q_hold_rad": target.tolist(),
        "active_arm_q_target_rad": active.tolist(),
        "simulation_time_seconds": float(handles.world.current_time),
        "simulation_step_index": int(handles.world.current_time_step_index),
        "wall_timestamp_unix_ns": time.time_ns(),
    }


def execute_chunk(
    handles: Any, config: dict, selected: np.ndarray,
    *, physics_dt: float, render: bool = False,
    arm_control_mode: str = "position_reference",
    velocity_scale_rad_s_per_action_unit: float = (
        DEFAULT_VELOCITY_SCALE_RAD_S_PER_ACTION_UNIT
    ),
    velocity_drive: ArmVelocityDrive | None = None,
    record_physics_steps: bool = False,
    gripper_runtime: GripperController | None = None,
    on_action: Any | None = None,
    contact_monitor: Any | None = None,
    terminal_evidence_steps: int = TERMINAL_EVIDENCE_STEPS,
) -> dict:
    """Issue eight arm actions at four-physics-step absolute deadlines."""
    if arm_control_mode not in ARM_CONTROL_MODES:
        raise ValueError(f"Unsupported arm control mode: {arm_control_mode}")
    if arm_control_mode == "joint_velocity" and velocity_drive is None:
        raise ValueError("Velocity mode requires an arm drive controller")
    if terminal_evidence_steps < 0:
        raise ValueError("terminal_evidence_steps must be nonnegative")
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
    gripper = (gripper_runtime if gripper_runtime is not None
               else create_gripper_controller(config))
    robotiq = config.get("gripper", {}).get("kind") == "robotiq_2f85"
    if robotiq != isinstance(gripper, RobotiqGripperController):
        raise ValueError("Gripper controller does not match scene embodiment")
    transitions = []
    failed = None
    drive_transition_start = (
        len(velocity_drive.transitions) if velocity_drive is not None else 0
    )
    physics_rows = []

    def advance_step() -> None:
        nonlocal simulated_steps
        if arm_control_mode == "joint_velocity":
            q_step_before, _ = _arm_state(handles)
        gripper.advance_step(handles, config, physics_dt)
        handles.world.step(render=render)
        simulated_steps += 1
        if arm_control_mode == "joint_velocity":
            q_step_after, dq_step_after = _arm_state(handles)
            if record_physics_steps:
                physics_rows.append({
                    "step_index": int(handles.world.current_time_step_index),
                    "simulation_time_seconds": float(handles.world.current_time),
                    "q_rad": q_step_after.tolist(),
                    "measured_dq_rad_s": dq_step_after.tolist(),
                    "finite_difference_dq_rad_s": (
                        (q_step_after - q_step_before) / physics_dt
                    ).tolist(),
                })
            if (not np.isfinite(q_step_after).all()
                    or not np.isfinite(dq_step_after).all()
                    or np.any(q_step_after < lower - 1e-5)
                    or np.any(q_step_after > upper + 1e-5)
                    or np.any(np.abs(dq_step_after)
                              > velocity_drive.max_velocity + 1e-3)):
                raise RuntimeError("Measured arm state exceeded safety limits")

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
        sample = gripper.read(handles, config)
        fingers = sample["positions"]
        width = sample["width_m"]
        gripper_scalar = float(sample["scalar"][0])
        row.update({
            "measured_q_before_rad": q_before.tolist(),
            "measured_dq_before_rad_s": dq_before.tolist(),
        })
        if robotiq:
            row["measured_gripper_driver_position_rad"] = float(fingers[0])
        else:
            row["measured_finger_positions_m"] = fingers.tolist()
        row.update({
            "measured_total_width_m": width,
            "normalized_gripper_observation": gripper_scalar,
            "policy_gripper_value": float(action[7]),
        })
        try:
            mapped = (
                velocity_command(
                    action, q_before, lower, upper,
                    velocity_drive.max_velocity,
                    velocity_scale_rad_s_per_action_unit,
                ) if arm_control_mode == "joint_velocity" else
                arm_target(action, q_before, lower, upper)
            )
            row.update({key: value.tolist() for key, value in mapped.items()})
            row["safety_result"] = "accepted"
            if arm_control_mode == "joint_velocity":
                velocity_drive.enter_velocity()
                velocity_drive.command(mapped["commanded_velocity_rad_s"])
            else:
                handles.fr3.set_dof_position_targets(
                    np.asarray([mapped["q_target_rad"]], dtype=np.float32),
                    dof_indices=handles.arm_indices,
                )
            intent_result = gripper.request_intent(float(action[7]), width)
            intent = intent_result["to"]
            row["binary_gripper_intent"] = intent
            row["gripper_transition_initiated"] = intent_result["transition"]
            if intent_result["transition"]:
                transitions.append({
                    "action_index": index, "from": intent_result["from"],
                    "to": intent, "measured_width_m": width,
                    "simulated_seconds": actual_sim_time,
                })
                gripper.apply_transition(handles, config)
        except (ValueError, RuntimeError) as error:
            row["safety_result"] = "rejected"
            row["rejection_reason"] = f"{type(error).__name__}: {error}"
            rows.append(row)
            if on_action is not None:
                on_action(row)
            failed = row["rejection_reason"]
            break
        try:
            for _ in range(PHYSICS_STEPS_PER_ACTION):
                advance_step()
        except RuntimeError as error:
            row["safety_result"] = "rejected"
            row["rejection_reason"] = f"{type(error).__name__}: {error}"
            rows.append(row)
            if on_action is not None:
                on_action(row)
            failed = row["rejection_reason"]
            break
        q_after, dq_after = _arm_state(handles)
        row["measured_q_after_rad"] = q_after.tolist()
        row["measured_dq_after_rad_s"] = dq_after.tolist()
        if arm_control_mode == "joint_velocity":
            row["finite_difference_dq_rad_s"] = (
                (q_after - q_before) / (PHYSICS_STEPS_PER_ACTION * physics_dt)
            ).tolist()
            row["velocity_tracking_error_rad_s"] = (
                dq_after - mapped["commanded_velocity_rad_s"]
            ).tolist()
        row["tcp_pose_after"] = _pose(*get_tcp_pose(config))
        if robotiq:
            row["gripper_state_after"] = gripper.report(handles, config)
            if contact_monitor is not None:
                row["contact_summary_after"] = contact_monitor.summary()
        rows.append(row)
        if on_action is not None:
            on_action(row)

    hold = {"scheduled_simulation_seconds": simulated_steps * physics_dt}
    if failed is not None:
        # Stop an active finger ramp after an arm safety rejection. The
        # previously requested binary intent remains recorded for provenance.
        try:
            hold.update(gripper.freeze(handles, config))
        except (ValueError, RuntimeError) as error:
            hold["gripper_freeze_error"] = f"{type(error).__name__}: {error}"
    try:
        if arm_control_mode == "joint_velocity":
            velocity_drive.hold()
        fresh_q, _ = _arm_state(handles)
        hold_q = terminal_hold_target(fresh_q, lower, upper)
        hold["fresh_q_measured_rad"] = fresh_q.tolist()
        hold["q_hold_rad"] = hold_q.tolist()
        hold["wall_timestamp_unix_ns"] = time.time_ns()
        handles.fr3.set_dof_position_targets(
            np.asarray([hold_q], dtype=np.float32),
            dof_indices=handles.arm_indices,
        )
        active = _array(handles.fr3.get_dof_position_targets(
            dof_indices=handles.arm_indices
        )).reshape(-1)
        if active.shape != (7,) or not np.allclose(
            active, hold_q, rtol=0, atol=1e-6
        ):
            raise RuntimeError("Terminal fresh-q hold is not the active target")
        hold["active_q_target_rad"] = active.tolist()
        hold["applied"] = True
        hold["simulation_step_index"] = int(
            handles.world.current_time_step_index
        )
        for _ in range(terminal_evidence_steps):
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
        "gripper_transitions": transitions,
        "gripper_intent_after": gripper.intent,
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
    if robotiq:
        result["gripper_state_after"] = gripper.report(handles, config)
        if contact_monitor is not None:
            result["contact_summary"] = contact_monitor.summary()
    else:
        result["gripper_ramp_steps_after"] = gripper.ramp_steps
    if arm_control_mode == "joint_velocity":
        result["arm_control_mode"] = arm_control_mode
        result["velocity_scale_rad_s_per_action_unit"] = (
            velocity_scale_rad_s_per_action_unit
        )
        result["arm_drive"] = velocity_drive.report()
        result["arm_drive_transitions"] = velocity_drive.transitions[
            drive_transition_start:
        ]
        if record_physics_steps:
            result["physics_steps"] = physics_rows
    return result
