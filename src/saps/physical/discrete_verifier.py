"""Non-actuating DROID targets anchored to each step's measured state."""

from __future__ import annotations

import hashlib
import time
from typing import Any, Callable
import xml.etree.ElementTree as ET

import numpy as np

from saps.physical.embodiment import FR3_JOINT_NAMES

FREQUENCY_HZ = 15
REFERENCE_HORIZON = 8


def limits_from_urdf(xml: str) -> dict[str, Any]:
    """Use the deployed robot description, in canonical arm joint order."""
    root = ET.fromstring(xml)
    lower, upper = [], []
    for name in FR3_JOINT_NAMES:
        joints = root.findall(f"./joint[@name='{name}']")
        if len(joints) != 1 or joints[0].get("type") != "revolute":
            raise ValueError(f"Expected one bounded revolute joint {name}.")
        limit = joints[0].find("limit")
        if limit is None:
            raise ValueError(f"Missing limits for {name}.")
        lower.append(float(limit.attrib["lower"]))
        upper.append(float(limit.attrib["upper"]))
    lo, hi = np.asarray(lower), np.asarray(upper)
    if not np.isfinite([lo, hi]).all() or np.any(lo >= hi):
        raise ValueError("Invalid joint-position limits.")
    return {"joint_names": FR3_JOINT_NAMES, "lower_rad": lo,
            "upper_rad": hi,
            "robot_description_sha256": hashlib.sha256(xml.encode()).hexdigest()}


def candidate_target(action: Any, measured_q: Any,
                     limits: dict[str, Any]) -> dict[str, Any]:
    """Accept only position-feasible candidates; this is not motion approval."""
    raw = np.asarray(action, dtype=np.float64)
    q = np.asarray(measured_q, dtype=np.float64)
    lo = np.asarray(limits["lower_rad"], dtype=np.float64)
    hi = np.asarray(limits["upper_rad"], dtype=np.float64)
    if (raw.shape != (8,) or q.shape != (7,) or lo.shape != (7,)
            or hi.shape != (7,) or not np.isfinite(raw).all()
            or not np.isfinite([q, lo, hi]).all() or np.any(lo >= hi)):
        raise ValueError("Require finite action[8], measured q[7], and bounds[7].")
    u = np.clip(raw[:7], -1.0, 1.0)
    delta = 0.2 * u
    target = q + delta
    lower_margin, upper_margin = target - lo, hi - target
    reasons = []
    if np.any(q < lo) or np.any(q > hi):
        reasons.append("measured_joint_position_outside_limits")
    if np.any(lower_margin < 0) or np.any(upper_margin < 0):
        reasons.append("target_joint_position_outside_limits")
    return {
        "raw_policy_action": raw, "clipped_u": u, "measured_q_rad": q,
        "delta_q_rad": delta, "proposed_q_target_rad": target,
        "joint_names": FR3_JOINT_NAMES, "joint_position_limits": limits,
        "target_lower_margin_rad": lower_margin,
        "target_upper_margin_rad": upper_margin,
        "measured_lower_margin_rad": q - lo,
        "measured_upper_margin_rad": hi - q,
        "minimum_target_margin_rad": float(np.min([lower_margin, upper_margin])),
        "position_gate_accepted": not reasons, "rejection_reasons": reasons,
        "gripper_reference": "closed" if raw[7] > 0.5 else "open",
        "rate_equivalent_diagnostic_rad_s": FREQUENCY_HZ * delta,
        "controller_acceptance_tested": False, "safe_to_execute": False,
        "policy_actions_executed": 0, "gripper_commands_issued": 0,
    }


def emulate_chunk(
    actions: Any, *, snapshot: Callable[[], Any], limits: dict[str, Any],
    request_start: float, response_end: float, observation_ros: float,
    ros_now: Callable[[], float], max_state_age: float,
    max_action_age: float, max_lateness: float,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> list[dict[str, Any]]:
    """Sample measured q per tick; late slots are rejected, never replayed.

    Tick zero follows response validation. The 8/15 s boundary includes the
    final action's nominal dwell. No previous target enters the next target.
    All thresholds are diagnostic verifier gates, not certified safety limits.
    """
    actions = np.asarray(actions)
    if actions.shape != (15, 8) or not np.isfinite(actions).all():
        raise ValueError("Require a finite native [15,8] chunk.")
    if any(not np.isfinite(x) or x <= 0 for x in
           (max_state_age, max_action_age, max_lateness)):
        raise ValueError("Diagnostic timing bounds must be positive and finite.")
    start = monotonic()
    start_ros = ros_now()
    rows = []
    previous_joint_stamp = None
    for index in range(REFERENCE_HORIZON):
        intended = start + index / FREQUENCY_HZ
        sleep(max(0.0, intended - monotonic()))
        state = snapshot()
        actual = monotonic()
        now_ros = ros_now()
        joint = state.latest_joint
        row = {"action_index": index,
               "raw_policy_action": actions[index],
               "intended_monotonic_seconds": intended,
               "intended_ros_seconds": start_ros + index / FREQUENCY_HZ,
               "sampled_monotonic_seconds": actual,
               "schedule_lateness_seconds": max(0.0, actual - intended),
               "policy_action_age_seconds": actual - request_start,
               "response_age_seconds": actual - response_end,
               "observation_age_seconds": now_ros - observation_ros,
               "callback_errors": dict(state.errors)}
        reasons = []
        if joint is None:
            reasons.append("missing_joint_state")
        else:
            row.update(candidate_target(actions[index], joint.position_rad, limits))
            reasons.extend(row["rejection_reasons"])
            age = now_ros - joint.stamp.ros_seconds
            received_age = actual - joint.stamp.receive_monotonic_seconds
            row.update({"joint_source_ros_seconds": joint.stamp.ros_seconds,
                        "joint_source_age_seconds": age,
                        "joint_receive_age_seconds": received_age})
            if not 0 <= age <= max_state_age or not 0 <= received_age <= max_state_age:
                reasons.append("stale_or_future_joint_state")
            if (previous_joint_stamp is not None
                    and joint.stamp.ros_seconds <= previous_joint_stamp):
                reasons.append("nonadvancing_joint_state")
            previous_joint_stamp = joint.stamp.ros_seconds
        if state.errors:
            reasons.append("observation_callback_error")
        if not 0 <= row["policy_action_age_seconds"] <= max_action_age:
            reasons.append("expired_policy_action")
        if row["schedule_lateness_seconds"] > max_lateness:
            reasons.append("missed_schedule_deadline")
        row["rejection_reasons"] = reasons
        row["verifier_accepted"] = not reasons
        row["controller_acceptance_tested"] = False
        row["safe_to_execute"] = False
        rows.append(row)
    sleep(max(0.0, start + REFERENCE_HORIZON / FREQUENCY_HZ - monotonic()))
    return rows
