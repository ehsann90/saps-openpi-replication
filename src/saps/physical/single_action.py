"""Controller-independent safety gates for one physical DROID action."""

from __future__ import annotations

from typing import Any

import numpy as np

from saps.physical.discrete_verifier import candidate_target
from saps.physical.fr3_forward_kinematics import (
    fr3_flange_fk,
    fr3_tcp_fk,
    fr3_tcp_jacobian,
)

STATE_AGE = 0.1
ACTION_AGE = 1.0
JOINT_MARGIN = 0.1
NATIVE_ACTION_SHAPE = (15, 8)

SINGULARITY_LOWER = 17.0
SINGULARITY_HARD = 30.0


def first_action_from_chunk(actions: Any) -> np.ndarray:
    """Return an immutable copy of action zero from one native pi05-DROID chunk."""

    chunk = np.asarray(actions)
    if (
        chunk.shape != NATIVE_ACTION_SHAPE
        or not np.issubdtype(chunk.dtype, np.floating)
        or not np.isfinite(chunk).all()
    ):
        raise ValueError(
            "P1-B requires one finite floating pi05-DROID chunk with "
            f"shape {NATIVE_ACTION_SHAPE}; received {chunk.shape}."
        )

    selected = np.array(chunk[0], dtype=np.float64, copy=True)
    selected.setflags(write=False)
    return selected


def singularity_metric(q: Any) -> dict[str, Any]:
    """Evaluate the FR3 arm Jacobian condition number.

    The metric matches the MoveIt Servo convention previously inspected:
    sigma_max / sigma_min of the unweighted fr3_arm Jacobian for
    fr3_link0 -> fr3_link8.

    This is an independent physical admissibility gate; it does not alter
    the policy action.
    """
    q = np.asarray(q, dtype=np.float64)

    jacobian = fr3_tcp_jacobian(q).copy()
    offset = fr3_flange_fk(q)[:3, 3] - fr3_tcp_fk(q)[:3, 3]
    jacobian[:3] += np.cross(jacobian[3:].T, offset).T

    values = np.linalg.svd(jacobian, compute_uv=False)

    condition = (
        float(values[0] / values[-1])
        if values[-1] > 1e-12
        else None
    )

    return {
        "condition_number": condition,
        "singular_values": values,
        "base": "fr3_link0",
        "tip": "fr3_link8",
        "lower_threshold": SINGULARITY_LOWER,
        "hard_threshold": SINGULARITY_HARD,
        "rejected": (
            condition is None
            or condition >= SINGULARITY_LOWER
        ),
    }


def safety_gate(
    action: Any,
    q: Any,
    limits: dict[str, Any],
    *,
    state_age: float,
    receive_age: float,
    action_age: float,
    readiness_reasons: list[str],
) -> dict[str, Any]:
    """Validate one native DROID action without modifying its target."""

    reasons = list(readiness_reasons)

    for name, value, limit in (
        ("state", state_age, STATE_AGE),
        ("state_receive", receive_age, STATE_AGE),
        ("action", action_age, ACTION_AGE),
    ):
        if (
            not np.isfinite(value)
            or not 0 <= value <= limit
        ):
            reasons.append(
                f"{name}_age_outside_0_to_{limit}s: {value}"
            )

    evidence: dict[str, Any] = {
        "state_age_seconds": state_age,
        "receive_age_seconds": receive_age,
        "action_age_seconds": action_age,
        "joint_margin_rad": JOINT_MARGIN,
    }

    try:
        # candidate_target preserves the authoritative DROID mapping:
        # q_target = q_measured + 0.2 * clip(u, -1, 1)
        evidence.update(
            candidate_target(action, q, limits)
        )

        # P1-B does not command the gripper.
        evidence.pop("gripper_reference", None)
        evidence["raw_gripper_policy_value"] = float(
            np.asarray(action)[7]
        )

        reasons.extend(evidence["rejection_reasons"])

        for side in ("lower", "upper"):
            margin = evidence[
                f"target_{side}_margin_rad"
            ]

            for index in np.flatnonzero(
                margin < JOINT_MARGIN
            ):
                reasons.append(
                    f"joint{index + 1}_{side}_margin="
                    f"{margin[index]:.6f}rad < "
                    f"{JOINT_MARGIN}rad"
                )

        for label, position in (
            ("measured", q),
            (
                "target",
                evidence["proposed_q_target_rad"],
            ),
        ):
            metric = singularity_metric(position)
            evidence[f"{label}_singularity"] = metric

            if metric["rejected"]:
                reasons.append(
                    f"{label}_singularity_condition="
                    f"{metric['condition_number']} "
                    f">={SINGULARITY_LOWER} "
                    "or rank deficient"
                )

    except (
        TypeError,
        ValueError,
        np.linalg.LinAlgError,
    ) as error:
        reasons.append(
            "invalid_state_action_or_kinematics: "
            f"{error}"
        )

    evidence["rejection_reasons"] = reasons
    evidence["accepted"] = not reasons
    return evidence
