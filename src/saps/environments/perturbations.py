"""Controlled object-position perturbations for LIBERO."""

from __future__ import annotations

import dataclasses
import math
from typing import Any

import numpy as np


FREE_JOINT_TYPE = 0


@dataclasses.dataclass(frozen=True)
class PlanarOffsetResult:
    joint_name: str
    body_name: str
    delta_x: float
    delta_y: float
    joint_qpos_before: list[float]
    joint_qpos_after: list[float]
    body_position_before: list[float]
    body_position_after: list[float]


def get_object_pose(
    env: Any,
    *,
    joint_name: str,
    body_name: str,
) -> tuple[np.ndarray, np.ndarray]:
    """Return the object's joint configuration and world body position."""

    sim = env.sim

    qpos_address = sim.model.get_joint_qpos_addr(joint_name)
    if not isinstance(qpos_address, tuple):
        raise ValueError(
            f"Expected {joint_name!r} to be a multi-dimensional joint, "
            f"but received address {qpos_address!r}."
        )

    start, end = qpos_address
    joint_qpos = np.asarray(sim.data.qpos[start:end], dtype=np.float64).copy()
    body_position = np.asarray(
        sim.data.get_body_xpos(body_name),
        dtype=np.float64,
    ).copy()

    return joint_qpos, body_position


def apply_planar_object_offset(
    env: Any,
    *,
    joint_name: str,
    body_name: str,
    delta_x: float,
    delta_y: float,
) -> tuple[dict[str, Any], PlanarOffsetResult]:
    """Shift one free object joint in x and y while preserving z and orientation."""

    if not math.isfinite(delta_x) or not math.isfinite(delta_y):
        raise ValueError("Object offsets must be finite numbers.")

    sim = env.sim

    joint_id = sim.model.joint_name2id(joint_name)
    joint_type = int(sim.model.jnt_type[joint_id])

    if joint_type != FREE_JOINT_TYPE:
        raise ValueError(
            f"Expected {joint_name!r} to be a free joint "
            f"(type {FREE_JOINT_TYPE}), but received type {joint_type}."
        )

    qpos_address = sim.model.get_joint_qpos_addr(joint_name)
    if not isinstance(qpos_address, tuple):
        raise ValueError(
            f"Expected a seven-dimensional joint address for {joint_name!r}."
        )

    start, end = qpos_address
    if end - start != 7:
        raise ValueError(
            f"Expected seven qpos values for {joint_name!r}, "
            f"but found {end - start}."
        )

    qpos_before, body_before = get_object_pose(
        env,
        joint_name=joint_name,
        body_name=body_name,
    )

    qpos_requested = qpos_before.copy()
    qpos_requested[0] += float(delta_x)
    qpos_requested[1] += float(delta_y)

    # Modify only the free-joint position. Height and orientation remain
    # exactly as provided by the original LIBERO initial state.
    sim.data.qpos[start:end] = qpos_requested
    sim.forward()

    # Rebuild observations using LIBERO's standard state-restoration path.
    # This updates camera images and observable values after the modification.
    modified_state = env.get_sim_state()
    obs = env.regenerate_obs_from_state(modified_state)

    qpos_after, body_after = get_object_pose(
        env,
        joint_name=joint_name,
        body_name=body_name,
    )

    expected_xy = qpos_before[:2] + np.asarray(
        [delta_x, delta_y],
        dtype=np.float64,
    )

    if not np.allclose(qpos_after[:2], expected_xy, atol=1e-8):
        raise RuntimeError(
            "The applied object position does not match the requested offset. "
            f"Expected {expected_xy}, received {qpos_after[:2]}."
        )

    result = PlanarOffsetResult(
        joint_name=joint_name,
        body_name=body_name,
        delta_x=float(delta_x),
        delta_y=float(delta_y),
        joint_qpos_before=qpos_before.tolist(),
        joint_qpos_after=qpos_after.tolist(),
        body_position_before=body_before.tolist(),
        body_position_after=body_after.tolist(),
    )

    return obs, result
