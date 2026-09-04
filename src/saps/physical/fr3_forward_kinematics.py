"""Transparent NumPy forward kinematics and Jacobian for the lab FR3."""

from __future__ import annotations

import numpy as np


JOINT_ORIGINS_M = (
    (0.0, 0.0, 0.333),
    (0.0, 0.0, 0.0),
    (0.0, -0.316, 0.0),
    (0.0825, 0.0, 0.0),
    (-0.0825, 0.384, 0.0),
    (0.0, 0.0, 0.0),
    (0.088, 0.0, 0.0),
)
JOINT_ROLLS_RAD = (
    0.0,
    -np.pi / 2.0,
    np.pi / 2.0,
    np.pi / 2.0,
    -np.pi / 2.0,
    np.pi / 2.0,
    np.pi / 2.0,
)
LINK7_TO_FLANGE_M = 0.107
FLANGE_TO_HAND_TCP_M = 0.1034
FLANGE_TO_HAND_TCP_YAW_RAD = -np.pi / 4.0


def translation(x_m: float, y_m: float, z_m: float) -> np.ndarray:
    """Return a homogeneous translation transform with metre inputs."""

    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = (x_m, y_m, z_m)
    return transform


def rotation_x(angle_rad: float) -> np.ndarray:
    """Return a homogeneous rotation about the local x-axis."""

    transform = np.eye(4, dtype=np.float64)
    cosine = np.cos(angle_rad)
    sine = np.sin(angle_rad)
    transform[1, 1] = cosine
    transform[1, 2] = -sine
    transform[2, 1] = sine
    transform[2, 2] = cosine
    return transform


def rotation_z(angle_rad: float) -> np.ndarray:
    """Return a homogeneous rotation about the local z-axis."""

    transform = np.eye(4, dtype=np.float64)
    cosine = np.cos(angle_rad)
    sine = np.sin(angle_rad)
    transform[0, 0] = cosine
    transform[0, 1] = -sine
    transform[1, 0] = sine
    transform[1, 1] = cosine
    return transform


def fr3_joint_transforms(joint_position_rad: np.ndarray) -> tuple[np.ndarray, ...]:
    """Return base-to-joint transforms for joints 1 through 7.

    Each joint origin follows ``T(x, y, z) Rx(alpha) Rz(q)``. Translations
    are in metres, angles are in radians, and the base frame is
    ``fr3_link0``.
    """

    q = _joint_vector(joint_position_rad)
    base_to_joint = np.eye(4, dtype=np.float64)
    transforms = []
    for origin_m, roll_rad, angle_rad in zip(
        JOINT_ORIGINS_M,
        JOINT_ROLLS_RAD,
        q,
    ):
        joint_relative = (
            translation(*origin_m)
            @ rotation_x(roll_rad)
            @ rotation_z(angle_rad)
        )
        base_to_joint = base_to_joint @ joint_relative
        transforms.append(base_to_joint.copy())
    return tuple(transforms)


def fr3_flange_fk(joint_position_rad: np.ndarray) -> np.ndarray:
    """Return ``fr3_link0`` to flange (``fr3_link8``) transform."""

    base_to_link7 = fr3_joint_transforms(joint_position_rad)[-1]
    link7_to_flange = translation(0.0, 0.0, LINK7_TO_FLANGE_M)
    return base_to_link7 @ link7_to_flange


def fr3_tcp_fk(joint_position_rad: np.ndarray) -> np.ndarray:
    """Return ``fr3_link0`` to ``fr3_hand_tcp`` transform."""

    flange_to_tcp = (
        translation(0.0, 0.0, FLANGE_TO_HAND_TCP_M)
        @ rotation_z(FLANGE_TO_HAND_TCP_YAW_RAD)
    )
    return fr3_flange_fk(joint_position_rad) @ flange_to_tcp


def fr3_tcp_finite_displacement(
    joint_position_rad: np.ndarray,
    delta_q_rad: np.ndarray,
) -> np.ndarray:
    """Return exact finite TCP displacement from the manual NumPy FK.

    Translation is ``p(q + delta_q) - p(q)`` in metres. Rotation is the
    base-resolved logarithm of ``R(q + delta_q) R(q)^T`` in radians.
    """

    q = _joint_vector(joint_position_rad)
    delta_q = _joint_vector(delta_q_rad)
    initial = fr3_tcp_fk(q)
    final = fr3_tcp_fk(q + delta_q)
    delta_position = final[:3, 3] - initial[:3, 3]
    delta_rotation = _rotation_vector(
        final[:3, :3] @ initial[:3, :3].T
    )
    return np.concatenate((delta_position, delta_rotation))


def fr3_tcp_jacobian(joint_position_rad: np.ndarray) -> np.ndarray:
    """Return the geometric TCP Jacobian resolved in ``fr3_link0``.

    Rows 0 through 2 are linear displacement in metres per radian and rows
    3 through 5 are angular displacement in radians per radian. Columns are
    ordered ``fr3_joint1`` through ``fr3_joint7``.
    """

    joint_transforms = fr3_joint_transforms(joint_position_rad)
    tcp_position_m = fr3_tcp_fk(joint_position_rad)[:3, 3]
    jacobian = np.zeros((6, 7), dtype=np.float64)
    for index, base_to_joint in enumerate(joint_transforms):
        joint_position_m = base_to_joint[:3, 3]
        joint_axis_base = base_to_joint[:3, 2]
        jacobian[:3, index] = np.cross(
            joint_axis_base,
            tcp_position_m - joint_position_m,
        )
        jacobian[3:, index] = joint_axis_base
    return jacobian


def _joint_vector(value: np.ndarray) -> np.ndarray:
    """Validate and copy one seven-joint vector in radians."""

    result = np.asarray(value)
    if not np.issubdtype(result.dtype, np.floating):
        raise TypeError("FR3 joint positions must have a floating dtype.")
    if result.shape != (7,):
        raise ValueError(
            "FR3 joint positions must have shape (7,), received "
            f"{result.shape}."
        )
    if not np.all(np.isfinite(result)):
        raise ValueError("FR3 joint positions must be finite.")
    return np.array(result, dtype=np.float64, copy=True)


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """Return the axis-angle vector for one relative rotation matrix."""

    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(cosine))
    antisymmetric = rotation - rotation.T
    vee = np.asarray(
        [
            antisymmetric[2, 1],
            antisymmetric[0, 2],
            antisymmetric[1, 0],
        ],
        dtype=np.float64,
    )
    if angle < 1e-8:
        return 0.5 * vee
    if np.pi - angle < 1e-6:
        raise RuntimeError(
            "Relative rotation is too close to pi for the finite-motion "
            "diagnostic logarithm."
        )
    return angle / (2.0 * np.sin(angle)) * vee
