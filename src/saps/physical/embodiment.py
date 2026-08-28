"""Pure DROID-joint to FR3 task-space embodiment mathematics."""

from __future__ import annotations

from collections.abc import Sequence
import dataclasses
from typing import Protocol

import numpy as np


DROID_CONTROL_HZ = 15.0
DROID_RELATIVE_MAX_JOINT_DELTA_RAD = 0.2
DROID_PANDA_JOINT_NAMES = tuple(
    f"panda_joint{index}" for index in range(1, 8)
)
FR3_JOINT_NAMES = tuple(
    f"fr3_joint{index}" for index in range(1, 8)
)
DROID_TO_FR3_JOINT_MAPPING = tuple(
    (DROID_PANDA_JOINT_NAMES[index], FR3_JOINT_NAMES[index])
    for index in range(7)
)


class JacobianProvider(Protocol):
    """State-dependent 6-by-7 geometric Jacobian provider."""

    joint_names: tuple[str, ...]
    base_frame: str
    end_effector_frame: str

    def jacobian(self, joint_position: np.ndarray) -> np.ndarray:
        """Return [linear; angular] velocity in the base frame."""


@dataclasses.dataclass(frozen=True)
class DroidJointAction:
    """Every stage of the native DROID relative-joint command."""

    native: np.ndarray
    clipping_scale: float
    clipped: np.ndarray
    delta_q_rad: np.ndarray
    nominal_qdot_rad_s: np.ndarray


@dataclasses.dataclass(frozen=True)
class NullSpaceDiagnostic:
    """Minimum-norm task component and discarded joint null component."""

    task_qdot_rad_s: np.ndarray
    null_qdot_rad_s: np.ndarray
    qdot_norm_rad_s: float
    task_norm_rad_s: float
    null_norm_rad_s: float
    null_fraction: float


@dataclasses.dataclass(frozen=True)
class JacobianDiagnostic:
    """Numerical rank and conditioning of one supplied Jacobian."""

    singular_values: np.ndarray
    rank: int
    condition_number: float
    near_singular: bool


@dataclasses.dataclass(frozen=True)
class CartesianPolicyAction:
    """Auditable projection of one native policy action at one state."""

    joint_position_rad: np.ndarray
    joint_action: DroidJointAction
    jacobian: np.ndarray
    twist_si: np.ndarray
    delta_x_linearized: np.ndarray
    normalized_motion: np.ndarray | None
    gripper_closure: float
    jacobian_diagnostic: JacobianDiagnostic
    null_space_diagnostic: NullSpaceDiagnostic


@dataclasses.dataclass(frozen=True)
class CartesianNormalization:
    """Explicit diagonal scaling for a Cartesian per-step displacement.

    Translation entries are metres per policy step. Rotation entries are
    radians per policy step. Normalization never clips its result.
    """

    translation_scale_m: float = 0.075
    rotation_scale_rad: float = 0.15

    def __post_init__(self) -> None:
        for name, value in (
            ("translation_scale_m", self.translation_scale_m),
            ("rotation_scale_rad", self.rotation_scale_rad),
        ):
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive.")

    @property
    def scales(self) -> np.ndarray:
        """Return [metres, radians] scales in Cartesian row order."""

        return _readonly(
            np.asarray(
                [self.translation_scale_m] * 3
                + [self.rotation_scale_rad] * 3,
                dtype=np.float64,
            )
        )

    def normalize_step(self, delta_x: np.ndarray) -> np.ndarray:
        """Normalize a physical step without clipping it."""

        return _readonly(
            _finite_float_array(
                delta_x,
                expected_shape=(6,),
                field_name="delta_x",
            )
            / self.scales
        )

    def denormalize_step(self, normalized: np.ndarray) -> np.ndarray:
        """Recover the physical Cartesian per-step displacement."""

        return _readonly(
            _finite_float_array(
                normalized,
                expected_shape=(6,),
                field_name="normalized",
            )
            * self.scales
        )


class DroidJointActionSemantics:
    """Exact 15-Hz DROID normalized relative-joint semantics."""

    def __init__(
        self,
        *,
        relative_max_joint_delta_rad: float = (
            DROID_RELATIVE_MAX_JOINT_DELTA_RAD
        ),
        control_hz: float = DROID_CONTROL_HZ,
    ) -> None:
        if (
            not np.isfinite(relative_max_joint_delta_rad)
            or relative_max_joint_delta_rad <= 0
        ):
            raise ValueError(
                "relative_max_joint_delta_rad must be finite and positive."
            )
        if not np.isfinite(control_hz) or control_hz <= 0:
            raise ValueError("control_hz must be finite and positive.")
        self.relative_max_joint_delta_rad = float(
            relative_max_joint_delta_rad
        )
        self.control_hz = float(control_hz)

    def transform(self, native: np.ndarray) -> DroidJointAction:
        """Clip by common scale, then expose delta and nominal qdot."""

        native_array = _finite_float_array(
            native,
            expected_shape=(7,),
            field_name="native_joint_action",
        )
        clipping_scale = max(1.0, float(np.max(np.abs(native_array))))
        clipped = native_array / clipping_scale
        delta_q = self.relative_max_joint_delta_rad * clipped
        nominal_qdot = self.control_hz * delta_q
        return DroidJointAction(
            native=_readonly(native_array),
            clipping_scale=clipping_scale,
            clipped=_readonly(clipped),
            delta_q_rad=_readonly(delta_q),
            nominal_qdot_rad_s=_readonly(nominal_qdot),
        )


class DroidToFr3TaskSpaceAdapter:
    """Project one DROID action using the FR3 state supplied for that step."""

    def __init__(
        self,
        jacobian_provider: JacobianProvider,
        *,
        normalization: CartesianNormalization | None = None,
        singular_value_tolerance: float = 1e-10,
    ) -> None:
        validate_joint_mapping(
            DROID_PANDA_JOINT_NAMES,
            jacobian_provider.joint_names,
        )
        if (
            not np.isfinite(singular_value_tolerance)
            or singular_value_tolerance <= 0
        ):
            raise ValueError(
                "singular_value_tolerance must be finite and positive."
            )
        self.jacobian_provider = jacobian_provider
        self.normalization = normalization
        self.singular_value_tolerance = float(
            singular_value_tolerance
        )
        self.joint_semantics = DroidJointActionSemantics()

    def project(
        self,
        policy_action: np.ndarray,
        joint_position: np.ndarray,
    ) -> CartesianPolicyAction:
        """Project at the current q; callers must supply fresh q each step."""

        action = _finite_float_array(
            policy_action,
            expected_shape=(8,),
            field_name="policy_action",
        )
        q = _finite_float_array(
            joint_position,
            expected_shape=(7,),
            field_name="joint_position",
        )
        gripper = validate_gripper_closure(float(action[7]))
        joint_action = self.joint_semantics.transform(action[:7])
        jacobian = _finite_float_array(
            self.jacobian_provider.jacobian(q),
            expected_shape=(6, 7),
            field_name="jacobian",
        )
        twist = jacobian @ joint_action.nominal_qdot_rad_s
        delta_x = jacobian @ joint_action.delta_q_rad
        normalized = (
            None
            if self.normalization is None
            else self.normalization.normalize_step(delta_x)
        )
        jacobian_diagnostic = diagnose_jacobian(
            jacobian,
            singular_value_tolerance=self.singular_value_tolerance,
        )
        null_diagnostic = diagnose_null_space(
            jacobian,
            joint_action.nominal_qdot_rad_s,
            singular_value_tolerance=self.singular_value_tolerance,
        )
        return CartesianPolicyAction(
            joint_position_rad=_readonly(q),
            joint_action=joint_action,
            jacobian=_readonly(jacobian),
            twist_si=_readonly(twist),
            delta_x_linearized=_readonly(delta_x),
            normalized_motion=normalized,
            gripper_closure=gripper,
            jacobian_diagnostic=jacobian_diagnostic,
            null_space_diagnostic=null_diagnostic,
        )


def validate_joint_mapping(
    policy_joint_names: Sequence[str],
    fr3_joint_names: Sequence[str],
) -> None:
    """Require the verified ordinal Panda-to-FR3 arm mapping."""

    policy = tuple(policy_joint_names)
    fr3 = tuple(fr3_joint_names)
    if policy != DROID_PANDA_JOINT_NAMES:
        raise ValueError(
            "DROID policy joints must be ordered panda_joint1 through "
            f"panda_joint7; received {policy!r}."
        )
    if fr3 != FR3_JOINT_NAMES:
        raise ValueError(
            "FR3 provider joints must be ordered fr3_joint1 through "
            f"fr3_joint7; received {fr3!r}."
        )


def validate_gripper_closure(value: float) -> float:
    """Validate canonical physical closure: zero open, one closed."""

    if not np.isfinite(value):
        raise ValueError("gripper closure must be finite.")
    if value < 0.0 or value > 1.0:
        raise ValueError("gripper closure must be within [0, 1].")
    return float(value)


def diagnose_jacobian(
    jacobian: np.ndarray,
    *,
    singular_value_tolerance: float = 1e-10,
) -> JacobianDiagnostic:
    """Compute deterministic rank and conditioning diagnostics."""

    matrix = _finite_float_array(
        jacobian,
        expected_shape=(6, 7),
        field_name="jacobian",
    )
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    maximum = float(singular_values[0])
    cutoff = singular_value_tolerance * max(1.0, maximum)
    rank = int(np.count_nonzero(singular_values > cutoff))
    minimum = float(singular_values[-1])
    near_singular = rank < 6
    condition = (
        float("inf")
        if near_singular
        else maximum / minimum
    )
    return JacobianDiagnostic(
        singular_values=_readonly(singular_values),
        rank=rank,
        condition_number=condition,
        near_singular=near_singular,
    )


def diagnose_null_space(
    jacobian: np.ndarray,
    qdot: np.ndarray,
    *,
    singular_value_tolerance: float = 1e-10,
) -> NullSpaceDiagnostic:
    """Decompose qdot into minimum-norm task and null components."""

    matrix = _finite_float_array(
        jacobian,
        expected_shape=(6, 7),
        field_name="jacobian",
    )
    velocity = _finite_float_array(
        qdot,
        expected_shape=(7,),
        field_name="qdot",
    )
    task_twist = matrix @ velocity
    task_qdot = np.linalg.pinv(
        matrix,
        rcond=singular_value_tolerance,
    ) @ task_twist
    null_qdot = velocity - task_qdot
    qdot_norm = float(np.linalg.norm(velocity))
    task_norm = float(np.linalg.norm(task_qdot))
    null_norm = float(np.linalg.norm(null_qdot))
    null_fraction = 0.0 if qdot_norm == 0.0 else null_norm / qdot_norm
    return NullSpaceDiagnostic(
        task_qdot_rad_s=_readonly(task_qdot),
        null_qdot_rad_s=_readonly(null_qdot),
        qdot_norm_rad_s=qdot_norm,
        task_norm_rad_s=task_norm,
        null_norm_rad_s=null_norm,
        null_fraction=null_fraction,
    )


def _finite_float_array(
    value: np.ndarray,
    *,
    expected_shape: tuple[int, ...],
    field_name: str,
) -> np.ndarray:
    result = np.asarray(value)
    if not np.issubdtype(result.dtype, np.floating):
        raise TypeError(
            f"{field_name} must have a floating dtype, received "
            f"{result.dtype}."
        )
    if result.shape != expected_shape:
        raise ValueError(
            f"{field_name} must have shape {expected_shape}, received "
            f"{result.shape}."
        )
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{field_name} must contain only finite values.")
    return np.array(result, dtype=np.float64, copy=True)


def _readonly(value: np.ndarray) -> np.ndarray:
    result = np.array(value, dtype=np.float64, copy=True)
    result.setflags(write=False)
    return result
