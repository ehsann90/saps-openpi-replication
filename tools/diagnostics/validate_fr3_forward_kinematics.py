#!/usr/bin/env python3
"""Validate the hand-derived FR3 FK and Jacobian independently."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from saps.physical.fr3_forward_kinematics import fr3_flange_fk
from saps.physical.fr3_forward_kinematics import fr3_tcp_fk
from saps.physical.fr3_forward_kinematics import fr3_tcp_jacobian
from saps.physical.fr3_kinematics import Fr3PinocchioKinematics


DEFAULT_FR3_XACRO = (
    Path.home()
    / "franka_ros2_ws/src/franka_description/robots/fr3/fr3.urdf.xacro"
)
DEFAULT_RANDOM_SAMPLES = 1000
DEFAULT_SEED = 20260902
POSITION_TOLERANCE_M = 1e-9
ROTATION_TOLERANCE_RAD = 1e-9
PINOCCHIO_JACOBIAN_TOLERANCE = 1e-10
FINITE_DIFFERENCE_JACOBIAN_TOLERANCE = 1e-7
FINITE_DIFFERENCE_EPSILONS = (1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9)

# A measured joint configuration from the accepted physical M3 capture.
M3_RECORDED_Q = np.asarray(
    [
        -0.023777075413167925,
        -0.26790710332609774,
        0.16103233919936977,
        -2.7272902786495137,
        0.04504917464769999,
        2.470422173191375,
        0.9354397407288674,
    ],
    dtype=np.float64,
)


def skew_to_vector(skew: np.ndarray) -> np.ndarray:
    """Return the vector whose cross-product matrix is ``skew``."""

    return np.asarray(
        [skew[2, 1], skew[0, 2], skew[1, 0]],
        dtype=np.float64,
    )


def rotation_angle_error(
    reference_rotation: np.ndarray,
    test_rotation: np.ndarray,
) -> float:
    """Return rotation-matrix angular distance in radians."""

    rotation_error = reference_rotation.T @ test_rotation
    cosine = np.clip((np.trace(rotation_error) - 1.0) / 2.0, -1.0, 1.0)
    skew_vector = np.asarray(
        [
            rotation_error[2, 1] - rotation_error[1, 2],
            rotation_error[0, 2] - rotation_error[2, 0],
            rotation_error[1, 0] - rotation_error[0, 1],
        ]
    )
    sine = 0.5 * np.linalg.norm(skew_vector)
    return float(np.arctan2(sine, cosine))


def transform_error(
    reference: np.ndarray,
    test: np.ndarray,
) -> tuple[float, float]:
    """Return position error in metres and orientation error in radians."""

    position_error_m = np.linalg.norm(reference[:3, 3] - test[:3, 3])
    rotation_error_rad = rotation_angle_error(
        reference[:3, :3],
        test[:3, :3],
    )
    return float(position_error_m), rotation_error_rad


def check_homogeneous_transform(transform: np.ndarray) -> None:
    """Require a finite proper homogeneous transform."""

    if transform.shape != (4, 4):
        raise AssertionError(f"Expected (4, 4), got {transform.shape}.")
    if not np.all(np.isfinite(transform)):
        raise AssertionError("Transform contains non-finite values.")
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-12):
        raise AssertionError("Invalid homogeneous bottom row.")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12):
        raise AssertionError("Rotation matrix is not orthonormal.")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-12):
        raise AssertionError("Rotation matrix determinant is not one.")


def compare_one_configuration(
    joint_position_rad: np.ndarray,
    flange_reference: Fr3PinocchioKinematics,
    tcp_reference: Fr3PinocchioKinematics,
) -> dict[str, float]:
    """Compare manual and Pinocchio FK at one joint configuration."""

    flange_manual = fr3_flange_fk(joint_position_rad)
    tcp_manual = fr3_tcp_fk(joint_position_rad)
    check_homogeneous_transform(flange_manual)
    check_homogeneous_transform(tcp_manual)
    flange_position, flange_rotation = transform_error(
        flange_reference.forward_kinematics(joint_position_rad),
        flange_manual,
    )
    tcp_position, tcp_rotation = transform_error(
        tcp_reference.forward_kinematics(joint_position_rad),
        tcp_manual,
    )
    return {
        "flange_position_error_m": flange_position,
        "flange_rotation_error_rad": flange_rotation,
        "tcp_position_error_m": tcp_position,
        "tcp_rotation_error_rad": tcp_rotation,
    }


def run_fk_validation(
    xacro_path: Path,
    *,
    number_random_samples: int = DEFAULT_RANDOM_SAMPLES,
    seed: int = DEFAULT_SEED,
) -> bool:
    """Validate manual flange and TCP FK against Pinocchio."""

    flange_reference = Fr3PinocchioKinematics.from_xacro(
        xacro_path,
        end_effector_frame="fr3_link8",
    )
    tcp_reference = Fr3PinocchioKinematics.from_xacro(
        xacro_path,
        end_effector_frame="fr3_hand_tcp",
    )
    print("Pinocchio version:", tcp_reference.backend_version)
    print("URDF SHA-256:", tcp_reference.urdf_sha256)

    recorded = compare_one_configuration(
        M3_RECORDED_Q,
        flange_reference,
        tcp_reference,
    )
    print("\n=== Recorded M3 configuration ===")
    print("q =", M3_RECORDED_Q)
    for name, value in recorded.items():
        print(f"{name}: {value:.12e}")

    lower, upper = _sampling_limits(tcp_reference)
    rng = np.random.default_rng(seed)
    worst: dict[str, tuple[float, np.ndarray | None]] = {
        name: (-1.0, None) for name in recorded
    }
    for _ in range(number_random_samples):
        q = rng.uniform(lower, upper)
        for name, value in compare_one_configuration(
            q,
            flange_reference,
            tcp_reference,
        ).items():
            if value > worst[name][0]:
                worst[name] = (value, q.copy())

    print(f"\n=== {number_random_samples} random configurations ===")
    for name, (value, q) in worst.items():
        print(f"{name}: {value:.12e}")
        print(f"  worst q: {q}")

    passed = (
        worst["flange_position_error_m"][0] <= POSITION_TOLERANCE_M
        and worst["tcp_position_error_m"][0] <= POSITION_TOLERANCE_M
        and worst["flange_rotation_error_rad"][0]
        <= ROTATION_TOLERANCE_RAD
        and worst["tcp_rotation_error_rad"][0] <= ROTATION_TOLERANCE_RAD
    )
    print("\n=== FK RESULT ===")
    print("PASS" if passed else "FAIL")
    return passed


def finite_difference_jacobian(
    joint_position_rad: np.ndarray,
    epsilon_rad: float = 1e-7,
) -> np.ndarray:
    """Differentiate manual TCP FK using centred joint perturbations."""

    if not np.isfinite(epsilon_rad) or epsilon_rad <= 0.0:
        raise ValueError("epsilon_rad must be finite and positive.")
    q = np.asarray(joint_position_rad, dtype=np.float64)
    initial_rotation = fr3_tcp_fk(q)[:3, :3]
    numeric = np.zeros((6, 7), dtype=np.float64)
    for index in range(7):
        q_plus = q.copy()
        q_minus = q.copy()
        q_plus[index] += epsilon_rad
        q_minus[index] -= epsilon_rad
        transform_plus = fr3_tcp_fk(q_plus)
        transform_minus = fr3_tcp_fk(q_minus)
        numeric[:3, index] = (
            transform_plus[:3, 3] - transform_minus[:3, 3]
        ) / (2.0 * epsilon_rad)
        rotation_derivative = (
            transform_plus[:3, :3] - transform_minus[:3, :3]
        ) / (2.0 * epsilon_rad)
        angular_skew = rotation_derivative @ initial_rotation.T
        angular_skew = 0.5 * (angular_skew - angular_skew.T)
        numeric[3:, index] = skew_to_vector(angular_skew)
    return numeric


def run_jacobian_validation(
    xacro_path: Path,
    *,
    number_random_samples: int = DEFAULT_RANDOM_SAMPLES,
    seed: int = DEFAULT_SEED,
) -> bool:
    """Validate the analytical Jacobian by two independent methods."""

    print("\n=== Numerical Jacobian epsilon sweep ===")
    analytical = fr3_tcp_jacobian(M3_RECORDED_Q)
    for epsilon in FINITE_DIFFERENCE_EPSILONS:
        error = analytical - finite_difference_jacobian(
            M3_RECORDED_Q,
            epsilon,
        )
        print(f"epsilon: {epsilon:.1e}")
        print(f"  maximum absolute error: {np.max(np.abs(error)):.12e}")
        print(f"  Frobenius error: {np.linalg.norm(error):.12e}")

    reference = Fr3PinocchioKinematics.from_xacro(
        xacro_path,
        end_effector_frame="fr3_hand_tcp",
    )
    lower, upper = _sampling_limits(reference)
    rng = np.random.default_rng(seed)
    worst_pinocchio = 0.0
    worst_finite_difference = 0.0
    for _ in range(number_random_samples):
        q = rng.uniform(lower, upper)
        analytical = fr3_tcp_jacobian(q)
        worst_pinocchio = max(
            worst_pinocchio,
            float(np.max(np.abs(analytical - reference.jacobian(q)))),
        )
        worst_finite_difference = max(
            worst_finite_difference,
            float(
                np.max(
                    np.abs(analytical - finite_difference_jacobian(q))
                )
            ),
        )

    print(f"\n=== {number_random_samples} random Jacobians ===")
    print(
        "worst maximum absolute error vs Pinocchio: "
        f"{worst_pinocchio:.12e}"
    )
    print(
        "worst maximum absolute error vs finite difference: "
        f"{worst_finite_difference:.12e}"
    )
    passed = (
        worst_pinocchio <= PINOCCHIO_JACOBIAN_TOLERANCE
        and worst_finite_difference <= FINITE_DIFFERENCE_JACOBIAN_TOLERANCE
    )
    print("\n=== JACOBIAN RESULT ===")
    print("PASS" if passed else "FAIL")
    return passed


def _sampling_limits(
    reference: Fr3PinocchioKinematics,
) -> tuple[np.ndarray, np.ndarray]:
    """Return limits inset two percent from the URDF bounds."""

    lower = reference.position_lower_rad
    upper = reference.position_upper_rad
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
        raise RuntimeError("FR3 joint limits must be finite.")
    span = upper - lower
    return lower + 0.02 * span, upper - 0.02 * span


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xacro-path", type=Path, default=DEFAULT_FR3_XACRO)
    parser.add_argument(
        "--random-samples",
        type=int,
        default=DEFAULT_RANDOM_SAMPLES,
    )
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    args = parser.parse_args()
    if args.random_samples <= 0:
        parser.error("--random-samples must be positive")
    return args


def main() -> int:
    """Run all FK and Jacobian checks and return a process status."""

    args = _parse_args()
    xacro_path = args.xacro_path.expanduser().resolve()
    fk_passed = run_fk_validation(
        xacro_path,
        number_random_samples=args.random_samples,
        seed=args.seed,
    )
    jacobian_passed = run_jacobian_validation(
        xacro_path,
        number_random_samples=args.random_samples,
        seed=args.seed,
    )
    return 0 if fk_passed and jacobian_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
