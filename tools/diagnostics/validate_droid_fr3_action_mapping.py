#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np

from saps.physical.fr3_forward_kinematics import (
    fr3_tcp_fk,
    fr3_tcp_jacobian,
)
from saps.physical.fr3_kinematics import Fr3PinocchioKinematics
from saps.policies.openpi_droid import map_droid_reference_joint_action


REFERENCE_EXECUTION_HORIZON = 8
FULL_CHUNK_HORIZON = 15
MOTION_NORM_EPSILON = 1e-9
COSINE_DENOMINATOR_EPSILON = 1e-12


def droid_arm_delta_q(
    action: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert native pi05-DROID arm action to reference joint increment."""

    mapped = map_droid_reference_joint_action(action)
    return (
        mapped.policy_joint_coordinates,
        mapped.reference_joint_coordinates,
        mapped.delta_q_rad,
    )


def skew_to_vector(skew: np.ndarray) -> np.ndarray:
    """Return the vector whose cross-product matrix is ``skew``."""

    return np.array(
        [
            skew[2, 1],
            skew[0, 2],
            skew[1, 0],
        ],
        dtype=np.float64,
    )


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """Return axis-angle rotation vector for a rotation matrix."""

    cos_theta = (np.trace(rotation) - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    theta = float(np.arccos(cos_theta))

    vee = skew_to_vector(rotation - rotation.T)

    if theta < 1e-8:
        # R - R.T ~= 2 [theta * axis]_x
        return 0.5 * vee

    if np.pi - theta < 1e-6:
        raise RuntimeError(
            "Relative rotation too close to pi for this simple "
            "diagnostic log implementation."
        )

    return theta / (2.0 * np.sin(theta)) * vee


def exact_tcp_displacement(
    q: np.ndarray,
    delta_q: np.ndarray,
) -> np.ndarray:
    """Exact finite TCP displacement from our manually validated FK."""

    T0 = fr3_tcp_fk(q)
    T1 = fr3_tcp_fk(q + delta_q)

    delta_position = T1[:3, 3] - T0[:3, 3]

    # Spatial/base-resolved relative rotation.
    R_delta = T1[:3, :3] @ T0[:3, :3].T
    delta_rotation = rotation_vector(R_delta)

    return np.concatenate(
        [delta_position, delta_rotation]
    )


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float | None:
    """Return directional similarity, or ``None`` for near-zero motion."""

    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)

    denom = np.linalg.norm(a) * np.linalg.norm(b)

    if denom < COSINE_DENOMINATOR_EPSILON:
        return None

    return float(np.dot(a, b) / denom)


def compare_one_step(
    q: np.ndarray,
    action: np.ndarray,
    pinocchio: Fr3PinocchioKinematics | None = None,
) -> dict[str, Any]:
    """Compare Jacobian-linearized and exact FK motion for one action."""

    q = np.asarray(q, dtype=np.float64)

    u_pi, u_ref, delta_q = droid_arm_delta_q(action)

    J = fr3_tcp_jacobian(q)

    linear = J @ delta_q
    exact = exact_tcp_displacement(q, delta_q)

    error = linear - exact

    translation_error = float(
        np.linalg.norm(error[:3])
    )
    rotation_error = float(
        np.linalg.norm(error[3:])
    )

    exact_translation_norm = float(
        np.linalg.norm(exact[:3])
    )
    exact_rotation_norm = float(
        np.linalg.norm(exact[3:])
    )

    translation_relative_error = (
        translation_error / exact_translation_norm
        if exact_translation_norm > MOTION_NORM_EPSILON
        else None
    )

    rotation_relative_error = (
        rotation_error / exact_rotation_norm
        if exact_rotation_norm > MOTION_NORM_EPSILON
        else None
    )

    result = {
        "u_pi": u_pi,
        "u_ref": u_ref,
        "clipped": bool(np.any(u_pi != u_ref)),
        "delta_q": delta_q,
        "linear": linear,
        "exact": exact,
        "translation_error_m": translation_error,
        "rotation_error_rad": rotation_error,
        "translation_relative_error": translation_relative_error,
        "rotation_relative_error": rotation_relative_error,
        "translation_cosine": cosine_similarity(
            linear[:3], exact[:3]
        ),
        "rotation_cosine": cosine_similarity(
            linear[3:], exact[3:]
        ),
    }

    # Independent validation of our exact-FK displacement convention.
    if pinocchio is not None:
        exact_pin = pinocchio.finite_step_displacement(
            q,
            delta_q,
        )

        result["exact_pinocchio_max_error"] = float(
            np.max(np.abs(exact - exact_pin))
        )

    return result


def print_step(result: dict[str, Any]) -> None:
    """Print one fully auditable finite-action comparison."""
    print("u_pi:")
    print(result["u_pi"])

    print("\nu_ref = clip(u_pi, -1, 1):")
    print(result["u_ref"])
    print("clipped:", result["clipped"])

    print("\ndelta_q [rad]:")
    print(result["delta_q"])

    print("\nJ(q) @ delta_q:")
    print(result["linear"])

    print("\nExact FK finite displacement:")
    print(result["exact"])

    print(
        "\nTranslation error [m]:",
        f"{result['translation_error_m']:.12e}",
    )
    print(
        "Rotation error [rad]:",
        f"{result['rotation_error_rad']:.12e}",
    )

    print(
        "Translation relative error:",
        result["translation_relative_error"],
    )
    print(
        "Rotation relative error:",
        result["rotation_relative_error"],
    )

    print(
        "Translation direction cosine:",
        result["translation_cosine"],
    )
    print(
        "Rotation direction cosine:",
        result["rotation_cosine"],
    )

    if "exact_pinocchio_max_error" in result:
        print(
            "Our exact FK vs Pinocchio finite-step max error:",
            f"{result['exact_pinocchio_max_error']:.12e}",
        )


def numeric_error_summary(values: list[float]) -> dict[str, float]:
    """Summarize non-negative errors or magnitudes explicitly."""

    values = np.asarray(values, dtype=np.float64)

    if values.size == 0:
        raise ValueError("Cannot summarize an empty error sequence.")

    return {
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "maximum": float(np.max(values)),
    }


def numeric_similarity_summary(values: list[float]) -> dict[str, float]:
    """Summarize cosine similarities with lower-tail statistics."""

    values = np.asarray(values, dtype=np.float64)

    if values.size == 0:
        raise ValueError("Cannot summarize an empty similarity sequence.")

    return {
        "minimum": float(np.min(values)),
        "p05": float(np.percentile(values, 5)),
        "median": float(np.median(values)),
    }


def summarize_results(
    results: list[dict[str, Any]],
    label: str,
) -> None:
    """Print error, direction, and relative norm-deviation summaries."""
    translation_errors = [
        r["translation_error_m"]
        for r in results
    ]

    rotation_errors = [
        r["rotation_error_rad"]
        for r in results
    ]

    translation_relative = [
        r["translation_relative_error"]
        for r in results
        if r["translation_relative_error"] is not None
    ]

    rotation_relative = [
        r["rotation_relative_error"]
        for r in results
        if r["rotation_relative_error"] is not None
    ]

    clipped_count = sum(r["clipped"] for r in results)

    print(f"\n=== {label} ===")
    print("step count:", len(results))
    print("arm actions requiring clipping:", clipped_count)

    print(
        "translation error [m]:",
        numeric_error_summary(translation_errors),
    )

    print(
        "rotation error [rad]:",
        numeric_error_summary(rotation_errors),
    )

    if translation_relative:
        print(
            "translation relative error:",
            numeric_error_summary(translation_relative),
        )

    if rotation_relative:
        print(
            "rotation relative error:",
            numeric_error_summary(rotation_relative),
        )

    translation_cosines = [
        r["translation_cosine"]
        for r in results
        if r["translation_cosine"] is not None
    ]

    rotation_cosines = [
        r["rotation_cosine"]
        for r in results
        if r["rotation_cosine"] is not None
    ]

    print(
        "translation direction cosine:",
        numeric_similarity_summary(translation_cosines),
    )

    print(
        "rotation direction cosine:",
        numeric_similarity_summary(rotation_cosines),
    )

    translation_relative_norm_deviation = []
    rotation_relative_norm_deviation = []

    translation_norm = []  # translation in meters/action
    rotation_norm = []     # rotation in radians/action

    for r in results:
        lin_t = np.linalg.norm(r["linear"][:3])
        exact_t = np.linalg.norm(r["exact"][:3])

        lin_r = np.linalg.norm(r["linear"][3:])
        exact_r = np.linalg.norm(r["exact"][3:])

        if exact_t > MOTION_NORM_EPSILON:
            translation_relative_norm_deviation.append(
                abs(lin_t / exact_t - 1.0)
            )

        if exact_r > MOTION_NORM_EPSILON:
            rotation_relative_norm_deviation.append(
                abs(lin_r / exact_r - 1.0)
            )

        translation_norm.append(
            np.linalg.norm(r["exact"][:3])
        )
        rotation_norm.append(
            np.linalg.norm(r["exact"][3:])
        )

    print(
        "translation relative norm deviation |linear/exact - 1|:",
        numeric_error_summary(translation_relative_norm_deviation),
    )

    print(
        "rotation relative norm deviation |linear/exact - 1|:",
        numeric_error_summary(rotation_relative_norm_deviation),
    )

    print(
        "translation norm [m] per action:",
        numeric_error_summary(translation_norm),
    )

    print(
        "rotation norm [rad] per action:",
        numeric_error_summary(rotation_norm),
    )

    pinocchio_errors = [
        r["exact_pinocchio_max_error"]
        for r in results
        if "exact_pinocchio_max_error" in r
    ]
    if pinocchio_errors:
        print(
            "manual exact FK vs Pinocchio maximum component error "
            "[m or rad]:",
            numeric_error_summary(pinocchio_errors),
        )


def main(run_dir: Path, franka_description_dir: Path) -> None:
    """Run measured-state and model-based finite-action diagnostics."""
    observation_path = (
        run_dir / "observation_bundle.npz"
    )
    action_path = (
        run_dir / "policy_actions.npz"
    )

    if not observation_path.is_file():
        raise FileNotFoundError(observation_path)

    if not action_path.is_file():
        raise FileNotFoundError(action_path)

    with np.load(
        observation_path,
        allow_pickle=False,
    ) as bundle:
        joint_positions = np.asarray(bundle["joint_positions"])

    with np.load(
        action_path,
        allow_pickle=False,
    ) as bundle:
        actions = np.asarray(bundle["actions"])

    if not np.issubdtype(joint_positions.dtype, np.floating):
        raise TypeError("M3 joint positions must have a floating dtype.")
    if (
        joint_positions.ndim != 2
        or joint_positions.shape[0] == 0
        or joint_positions.shape[1] != 7
    ):
        raise ValueError(
            f"Unexpected q shape: {joint_positions.shape}"
        )

    if not np.issubdtype(actions.dtype, np.floating):
        raise TypeError("M3 policy actions must have a floating dtype.")
    if actions.shape != (
        joint_positions.shape[0],
        FULL_CHUNK_HORIZON,
        8,
    ):
        raise ValueError(
            f"Unexpected action shape: {actions.shape}"
        )
    if not np.all(np.isfinite(joint_positions)):
        raise ValueError("M3 joint positions must be finite.")
    if not np.all(np.isfinite(actions)):
        raise ValueError("M3 policy actions must be finite.")

    joint_positions = np.array(joint_positions, dtype=np.float64, copy=True)
    actions = np.array(actions, dtype=np.float64, copy=True)

    print("joint_positions shape:", joint_positions.shape)
    print("actions shape:", actions.shape)

    xacro = (
        franka_description_dir
        / "robots/fr3/fr3.urdf.xacro"
    )

    pinocchio = Fr3PinocchioKinematics.from_xacro(
        xacro,
        end_effector_frame="fr3_hand_tcp",
    )

    # --------------------------------------------------------
    # A. One genuine M3 measured-state example
    # --------------------------------------------------------

    q0 = joint_positions[0]
    action0 = actions[0, 0]

    print("\n========================================")
    print("OBSERVATION 0, ACTION 0")
    print("========================================")
    print("q:")
    print(q0)
    print("\nraw 8-D policy action:")
    print(action0)
    print("raw gripper value:", action0[7])

    first = compare_one_step(
        q0,
        action0,
        pinocchio=pinocchio,
    )

    print_step(first)

    # --------------------------------------------------------
    # B. First action from every measured M3 observation
    #
    # Each one uses a genuine measured q.
    # --------------------------------------------------------

    measured_state_results = []

    for observation_index in range(
        joint_positions.shape[0]
    ):
        result = compare_one_step(
            joint_positions[observation_index],
            actions[observation_index, 0],
            pinocchio=pinocchio,
        )

        measured_state_results.append(result)

    summarize_results(
        measured_state_results,
        "First action at each measured M3 state",
    )

    # --------------------------------------------------------
    # C. Idealized open-loop kinematic rollouts
    #
    # q[0] = actual captured M3 q.
    # q[k+1] = q[k] + delta_q[k].
    #
    # This is a kinematic model rollout, NOT measured FR3 motion.
    # --------------------------------------------------------

    rollout_results = []
    full_rollout_results = []

    lower = pinocchio.position_lower_rad
    upper = pinocchio.position_upper_rad

    reference_position_violation_count = 0
    full_chunk_position_violation_count = 0

    for observation_index in range(
        joint_positions.shape[0]
    ):
        q = joint_positions[observation_index].copy()

        for action_index in range(FULL_CHUNK_HORIZON):
            action = actions[
                observation_index,
                action_index,
            ]

            result = compare_one_step(
                q,
                action,
                pinocchio=pinocchio,
            )

            if action_index < REFERENCE_EXECUTION_HORIZON:
                rollout_results.append(result)

            full_rollout_results.append(result)

            q_next = q + result["delta_q"]

            if np.any(q_next < lower) or np.any(q_next > upper):
                full_chunk_position_violation_count += 1
                if action_index < REFERENCE_EXECUTION_HORIZON:
                    reference_position_violation_count += 1

            q = q_next

    summarize_results(
        rollout_results,
        "Kinematic/model-based sequential 8-action reference rollout",
    )

    summarize_results(
        full_rollout_results,
        "Kinematic/model-based 15-action full-chunk stress diagnostic",
    )

    print(
        "8-action model rollout next-state joint-limit violations:",
        reference_position_violation_count,
    )
    print(
        "15-action stress rollout next-state joint-limit violations:",
        full_chunk_position_violation_count,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--run-dir",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--franka-description-dir",
        type=Path,
        default=Path(
            "~/franka_ros2_ws/src/franka_description"
        ).expanduser(),
    )

    args = parser.parse_args()

    main(
        args.run_dir.resolve(),
        args.franka_description_dir.resolve(),
    )
