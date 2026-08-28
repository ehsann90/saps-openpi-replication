#!/usr/bin/env python3
"""Project genuine M1 pi05-DROID actions through the FR3 model offline."""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
from typing import Any

import numpy as np

from saps.physical.embodiment import CartesianNormalization
from saps.physical.embodiment import DROID_CONTROL_HZ
from saps.physical.embodiment import DROID_PANDA_JOINT_NAMES
from saps.physical.embodiment import DROID_RELATIVE_MAX_JOINT_DELTA_RAD
from saps.physical.embodiment import DROID_TO_FR3_JOINT_MAPPING
from saps.physical.embodiment import DroidToFr3TaskSpaceAdapter
from saps.physical.embodiment import FR3_JOINT_NAMES
from saps.physical.fr3_kinematics import Fr3PinocchioKinematics


EXPECTED_EPISODE = "IRIS+7dfa2da3+2023-12-04-15h-44m-25s"
EXPECTED_STEPS = (0, 76, 152)
M1_CHECKPOINT = "gs://openpi-assets/checkpoints/pi05_droid"
OPENPI_COMMIT = "15a9616a00943ada6c20a0f158e3adb39df2ccac"


@dataclasses.dataclass(frozen=True)
class Args:
    m1_run_path: Path
    output_dir: Path
    repository_root: Path
    franka_description_dir: Path
    igd_control_dir: Path
    m1_starting_commit: str
    translation_scale_m: float
    rotation_scale_rad: float


def main(args: Args) -> None:
    """Run a non-actuating kinematic rollout and write structured evidence."""

    _validate_paths(args)
    m1_run = json.loads(args.m1_run_path.read_text())
    source_samples = _validate_m1_run(m1_run)
    xacro_path = (
        args.franka_description_dir / "robots/fr3/fr3.urdf.xacro"
    )
    kinematics = Fr3PinocchioKinematics.from_xacro(xacro_path)
    normalization = CartesianNormalization(
        translation_scale_m=args.translation_scale_m,
        rotation_scale_rad=args.rotation_scale_rad,
    )
    adapter = DroidToFr3TaskSpaceAdapter(
        kinematics,
        normalization=normalization,
    )

    projected_samples = []
    for source_sample in source_samples:
        q = np.asarray(
            source_sample["observation"]["joint_position"],
            dtype=np.float64,
        )
        initial_q = q.copy()
        projected_actions = []
        for action_index, source_action in enumerate(
            source_sample["policy_response"]["actions"]
        ):
            action = np.asarray(source_action, dtype=np.float64)
            projected = adapter.project(action, q)
            repeated = adapter.project(action, q)
            actual_delta = kinematics.finite_step_displacement(
                q,
                projected.joint_action.delta_q_rad,
            )
            q_after = q + projected.joint_action.delta_q_rad
            projected_actions.append(
                _action_record(
                    action_index=action_index,
                    projected=projected,
                    repeated=repeated,
                    actual_delta=actual_delta,
                    q_after=q_after,
                    kinematics=kinematics,
                )
            )
            q = q_after
        projected_samples.append(
            {
                "sample_identity": source_sample["sample_identity"],
                "episode_uuid": source_sample["episode_uuid"],
                "step_index": int(source_sample["step_index"]),
                "trajectory_kind": (
                    "DROID-command kinematic rollout; q[0] is the recorded "
                    "DROID Panda observation and q[k+1] = q[k] + delta_q[k]; "
                    "states are not measured FR3 states"
                ),
                "initial_joint_position_rad": initial_q.tolist(),
                "final_joint_position_rad": q.tolist(),
                "actions": projected_actions,
            }
        )

    run_record = {
        "schema_version": 1,
        "diagnostic_scope": (
            "offline, non-actuating FR3 kinematics only; no ROS graph, "
            "robot state, Servo publication, or gripper command"
        ),
        "provenance": _provenance(args, kinematics),
        "joint_mapping": {
            "droid_robot": "Franka Emika Panda",
            "selected_episode_robot_serial": "panda-295341-1326372",
            "policy_joint_order": list(DROID_PANDA_JOINT_NAMES),
            "fr3_joint_order": list(FR3_JOINT_NAMES),
            "mapping": [
                {"policy": source, "fr3": target}
                for source, target in DROID_TO_FR3_JOINT_MAPPING
            ],
            "supporting_sources": [
                (
                    "selected DROID episode metadata robot_serial field "
                    "(panda-295341-1326372)"
                ),
                (
                    "pinned OpenPI examples/droid/"
                    "convert_droid_data_to_lerobot.py copies DROID joint "
                    "arrays unchanged and declares robot_type=panda"
                ),
                (
                    "DROID droid/franka/launch_robot.sh selects Polymetis "
                    "franka_hardware"
                ),
                (
                    "Polymetis franka_panda.yaml controls URDF indices "
                    "0..6; panda_arm.urdf orders panda_joint1..7"
                ),
                (
                    "laboratory franka_description franka_arm.xacro and "
                    "fr3_arm SRDF chain order fr3_joint1..7"
                ),
            ],
        },
        "mathematical_contract": {
            "native_clipping": "s=max(1,max_i(abs(u_i))); u_clipped=u/s",
            "delta_q": (
                f"delta_q={DROID_RELATIVE_MAX_JOINT_DELTA_RAD}*"
                "u_clipped rad/policy-step"
            ),
            "nominal_qdot": (
                f"qdot_nominal={DROID_CONTROL_HZ}*delta_q rad/s"
            ),
            "twist_si": "J_FR3(q)*qdot_nominal [m/s; rad/s]",
            "delta_x_linearized": (
                "J_FR3(q)*delta_q [m/policy-step; rad/policy-step]"
            ),
            "normalized_motion": (
                "delta_x_linearized/[0.075 m,0.075 m,0.075 m,"
                "0.15 rad,0.15 rad,0.15 rad]; no clipping"
            ),
            "state_contract": (
                "one action and the current q produce one projection; a "
                "fresh q/Jacobian is required for every eventual runtime step"
            ),
        },
        "jacobian_contract": {
            "backend": "Pinocchio",
            "backend_version": kinematics.backend_version,
            "shape": [6, 7],
            "row_order": [
                "linear_x_m_s",
                "linear_y_m_s",
                "linear_z_m_s",
                "angular_x_rad_s",
                "angular_y_rad_s",
                "angular_z_rad_s",
            ],
            "column_order": list(FR3_JOINT_NAMES),
            "base_reference_frame": kinematics.base_frame,
            "end_effector_point": kinematics.end_effector_frame,
            "component_expression_frame": kinematics.base_frame,
            "pinocchio_reference": "LOCAL_WORLD_ALIGNED, rotated to base",
        },
        "normalization": {
            "status": "defined as explicit DROID-reference representation",
            "translation_scale_m_per_policy_step": (
                normalization.translation_scale_m
            ),
            "rotation_scale_rad_per_policy_step": (
                normalization.rotation_scale_rad
            ),
            "clipping": "none",
            "servo_scaling_is_definition": False,
        },
        "gripper_contract": {
            "representation": "continuous normalized closure fraction",
            "range": [0.0, 1.0],
            "open": 0.0,
            "closed": 1.0,
            "thresholded": False,
            "commanded": False,
        },
        "fr3_limits": {
            "joint_order": list(FR3_JOINT_NAMES),
            "position_lower_rad": kinematics.position_lower_rad.tolist(),
            "position_upper_rad": kinematics.position_upper_rad.tolist(),
            "velocity_rad_s": kinematics.velocity_limit_rad_s.tolist(),
            "source": "limits expanded from laboratory FR3 URDF xacro",
        },
        "source_m1": {
            "run_path": str(args.m1_run_path.resolve()),
            "run_sha256": _sha256(args.m1_run_path),
            "checkpoint": M1_CHECKPOINT,
            "openpi_commit": OPENPI_COMMIT,
            "episode_uuid": EXPECTED_EPISODE,
            "steps": list(EXPECTED_STEPS),
            "selected_episode_metadata_sha256": _sha256(
                args.repository_root
                / "data/droid_m1/raw"
                / EXPECTED_EPISODE
                / f"metadata_{EXPECTED_EPISODE}.json"
            ),
        },
        "samples": projected_samples,
    }
    run_record["aggregate"] = _aggregate(projected_samples)
    _write_json_exclusive(args.output_dir, run_record)
    print(json.dumps(run_record["aggregate"], indent=2, sort_keys=True))
    print(f"Wrote M2 offline projection to {args.output_dir / 'run.json'}")


def _action_record(
    *,
    action_index: int,
    projected: Any,
    repeated: Any,
    actual_delta: np.ndarray,
    q_after: np.ndarray,
    kinematics: Fr3PinocchioKinematics,
) -> dict[str, Any]:
    joint = projected.joint_action
    jacobian_diagnostic = projected.jacobian_diagnostic
    null = projected.null_space_diagnostic
    linearized = projected.delta_x_linearized
    velocity_ratio = (
        np.abs(joint.nominal_qdot_rad_s)
        / kinematics.velocity_limit_rad_s
    )
    current_position_violation = np.logical_or(
        projected.joint_position_rad < kinematics.position_lower_rad,
        projected.joint_position_rad > kinematics.position_upper_rad,
    )
    next_position_violation = np.logical_or(
        q_after < kinematics.position_lower_rad,
        q_after > kinematics.position_upper_rad,
    )
    translation_error = float(
        np.linalg.norm(actual_delta[:3] - linearized[:3])
    )
    rotation_error = float(
        np.linalg.norm(actual_delta[3:] - linearized[3:])
    )
    deterministic = all(
        np.array_equal(first, second)
        for first, second in (
            (projected.jacobian, repeated.jacobian),
            (projected.twist_si, repeated.twist_si),
            (projected.delta_x_linearized, repeated.delta_x_linearized),
            (projected.normalized_motion, repeated.normalized_motion),
        )
    )
    condition = jacobian_diagnostic.condition_number
    return {
        "action_index": action_index,
        "joint_position_rad": projected.joint_position_rad.tolist(),
        "native_policy_motion": joint.native.tolist(),
        "gripper_closure": projected.gripper_closure,
        "clipping_scale": joint.clipping_scale,
        "clipped_policy_motion": joint.clipped.tolist(),
        "delta_q_rad": joint.delta_q_rad.tolist(),
        "nominal_qdot_rad_s": joint.nominal_qdot_rad_s.tolist(),
        "jacobian": projected.jacobian.tolist(),
        "cartesian_twist_si": projected.twist_si.tolist(),
        "cartesian_delta_linearized": linearized.tolist(),
        "normalized_task_motion": projected.normalized_motion.tolist(),
        "jacobian_diagnostic": {
            "singular_values": (
                jacobian_diagnostic.singular_values.tolist()
            ),
            "rank": jacobian_diagnostic.rank,
            "condition_number": (
                condition if np.isfinite(condition) else None
            ),
            "condition_number_finite": bool(np.isfinite(condition)),
            "near_singular": jacobian_diagnostic.near_singular,
        },
        "null_space_diagnostic": {
            "task_qdot_rad_s": null.task_qdot_rad_s.tolist(),
            "null_qdot_rad_s": null.null_qdot_rad_s.tolist(),
            "qdot_norm_rad_s": null.qdot_norm_rad_s,
            "task_norm_rad_s": null.task_norm_rad_s,
            "null_norm_rad_s": null.null_norm_rad_s,
            "null_fraction": null.null_fraction,
        },
        "finite_step_fk": {
            "actual_delta": actual_delta.tolist(),
            "translation_error_m": translation_error,
            "rotation_error_rad": rotation_error,
        },
        "limit_diagnostic": {
            "nominal_velocity_ratio": velocity_ratio.tolist(),
            "velocity_violation": (velocity_ratio > 1.0).tolist(),
            "current_position_violation": (
                current_position_violation.tolist()
            ),
            "next_position_violation": next_position_violation.tolist(),
            "q_after_rad": q_after.tolist(),
        },
        "deterministic_repeat": deterministic,
    }


def _aggregate(samples: list[dict[str, Any]]) -> dict[str, Any]:
    actions = [
        action
        for sample in samples
        for action in sample["actions"]
    ]
    native = np.asarray(
        [action["native_policy_motion"] for action in actions]
    )
    delta_q = np.asarray([action["delta_q_rad"] for action in actions])
    qdot = np.asarray(
        [action["nominal_qdot_rad_s"] for action in actions]
    )
    twist = np.asarray(
        [action["cartesian_twist_si"] for action in actions]
    )
    delta_x = np.asarray(
        [action["cartesian_delta_linearized"] for action in actions]
    )
    normalized = np.asarray(
        [action["normalized_task_motion"] for action in actions]
    )
    velocity_ratio = np.asarray(
        [
            action["limit_diagnostic"]["nominal_velocity_ratio"]
            for action in actions
        ]
    )
    velocity_violation = velocity_ratio > 1.0
    next_position_violation = np.asarray(
        [
            action["limit_diagnostic"]["next_position_violation"]
            for action in actions
        ]
    )
    conditions = np.asarray(
        [
            action["jacobian_diagnostic"]["condition_number"]
            for action in actions
            if action["jacobian_diagnostic"]["condition_number"]
            is not None
        ],
        dtype=np.float64,
    )
    minimum_singular_values = np.asarray(
        [
            action["jacobian_diagnostic"]["singular_values"][-1]
            for action in actions
        ]
    )
    null_fractions = np.asarray(
        [
            action["null_space_diagnostic"]["null_fraction"]
            for action in actions
        ]
    )
    task_norms = np.asarray(
        [
            action["null_space_diagnostic"]["task_norm_rad_s"]
            for action in actions
        ]
    )
    null_norms = np.asarray(
        [
            action["null_space_diagnostic"]["null_norm_rad_s"]
            for action in actions
        ]
    )
    translation_errors = np.asarray(
        [
            action["finite_step_fk"]["translation_error_m"]
            for action in actions
        ]
    )
    rotation_errors = np.asarray(
        [
            action["finite_step_fk"]["rotation_error_rad"]
            for action in actions
        ]
    )
    normalized_above_unit = np.abs(normalized) > 1.0
    clipping_count = sum(
        action["clipping_scale"] > 1.0 for action in actions
    )
    affected_velocity_joints = [
        FR3_JOINT_NAMES[index]
        for index in range(7)
        if np.any(velocity_violation[:, index])
    ]
    affected_position_joints = [
        FR3_JOINT_NAMES[index]
        for index in range(7)
        if np.any(next_position_violation[:, index])
    ]
    return {
        "sample_count": len(samples),
        "action_count": len(actions),
        "native_policy_motion": _array_summary(native),
        "clipping": {
            "action_count": clipping_count,
            "action_fraction": clipping_count / len(actions),
            "maximum_scale": max(
                action["clipping_scale"] for action in actions
            ),
        },
        "delta_q_rad": _array_summary(delta_q),
        "nominal_qdot_rad_s": _array_summary(qdot),
        "cartesian_twist_si": _array_summary(twist),
        "cartesian_delta_linearized": _array_summary(delta_x),
        "normalized_task_motion": {
            **_array_summary(normalized),
            "component_count_above_unit": int(
                np.count_nonzero(normalized_above_unit)
            ),
            "component_fraction_above_unit": float(
                np.mean(normalized_above_unit)
            ),
            "action_count_with_any_component_above_unit": int(
                np.count_nonzero(np.any(normalized_above_unit, axis=1))
            ),
        },
        "fr3_limits": {
            "velocity_component_violation_count": int(
                np.count_nonzero(velocity_violation)
            ),
            "velocity_component_violation_fraction": float(
                np.mean(velocity_violation)
            ),
            "action_count_with_velocity_violation": int(
                np.count_nonzero(np.any(velocity_violation, axis=1))
            ),
            "worst_velocity_limit_ratio": float(
                np.max(velocity_ratio)
            ),
            "velocity_affected_joints": affected_velocity_joints,
            "next_position_component_violation_count": int(
                np.count_nonzero(next_position_violation)
            ),
            "action_count_with_next_position_violation": int(
                np.count_nonzero(
                    np.any(next_position_violation, axis=1)
                )
            ),
            "position_affected_joints": affected_position_joints,
        },
        "jacobian": {
            "condition_number": _numeric_summary(conditions),
            "minimum_singular_value": _numeric_summary(
                minimum_singular_values
            ),
            "near_singular_action_count": sum(
                action["jacobian_diagnostic"]["near_singular"]
                for action in actions
            ),
        },
        "linearization": {
            "translation_error_m": _numeric_summary(translation_errors),
            "rotation_error_rad": _numeric_summary(rotation_errors),
            "worst_translation": _worst_case(
                samples,
                "translation_error_m",
            ),
            "worst_rotation": _worst_case(
                samples,
                "rotation_error_rad",
            ),
        },
        "null_space": {
            "task_qdot_norm_rad_s": _numeric_summary(task_norms),
            "null_qdot_norm_rad_s": _numeric_summary(null_norms),
            "null_fraction": _numeric_summary(null_fractions),
            "worst": _worst_null_case(samples),
        },
        "all_deterministic_repeats_exact": all(
            action["deterministic_repeat"] for action in actions
        ),
    }


def _worst_case(
    samples: list[dict[str, Any]],
    metric: str,
) -> dict[str, Any]:
    candidates = [
        (
            action["finite_step_fk"][metric],
            sample["sample_identity"],
            action["action_index"],
        )
        for sample in samples
        for action in sample["actions"]
    ]
    value, identity, action_index = max(candidates)
    return {
        "value": value,
        "sample_identity": identity,
        "action_index": action_index,
    }


def _worst_null_case(
    samples: list[dict[str, Any]],
) -> dict[str, Any]:
    candidates = [
        (
            action["null_space_diagnostic"]["null_fraction"],
            sample["sample_identity"],
            action["action_index"],
            action["null_space_diagnostic"],
        )
        for sample in samples
        for action in sample["actions"]
    ]
    fraction, identity, action_index, diagnostic = max(candidates)
    return {
        "null_fraction": fraction,
        "sample_identity": identity,
        "action_index": action_index,
        "qdot_norm_rad_s": diagnostic["qdot_norm_rad_s"],
        "task_norm_rad_s": diagnostic["task_norm_rad_s"],
        "null_norm_rad_s": diagnostic["null_norm_rad_s"],
    }


def _array_summary(values: np.ndarray) -> dict[str, Any]:
    return {
        "minimum": float(np.min(values)),
        "maximum": float(np.max(values)),
        "maximum_absolute": float(np.max(np.abs(values))),
        "per_dimension_minimum": np.min(values, axis=0).tolist(),
        "per_dimension_maximum": np.max(values, axis=0).tolist(),
    }


def _numeric_summary(values: np.ndarray) -> dict[str, Any]:
    return {
        "minimum": float(np.min(values)),
        "maximum": float(np.max(values)),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
    }


def _validate_m1_run(run: dict[str, Any]) -> list[dict[str, Any]]:
    provenance = run.get("provenance", {})
    if provenance.get("openpi_commit") != OPENPI_COMMIT:
        raise ValueError("M1 run does not use the pinned OpenPI commit.")
    if provenance.get("checkpoint") != M1_CHECKPOINT:
        raise ValueError("M1 run does not use the pi05_droid checkpoint.")
    samples = run.get("samples")
    if not isinstance(samples, list) or len(samples) != 3:
        raise ValueError("M2 requires exactly the three M1 samples.")
    if tuple(sample.get("step_index") for sample in samples) != EXPECTED_STEPS:
        raise ValueError("M1 run does not contain steps 0, 76, 152 in order.")
    for sample in samples:
        if sample.get("episode_uuid") != EXPECTED_EPISODE:
            raise ValueError("M1 sample episode identity is unexpected.")
        action = np.asarray(sample["policy_response"]["actions"])
        if action.shape != (15, 8) or not np.all(np.isfinite(action)):
            raise ValueError("Every M1 action chunk must be finite [15, 8].")
        q = np.asarray(sample["observation"]["joint_position"])
        if q.shape != (7,) or not np.all(np.isfinite(q)):
            raise ValueError("Every M1 joint observation must be finite [7].")
    return samples


def _provenance(
    args: Args,
    kinematics: Fr3PinocchioKinematics,
) -> dict[str, Any]:
    franka_files = (
        "robots/fr3/fr3.urdf.xacro",
        "robots/fr3/joint_limits.yaml",
        "robots/fr3/kinematics.yaml",
        "robots/common/franka_robot.xacro",
        "robots/common/franka_arm.xacro",
        "robots/common/group_definition.xacro",
        "end_effectors/common/franka_hand.xacro",
    )
    source_hashes = {
        relative: _sha256(args.franka_description_dir / relative)
        for relative in franka_files
    }
    franka_diff = _git_bytes(
        args.franka_description_dir,
        "diff",
        "--binary",
        "--no-ext-diff",
    )
    group_diff = _git_bytes(
        args.franka_description_dir,
        "diff",
        "--binary",
        "--no-ext-diff",
        "--",
        "robots/common/group_definition.xacro",
    )
    igd_diff = _git_bytes(
        args.igd_control_dir,
        "diff",
        "--binary",
        "--no-ext-diff",
    )
    return {
        "repository": {
            "branch": _git_text(args.repository_root, "branch", "--show-current"),
            "m1_starting_commit": args.m1_starting_commit,
            "head": _git_text(args.repository_root, "rev-parse", "HEAD"),
            "status": _git_text(
                args.repository_root,
                "status",
                "--short",
            ).splitlines(),
        },
        "openpi_commit": OPENPI_COMMIT,
        "franka_description": {
            "path": str(args.franka_description_dir.resolve()),
            "commit": _git_text(
                args.franka_description_dir,
                "rev-parse",
                "HEAD",
            ),
            "status": _git_text(
                args.franka_description_dir,
                "status",
                "--short",
            ).splitlines(),
            "full_local_diff_sha256": _sha256_bytes(franka_diff),
            "group_definition_diff_sha256": _sha256_bytes(group_diff),
            "source_file_sha256": source_hashes,
            "local_diff_assessment": (
                "Only named fr3_arm group_state postures are added to the "
                "SRDF helper. The diff changes no chain, joint ordering, "
                "frame, group membership, URDF, or Jacobian input."
            ),
        },
        "igd_fr3_control": {
            "path": str(args.igd_control_dir.resolve()),
            "commit": _git_text(args.igd_control_dir, "rev-parse", "HEAD"),
            "status": _git_text(
                args.igd_control_dir,
                "status",
                "--short",
            ).splitlines(),
            "full_local_diff_sha256": _sha256_bytes(igd_diff),
            "servo_config_sha256": _sha256(
                args.igd_control_dir / "config/servo_config.yaml"
            ),
        },
        "kinematic_backend": {
            "name": "Pinocchio",
            "version": kinematics.backend_version,
            "generated_urdf_sha256": kinematics.urdf_sha256,
            "finger_joints": "locked at neutral in reduced offline model",
        },
        "runtime": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": np.__version__,
        },
    }


def _validate_paths(args: Args) -> None:
    if args.output_dir.exists():
        raise FileExistsError(
            f"Output directory already exists: {args.output_dir}."
        )
    for name, path in (
        ("M1 run", args.m1_run_path),
        ("repository", args.repository_root),
        ("franka_description", args.franka_description_dir),
        ("igd_fr3_control", args.igd_control_dir),
    ):
        if not path.exists():
            raise FileNotFoundError(f"{name} path not found: {path}")


def _write_json_exclusive(
    output_dir: Path,
    record: dict[str, Any],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=False)
    temporary = output_dir / "run.json.tmp"
    destination = output_dir / "run.json"
    with temporary.open("x") as file:
        json.dump(record, file, indent=2, sort_keys=True, allow_nan=False)
        file.write("\n")
    temporary.replace(destination)


def _git_text(path: Path, *arguments: str) -> str:
    return _git_bytes(path, *arguments).decode("utf-8").rstrip()


def _git_bytes(path: Path, *arguments: str) -> bytes:
    return subprocess.run(
        ("git", "-C", str(path), *arguments),
        check=True,
        stdout=subprocess.PIPE,
    ).stdout


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _parse_args() -> Args:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--m1-run-path",
        type=Path,
        default=Path(
            "outputs/physical_pi05_droid_m1/"
            "validation_final_20260827T1318Z/run.json"
        ),
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repository-root", type=Path, default=Path("."))
    parser.add_argument(
        "--franka-description-dir",
        type=Path,
        default=Path(
            "/home/hvl-robotics2404/franka_ros2_ws/src/"
            "franka_description"
        ),
    )
    parser.add_argument(
        "--igd-control-dir",
        type=Path,
        default=Path(
            "/home/hvl-robotics2404/franka_ros2_ws/src/igd_fr3_control"
        ),
    )
    parser.add_argument(
        "--m1-starting-commit",
        default="ea762d782c68024ce1f2ce3a9f764b2e6122f198",
    )
    parser.add_argument("--translation-scale-m", type=float, default=0.075)
    parser.add_argument("--rotation-scale-rad", type=float, default=0.15)
    parsed = parser.parse_args()
    return Args(**vars(parsed))


if __name__ == "__main__":
    main(_parse_args())
