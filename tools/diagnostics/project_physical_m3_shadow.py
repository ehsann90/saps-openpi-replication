#!/usr/bin/env python3
"""Reproduce the superseded M2 projection of captured M3 actions.

This is an explicit legacy/provenance path, not the current physical mapping.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from saps.physical.embodiment import CartesianNormalization
from saps.physical.embodiment import DroidToFr3TaskSpaceAdapter
from saps.physical.fr3_kinematics import Fr3PinocchioKinematics


def main(args: argparse.Namespace) -> None:
    """Apply legacy M2 kinematics without robot transport."""

    run_dir = args.run_dir.resolve()
    capture_path = run_dir / "run.json"
    observation_bundle_path = run_dir / "observation_bundle.npz"
    policy_path = run_dir / "shadow_policy.json"
    action_bundle_path = run_dir / "policy_actions.npz"
    output_path = run_dir / "shadow_projection.json"
    for path in (
        capture_path,
        observation_bundle_path,
        policy_path,
        action_bundle_path,
    ):
        if not path.is_file():
            raise FileNotFoundError(path)
    if output_path.exists():
        raise FileExistsError(
            f"Refusing to overwrite existing M3 projection: {output_path}."
        )

    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    if _sha256(observation_bundle_path) != capture["bundle"]["sha256"]:
        raise ValueError("Observation bundle hash does not match capture.")
    if _sha256(action_bundle_path) != policy["action_bundle"]["sha256"]:
        raise ValueError("Policy action bundle hash does not match policy log.")
    with np.load(observation_bundle_path, allow_pickle=False) as bundle:
        joint_positions = np.array(bundle["joint_positions"])
    with np.load(action_bundle_path, allow_pickle=False) as bundle:
        actions = np.array(bundle["actions"])
    if actions.shape != (joint_positions.shape[0], 15, 8):
        raise ValueError(
            "M3 actions must have shape [observation_count, 15, 8]; "
            f"received {actions.shape}."
        )

    xacro_path = (
        args.franka_description_dir.resolve()
        / "robots/fr3/fr3.urdf.xacro"
    )
    kinematics = Fr3PinocchioKinematics.from_xacro(xacro_path)
    normalization = CartesianNormalization()
    adapter = DroidToFr3TaskSpaceAdapter(
        kinematics,
        normalization=normalization,
    )
    all_normalized = []
    gripper_boundary_records = []
    samples = []
    for observation_index, q in enumerate(joint_positions):
        projected_actions = []
        first_projection = None
        for action_index, action in enumerate(actions[observation_index]):
            projection_action, gripper_clipped = (
                _canonical_projection_action(action)
            )
            projected = adapter.project(projection_action, q)
            if first_projection is None:
                first_projection = projected
            all_normalized.append(projected.normalized_motion)
            gripper_boundary_records.append(
                {
                    "observation_index": observation_index,
                    "action_index": action_index,
                    "raw_closure": float(action[7]),
                    "canonical_closure": float(projection_action[7]),
                    "clipped": gripper_clipped,
                }
            )
            if action_index in (0, 7, 14):
                projected_actions.append(
                    _projection_record(
                        action_index,
                        action,
                        projected,
                    )
                )
        samples.append(
            {
                "observation_index": observation_index,
                "jacobian_state_rad": q.tolist(),
                "jacobian": first_projection.jacobian.tolist(),
                "jacobian_diagnostic": _jacobian_record(
                    first_projection.jacobian_diagnostic
                ),
                "selected_actions": projected_actions,
            }
        )
        print(
            f"shadow projection {observation_index + 1}/"
            f"{joint_positions.shape[0]}: condition="
            f"{first_projection.jacobian_diagnostic.condition_number:.3f}",
            flush=True,
        )

    normalized = np.stack(all_normalized)
    record = {
        "schema_version": 1,
        "milestone": "physical_pi05_droid_m3",
        "diagnostic_scope": (
            "offline M2 projection using captured live q; no ROS graph, "
            "publisher, service, action client, Servo command, robot "
            "command, or gripper command"
        ),
        "created_utc": _utc_now(),
        "inputs": {
            "capture_path": str(capture_path),
            "capture_sha256": _sha256(capture_path),
            "policy_path": str(policy_path),
            "policy_sha256": _sha256(policy_path),
            "observation_count": int(joint_positions.shape[0]),
            "native_action_shape": list(actions.shape),
        },
        "m2_contract": {
            "base_frame": kinematics.base_frame,
            "tcp_point": kinematics.end_effector_frame,
            "order": [
                "linear_x",
                "linear_y",
                "linear_z",
                "angular_x",
                "angular_y",
                "angular_z",
            ],
            "translation_scale_m_per_policy_step": (
                normalization.translation_scale_m
            ),
            "rotation_scale_rad_per_policy_step": (
                normalization.rotation_scale_rad
            ),
            "clipping": "none for Cartesian normalization",
            "gripper_boundary": (
                "native finite policy closure is explicitly clipped to "
                "[0,1] before the unchanged strict M2 adapter; raw and "
                "canonical values are recorded"
            ),
            "state_semantics": (
                "each 15-action chunk is projected at the fresh q captured "
                "with its observation; this diagnostic does not roll q forward"
            ),
            "kinematics": {
                "backend": "Pinocchio",
                "version": kinematics.backend_version,
                "generated_urdf_sha256": kinematics.urdf_sha256,
                "source_xacro": str(xacro_path),
            },
        },
        "gripper_boundary": {
            "raw": _numeric_summary(actions[:, :, 7]),
            "canonical": _numeric_summary(
                np.clip(actions[:, :, 7], 0.0, 1.0)
            ),
            "clipped_count": sum(
                item["clipped"] for item in gripper_boundary_records
            ),
            "records": gripper_boundary_records,
        },
        "policy_normalized_distribution": _motion_summary(normalized),
        "samples": samples,
        "actuation": {
            "published_topics": [],
            "called_services": [],
            "called_actions": [],
            "robot_commands_issued": 0,
        },
    }
    with output_path.open("x", encoding="utf-8") as file:
        json.dump(record, file, indent=2, sort_keys=True, allow_nan=False)
        file.write("\n")
    print(f"Wrote M3 shadow projection evidence to {output_path}")


def _projection_record(
    action_index: int,
    raw_action: np.ndarray,
    projected: Any,
) -> dict[str, Any]:
    condition = projected.jacobian_diagnostic.condition_number
    return {
        "action_index": action_index,
        "native_action": np.asarray(raw_action).tolist(),
        "canonical_projection_action": projected.joint_action.native.tolist()
        + [projected.gripper_closure],
        "native_joint_clipping_scale": (
            projected.joint_action.clipping_scale
        ),
        "delta_q_rad": projected.joint_action.delta_q_rad.tolist(),
        "cartesian_delta_linearized": (
            projected.delta_x_linearized.tolist()
        ),
        "normalized_policy_motion": projected.normalized_motion.tolist(),
        "gripper_closure": projected.gripper_closure,
        "gripper_clipped": (
            float(raw_action[7]) != projected.gripper_closure
        ),
        "jacobian_condition_number": (
            condition if np.isfinite(condition) else None
        ),
        "null_fraction": projected.null_space_diagnostic.null_fraction,
    }


def _canonical_projection_action(
    action: np.ndarray,
) -> tuple[np.ndarray, bool]:
    """Clip only canonical closure while preserving native policy motion."""

    value = np.asarray(action)
    if value.shape != (8,):
        raise ValueError(
            f"native policy action must have shape (8,), received {value.shape}."
        )
    if not np.issubdtype(value.dtype, np.floating):
        raise TypeError("native policy action must have a floating dtype.")
    if not np.all(np.isfinite(value)):
        raise ValueError("native policy action must be finite.")
    result = np.array(value, dtype=np.float64, copy=True)
    raw_closure = float(result[7])
    result[7] = np.clip(raw_closure, 0.0, 1.0)
    return result, float(result[7]) != raw_closure


def _jacobian_record(diagnostic: Any) -> dict[str, Any]:
    condition = diagnostic.condition_number
    return {
        "singular_values": diagnostic.singular_values.tolist(),
        "rank": diagnostic.rank,
        "condition_number": condition if np.isfinite(condition) else None,
        "near_singular": diagnostic.near_singular,
    }


def _motion_summary(values: np.ndarray) -> dict[str, Any]:
    motion = np.asarray(values, dtype=np.float64).reshape(-1, 6)
    translation_norm = np.linalg.norm(motion[:, :3], axis=1)
    rotation_norm = np.linalg.norm(motion[:, 3:], axis=1)
    overall_norm = np.linalg.norm(motion, axis=1)
    return {
        "sample_count": int(motion.shape[0]),
        "component_minimum": np.min(motion, axis=0).tolist(),
        "component_maximum": np.max(motion, axis=0).tolist(),
        "translation_norm": _numeric_summary(translation_norm),
        "rotation_norm": _numeric_summary(rotation_norm),
        "overall_motion_norm": _numeric_summary(overall_norm),
        "component_fraction_above_unit_magnitude": float(
            np.mean(np.abs(motion) > 1.0)
        ),
        "action_fraction_with_any_component_above_unit_magnitude": float(
            np.mean(np.any(np.abs(motion) > 1.0, axis=1))
        ),
        "clipping": "none",
    }


def _numeric_summary(values: np.ndarray) -> dict[str, Any]:
    return {
        "minimum": float(np.min(values)),
        "maximum": float(np.max(values)),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument(
        "--franka-description-dir",
        type=Path,
        default=Path(
            "/home/hvl-robotics2404/franka_ros2_ws/src/"
            "franka_description"
        ),
    )
    return parser.parse_args()


if __name__ == "__main__":
    main(_parse_args())
