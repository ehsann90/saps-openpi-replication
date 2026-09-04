#!/usr/bin/env python3
"""Estimate FR3 TCP characteristic length from saved M3 shadow runs."""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from saps.physical.fr3_forward_kinematics import (
    fr3_tcp_finite_displacement,
)
from saps.physical.fr3_kinematics import Fr3PinocchioKinematics
from saps.policies.openpi_droid import map_droid_reference_joint_action
from saps.policies.openpi_droid import prepare_droid_observation
from saps.policies.openpi_droid import validate_droid_action_response


REFERENCE_EXECUTION_HORIZON = 8
FULL_CHUNK_HORIZON = 15
DEFAULT_ELL_0_M_PER_RAD = 0.30
DEFAULT_ROTATION_EXCLUSION_THRESHOLD_RAD = 1e-9
M3_MILESTONE = "physical_pi05_droid_m3"


@dataclasses.dataclass(frozen=True)
class M3RunData:
    """Validated arrays and provenance needed for one M3 analysis run."""

    run_id: str
    run_dir: Path
    joint_positions: np.ndarray
    actions: np.ndarray
    prompts: tuple[str, ...]
    validation: dict[str, str]


def load_m3_run(run_dir: Path) -> M3RunData:
    """Load one saved M3 run and enforce its observation/action contracts."""

    source = run_dir.resolve()
    if not source.is_dir():
        raise NotADirectoryError(source)
    observation_path = source / "observation_bundle.npz"
    action_path = source / "policy_actions.npz"
    for path in (observation_path, action_path):
        if not path.is_file():
            raise FileNotFoundError(path)

    arrays = _load_observation_arrays(observation_path)
    joint_positions = arrays["joint_positions"]
    actions = _load_actions(
        action_path,
        observation_count=joint_positions.shape[0],
    )
    validation = _validate_sidecars(
        source,
        arrays=arrays,
        actions=actions,
        observation_path=observation_path,
        action_path=action_path,
    )
    prompts = tuple(str(prompt) for prompt in arrays["prompts"])
    return M3RunData(
        run_id=source.name,
        run_dir=source,
        joint_positions=np.array(joint_positions, dtype=np.float64, copy=True),
        actions=actions,
        prompts=prompts,
        validation=validation,
    )


def analyze_run(
    run: M3RunData,
    *,
    position_lower_rad: np.ndarray,
    position_upper_rad: np.ndarray,
    ell_0_m_per_rad: float,
    rotation_exclusion_threshold_rad: float,
) -> dict[str, Any]:
    """Analyze sequential first-eight model rollouts for one saved run."""

    _validate_positive_finite(ell_0_m_per_rad, "ell_0_m_per_rad")
    _validate_positive_finite(
        rotation_exclusion_threshold_rad,
        "rotation_exclusion_threshold_rad",
    )
    lower, upper = _validate_joint_limits(
        position_lower_rad,
        position_upper_rad,
    )
    samples = []
    for observation_index in range(run.joint_positions.shape[0]):
        q = run.joint_positions[observation_index].copy()
        for action_index in range(REFERENCE_EXECUTION_HORIZON):
            mapped = map_droid_reference_joint_action(
                run.actions[observation_index, action_index]
            )
            delta_q = mapped.delta_q_rad
            displacement = fr3_tcp_finite_displacement(q, delta_q)
            q_next = q + delta_q
            violated = np.flatnonzero(
                (q_next < lower) | (q_next > upper)
            )
            translation_norm = float(np.linalg.norm(displacement[:3]))
            rotation_norm = float(np.linalg.norm(displacement[3:]))
            valid_ratio = (
                rotation_norm > rotation_exclusion_threshold_rad
            )
            ratio = (
                translation_norm / rotation_norm
                if valid_ratio
                else None
            )
            samples.append(
                {
                    "run_id": run.run_id,
                    "observation_index": observation_index,
                    "action_index": action_index,
                    "translation_norm_m": translation_norm,
                    "rotation_norm_rad": rotation_norm,
                    "ratio_m_per_rad": ratio,
                    "ell_0_block_balance": (
                        ratio / ell_0_m_per_rad
                        if ratio is not None
                        else None
                    ),
                    "arm_action_clipped": bool(
                        np.any(
                            mapped.policy_joint_coordinates
                            != mapped.reference_joint_coordinates
                        )
                    ),
                    "next_state_joint_limit_violation": bool(violated.size),
                    "violated_joint_indices": violated.tolist(),
                }
            )
            q = q_next

    return {
        "run_id": run.run_id,
        "run_dir": str(run.run_dir),
        "prompts": list(dict.fromkeys(run.prompts)),
        "prompt_by_observation": list(run.prompts),
        "observation_count": int(run.joint_positions.shape[0]),
        "first_eight_action_count": len(samples),
        "physical_configuration_change": (
            "not encoded in M3 artifacts; establish from the lab procedure"
        ),
        "input_validation": run.validation,
        "statistics": summarize_motion_samples(
            samples,
            ell_0_m_per_rad=ell_0_m_per_rad,
            rotation_exclusion_threshold_rad=(
                rotation_exclusion_threshold_rad
            ),
        ),
        "samples": samples,
    }


def build_report(
    run_dirs: list[Path],
    *,
    position_lower_rad: np.ndarray,
    position_upper_rad: np.ndarray,
    ell_0_m_per_rad: float = DEFAULT_ELL_0_M_PER_RAD,
    rotation_exclusion_threshold_rad: float = (
        DEFAULT_ROTATION_EXCLUSION_THRESHOLD_RAD
    ),
    joint_limit_source: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Load multiple M3 runs and build per-run and pooled summaries."""

    _validate_positive_finite(ell_0_m_per_rad, "ell_0_m_per_rad")
    _validate_positive_finite(
        rotation_exclusion_threshold_rad,
        "rotation_exclusion_threshold_rad",
    )
    if not run_dirs:
        raise ValueError("At least one M3 run directory is required.")
    resolved = [path.resolve() for path in run_dirs]
    if len(set(resolved)) != len(resolved):
        raise ValueError("M3 run directories must be unique.")

    runs = [load_m3_run(path) for path in resolved]
    run_ids = [run.run_id for run in runs]
    if len(set(run_ids)) != len(run_ids):
        raise ValueError(
            "M3 run directory names must be unique so sample identities "
            "remain auditable."
        )
    run_reports = [
        analyze_run(
            run,
            position_lower_rad=position_lower_rad,
            position_upper_rad=position_upper_rad,
            ell_0_m_per_rad=ell_0_m_per_rad,
            rotation_exclusion_threshold_rad=(
                rotation_exclusion_threshold_rad
            ),
        )
        for run in runs
    ]
    pooled_samples = [
        sample
        for run_report in run_reports
        for sample in run_report["samples"]
    ]
    return {
        "schema_version": 1,
        "diagnostic_scope": (
            "offline non-actuating first-eight pi0.5-DROID model rollouts; "
            "no ROS graph, publisher, service, action client, Servo command, "
            "robot command, or gripper command"
        ),
        "motion_contract": {
            "native_arm_action": (
                "u_ref = clip(u_pi, -1, 1) component-wise"
            ),
            "joint_increment": "delta_q = 0.2 * u_ref",
            "rollout": "q[k+1] = q[k] + delta_q[k] for k=0,...,7",
            "tcp_motion": (
                "manual NumPy finite FK: p1-p0 and Log(R1 R0^T)"
            ),
            "jacobian_used_for_motion": False,
        },
        "candidate_ell_0_m_per_rad": float(ell_0_m_per_rad),
        "rotation_exclusion_threshold_rad": float(
            rotation_exclusion_threshold_rad
        ),
        "joint_limits": {
            "lower_rad": np.asarray(position_lower_rad).tolist(),
            "upper_rad": np.asarray(position_upper_rad).tolist(),
            "source": joint_limit_source,
        },
        "interpretation": (
            "evidence for scientific review only; no pass/fail threshold or "
            "accepted final characteristic length is assigned"
        ),
        "runs": run_reports,
        "pooled": {
            "run_count": len(run_reports),
            "observation_count": sum(
                report["observation_count"] for report in run_reports
            ),
            "first_eight_action_count": len(pooled_samples),
            "statistics": summarize_motion_samples(
                pooled_samples,
                ell_0_m_per_rad=ell_0_m_per_rad,
                rotation_exclusion_threshold_rad=(
                    rotation_exclusion_threshold_rad
                ),
            ),
        },
    }


def summarize_motion_samples(
    samples: list[dict[str, Any]],
    *,
    ell_0_m_per_rad: float,
    rotation_exclusion_threshold_rad: float,
) -> dict[str, Any]:
    """Compute norm, characteristic-length, ratio, and limit summaries."""

    if not samples:
        raise ValueError("Cannot summarize an empty motion sample sequence.")
    translation = np.asarray(
        [sample["translation_norm_m"] for sample in samples],
        dtype=np.float64,
    )
    rotation = np.asarray(
        [sample["rotation_norm_rad"] for sample in samples],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(translation)) or np.any(translation < 0.0):
        raise ValueError("Translation norms must be finite and non-negative.")
    if not np.all(np.isfinite(rotation)) or np.any(rotation < 0.0):
        raise ValueError("Rotation norms must be finite and non-negative.")

    valid = rotation > rotation_exclusion_threshold_rad
    ratio = translation[valid] / rotation[valid]
    block_balance = ratio / ell_0_m_per_rad
    translation_energy = float(np.sum(np.square(translation)))
    rotation_energy = float(np.sum(np.square(rotation)))
    weighted_rotation_energy = ell_0_m_per_rad**2 * rotation_energy
    violations = [
        {
            "run_id": sample["run_id"],
            "observation_index": sample["observation_index"],
            "action_index": sample["action_index"],
            "violated_joint_indices": sample["violated_joint_indices"],
        }
        for sample in samples
        if sample["next_state_joint_limit_violation"]
    ]
    return {
        "translation_norm_m": norm_statistics(translation),
        "rotation_norm_rad": norm_statistics(rotation),
        "characteristic_length_m_per_rad": {
            "ell_median": _ratio_or_none(
                float(np.median(translation)),
                float(np.median(rotation)),
            ),
            "ell_p95": _ratio_or_none(
                float(np.percentile(translation, 95)),
                float(np.percentile(rotation, 95)),
            ),
            "ell_rms": (
                float(np.sqrt(translation_energy / rotation_energy))
                if rotation_energy > 0.0
                else None
            ),
            "ell_max": _ratio_or_none(
                float(np.max(translation)),
                float(np.max(rotation)),
            ),
            "ell_max_role": "diagnostic only; not a robust estimator",
        },
        "per_action_ratio_m_per_rad": {
            "rotation_exclusion_threshold_rad": float(
                rotation_exclusion_threshold_rad
            ),
            "valid_count": int(np.count_nonzero(valid)),
            "excluded_count": int(np.count_nonzero(~valid)),
            "distribution": ratio_statistics(ratio),
            "role": (
                "secondary evidence; near-pure translation can be unstable"
            ),
        },
        "candidate_ell_0_block_balance": {
            "ell_0_m_per_rad": float(ell_0_m_per_rad),
            "valid_count": int(np.count_nonzero(valid)),
            "excluded_count": int(np.count_nonzero(~valid)),
            "distribution": ratio_statistics(block_balance),
            "interpretation": (
                "B=1 equal blocks; B>1 translation larger; "
                "B<1 weighted rotation larger"
            ),
        },
        "squared_energy_balance_under_ell_0": {
            "translation_energy_m2": translation_energy,
            "weighted_rotation_energy_m2": weighted_rotation_energy,
            "translation_to_weighted_rotation_ratio": (
                translation_energy / weighted_rotation_energy
                if weighted_rotation_energy > 0.0
                else None
            ),
        },
        "joint_limit_violations": {
            "next_state_count": len(violations),
            "samples": violations,
        },
    }


def norm_statistics(values: np.ndarray) -> dict[str, float | int]:
    """Return required non-negative norm statistics."""

    data = np.asarray(values, dtype=np.float64).reshape(-1)
    if data.size == 0:
        raise ValueError("Cannot summarize an empty norm sequence.")
    return {
        "count": int(data.size),
        "median": float(np.median(data)),
        "p75": float(np.percentile(data, 75)),
        "p90": float(np.percentile(data, 90)),
        "p95": float(np.percentile(data, 95)),
        "maximum": float(np.max(data)),
    }


def ratio_statistics(
    values: np.ndarray,
) -> dict[str, float | int | None]:
    """Return the requested central and tail ratio quantiles."""

    data = np.asarray(values, dtype=np.float64).reshape(-1)
    if data.size == 0:
        return {
            "count": 0,
            "p05": None,
            "p25": None,
            "median": None,
            "p75": None,
            "p95": None,
        }
    return {
        "count": int(data.size),
        "p05": float(np.percentile(data, 5)),
        "p25": float(np.percentile(data, 25)),
        "median": float(np.median(data)),
        "p75": float(np.percentile(data, 75)),
        "p95": float(np.percentile(data, 95)),
    }


def _load_observation_arrays(path: Path) -> dict[str, np.ndarray]:
    required = {
        "exterior_images",
        "wrist_images",
        "joint_positions",
        "gripper_positions",
        "prompts",
        "source_ros_seconds",
    }
    with np.load(path, allow_pickle=False) as bundle:
        missing = required.difference(bundle.files)
        if missing:
            raise ValueError(
                f"M3 observation bundle is missing arrays: {sorted(missing)}"
            )
        arrays = {name: np.array(bundle[name]) for name in required}

    joints = arrays["joint_positions"]
    if not np.issubdtype(joints.dtype, np.floating):
        raise TypeError("M3 joint positions must have a floating dtype.")
    if joints.ndim != 2 or joints.shape[0] == 0 or joints.shape[1] != 7:
        raise ValueError(
            "M3 joint positions must have shape [positive_count, 7], "
            f"received {joints.shape}."
        )
    count = joints.shape[0]
    if any(value.ndim == 0 or value.shape[0] != count for value in arrays.values()):
        raise ValueError("M3 observation arrays have inconsistent counts.")
    prompts = arrays["prompts"]
    if prompts.shape != (count,) or not np.issubdtype(prompts.dtype, np.str_):
        raise ValueError(
            "M3 prompts must be a string array with shape "
            f"({count},), received {prompts.shape} and {prompts.dtype}."
        )
    source_times = arrays["source_ros_seconds"]
    if source_times.shape != (count, 4):
        raise ValueError(
            "M3 source_ros_seconds must have shape "
            f"({count}, 4), received {source_times.shape}."
        )
    if not np.issubdtype(source_times.dtype, np.floating):
        raise TypeError("M3 source timestamps must have a floating dtype.")
    if not np.all(np.isfinite(source_times)):
        raise ValueError("M3 source timestamps must be finite.")

    for index in range(count):
        prepare_droid_observation(
            exterior_image=arrays["exterior_images"][index],
            wrist_image=arrays["wrist_images"][index],
            joint_position=joints[index],
            gripper_position=arrays["gripper_positions"][index],
            prompt=str(prompts[index]),
        )
    return arrays


def _load_actions(path: Path, *, observation_count: int) -> np.ndarray:
    with np.load(path, allow_pickle=False) as bundle:
        if "actions" not in bundle.files:
            raise ValueError("M3 action bundle is missing the actions array.")
        actions = np.asarray(bundle["actions"])
    expected = (observation_count, FULL_CHUNK_HORIZON, 8)
    if actions.shape != expected:
        raise ValueError(
            f"M3 policy actions must have shape {expected}, "
            f"received {actions.shape}."
        )
    if not np.issubdtype(actions.dtype, np.floating):
        raise TypeError("M3 policy actions must have a floating dtype.")
    for chunk in actions:
        validate_droid_action_response({"actions": chunk})
    return np.array(actions, copy=True)


def _validate_sidecars(
    run_dir: Path,
    *,
    arrays: dict[str, np.ndarray],
    actions: np.ndarray,
    observation_path: Path,
    action_path: Path,
) -> dict[str, str]:
    validation = {
        "observation_bundle": "canonical arrays validated",
        "policy_actions": "canonical shape, dtype, and finiteness validated",
    }
    run_path = run_dir / "run.json"
    if run_path.is_file():
        capture = _read_mapping(run_path)
        _require_m3_identity(capture, run_path)
        bundle = _mapping_field(capture, "bundle", run_path)
        if bundle.get("sha256") != _sha256(observation_path):
            raise ValueError(
                f"M3 observation bundle hash does not match {run_path}."
            )
        if len(capture.get("observations", [])) != actions.shape[0]:
            raise ValueError(
                f"M3 observation count does not match {run_path}."
            )
        recorded_arrays = _mapping_field(bundle, "arrays", run_path)
        for name, value in arrays.items():
            recorded = _mapping_field(recorded_arrays, name, run_path)
            if recorded.get("shape") != list(value.shape):
                raise ValueError(
                    f"M3 array shape for {name} does not match {run_path}."
                )
            if recorded.get("dtype") != str(value.dtype):
                raise ValueError(
                    f"M3 array dtype for {name} does not match {run_path}."
                )
        validation["run.json"] = "identity, hash, arrays, and count validated"
    else:
        validation["run.json"] = "not present; NPZ contracts validated only"

    shadow_path = run_dir / "shadow_policy.json"
    if shadow_path.is_file():
        shadow = _read_mapping(shadow_path)
        _require_m3_identity(shadow, shadow_path)
        capture = _mapping_field(shadow, "capture", shadow_path)
        if capture.get("run_sha256") != _sha256(run_path):
            raise ValueError(
                f"M3 capture record hash does not match {shadow_path}."
            )
        if capture.get("bundle_sha256") != _sha256(observation_path):
            raise ValueError(
                f"M3 captured bundle hash does not match {shadow_path}."
            )
        bundle = _mapping_field(shadow, "action_bundle", shadow_path)
        if bundle.get("sha256") != _sha256(action_path):
            raise ValueError(
                f"M3 policy action hash does not match {shadow_path}."
            )
        if bundle.get("shape") != list(actions.shape):
            raise ValueError(
                f"M3 action shape does not match {shadow_path}."
            )
        if bundle.get("dtype") != str(actions.dtype):
            raise ValueError(
                f"M3 action dtype does not match {shadow_path}."
            )
        if len(shadow.get("samples", [])) != actions.shape[0]:
            raise ValueError(
                f"M3 policy sample count does not match {shadow_path}."
            )
        validation["shadow_policy.json"] = (
            "identity, capture/action hashes, shape, dtype, and count validated"
        )
    else:
        validation["shadow_policy.json"] = (
            "not present; NPZ contracts validated only"
        )
    return validation


def _read_mapping(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise TypeError(f"Expected a JSON object in {path}.")
    return value


def _mapping_field(
    value: dict[str, Any],
    field: str,
    source: Path,
) -> dict[str, Any]:
    result = value.get(field)
    if not isinstance(result, dict):
        raise ValueError(f"Expected object field {field!r} in {source}.")
    return result


def _require_m3_identity(value: dict[str, Any], source: Path) -> None:
    if value.get("schema_version") != 1:
        raise ValueError(f"Unsupported or missing schema_version in {source}.")
    if value.get("milestone") != M3_MILESTONE:
        raise ValueError(f"Unexpected M3 milestone identity in {source}.")


def _validate_joint_limits(
    lower: np.ndarray,
    upper: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    low = np.asarray(lower, dtype=np.float64)
    high = np.asarray(upper, dtype=np.float64)
    if low.shape != (7,) or high.shape != (7,):
        raise ValueError("FR3 position limits must each have shape (7,).")
    if not np.all(np.isfinite(low)) or not np.all(np.isfinite(high)):
        raise ValueError("FR3 position limits must be finite.")
    if np.any(low >= high):
        raise ValueError("Every FR3 lower position limit must be below upper.")
    return low, high


def _validate_positive_finite(value: float, name: str) -> None:
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive.")


def _ratio_or_none(numerator: float, denominator: float) -> float | None:
    return numerator / denominator if denominator > 0.0 else None


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _format_values(values: dict[str, Any]) -> str:
    return " ".join(
        f"{name}={value:.10g}" if isinstance(value, float) else f"{name}={value}"
        for name, value in values.items()
    )


def _print_statistics(label: str, statistics: dict[str, Any]) -> None:
    print(f"\n{label}")
    print(
        "  translation [m]:",
        _format_values(statistics["translation_norm_m"]),
    )
    print(
        "  rotation [rad]: ",
        _format_values(statistics["rotation_norm_rad"]),
    )
    print(
        "  anchors [m/rad]:",
        _format_values(
            {
                key: value
                for key, value in statistics[
                    "characteristic_length_m_per_rad"
                ].items()
                if key.startswith("ell_") and key != "ell_max_role"
            }
        ),
    )
    ratios = statistics["per_action_ratio_m_per_rad"]
    print(
        "  per-action r [m/rad]:",
        f"valid={ratios['valid_count']} excluded={ratios['excluded_count']}",
        _format_values(ratios["distribution"]),
    )
    balance = statistics["candidate_ell_0_block_balance"]
    print(
        f"  B under ell_0={balance['ell_0_m_per_rad']:.10g} m/rad:",
        _format_values(balance["distribution"]),
    )
    energy = statistics["squared_energy_balance_under_ell_0"]
    print(
        "  E_translation/E_weighted_rotation:",
        energy["translation_to_weighted_rotation_ratio"],
    )
    violations = statistics["joint_limit_violations"]
    print("  next-state joint-limit violations:", violations["next_state_count"])


def print_report(report: dict[str, Any]) -> None:
    """Print the concise human-readable form of an analysis report."""

    print("FR3 characteristic-length evidence (first 8 actions per chunk)")
    print(
        "rotation ratio exclusion threshold [rad]:",
        report["rotation_exclusion_threshold_rad"],
    )
    for run in report["runs"]:
        print(
            f"\nRun {run['run_id']}: observations={run['observation_count']} "
            f"actions={run['first_eight_action_count']}"
        )
        print("  prompts:", run["prompts"])
        print("  configuration change:", run["physical_configuration_change"])
        _print_statistics("  Per-run statistics", run["statistics"])
    pooled = report["pooled"]
    _print_statistics(
        "Pooled statistics: "
        f"runs={pooled['run_count']} observations={pooled['observation_count']} "
        f"actions={pooled['first_eight_action_count']}",
        pooled["statistics"],
    )
    print("\nNo pass/fail decision or final ell is assigned.")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "run_dirs",
        type=Path,
        nargs="+",
        help="M3 directories containing observation and policy NPZ bundles",
    )
    parser.add_argument(
        "--franka-description-dir",
        type=Path,
        default=Path.home() / "franka_ros2_ws/src/franka_description",
    )
    parser.add_argument(
        "--ell-0-m-per-rad",
        type=float,
        default=DEFAULT_ELL_0_M_PER_RAD,
    )
    parser.add_argument(
        "--rotation-exclusion-threshold-rad",
        type=float,
        default=DEFAULT_ROTATION_EXCLUSION_THRESHOLD_RAD,
    )
    parser.add_argument("--output-json", type=Path)
    return parser.parse_args()


def main(args: argparse.Namespace) -> None:
    """Load FR3 limits, analyze saved runs, and print/write the evidence."""

    xacro_path = (
        args.franka_description_dir.resolve()
        / "robots/fr3/fr3.urdf.xacro"
    )
    model = Fr3PinocchioKinematics.from_xacro(
        xacro_path,
        end_effector_frame="fr3_hand_tcp",
    )
    report = build_report(
        args.run_dirs,
        position_lower_rad=model.position_lower_rad,
        position_upper_rad=model.position_upper_rad,
        ell_0_m_per_rad=args.ell_0_m_per_rad,
        rotation_exclusion_threshold_rad=(
            args.rotation_exclusion_threshold_rad
        ),
        joint_limit_source={
            "xacro_path": str(xacro_path),
            "expanded_urdf_sha256": model.urdf_sha256,
            "pinocchio_version": model.backend_version,
        },
    )
    print_report(report)
    if args.output_json is not None:
        output_path = args.output_json.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("x", encoding="utf-8") as file:
            json.dump(report, file, indent=2, sort_keys=True, allow_nan=False)
            file.write("\n")
        print(f"Wrote JSON summary to {output_path}")


if __name__ == "__main__":
    main(_parse_args())
