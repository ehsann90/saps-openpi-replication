#!/usr/bin/env python3
"""Run genuine extracted DROID observations through pi05_droid offline."""

from __future__ import annotations

import dataclasses
import hashlib
import json
import logging
import os
from pathlib import Path
import platform
import sys
import time
from typing import Any

import numpy as np
import tyro

from saps.evaluation.experiment_session import write_json_atomic
from saps.policies.openpi_droid import DroidRunProvenance
from saps.policies.openpi_droid import json_compatible
from saps.policies.openpi_droid import OpenPiDroidPolicy
from saps.policies.openpi_droid import prepare_droid_observation
from saps.policies.openpi_droid import summarize_action_chunk
from saps.policies.openpi_droid import validate_droid_action_response


@dataclasses.dataclass
class Args:
    sample_bundle_path: str = "data/droid_m1/droid_m1_samples.npz"
    sample_metadata_path: str = "data/droid_m1/droid_m1_samples.json"
    num_samples: int = 3
    repeat_count: int = 2

    host: str = "0.0.0.0"
    port: int = 8000
    policy_config: str = "pi05_droid"
    checkpoint: str = "gs://openpi-assets/checkpoints/pi05_droid"
    policy_episode_seed: int = 20260827

    repository_commit: str = "unknown"
    repository_dirty: bool = False
    openpi_commit: str = "unknown"
    output_dir: str = "outputs/physical_pi05_droid_m1/manual_run"


def main(args: Args) -> None:
    _validate_args(args)
    output_dir = Path(args.output_dir)
    if output_dir.exists():
        raise FileExistsError(
            f"Output directory already exists: {output_dir}. Use a new "
            "identity so an earlier diagnostic is never overwritten."
        )

    samples, bundle_metadata = _load_sample_bundle(
        Path(args.sample_bundle_path),
        Path(args.sample_metadata_path),
    )
    selected_samples = samples[: args.num_samples]

    policy = OpenPiDroidPolicy(host=args.host, port=args.port)
    policy.validate_policy_identity(
        config_name=args.policy_config,
        checkpoint=args.checkpoint,
    )

    sample_results = []
    all_round_trip_seconds = []
    all_server_infer_ms = []
    all_policy_infer_ms = []

    for sample_position, sample in enumerate(selected_samples):
        preprocess_start = time.perf_counter()
        policy_input = prepare_droid_observation(
            exterior_image=sample["exterior_image"],
            wrist_image=sample["wrist_image"],
            joint_position=sample["joint_position"],
            gripper_position=sample["gripper_position"],
            prompt=sample["language_instruction"],
        )
        preprocessing_seconds = time.perf_counter() - preprocess_start

        responses = []
        replan_index = sample_position
        for _ in range(args.repeat_count):
            response = policy.infer(
                policy_input,
                policy_episode_seed=args.policy_episode_seed,
                replan_index=replan_index,
            )
            responses.append(response)
            all_round_trip_seconds.append(
                response.client_round_trip_seconds
            )
            _append_timing(
                all_server_infer_ms,
                response.server_timing,
            )
            _append_timing(
                all_policy_infer_ms,
                response.policy_timing,
            )

        actions = responses[0].actions
        repeated_actions = [response.actions for response in responses]
        exact_repeat = all(
            np.array_equal(actions, repeated)
            for repeated in repeated_actions[1:]
        )
        maximum_repeat_difference = max(
            (
                float(np.max(np.abs(actions - repeated)))
                for repeated in repeated_actions[1:]
            ),
            default=0.0,
        )

        recorded_actions = validate_droid_action_response(
            {"actions": sample["recorded_actions"]}
        )
        result = {
            "sample_identity": sample["identity"],
            "source_dataset": bundle_metadata["dataset"],
            "episode_uuid": sample["episode_uuid"],
            "step_index": sample["step_index"],
            "language_instruction": sample["language_instruction"],
            "observation": {
                "exact_policy_input_keys": list(policy_input),
                "exterior_image": _array_contract(
                    policy_input[
                        "observation/exterior_image_1_left"
                    ]
                ),
                "wrist_image": _array_contract(
                    policy_input["observation/wrist_image_left"]
                ),
                "joint_position": policy_input[
                    "observation/joint_position"
                ].tolist(),
                "joint_position_contract": _array_contract(
                    policy_input["observation/joint_position"]
                ),
                "gripper_position": policy_input[
                    "observation/gripper_position"
                ].tolist(),
                "gripper_position_contract": _array_contract(
                    policy_input["observation/gripper_position"]
                ),
                "preprocessing_seconds": preprocessing_seconds,
            },
            "policy_response": {
                "response_keys": list(responses[0].response_keys),
                "actions": actions.tolist(),
                "action_shape": list(actions.shape),
                "action_horizon": int(actions.shape[0]),
                "action_dimension": int(actions.shape[1]),
                "dtype": str(actions.dtype),
                "all_finite": bool(np.all(np.isfinite(actions))),
                "first_action": actions[0].tolist(),
                "per_dimension_summary": summarize_action_chunk(actions),
                "gripper_values": actions[:, -1].tolist(),
                "calls": [
                    {
                        "client_round_trip_seconds": (
                            response.client_round_trip_seconds
                        ),
                        "policy_timing": json_compatible(
                            response.policy_timing
                        ),
                        "server_timing": json_compatible(
                            response.server_timing
                        ),
                        "sampling_metadata": json_compatible(
                            response.sampling_metadata
                        ),
                    }
                    for response in responses
                ],
            },
            "repeatability": {
                "repeat_count": args.repeat_count,
                "same_seed": args.policy_episode_seed,
                "same_replan_index": replan_index,
                "exact_array_equal": exact_repeat,
                "maximum_absolute_difference": (
                    maximum_repeat_difference
                ),
            },
            "recorded_action_reference": {
                "purpose": (
                    "representation and scale inspection only; no accuracy "
                    "metric is computed"
                ),
                "actions": recorded_actions.tolist(),
                "action_shape": list(recorded_actions.shape),
                "dtype": str(recorded_actions.dtype),
                "first_action": recorded_actions[0].tolist(),
                "per_dimension_summary": summarize_action_chunk(
                    recorded_actions
                ),
                "gripper_values": recorded_actions[:, -1].tolist(),
            },
        }
        sample_results.append(result)

        logging.info(
            "%s -> actions=%s dtype=%s round_trip=%.3fs exact_repeat=%s",
            sample["identity"],
            actions.shape,
            actions.dtype,
            responses[0].client_round_trip_seconds,
            exact_repeat,
        )

    action_shapes = sorted(
        {
            tuple(result["policy_response"]["action_shape"])
            for result in sample_results
        }
    )
    action_dtypes = sorted(
        {
            result["policy_response"]["dtype"]
            for result in sample_results
        }
    )
    provenance = DroidRunProvenance(
        repository_commit=args.repository_commit,
        repository_dirty=args.repository_dirty,
        openpi_commit=args.openpi_commit,
        checkpoint=args.checkpoint,
        policy_config=args.policy_config,
        dataset_source=(
            f"{bundle_metadata['dataset']['name']} "
            f"{bundle_metadata['dataset']['version']}"
        ),
        sample_identities=[
            sample["identity"] for sample in selected_samples
        ],
        runtime={
            "python": sys.version,
            "platform": platform.platform(),
            "is_docker": (
                os.environ.get("IS_DOCKER") == "true"
                or Path("/.dockerenv").exists()
            ),
        },
        server_metadata=policy.server_metadata,
    )

    run_record = {
        "schema_version": 1,
        "diagnostic_scope": (
            "offline only; no robot, ROS, SpaceMouse, cameras, or commands"
        ),
        "arguments": dataclasses.asdict(args),
        "provenance": provenance.as_dict(),
        "sample_bundle": bundle_metadata,
        "empirical_contract": {
            "observed_action_shapes": [
                list(shape) for shape in action_shapes
            ],
            "observed_action_dtypes": action_dtypes,
            "observed_action_horizons": sorted(
                {shape[0] for shape in action_shapes}
            ),
            "observed_action_dimensions": sorted(
                {shape[1] for shape in action_shapes}
            ),
            "all_actions_finite": all(
                result["policy_response"]["all_finite"]
                for result in sample_results
            ),
            "all_seeded_repeats_exact": all(
                result["repeatability"]["exact_array_equal"]
                for result in sample_results
            ),
        },
        "latency_diagnostics": {
            "client_round_trip_seconds": _numeric_summary(
                all_round_trip_seconds
            ),
            "server_infer_milliseconds": _numeric_summary(
                all_server_infer_ms
            ),
            "policy_infer_milliseconds": _numeric_summary(
                all_policy_infer_ms
            ),
            "call_count": len(all_round_trip_seconds),
            "interpretation": (
                "small offline diagnostic only; not a benchmark"
            ),
        },
        "samples": sample_results,
    }

    output_dir.mkdir(parents=True)
    write_json_atomic(output_dir / "run.json", run_record)
    logging.info("Wrote offline DROID diagnostic to %s", output_dir)


def _validate_args(args: Args) -> None:
    if args.num_samples <= 0:
        raise ValueError("num_samples must be positive.")
    if args.repeat_count < 2:
        raise ValueError(
            "repeat_count must be at least two for the M1 repeatability check."
        )
    if not 0 <= args.policy_episode_seed <= 0x7FFFFFFF:
        raise ValueError(
            "policy_episode_seed must be within [0, 2^31 - 1]."
        )


def _load_sample_bundle(
    bundle_path: Path,
    metadata_path: Path,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    metadata = json.loads(metadata_path.read_text())
    actual_bundle_sha256 = _sha256(bundle_path)
    if metadata.get("bundle_sha256") != actual_bundle_sha256:
        raise ValueError(
            "DROID bundle SHA-256 does not match its metadata."
        )

    with np.load(bundle_path, allow_pickle=False) as bundle:
        required = {
            "step_indices",
            "exterior_images",
            "wrist_images",
            "joint_positions",
            "gripper_positions",
            "recorded_actions",
        }
        missing = required.difference(bundle.files)
        if missing:
            raise ValueError(
                f"DROID sample bundle is missing arrays: {sorted(missing)}."
            )
        arrays = {name: np.array(bundle[name]) for name in required}

    sample_records = metadata.get("samples")
    if not isinstance(sample_records, list):
        raise ValueError("DROID bundle metadata has no sample records.")
    sample_count = len(sample_records)
    if any(len(value) != sample_count for value in arrays.values()):
        raise ValueError(
            "DROID bundle arrays and metadata have different sample counts."
        )

    samples = []
    for index, record in enumerate(sample_records):
        if int(arrays["step_indices"][index]) != int(
            record["step_index"]
        ):
            raise ValueError(
                "DROID bundle step indices differ from sample metadata."
            )
        samples.append(
            {
                **record,
                "exterior_image": arrays["exterior_images"][index],
                "wrist_image": arrays["wrist_images"][index],
                "joint_position": arrays["joint_positions"][index],
                "gripper_position": arrays["gripper_positions"][index],
                "recorded_actions": arrays["recorded_actions"][index],
            }
        )
    return samples, metadata


def _append_timing(
    destination: list[float],
    timing: dict[str, Any] | None,
) -> None:
    if timing is not None and timing.get("infer_ms") is not None:
        destination.append(float(timing["infer_ms"]))


def _array_contract(array: np.ndarray) -> dict[str, Any]:
    return {
        "shape": list(array.shape),
        "dtype": str(array.dtype),
    }


def _numeric_summary(values: list[float]) -> dict[str, Any] | None:
    if not values:
        return None
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": len(values),
        "minimum": float(np.min(array)),
        "maximum": float(np.max(array)),
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )
    main(tyro.cli(Args))
