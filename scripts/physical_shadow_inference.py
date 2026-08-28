#!/usr/bin/env python3
"""Run captured live M3 observations through pi05_droid in shadow mode."""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
from pathlib import Path
import platform
import sys
import time
from typing import Any

import numpy as np

from saps.policies.openpi_droid import json_compatible
from saps.policies.openpi_droid import OpenPiDroidPolicy
from saps.policies.openpi_droid import prepare_droid_observation
from saps.policies.openpi_droid import summarize_action_chunk


POLICY_CONFIG = "pi05_droid"
POLICY_CHECKPOINT = "gs://openpi-assets/checkpoints/pi05_droid"
EXPECTED_ACTION_SHAPE = (15, 8)


def main(args: argparse.Namespace) -> None:
    """Infer one action chunk per capture and write policy-only evidence."""

    run_dir = args.run_dir.resolve()
    run_path = run_dir / "run.json"
    bundle_path = run_dir / "observation_bundle.npz"
    action_path = run_dir / "policy_actions.npz"
    output_path = run_dir / "shadow_policy.json"
    for path in (run_path, bundle_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    for path in (action_path, output_path):
        if path.exists():
            raise FileExistsError(
                f"Refusing to overwrite existing M3 artifact: {path}."
            )

    capture = json.loads(run_path.read_text(encoding="utf-8"))
    expected_hash = capture.get("bundle", {}).get("sha256")
    actual_hash = _sha256(bundle_path)
    if expected_hash != actual_hash:
        raise ValueError("M3 observation bundle hash does not match run.json.")
    arrays = _load_bundle(bundle_path)
    observation_count = arrays["joint_positions"].shape[0]
    if observation_count != len(capture.get("observations", [])):
        raise ValueError("M3 bundle and run.json observation counts differ.")

    policy = OpenPiDroidPolicy(host=args.host, port=args.port)
    policy.validate_policy_identity(
        config_name=POLICY_CONFIG,
        checkpoint=POLICY_CHECKPOINT,
    )
    samples = []
    action_chunks = []
    latencies = []
    for index in range(observation_count):
        policy_input = prepare_droid_observation(
            exterior_image=arrays["exterior_images"][index],
            wrist_image=arrays["wrist_images"][index],
            joint_position=arrays["joint_positions"][index],
            gripper_position=arrays["gripper_positions"][index],
            prompt=str(arrays["prompts"][index]),
        )
        source_stamps = arrays["source_ros_seconds"][index]
        newest_source_ros_seconds = float(np.max(source_stamps))
        inference_started_unix_seconds = time.time()
        response = policy.infer(
            policy_input,
            policy_episode_seed=args.policy_episode_seed,
            replan_index=index,
        )
        inference_completed_unix_seconds = time.time()
        if response.actions.shape != EXPECTED_ACTION_SHAPE:
            raise ValueError(
                "Pinned physical M3 policy must return shape (15, 8); "
                f"received {response.actions.shape}."
            )
        action_chunks.append(response.actions)
        latencies.append(response.client_round_trip_seconds)
        samples.append(
            {
                "observation_index": index,
                "observation_source_ros_seconds": (
                    source_stamps.tolist()
                ),
                "newest_observation_source_ros_seconds": (
                    newest_source_ros_seconds
                ),
                "inference_started_unix_seconds": (
                    inference_started_unix_seconds
                ),
                "inference_completed_unix_seconds": (
                    inference_completed_unix_seconds
                ),
                "observation_age_at_inference_start_seconds": (
                    inference_started_unix_seconds
                    - newest_source_ros_seconds
                ),
                "observation_age_at_inference_completion_seconds": (
                    inference_completed_unix_seconds
                    - newest_source_ros_seconds
                ),
                "response_keys": list(response.response_keys),
                "action_shape": list(response.actions.shape),
                "action_dtype": str(response.actions.dtype),
                "actions": response.actions.tolist(),
                "first_action": response.actions[0].tolist(),
                "selected_actions": {
                    str(selected): response.actions[selected].tolist()
                    for selected in (0, 7, 14)
                },
                "per_dimension_summary": summarize_action_chunk(
                    response.actions
                ),
                "client_round_trip_seconds": (
                    response.client_round_trip_seconds
                ),
                "policy_timing": json_compatible(response.policy_timing),
                "server_timing": json_compatible(response.server_timing),
                "sampling_metadata": json_compatible(
                    response.sampling_metadata
                ),
            }
        )
        print(
            f"shadow inference {index + 1}/{observation_count}: "
            f"shape={response.actions.shape} "
            f"latency={response.client_round_trip_seconds:.3f}s",
            flush=True,
        )

    actions = np.stack(action_chunks)
    np.savez_compressed(action_path, actions=actions)
    record = {
        "schema_version": 1,
        "milestone": "physical_pi05_droid_m3",
        "diagnostic_scope": (
            "pi05_droid shadow inference over captured physical inputs; "
            "no ROS imports, robot-facing transport, Servo publication, "
            "or gripper command"
        ),
        "created_utc": _utc_now(),
        "capture": {
            "run_path": str(run_path),
            "run_sha256": _sha256(run_path),
            "bundle_path": str(bundle_path),
            "bundle_sha256": actual_hash,
        },
        "policy": {
            "config": POLICY_CONFIG,
            "checkpoint": POLICY_CHECKPOINT,
            "server_metadata": json_compatible(policy.server_metadata),
            "policy_episode_seed": args.policy_episode_seed,
            "expected_action_shape": list(EXPECTED_ACTION_SHAPE),
        },
        "runtime": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": np.__version__,
        },
        "latency": _numeric_summary(np.asarray(latencies)),
        "aggregate_native_actions": _motion_summary(actions[:, :, :7]),
        "aggregate_gripper_closure": _numeric_summary(actions[:, :, 7]),
        "samples": samples,
        "action_bundle": {
            "path": str(action_path),
            "sha256": _sha256(action_path),
            "shape": list(actions.shape),
            "dtype": str(actions.dtype),
        },
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
    print(f"Wrote M3 shadow policy evidence to {output_path}")


def _load_bundle(path: Path) -> dict[str, np.ndarray]:
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
            raise ValueError(f"M3 bundle is missing arrays: {sorted(missing)}")
        arrays = {name: np.array(bundle[name]) for name in required}
    count = arrays["joint_positions"].shape[0]
    if any(value.shape[0] != count for value in arrays.values()):
        raise ValueError("M3 bundle arrays have inconsistent sample counts.")
    return arrays


def _motion_summary(values: np.ndarray) -> dict[str, Any]:
    flattened = values.reshape(-1, values.shape[-1])
    return {
        "component": {
            "minimum": np.min(flattened, axis=0).tolist(),
            "maximum": np.max(flattened, axis=0).tolist(),
        },
        "absolute_maximum": float(np.max(np.abs(flattened))),
        "component_fraction_above_unit": float(
            np.mean(np.abs(flattened) > 1.0)
        ),
    }


def _numeric_summary(values: np.ndarray) -> dict[str, Any]:
    flat = np.asarray(values, dtype=np.float64).reshape(-1)
    return {
        "count": int(flat.size),
        "minimum": float(np.min(flat)),
        "maximum": float(np.max(flat)),
        "mean": float(np.mean(flat)),
        "median": float(np.median(flat)),
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
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--policy-episode-seed", type=int, default=20260828)
    return parser.parse_args()


if __name__ == "__main__":
    main(_parse_args())
