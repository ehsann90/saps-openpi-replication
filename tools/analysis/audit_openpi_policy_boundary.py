#!/usr/bin/env python3
"""Replay archived DROID requests through one pinned OpenPI policy instance."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np
from PIL import Image
from openpi_client import image_tools, msgpack_numpy

from openpi.policies import policy_config
from openpi.training import config as training_config
from openpi import transforms

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from saps.policies.model_input_audit import infer_with_model_audit
from saps.policies.openpi_droid import OpenPiDroidPolicy, prepare_droid_observation
from scripts.serve_seeded_policy import SeededWebsocketPolicyServer


INDICES = (11, 12, 115, 119, 146, 148)
CONFIG_NAME = "pi05_droid"
CHECKPOINT_URI = "gs://openpi-assets/checkpoints/pi05_droid"


def digest(value: np.ndarray) -> str:
    return hashlib.sha256(np.asarray(value).tobytes(order="C")).hexdigest()


def file_digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compare(left: Any, right: Any) -> dict:
    a, b = np.asarray(left), np.asarray(right)
    if a.shape != b.shape:
        return {"shape_left": list(a.shape), "shape_right": list(b.shape),
                "shape_match": False}
    difference = np.abs(a.astype(np.float64) - b.astype(np.float64))
    return {
        "shape": list(a.shape), "dtype_left": str(a.dtype),
        "dtype_right": str(b.dtype),
        "max_abs_difference": float(difference.max(initial=0)),
        "mean_abs_difference": float(difference.mean()) if difference.size else 0.0,
        "exactly_equal": bool(np.array_equal(a, b)),
        "left_sha256": digest(a), "right_sha256": digest(b),
    }


def reconstruct_request(directory: Path, record: dict) -> tuple[dict, dict]:
    exterior = np.asarray(Image.open(directory / "exterior_raw.png").convert("RGB"))
    wrist = np.asarray(Image.open(directory / "wrist_raw.png").convert("RGB"))
    request = prepare_droid_observation(
        exterior_image=exterior,
        wrist_image=wrist,
        joint_position=np.asarray(record["measured_arm_q_rad"], dtype=np.float32),
        gripper_position=np.asarray(record["droid_gripper_scalar"], dtype=np.float32),
        prompt=record["prompt"],
    )
    observed_hashes = {
        key: digest(value) == record["array_evidence"][key]["sha256"]
        for key, value in request.items() if isinstance(value, np.ndarray)
    }
    if not all(observed_hashes.values()):
        raise ValueError(f"Archived canonical input hash mismatch: {directory}")
    resize_matches = {}
    for key in ("observation/exterior_image_1_left",
                "observation/wrist_image_left"):
        once = image_tools.resize_with_pad(request[key], 224, 224)
        twice = image_tools.resize_with_pad(once, 224, 224)
        resize_matches[key] = bool(np.array_equal(once, twice))
    return request, {"canonical_hash_matches": observed_hashes,
                     "official_client_resize_idempotent": resize_matches}


def infer_captured(policy: Any, request: dict, noise: np.ndarray,
                   *, with_local_audit: bool) -> tuple[dict, dict]:
    """Capture stages while calling the same Policy.infer implementation."""
    original_input = policy._input_transform
    original_sampler = policy._sample_actions
    original_output = policy._output_transform
    captured: dict[str, Any] = {}

    def input_hook(inputs: dict) -> dict:
        transformed = original_input(inputs)
        captured["transformed_images"] = {
            key: np.asarray(value).copy()
            for key, value in transformed["image"].items()
        }
        captured["transformed_state"] = np.asarray(
            transformed["state"]
        ).copy()
        captured["tokens"] = np.asarray(
            transformed["tokenized_prompt"]
        ).copy()
        return transformed

    def sampler_hook(rng: Any, model_observation: Any,
                     **kwargs: Any) -> Any:
        captured["model_state"] = np.asarray(
            model_observation.state
        ).copy()
        captured["model_images"] = {
            key: np.asarray(value).copy()
            for key, value in model_observation.images.items()
        }
        captured["model_image_masks"] = {
            key: np.asarray(value).copy()
            for key, value in model_observation.image_masks.items()
        }
        captured["model_tokens"] = np.asarray(
            model_observation.tokenized_prompt
        ).copy()
        captured["model_token_mask"] = np.asarray(
            model_observation.tokenized_prompt_mask
        ).copy()
        captured["supplied_noise"] = np.asarray(kwargs["noise"]).copy()
        captured["sampling_kwargs"] = sorted(kwargs)
        sampled = original_sampler(rng, model_observation, **kwargs)
        captured["raw_sample"] = np.asarray(sampled).copy()
        return sampled

    def output_hook(outputs: dict) -> dict:
        captured["unbatched_sample"] = np.asarray(outputs["actions"]).copy()
        # This extra call is diagnostic only; the original output composite
        # below still transforms the official result exactly once.
        for transform in original_output.transforms:
            if isinstance(transform, transforms.Unnormalize):
                staged = transform({key: np.asarray(value).copy()
                                    for key, value in outputs.items()})
                captured["denormalized_actions"] = np.asarray(
                    staged["actions"]
                ).copy()
                break
        return original_output(outputs)

    policy._input_transform = input_hook
    policy._sample_actions = sampler_hook
    policy._output_transform = output_hook
    try:
        if with_local_audit:
            result = infer_with_model_audit(policy, request, noise=noise)
        else:
            result = policy.infer(request, noise=noise)
    finally:
        policy._input_transform = original_input
        policy._sample_actions = original_sampler
        policy._output_transform = original_output
    return result, captured


class SerializedClient:
    """Exercise the real msgpack format and DROID adapter without a socket."""

    def __init__(self, result: dict, metadata: dict) -> None:
        self.result = result
        self.metadata = metadata
        self.request = None

    def get_server_metadata(self) -> dict:
        return self.metadata

    def infer(self, request: dict) -> dict:
        self.request = msgpack_numpy.unpackb(msgpack_numpy.packb(request))
        return msgpack_numpy.unpackb(msgpack_numpy.packb(self.result))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--checkpoint-dir", type=Path, required=True)
    args = parser.parse_args()
    run, output, checkpoint = (args.run.resolve(), args.output.resolve(),
                               args.checkpoint_dir.resolve())
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Refusing to overwrite {output}")
    if not (checkpoint / "assets/droid/norm_stats.json").is_file():
        raise FileNotFoundError("Cached DROID normalization asset is missing")
    provenance = json.loads((run / "provenance.json").read_text())
    if (provenance["policy_config"] != CONFIG_NAME
            or provenance["checkpoint"] != CHECKPOINT_URI):
        raise ValueError("Archived model identity differs from requested audit")

    config = training_config.get_config(CONFIG_NAME)
    policy = policy_config.create_trained_policy(config, checkpoint)
    server = SeededWebsocketPolicyServer(
        policy=policy, host="127.0.0.1", port=0,
        config_name=CONFIG_NAME, checkpoint_dir=CHECKPOINT_URI,
    )
    if server._metadata["saps_seeded_sampling"] != provenance[
            "server_metadata"]["saps_seeded_sampling"]:
        raise ValueError("Current server identity differs from archived handshake")

    rows = []
    for index in INDICES:
        directory = run / "replans" / f"{index:04d}"
        observation = json.loads((directory / "observation.json").read_text())
        response = json.loads((directory / "response.json").read_text())
        request, input_checks = reconstruct_request(directory, observation)
        noise = server._make_noise(
            policy_episode_seed=provenance["policy_episode_seed"],
            replan_index=index,
        )
        noise_sha = digest(noise)
        archive_noise_sha = response["sampling_metadata"]["noise_sha256"]
        if noise_sha != archive_noise_sha:
            raise ValueError(f"Sampling noise differs from archive at {index}")
        official, direct_stage = infer_captured(
            policy, request, noise, with_local_audit=False
        )
        audited, audited_stage = infer_captured(
            policy, request, noise, with_local_audit=True
        )
        archived_actions = np.load(directory / "actions.npy", allow_pickle=False)
        wrapped_result = dict(audited)
        wrapped_result["saps_sampling"] = response["sampling_metadata"]
        client = SerializedClient(wrapped_result, server._metadata)
        adapter = OpenPiDroidPolicy(client=client)
        adapter.validate_policy_identity(config_name=CONFIG_NAME,
                                         checkpoint=CHECKPOINT_URI)
        returned = adapter.infer(
            request, policy_episode_seed=provenance["policy_episode_seed"],
            replan_index=index,
        )
        if client.request["observation"].keys() != request.keys():
            raise ValueError("Serialized client changed observation keys")
        stage_pairs = {
            "transformed_state": (direct_stage["transformed_state"],
                                  audited_stage["transformed_state"]),
            "model_state": (direct_stage["model_state"],
                            audited_stage["model_state"]),
            "tokens": (direct_stage["model_tokens"],
                       audited_stage["model_tokens"]),
            "token_mask": (direct_stage["model_token_mask"],
                           audited_stage["model_token_mask"]),
            "supplied_noise": (direct_stage["supplied_noise"],
                               audited_stage["supplied_noise"]),
            "raw_sample": (direct_stage["raw_sample"],
                           audited_stage["raw_sample"]),
            "denormalized_actions": (
                direct_stage["denormalized_actions"],
                audited_stage["denormalized_actions"],
            ),
            "returned_actions": (official["actions"], returned.actions),
            "archived_actions": (official["actions"], archived_actions),
        }
        for name in direct_stage["model_images"]:
            stage_pairs["image_" + name] = (
                direct_stage["model_images"][name],
                audited_stage["model_images"][name],
            )
            stage_pairs["image_mask_" + name] = (
                direct_stage["model_image_masks"][name],
                audited_stage["model_image_masks"][name],
            )
        for name in direct_stage["transformed_images"]:
            stage_pairs["transformed_image_" + name] = (
                direct_stage["transformed_images"][name],
                audited_stage["transformed_images"][name],
            )
        metrics = {name: compare(*pair) for name, pair in stage_pairs.items()}
        gripper_original = np.asarray(archived_actions)[:, 7]
        gripper_replay = np.asarray(official["actions"])[:, 7]
        rows.append({
            "replan_index": index,
            "input_checks": input_checks,
            "noise_sha256": noise_sha,
            "sampling_kwargs": direct_stage["sampling_kwargs"],
            "stage_comparisons": metrics,
            "gripper_archived": gripper_original.tolist(),
            "gripper_replay": gripper_replay.tolist(),
            "gripper_threshold_mismatch_indices": np.flatnonzero(
                (gripper_original > 0.5) != (gripper_replay > 0.5)
            ).tolist(),
            "archived_first_close_index": (
                int(np.flatnonzero(gripper_original > 0.5)[0])
                if np.any(gripper_original > 0.5) else None
            ),
            "replay_first_close_index": (
                int(np.flatnonzero(gripper_replay > 0.5)[0])
                if np.any(gripper_replay > 0.5) else None
            ),
        })
        print(f"Replan {index}: archive max difference "
              f"{metrics['archived_actions']['max_abs_difference']:.6g}",
              flush=True)

    output.mkdir(parents=True, exist_ok=True)
    report = {
        "source_run": str(run),
        "openpi_commit": server._metadata[
            "saps_model_input_audit"
        ]["openpi_commit"],
        "config_name": CONFIG_NAME,
        "checkpoint_uri": CHECKPOINT_URI,
        "checkpoint_cache": str(checkpoint),
        "checkpoint_metadata_sha256": file_digest(
            checkpoint / "params/_CHECKPOINT_METADATA"
        ),
        "norm_stats_sha256": file_digest(
            checkpoint / "assets/droid/norm_stats.json"
        ),
        "model_type": type(policy._model).__name__,
        "is_pytorch": policy._is_pytorch_model,
        "model_action_horizon": server._action_horizon,
        "model_action_dim": server._action_dim,
        "output_transform_types": [
            type(value).__name__ for value in policy._output_transform.transforms
        ],
        "rows": rows,
    }
    (output / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"Saved {output / 'comparison.json'}")


if __name__ == "__main__":
    main()
