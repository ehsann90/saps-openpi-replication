#!/usr/bin/env python3
"""Compare two prompts at one archived DROID observation, without actuation."""

from __future__ import annotations

import argparse
import datetime as dt
import json
from pathlib import Path
from typing import Any

import numpy as np

from saps.physical.droid_gripper import droid_gripper_decision
from saps.policies.model_input_audit import array_evidence
from saps.policies.openpi_droid import DROID_POLICY_INPUT_KEYS, OpenPiDroidPolicy
from tools.diagnostics.sweep_archived_droid_gripper_state import (
    ACTION_SHAPE, CHECKPOINT, CONFIG_NAME, GRIPPER_KEY, JOINT_KEY,
    OPENPI_COMMIT, action_evidence, file_sha256, ideal_rollout,
    load_archive, select_request,
)


ARCHIVED_PROMPT = "Pick up the red object"
ALTERNATE_PROMPT = "Pick up the red T-shaped object"


def with_prompt(original: dict[str, Any], prompt: str) -> dict[str, Any]:
    """Copy one canonical input and verify that only its prompt differs."""
    if not isinstance(prompt, str) or not prompt.strip():
        raise ValueError("Prompt must be a nonempty string")
    candidate = {key: value.copy() if isinstance(value, np.ndarray) else value
                 for key, value in original.items()}
    candidate["prompt"] = prompt
    if set(candidate) != set(DROID_POLICY_INPUT_KEYS):
        raise ValueError("Noncanonical archived input")
    if any(array_evidence(candidate[key]) != array_evidence(original[key])
           for key in DROID_POLICY_INPUT_KEYS[:4]):
        raise AssertionError("Prompt intervention changed a canonical array")
    return candidate


def _infer(policy: OpenPiDroidPolicy, observation: dict[str, Any],
           seed: int, replan: int) -> tuple[np.ndarray, dict[str, Any]]:
    before = {key: array_evidence(observation[key]) for key in DROID_POLICY_INPUT_KEYS[:4]}
    prompt_before = observation["prompt"]
    response = policy.infer(observation, policy_episode_seed=seed, replan_index=replan)
    after = {key: array_evidence(observation[key]) for key in DROID_POLICY_INPUT_KEYS[:4]}
    if before != after or prompt_before != observation["prompt"]:
        raise RuntimeError("Inference mutated canonical input")
    actions = np.array(response.actions, copy=True)
    action_evidence(actions)
    return actions, response.sampling_metadata or {}


def _valid_sampling(metadata: dict[str, Any], seed: int, replan: int,
                    archived_noise: str) -> bool:
    return (metadata.get("policy_episode_seed") == seed
            and metadata.get("replan_index") == replan
            and metadata.get("noise_sha256") == archived_noise)


def describe(actions: np.ndarray, joint_position: np.ndarray) -> dict[str, Any]:
    """Keep raw binary decisions separate from ideal FK displacement."""
    gripper = actions[:, 7]
    decisions = ["CLOSED" if droid_gripper_decision(float(value)).binary_closure else "OPEN"
                 for value in gripper[:8]]
    return {
        "action": action_evidence(actions),
        "first_eight_arm_actions": actions[:8, :7].tolist(),
        "raw_gripper_actions_15": gripper.tolist(),
        "first_eight_gripper_decisions": decisions,
        "first_eight_close_count": decisions.count("CLOSED"),
        **ideal_rollout(joint_position, actions),
    }


def compare(baseline: np.ndarray, alternative: np.ndarray,
            joints: np.ndarray) -> dict[str, Any]:
    """Report effects without assigning an arbitrary success score."""
    baseline_rollout = ideal_rollout(joints, baseline)
    alternate_rollout = ideal_rollout(joints, alternative)
    arm_diff = alternative[:8, :7] - baseline[:8, :7]
    raw_diff = alternative - baseline
    baseline_translation = np.asarray(baseline_rollout["cumulative_translation_m"])
    alternate_translation = np.asarray(alternate_rollout["cumulative_translation_m"])
    denominator = np.linalg.norm(baseline_translation) * np.linalg.norm(alternate_translation)
    cosine = (float(np.dot(baseline_translation, alternate_translation) / denominator)
              if denominator > 0 else None)
    baseline_close = baseline[:8, 7] > .5
    alternate_close = alternative[:8, 7] > .5
    return {
        "exactly_equal": bool(np.array_equal(baseline, alternative)
                              and action_evidence(baseline) == action_evidence(alternative)),
        "max_abs_native_action_difference": float(np.max(np.abs(raw_diff))),
        "max_abs_first_eight_arm_difference": float(np.max(np.abs(arm_diff))),
        "rms_first_eight_arm_difference": float(np.sqrt(np.mean(arm_diff**2))),
        "max_abs_first_eight_gripper_difference": float(np.max(np.abs(raw_diff[:8, 7]))),
        "first_eight_gripper_decision_changed_indices":
            np.flatnonzero(baseline_close != alternate_close).tolist(),
        "ideal_final_tcp_delta_m": (alternate_translation - baseline_translation).tolist(),
        "ideal_final_tcp_delta_norm_m": float(np.linalg.norm(alternate_translation - baseline_translation)),
        "ideal_cumulative_translation_direction_cosine": cosine,
    }


def run_gate(policy: OpenPiDroidPolicy, observation: dict[str, Any],
             archived: np.ndarray, seed: int, replan: int, noise: str,
             report: dict[str, Any], arrays: dict[str, np.ndarray]) -> None:
    """Require an exact archived baseline before any alternate-prompt request."""
    if observation["prompt"] != ARCHIVED_PROMPT:
        raise ValueError("Unexpected archived baseline prompt")
    arrays["archived_actions"] = archived
    baseline_input = with_prompt(observation, ARCHIVED_PROMPT)
    alternate_input = with_prompt(observation, ALTERNATE_PROMPT)
    report["baseline_gate"] = {"passed": False}
    baseline, sampling = _infer(policy, baseline_input, seed, replan)
    arrays["baseline_replay"] = baseline
    report["baseline_gate"].update({
        "sampling_metadata": sampling, "action": action_evidence(baseline),
        "max_abs_difference_from_archive": float(np.max(np.abs(baseline - archived))),
    })
    if (not _valid_sampling(sampling, seed, replan, noise)
            or not np.array_equal(baseline, archived)
            or action_evidence(baseline) != action_evidence(archived)):
        raise RuntimeError("Archived baseline/noise mismatch; alternate prompt was not submitted")
    report["baseline_gate"]["passed"] = True
    report["baseline"] = describe(baseline, observation[JOINT_KEY])

    report["alternate_replays"] = []
    alternatives: list[np.ndarray] = []
    for index in range(3):
        actions, sampling = _infer(policy, alternate_input, seed, replan)
        arrays[f"alternate_replay_{index}"] = actions
        alternatives.append(actions)
        report["alternate_replays"].append({
            "sampling_metadata": sampling,
            "action": action_evidence(actions),
            "max_abs_difference_from_first": float(np.max(np.abs(actions - alternatives[0]))),
        })
        if not _valid_sampling(sampling, seed, replan, noise):
            raise RuntimeError("Alternate-prompt seed/replan/noise mismatch")
    identical = all(np.array_equal(item, alternatives[0])
                    and action_evidence(item) == action_evidence(alternatives[0])
                    for item in alternatives[1:])
    report["alternate_repeatability"] = {"passed": bool(identical)}
    if not identical:
        raise RuntimeError("Alternate prompt was not repeatable")
    report["alternate"] = describe(alternatives[0], observation[JOINT_KEY])
    report["comparison"] = compare(baseline, alternatives[0], observation[JOINT_KEY])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--expected-request", default="request_0006")
    parser.add_argument("--config", type=Path, default=Path("configs/physical_pi05_fr3.json"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    report: dict[str, Any] = {
        "schema_version": 1, "diagnostic": "g2_archived_prompt_comparison",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "status": "failed", "zero_actuation": True,
        "prompts": {"baseline": ARCHIVED_PROMPT, "alternate": ALTERNATE_PROMPT},
    }
    arrays: dict[str, np.ndarray] = {}
    try:
        selected = select_request(args.run_dir)
        if selected.name != args.expected_request:
            raise ValueError(f"Earliest qualifying request is {selected.name}; expected {args.expected_request}")
        observation, archived, seed, replan, noise, provenance = load_archive(selected)
        report.update(archive=provenance, seed=seed, replan_index=replan,
                      noise_sha256=noise, selection=selected.name)
        policy_config = json.loads(args.config.read_text(encoding="utf-8"))["policy"]
        expected = {"config": CONFIG_NAME, "checkpoint": CHECKPOINT,
                    "openpi_commit": OPENPI_COMMIT, "action_shape": list(ACTION_SHAPE),
                    "reference_future_open_loop_horizon": 8}
        if policy_config != expected:
            raise ValueError("Local pinned DROID policy config differs")
        report["config_sha256"] = file_sha256(args.config)
        policy = OpenPiDroidPolicy(host=args.host, port=args.port)
        policy.validate_policy_identity(config_name=CONFIG_NAME, checkpoint=CHECKPOINT)
        metadata = policy.server_metadata
        seeded = metadata["saps_seeded_sampling"]
        audit = metadata.get("saps_model_input_audit", {})
        if (seeded.get("action_horizon") != ACTION_SHAPE[0]
                or audit.get("schema_version") != 1
                or audit.get("openpi_commit") != OPENPI_COMMIT):
            raise RuntimeError("Seeded server horizon or OpenPI identity differs")
        report["server_metadata"] = metadata
        run_gate(policy, observation, archived, seed, replan, noise, report, arrays)
        report["status"] = "passed"
    except Exception as error:
        report["error"] = {"type": type(error).__name__, "message": str(error)}
        raise
    finally:
        np.savez_compressed(args.output_dir / "actions.npz", **arrays)
        with (args.output_dir / "run.json").open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write("\n")
        print(f"{report['status']}: {args.output_dir / 'run.json'}", flush=True)


if __name__ == "__main__":
    main()
