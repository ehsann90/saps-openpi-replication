#!/usr/bin/env python3
"""Offline, staged, one-variable DROID input isolation; never publishes robot commands."""

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
    OPENPI_COMMIT, _sample, action_evidence, file_sha256, ideal_rollout,
    load_archive,
)


EXTERIOR_KEY, WRIST_KEY = DROID_POLICY_INPUT_KEYS[:2]
BASELINE_PROMPT = "Pick up the red object"
ALTERNATE_PROMPT = "Pick up the red T-shaped object"
STAGES = ("replay", "gripper", "prompt", "modalities")
SWEEP = (0.0, 0.25, 0.5, 0.75, 1.0)


def change_only(base: dict[str, Any], replacement: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    """Copy canonical input; verify all and only the declared fields can change."""
    if not set(replacement) <= set(DROID_POLICY_INPUT_KEYS):
        raise ValueError("Noncanonical replacement key")
    candidate = {key: value.copy() if isinstance(value, np.ndarray) else value
                 for key, value in base.items()}
    if set(candidate) != set(DROID_POLICY_INPUT_KEYS):
        raise ValueError("Noncanonical baseline")
    for key, value in replacement.items():
        if key == "prompt":
            if not isinstance(value, str) or not value.strip():
                raise ValueError("Replacement prompt must be a nonempty string")
            candidate[key] = value
        else:
            original = base[key]
            if not isinstance(value, np.ndarray) or value.shape != original.shape or value.dtype != original.dtype:
                raise ValueError(f"Invalid replacement shape or dtype: {key}")
            if not np.isfinite(value).all():
                raise ValueError(f"Nonfinite replacement: {key}")
            candidate[key] = value.copy()
    before = canonical_evidence(base)
    after = canonical_evidence(candidate)
    changed = [key for key in DROID_POLICY_INPUT_KEYS if before[key] != after[key]]
    if not set(changed) <= set(replacement):
        raise AssertionError("An unrequested canonical input changed")
    return candidate, changed


def canonical_evidence(observation: dict[str, Any]) -> dict[str, Any]:
    return {key: ({"type": "str", "value": observation[key]} if key == "prompt"
                  else array_evidence(observation[key])) for key in DROID_POLICY_INPUT_KEYS}


def summarize(actions: np.ndarray, baseline: np.ndarray, fixed_joints: np.ndarray) -> dict[str, Any]:
    action_evidence(actions)
    difference = actions.astype(np.float64) - baseline.astype(np.float64)
    arm_difference = difference[:8, :7]
    result = ideal_rollout(fixed_joints, actions)
    baseline_result = ideal_rollout(fixed_joints, baseline)
    shift = np.asarray(result["ideal_tcp_xyz_m"][-1]) - np.asarray(baseline_result["ideal_tcp_xyz_m"][-1])
    first_eight = ["CLOSED" if droid_gripper_decision(float(g)).binary_closure else "OPEN"
                   for g in actions[:8, 7]]
    return {
        "action": action_evidence(actions),
        "raw_gripper_actions_15": actions[:, 7].tolist(),
        "first_eight_gripper_decisions": first_eight,
        "first_eight_close_count": first_eight.count("CLOSED"),
        "first_eight_arm_actions": actions[:8, :7].tolist(),
        "identical_to_baseline": bool(np.array_equal(actions, baseline)),
        "max_abs_native_action_difference": float(np.max(np.abs(difference))),
        "max_abs_first_eight_arm_difference": float(np.max(np.abs(arm_difference))),
        "rms_first_eight_arm_difference": float(np.sqrt(np.mean(arm_difference ** 2))),
        "changed_first_eight_gripper_decision_indices": [i for i in range(8)
            if (actions[i, 7] > 0.5) != (baseline[i, 7] > 0.5)],
        "ideal_tcp_endpoint_shift_at_fixed_baseline_q_m": shift.tolist(),
        "ideal_tcp_endpoint_shift_at_fixed_baseline_q_norm_m": float(np.linalg.norm(shift)),
    }


def infer_checked(policy: OpenPiDroidPolicy, observation: dict[str, Any],
                  seed: int, replan: int, noise: str) -> tuple[np.ndarray, dict[str, Any]]:
    actions, sampling = _sample(policy, observation, seed, replan)
    if (sampling.get("policy_episode_seed") != seed or sampling.get("replan_index") != replan
            or sampling.get("noise_sha256") != noise):
        raise RuntimeError("Seed/replan/noise mismatch; subsequent interventions stopped")
    return actions, sampling


def submit_variant(policy: OpenPiDroidPolicy, base: dict[str, Any], baseline: np.ndarray,
                   replacement: dict[str, Any], name: str, seed: int, replan: int,
                   noise: str, report: dict[str, Any], arrays: dict[str, np.ndarray]) -> np.ndarray:
    candidate, changed = change_only(base, replacement)
    actions, sampling = infer_checked(policy, candidate, seed, replan, noise)
    arrays[name] = actions
    report[name] = {
        "changed_canonical_fields": changed, "canonical_fields": canonical_evidence(candidate),
        "sampling_metadata": sampling,
        **summarize(actions, baseline, base[JOINT_KEY]),
    }
    return actions


def run_gate(policy: OpenPiDroidPolicy, archive_dir: Path, base: dict[str, Any],
             archived: np.ndarray, seed: int, replan: int, noise: str,
             through: str, report: dict[str, Any], arrays: dict[str, np.ndarray]) -> None:
    arrays["archived_actions"] = archived
    report["stages"] = {}
    replays = []
    for index in range(3):
        actions, sampling = infer_checked(policy, base, seed, replan, noise)
        arrays[f"replay_{index}"] = actions
        replays.append(actions)
        report.setdefault("replays", []).append({"sampling_metadata": sampling,
                                                  "action": action_evidence(actions),
                                                  "equal_to_archive": bool(np.array_equal(actions, archived))})
        if not np.array_equal(actions, archived) or action_evidence(actions) != action_evidence(archived):
            raise RuntimeError("Exact archived replay failed; interventions stopped")
    report["stages"]["replay"] = {"passed": True, "criterion": "Three byte-exact archive replays"}
    if through == "replay":
        return

    baseline = replays[0]
    for index, closure in enumerate(SWEEP):
        name = f"gripper_{index}"
        candidate = {GRIPPER_KEY: np.array([closure], dtype=np.float32)}
        submit_variant(policy, base, baseline, candidate, name, seed, replan, noise,
                       report.setdefault("gripper_sweep", {}), arrays)
        report["gripper_sweep"][name]["input_gripper_closure"] = closure
    report["stages"]["gripper"] = {"passed": True}
    if through == "gripper":
        return

    alternate = []
    for index in range(3):
        action = submit_variant(policy, base, baseline, {"prompt": ALTERNATE_PROMPT},
                                f"prompt_{index}", seed, replan, noise,
                                report.setdefault("prompt_comparison", {}), arrays)
        alternate.append(action)
        if index and (not np.array_equal(action, alternate[0])
                      or action_evidence(action) != action_evidence(alternate[0])):
            raise RuntimeError("Alternate prompt is not deterministic; modality interventions stopped")
    report["stages"]["prompt"] = {"passed": True, "criterion": "Three identical alternate-prompt replays"}
    if through == "prompt":
        return

    for offset, label in ((-1, "previous"), (1, "next")):
        directory = archive_dir.parent / f"request_{replan + offset:04d}"
        if not directory.is_dir():
            raise ValueError(f"Missing adjacent archived request: {directory}")
        neighbor, _, neighbor_seed, _, _, provenance = load_archive(directory)
        if neighbor_seed != seed or neighbor["prompt"] != base["prompt"]:
            raise ValueError("Adjacent request must share episode seed and prompt")
        report.setdefault("adjacent_archives", {})[label] = provenance
        for group, keys in (
            ("joints", (JOINT_KEY,)),
            ("gripper", (GRIPPER_KEY,)),
            ("proprioception", (JOINT_KEY, GRIPPER_KEY)),
            ("exterior", (EXTERIOR_KEY,)),
            ("wrist", (WRIST_KEY,)),
            ("both_images", (EXTERIOR_KEY, WRIST_KEY)),
        ):
            replacement = {key: neighbor[key] for key in keys}
            candidate, changed = change_only(base, replacement)
            if not changed:
                report.setdefault("skipped", {})[f"{label}_{group}"] = "Adjacent field identical to baseline"
                continue
            if group == "proprioception" and len(changed) != 2:
                report.setdefault("skipped", {})[f"{label}_{group}"] = "One adjacent proprioception field identical to baseline"
                continue
            # Images stay fixed for proprioception swaps; joint and gripper state
            # stay fixed for image swaps. All comparisons use baseline noise.
            expected = set(keys)
            if set(changed) != expected or candidate["prompt"] != base["prompt"]:
                raise AssertionError("Modality intervention did not change exactly the requested fields")
            name = f"{label}_{group}"
            submit_variant(policy, base, baseline, replacement, name, seed, replan,
                           noise, report.setdefault("modality_comparison", {}), arrays)
    report["stages"]["modalities"] = {"passed": True}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--baseline-request", default="request_0001")
    parser.add_argument("--through", choices=STAGES, required=True)
    parser.add_argument("--config", type=Path, default=Path("configs/physical_pi05_fr3.json"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    report: dict[str, Any] = {
        "schema_version": 1, "diagnostic": "g2_archived_input_isolation",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "status": "failed", "zero_actuation": True,
        "through_stage": args.through, "baseline_request": args.baseline_request,
        "prompts": {"baseline": BASELINE_PROMPT, "alternate": ALTERNATE_PROMPT},
        "interpretation": "Synthetic adjacent observations measure policy sensitivity, not physical outcomes; ideal TCP comparisons start from fixed baseline joints.",
    }
    arrays: dict[str, np.ndarray] = {}
    try:
        if not args.baseline_request.startswith("request_") or len(args.baseline_request) != 12 or not args.baseline_request[8:].isdigit():
            raise ValueError("Baseline request must be request_NNNN")
        archive_dir = args.run_dir / args.baseline_request
        base, archived, seed, replan, noise, provenance = load_archive(archive_dir)
        if base["prompt"] != BASELINE_PROMPT:
            raise ValueError("Archived baseline prompt differs from the intended comparison")
        report.update(archive=provenance, seed=seed, replan_index=replan,
                      noise_sha256=noise, baseline_input=canonical_evidence(base))
        expected = {"config": CONFIG_NAME, "checkpoint": CHECKPOINT,
                    "openpi_commit": OPENPI_COMMIT, "action_shape": list(ACTION_SHAPE),
                    "reference_future_open_loop_horizon": 8}
        config = json.loads(args.config.read_text(encoding="utf-8"))
        if config.get("policy") != expected:
            raise ValueError("Local physical policy config differs from pinned DROID identity")
        report["config_sha256"] = file_sha256(args.config)
        policy = OpenPiDroidPolicy(host=args.host, port=args.port)
        policy.validate_policy_identity(config_name=CONFIG_NAME, checkpoint=CHECKPOINT)
        metadata = policy.server_metadata
        seeded, audit = metadata["saps_seeded_sampling"], metadata.get("saps_model_input_audit", {})
        if (seeded.get("action_horizon") != ACTION_SHAPE[0]
                or audit.get("schema_version") != 1 or audit.get("openpi_commit") != OPENPI_COMMIT):
            raise RuntimeError("Seeded server or model-input identity differs")
        report["server_metadata"] = metadata
        run_gate(policy, archive_dir, base, archived, seed, replan, noise,
                 args.through, report, arrays)
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
