#!/usr/bin/env python3
"""Validate and print the excluded Gate-2 operator-pilot schedule."""

from __future__ import annotations

from collections import Counter
import dataclasses
from pathlib import Path

import tyro

from saps.evaluation.experiment_session import load_manifest
from saps.evaluation.experiment_session import manifest_sha256
from saps.evaluation.gate2_protocol import GATE2_MANIFEST_PATH
from saps.evaluation.gate2_protocol import GATE2_OUTPUT_ROOT
from saps.evaluation.gate2_protocol import GATE2_PROFILE_PATH
from saps.evaluation.gate2_protocol import gate2_ordering_metrics
from saps.evaluation.gate2_protocol import validate_gate2_protocol


@dataclasses.dataclass
class Args:
    spacemouse_device_path: str
    manifest_path: str = GATE2_MANIFEST_PATH
    spacemouse_profile_path: str = GATE2_PROFILE_PATH
    output_dir: str = GATE2_OUTPUT_ROOT


def main(args: Args) -> None:
    """Fail on protocol drift and print the deterministic 60-row plan."""

    manifest = load_manifest(Path(args.manifest_path))
    result = validate_gate2_protocol(
        manifest=manifest,
        input_source="spacemouse",
        spacemouse_profile_path=args.spacemouse_profile_path,
        spacemouse_device_path=args.spacemouse_device_path,
        output_root=Path(args.output_dir),
    )
    episodes = result["schedule"]["episodes"]
    mode_counts = Counter(episode["mode"] for episode in episodes)
    condition_counts = Counter(
        episode["condition_id"] for episode in episodes
    )
    pair_counts = Counter(
        (episode["mode"], episode["condition_id"])
        for episode in episodes
    )
    ordering = gate2_ordering_metrics(result["schedule"])

    print("Gate-2 excluded operator pilot preflight: PASSED")
    print(f"Experiment ID: {manifest.experiment_id}")
    print(f"Manifest path: {args.manifest_path}")
    print(f"Manifest SHA-256: {manifest_sha256(manifest)}")
    print(
        "Perturbation config: "
        f"{result['task_config']['path']} "
        f"sha256={result['task_config']['sha256']}"
    )
    print(
        "SpaceMouse profile: "
        f"{result['spacemouse_profile']['path']} "
        f"sha256={result['spacemouse_profile']['sha256']}"
    )
    print(f"SpaceMouse device: {result['spacemouse_device_path']}")
    print(f"Output root: {args.output_dir}")
    print(f"Conditions: {', '.join(manifest.conditions)}")
    print(f"Modes: {', '.join(manifest.modes)}")
    print(f"Trials per mode-condition: {manifest.trials_per_condition}")
    print(f"Episodes: {len(episodes)}")
    print(f"Episodes per mode: {dict(mode_counts)}")
    print(f"Episodes per condition: {dict(condition_counts)}")
    print(
        "Episodes per mode-condition: "
        f"min={min(pair_counts.values())} max={max(pair_counts.values())}"
    )
    print(f"Initial state: {manifest.initial_state_index}")
    print(f"Environment seed: {manifest.environment_seed}")
    print(f"Policy base seed: {manifest.policy_base_seed}")
    print(f"Ordering seed: {manifest.ordering_seed}")
    print(f"Ordering method: {ordering['ordering_method']}")
    print(f"Fixed autonomy weight: {manifest.fixed_autonomy_weight}")
    print(f"Cosine gain: {manifest.cosine_gain}")
    print(f"Control frequency: {manifest.control_frequency_hz} Hz")
    print(f"Horizon: {manifest.operator_max_steps} steps")
    print(
        "Maximum same-mode run length: "
        f"{ordering['maximum_same_mode_run_length']}"
    )
    print(
        "Maximum same-condition run length: "
        f"{ordering['maximum_same_condition_run_length']}"
    )
    print(
        "Minimum matched condition-trial intervening episodes: "
        f"{ordering['minimum_same_identity_intervening_episodes']}"
    )
    print("Pairwise mode precedence by condition:")
    for condition_id, precedence in ordering[
        "pairwise_mode_precedence"
    ].items():
        print(f"  {condition_id}:")
        for label, count in precedence.items():
            first_mode, second_mode = label.split("_before_", 1)
            print(
                f"    {first_mode} before {second_mode}: {count}/5"
            )
    print()
    print("index  mode            condition  trial  policy_seed")
    for episode in episodes:
        print(
            f"{episode['order_index']:>5}  "
            f"{episode['mode']:<15} "
            f"{episode['condition_id']:<10} "
            f"{episode['trial_index']:>5}  "
            f"{episode['policy_episode_seed']:>11}"
        )
    print()
    print("Preflight does not create outputs or launch an episode.")


if __name__ == "__main__":
    main(tyro.cli(Args))
