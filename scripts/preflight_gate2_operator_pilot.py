#!/usr/bin/env python3
"""Validate and print the excluded Gate-2 v2 shared pilot design."""

from __future__ import annotations

from collections import Counter
import dataclasses
from pathlib import Path

import tyro

from saps.evaluation.experiment_session import load_manifest
from saps.evaluation.experiment_session import manifest_sha256
from saps.evaluation.gate2_v2_protocol import (
    GATE2_V2_AUTONOMOUS_PROTOCOL_PATH,
)
from saps.evaluation.gate2_v2_protocol import GATE2_V2_MANIFEST_PATH
from saps.evaluation.gate2_v2_protocol import GATE2_V2_PROFILE_PATH
from saps.evaluation.gate2_v2_protocol import GATE2_V2_SHARED_OUTPUT_ROOT
from saps.evaluation.gate2_v2_protocol import gate2_v2_ordering_metrics
from saps.evaluation.gate2_v2_protocol import (
    validate_gate2_v2_shared_protocol,
)


@dataclasses.dataclass
class Args:
    spacemouse_device_path: str
    manifest_path: str = GATE2_V2_MANIFEST_PATH
    spacemouse_profile_path: str = GATE2_V2_PROFILE_PATH
    autonomous_protocol_path: str = GATE2_V2_AUTONOMOUS_PROTOCOL_PATH
    output_dir: str = GATE2_V2_SHARED_OUTPUT_ROOT


def main(args: Args) -> None:
    """Fail on protocol drift and print the deterministic 40-row plan."""

    manifest = load_manifest(Path(args.manifest_path))
    result = validate_gate2_v2_shared_protocol(
        manifest=manifest,
        input_source="spacemouse",
        spacemouse_profile_path=args.spacemouse_profile_path,
        spacemouse_device_path=args.spacemouse_device_path,
        output_root=Path(args.output_dir),
        autonomous_protocol_path=Path(args.autonomous_protocol_path),
    )
    episodes = result["schedule"]["episodes"]
    mode_counts = Counter(str(row["mode"]) for row in episodes)
    condition_counts = Counter(str(row["condition_id"]) for row in episodes)
    pair_counts = Counter(
        (str(row["mode"]), str(row["condition_id"])) for row in episodes
    )
    ordering = gate2_v2_ordering_metrics(result["schedule"])

    print("Gate-2 v2 shared-autonomy pilot preflight: PASSED")
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
    print(f"Shared modes: {', '.join(manifest.modes)}")
    print(f"Episodes: {len(episodes)}")
    print(f"Episodes per mode: {dict(mode_counts)}")
    print(f"Episodes per condition: {dict(condition_counts)}")
    print(
        "Episodes per mode-condition: "
        f"min={min(pair_counts.values())} max={max(pair_counts.values())}"
    )
    print(f"Ordering method: {ordering['ordering_method']}")
    print(
        "Maximum same-mode run length: "
        f"{ordering['maximum_same_mode_run_length']}"
    )
    print(
        "Maximum same-condition run length: "
        f"{ordering['maximum_same_condition_run_length']}"
    )
    print(
        "Minimum pair intervening episodes: "
        f"{ordering['minimum_pair_intervening_episodes']}"
    )
    print(
        "Fixed-before-cosine counts: "
        f"{ordering['fixed_before_cosine_by_condition']}"
    )
    print(
        "Combined matched design: "
        f"{result['matched_design']['matched_triplets']} triplets, "
        f"{result['matched_design']['total_episodes']} total outcomes"
    )
    print()
    print("index  mode            condition  trial  matched_autonomous_seed")
    for episode in episodes:
        print(
            f"{episode['order_index']:>5}  "
            f"{episode['mode']:<15} "
            f"{episode['condition_id']:<10} "
            f"{episode['trial_index']:>5}  "
            f"{episode['policy_episode_seed']:>23}"
        )
    print()
    print("Preflight does not create outputs or launch an episode.")


if __name__ == "__main__":
    main(tyro.cli(Args))
