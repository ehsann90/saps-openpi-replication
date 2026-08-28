#!/usr/bin/env python3
"""Compare live M3 policy and human normalized-action diagnostics."""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
from pathlib import Path
from typing import Any


def main(args: argparse.Namespace) -> None:
    """Write a read-only scale comparison without modifying either input."""

    if args.output_path.exists():
        raise FileExistsError(args.output_path)
    projection = _load_json(args.projection_path)
    spnav = _load_json(args.spnav_path)
    policy = projection.get("policy_normalized_distribution")
    human = spnav.get("distribution", {}).get("active_samples")
    if not isinstance(policy, dict):
        raise ValueError("Projection has no policy normalized distribution.")
    if not isinstance(human, dict):
        raise ValueError(
            "SpaceMouse diagnostic has no active motion samples; collect "
            "deliberate operator motion before comparing distributions."
        )

    comparisons = {}
    for metric in (
        "translation_norm",
        "rotation_norm",
        "overall_motion_norm",
    ):
        policy_median = float(policy[metric]["median"])
        human_median = float(human[metric]["median"])
        comparisons[metric] = {
            "policy_median": policy_median,
            "human_active_median": human_median,
            "human_to_policy_median_ratio": (
                human_median / policy_median
                if policy_median != 0.0
                else None
            ),
        }
    record = {
        "schema_version": 1,
        "milestone": "physical_pi05_droid_m3",
        "created_utc": datetime.datetime.now(
            datetime.timezone.utc
        ).isoformat(),
        "scope": (
            "diagnostic comparison only; no gain selection, clipping, "
            "retuning, arbitration, or robot command"
        ),
        "inputs": {
            "projection_path": str(args.projection_path.resolve()),
            "projection_sha256": _sha256(args.projection_path),
            "spnav_path": str(args.spnav_path.resolve()),
            "spnav_sha256": _sha256(args.spnav_path),
        },
        "policy": policy,
        "human_active": human,
        "comparison": comparisons,
        "component_range": {
            "policy_minimum": policy["component_minimum"],
            "policy_maximum": policy["component_maximum"],
            "human_active_minimum": human["component_minimum"],
            "human_active_maximum": human["component_maximum"],
        },
        "above_unit_frequency": {
            "policy_component_fraction": policy[
                "component_fraction_above_unit_magnitude"
            ],
            "policy_action_fraction": policy[
                "action_fraction_with_any_component_above_unit_magnitude"
            ],
            "human_component_fraction": human[
                "component_fraction_above_unit_magnitude"
            ],
            "human_action_fraction": human[
                "action_fraction_with_any_component_above_unit_magnitude"
            ],
        },
        "interpretation": (
            "Any serious mismatch is an M4 scaling decision. These M3 "
            "artifacts preserve the pinned input mapping and M2 policy "
            "normalization unchanged."
        ),
        "hidden_gain_tuning": False,
    }
    args.output_path.parent.mkdir(parents=True, exist_ok=True)
    with args.output_path.open("x", encoding="utf-8") as file:
        json.dump(record, file, indent=2, sort_keys=True, allow_nan=False)
        file.write("\n")
    print(json.dumps(comparisons, indent=2, sort_keys=True))
    print(f"Wrote M3 action comparison to {args.output_path}")


def _load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--projection-path", type=Path, required=True)
    parser.add_argument("--spnav-path", type=Path, required=True)
    parser.add_argument("--output-path", type=Path, required=True)
    return parser.parse_args()


if __name__ == "__main__":
    main(_parse_args())
