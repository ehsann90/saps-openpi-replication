#!/usr/bin/env python3
"""Generate descriptive Gate-2 metrics and readiness artifacts."""

from __future__ import annotations

import dataclasses
from pathlib import Path

import tyro

from saps.evaluation.gate2_analysis import analyze_gate2_collection
from saps.evaluation.gate2_protocol import GATE2_OUTPUT_ROOT


@dataclasses.dataclass
class Args:
    session_root: str = GATE2_OUTPUT_ROOT
    autonomous_root: str = (
        "outputs/autonomous_deterministic_n20_state0_v1"
    )
    output_dir: str = "results/gate2_operator_pilot_v1"


def main(args: Args) -> None:
    """Run read-only analysis and report validation readiness."""

    report = analyze_gate2_collection(
        session_root=Path(args.session_root),
        autonomous_root=Path(args.autonomous_root),
        output_dir=Path(args.output_dir),
    )
    print(
        "Gate-2 analysis wrote derived artifacts to "
        f"{args.output_dir}."
    )
    print(
        "Selected analyzable episodes: "
        f"{report['selected_analyzable_episode_count']}/60"
    )
    print(f"Analysis valid: {report['analysis_valid']}")
    print(f"Collection complete: {report['collection_complete']}")
    if report["blocking_errors"]:
        print("Blocking validation errors:")
        for error in report["blocking_errors"]:
            print(f"- {error}")
        raise SystemExit(1)


if __name__ == "__main__":
    main(tyro.cli(Args))
