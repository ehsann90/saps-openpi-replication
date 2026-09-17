#!/usr/bin/env python3
"""C1-C2: repeated measured-state-anchored policy chunks; arm only."""

from __future__ import annotations

import argparse
from pathlib import Path

from saps.physical.live_inference_hold_ros import run_inference_hold


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", required=True,
                        help="Explicitly enable policy arm execution")
    parser.add_argument("--config", type=Path,
                        default=Path("configs/physical_pi05_fr3.json"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--prompt", required=True)
    parser.set_defaults(policy_episode_seed=20260827)
    parser.add_argument("--warmup-policy-seed", type=int, default=20260917)
    parser.add_argument("--max-replans", type=int, default=100,
                        help="Hard finite safety bound; never a task failure")
    parser.add_argument("--max-executed-policy-chunks", type=int, default=1,
                        help="Test-only cap; initial physical validation requires 1")
    parser.add_argument("--observation-timeout", type=float, default=30)
    parser.add_argument("--policy-timeout", type=float, default=120)
    parser.add_argument("--application-confirmation-timeout", type=float,
                        required=True,
                        help="Positive seconds to await hold T4 evidence; "
                             "bounded failure handling, not a latency threshold")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    workspace = Path.home() / "franka_ros2_ws/src"
    parser.add_argument("--lab-stack-dir", type=Path,
                        default=workspace / "fr3_lab_stack")
    parser.add_argument("--igd-control-dir", type=Path,
                        default=workspace / "igd_fr3_control")
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run_inference_hold(parse_args(), policy_execution=True))
