#!/usr/bin/env python3
"""Finite live P1-A discrete-target verifier; no actuation interfaces."""
from __future__ import annotations

import argparse
from pathlib import Path

from saps.physical.verifier_ros import run_verifier


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/physical_pi05_fr3.json"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--prompt", required=True)
    parser.add_argument("--requests", type=int, default=1)
    parser.add_argument("--policy-episode-seed", type=int, default=20260827)
    parser.add_argument("--observation-timeout", type=float, default=30)
    parser.add_argument("--policy-timeout", type=float, default=120)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--max-state-age", type=float, default=0.1)
    parser.add_argument("--max-action-age", type=float, default=1.0)
    parser.add_argument("--max-lateness", type=float, default=1 / 15)
    return parser.parse_args()


if __name__ == "__main__":
    run_verifier(parse_args())
