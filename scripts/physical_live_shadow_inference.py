#!/usr/bin/env python3
"""Finite live FR3 pi05-DROID shadow inference; no execution interface."""

from __future__ import annotations

import argparse
from pathlib import Path

from saps.physical.shadow_ros import run_shadow


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/physical_pi05_fr3.json"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--prompt", required=True)
    parser.add_argument("--requests", type=int, default=10)
    parser.add_argument("--policy-episode-seed", type=int, default=20260827)
    parser.add_argument("--observation-timeout", type=float, default=30)
    parser.add_argument("--policy-timeout", type=float, default=120)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    workspace = Path.home() / "franka_ros2_ws/src"
    parser.add_argument("--lab-stack-dir", type=Path, default=workspace / "fr3_lab_stack")
    parser.add_argument("--igd-control-dir", type=Path, default=workspace / "igd_fr3_control")
    return parser.parse_args()


if __name__ == "__main__":
    run_shadow(parse_args())
