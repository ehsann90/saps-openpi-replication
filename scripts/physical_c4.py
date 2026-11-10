#!/usr/bin/env python3
"""C4 observations, readiness, or warmed shadow inference; never actuation."""
import argparse
from pathlib import Path

from saps.physical.c4_observation import run


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', required=True, choices=['observations', 'shadow', 'readiness'])
    parser.add_argument('--config', type=Path, default=Path('configs/physical_pi05_fr3_c4.json'))
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--prompt', required=True)
    parser.add_argument('--samples', type=int, default=1)
    parser.add_argument('--provenance-mode', choices=['development', 'strict'], default='development',
                        help='Record development sources, or require strict experiment source qualification')
    parser.add_argument('--observation-timeout', type=float, default=30.)
    parser.add_argument('--policy-timeout', type=float, default=120.)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--policy-episode-seed', type=int, default=20260827)
    parser.add_argument('--warmup-policy-seed', type=int, default=20260917)
    parser.add_argument('--robot-description', type=Path)
    parser.add_argument('--controller-manager', default='/controller_manager')
    workspace = Path.home() / 'franka_ehsan_ros2_ws/src'
    parser.add_argument('--lab-stack-dir', type=Path, default=workspace / 'fr3_lab_stack')
    parser.add_argument('--igd-control-dir', type=Path, default=workspace / 'igd_fr3_control')
    return parser.parse_args()


if __name__ == '__main__':
    raise SystemExit(run(parse_args()))
