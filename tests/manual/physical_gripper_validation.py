#!/usr/bin/env python3
"""Supervised stationary-arm G1B validation; never executes policy arm motion."""

import argparse
from pathlib import Path

from saps.physical.gripper_validation import run_validation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--execute', action='store_true', required=True)
parser.add_argument(
    '--mode',
    choices=['grasp-release', 'archived'],
    required=True,
)
parser.add_argument('--output-dir', type=Path, required=True)
parser.add_argument(
    '--config',
    type=Path,
    default=Path('configs/physical_pi05_fr3.json'),
)
parser.add_argument(
    '--lab-stack-dir',
    type=Path,
    default=Path.home() / 'franka_ros2_ws/src/fr3_lab_stack',
)
parser.add_argument('--prior-validation', type=Path)
parser.add_argument('--actions', type=Path)

if __name__ == '__main__':
    args = parser.parse_args()

    if args.mode == 'archived' and args.prior_validation is None:
        parser.error('--prior-validation is required for archived replay')

    if args.mode == 'archived' and args.actions is None:
        parser.error('--actions is required for archived replay')

    raise SystemExit(run_validation(args))
