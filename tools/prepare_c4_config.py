#!/usr/bin/env python3
"""Create a new C4 local profile preserving operator-owned gripper settings."""
import argparse
from pathlib import Path
from saps.physical.c4_readiness import prepare_profile

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare_profile(args.hardware_config, args.output)
