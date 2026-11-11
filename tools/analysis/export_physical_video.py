#!/usr/bin/env python3
"""Export saved physical policy observations; requires no ROS or hardware."""
from __future__ import annotations

import argparse
from pathlib import Path

from saps.physical.video_export import export_physical_video


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True,
                        help='New derived directory outside the original run')
    parser.add_argument('--fps', type=float, default=10.)
    parser.add_argument('--side-by-side', action='store_true')
    args = parser.parse_args()
    report = export_physical_video(args.run, args.output_dir, fps=args.fps,
                                   side_by_side=args.side_by_side)
    print(f"Exported {report['output_frames']} frames per stream to {args.output_dir}")


if __name__ == '__main__':
    main()
