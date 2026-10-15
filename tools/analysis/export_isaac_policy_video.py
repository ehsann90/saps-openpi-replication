#!/usr/bin/env python3
"""Export saved OpenPI request images as sparse policy-observation video."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "sim/src"))

from isaac_fr3.policy_video import archived_frames, export_video, write_metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    frames, period, missing = archived_frames(args.run)
    output = args.run / "videos"
    result = export_video(
        frames, output, period_s=period,
        label="replan-sampled policy observations",
        video_name="policy_observations",
        source_rollout_id=args.run.name,
    )
    result["source_rollout"] = str(args.run.resolve())
    result["missing_or_dropped_frames"] = missing
    if missing:
        result["status"] = "partial"
    result["observation_source"] = (
        "saved raw RGB request arrays, checked against observation.json SHA-256"
    )
    write_metadata(output / "policy_observations.json", result)
    print(json.dumps({"output": str(output / "policy_observations.mp4"),
                      "frames": result["frame_count"],
                      "simulated_seconds": result["encoded_presentation_seconds"]}))


if __name__ == "__main__":
    main()
