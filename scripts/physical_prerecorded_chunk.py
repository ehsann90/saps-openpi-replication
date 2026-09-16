#!/usr/bin/env python3
"""C1-B: one canonical prerecorded eight-action chunk; dry run by default."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import time
import uuid

import numpy as np

from saps.physical.discrete_verifier import limits_from_urdf
from saps.physical.live_shadow import write_json
from saps.physical.shadow_ros import git_identity
from saps.physical.streaming_playback import (
    ARTIFACT, ARTIFACT_SHA256, MeasuredState, load_canonical, playback,
)
from saps.physical.streaming_ros import (
    StreamingBoundary,
    correlate,
    pinned_timing_helpers,
    validate_c1b_delivery,
    validate_c1b_runtime_health
)


class DryBoundary:
    """Nine independently supplied diagnostic states; never constructs ROS objects."""

    def __init__(self, references: np.ndarray) -> None:
        if references.shape != (9, 7) or not np.isfinite(references).all():
            raise ValueError("Dry run requires nine finite independent q_ref[7] rows")
        self.references = references.copy()
        self.index = 0
        self.now_ns = 1_000_000_000

    def wait(self, deadline: int) -> None:
        self.now_ns = deadline

    def snapshot(self) -> MeasuredState:
        q = self.references[self.index].copy()
        self.index += 1
        return MeasuredState(q, self.now_ns, self.now_ns, self.now_ns)

    def publish(self, *args: object) -> None:
        raise AssertionError("Dry run must never publish")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-q-refs", type=Path,
                        help="JSON array of nine independent diagnostic q[7] states")
    parser.add_argument("--lab-stack-dir", type=Path,
                        default=Path.home() / "franka_ros2_ws/src/fr3_lab_stack")
    args = parser.parse_args()
    if args.execute == (args.dry_q_refs is not None):
        parser.error("Supply --dry-q-refs for dry run, or --execute alone")
    root = Path(__file__).resolve().parents[1]
    chunk, archive = load_canonical(root)
    timing, lab_identity = pinned_timing_helpers(args.lab_stack_dir)
    clock = timing.clock_provenance()
    run = str(uuid.uuid4())
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir / "actions.npz").write_bytes(archive)
    provenance = {
        "run": run, "repository": git_identity(root), "lab_stack": lab_identity,
        "artifact": str(root / ARTIFACT), "artifact_sha256": ARTIFACT_SHA256,
        "selected_action_indices": list(range(8)), "full_chunk_shape": list(chunk.shape),
        "execute": args.execute, "clock": clock,
    }
    write_json(args.output_dir / "provenance.json", provenance)
    boundary = None
    initialized = False
    analysis_start = 0
    result = {"status": "startup_failed", "rows": []}
    try:
        if not args.execute:
            refs = np.asarray(json.loads(args.dry_q_refs.read_text()), dtype=float)
            boundary = DryBoundary(refs)
            write_json(args.output_dir / "dry_q_refs.json", refs)
            xml = (root / ARTIFACT.parent / "robot_description.urdf").read_text()
            limits = limits_from_urdf(xml)
            result = playback(chunk, boundary, limits, dry_run=True,
                              now=lambda: boundary.now_ns, wait=boundary.wait)
        else:
            # Read deployed limits once before playback; no controller changes.
            query = subprocess.run(
                ["ros2", "param", "get", "--hide-type",
                 "/robot_state_publisher", "robot_description"],
                check=True, capture_output=True, text=True, timeout=20,
            )
            xml = query.stdout.strip()
            limits = limits_from_urdf(xml)
            import rclpy
            rclpy.init(args=[])
            initialized = True

            boundary = StreamingBoundary(run, clock)

            # Discovery only: no commands during this bounded startup window.
            time.sleep(2.0)

            with boundary.lock:
                analysis_start = len(boundary.records)

            result = playback(chunk, boundary, limits)

            # Capture delayed timing and measured q/dq after the single hold.
            time.sleep(2.0)

        (args.output_dir / "robot_description.urdf").write_text(xml)
        write_json(args.output_dir / "joint_limits.json", limits)
    except BaseException as error:
        result["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        if isinstance(boundary, StreamingBoundary):
            try:
                boundary.close()
            except RuntimeError as error:
                result["cleanup_error"] = str(error)
            # All raw JSON strings are retained unchanged under `raw`.
            with boundary.lock:
                records = list(boundary.records)
            with (args.output_dir / "timing.jsonl").open("x") as output:
                for record in records:
                    output.write(json.dumps(record, allow_nan=False) + "\n")
            try:
                analysis_records = records[analysis_start:]

                result["timing_summary"] = correlate(
                    analysis_records,
                    result["rows"],
                    timing.analyze,
                )
                result["delivery_validation"] = validate_c1b_delivery(
                    result["rows"],
                    result["timing_summary"],
                )
                result["runtime_health_validation"] = validate_c1b_runtime_health(
                    analysis_records,
                    result["rows"],
                )
            except (KeyError, TypeError, ValueError) as error:
                result["timing_analysis_error"] = str(error)
        if initialized:
            rclpy.shutdown()
        write_json(args.output_dir / "playback.json", result)
    for row in result["rows"]:
        offset = (row["scheduled_monotonic_ns"] - result["start_monotonic_ns"]) / 1e6
        print(f"{row['type']} {row['action_index']}: {offset:.6f} ms; "
              f"admitted={row['safety_gate']['accepted']}")
    print(result["status"])
    if args.execute:
        success = (
            result["status"] == "published"
            and "cleanup_error" not in result
            and "timing_analysis_error" not in result
            and result.get("delivery_validation", {}).get("accepted") is True
            and result.get("runtime_health_validation", {}).get("accepted") is True
        )
    else:
        success = result["status"] == "dry_run_complete"
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
