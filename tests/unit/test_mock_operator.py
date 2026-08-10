"""Tests for deterministic, control-step-indexed mock operator traces."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from saps.human_input.mock_operator import MockOperatorTrace


class MockOperatorTraceTest(unittest.TestCase):
    def make_trace(self, directory: Path) -> Path:
        path = directory / "operator_trace.json"
        path.write_text(
            json.dumps(
                {
                    "trace_format_version": 1,
                    "segments": [
                        {
                            "start_control_step": 1,
                            "end_control_step": 3,
                            "action": [0.2, 0, 0, 0, 0, 0, -1],
                            "label": "move_right",
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        return path

    def test_trace_advances_only_after_control_step(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace = MockOperatorTrace.from_json(
                self.make_trace(Path(directory)),
                control_frequency_hz=20.0,
            )

        initial = trace.sample()
        repeated_wait_sample = trace.sample()
        trace.control_step_completed()
        active = trace.sample()
        trace.control_step_completed()
        trace.control_step_completed()
        idle = trace.sample()

        np.testing.assert_array_equal(initial.action[:6], np.zeros(6))
        np.testing.assert_array_equal(
            repeated_wait_sample.action,
            initial.action,
        )
        self.assertTrue(active.motion_active)
        self.assertEqual(active.sample_monotonic_seconds, 0.05)
        self.assertEqual(active.last_event_monotonic_seconds, 0.05)
        np.testing.assert_array_equal(idle.action[:6], np.zeros(6))

    def test_trace_rejects_overlapping_segments(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "overlap.json"
            path.write_text(
                json.dumps(
                    {
                        "trace_format_version": 1,
                        "segments": [
                            {
                                "start_control_step": 0,
                                "end_control_step": 2,
                                "action": [0, 0, 0, 0, 0, 0, -1],
                            },
                            {
                                "start_control_step": 1,
                                "end_control_step": 3,
                                "action": [0, 0, 0, 0, 0, 0, -1],
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "non-overlapping"):
                MockOperatorTrace.from_json(
                    path,
                    control_frequency_hz=20.0,
                )


if __name__ == "__main__":
    unittest.main()
