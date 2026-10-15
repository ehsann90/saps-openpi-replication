"""Focused checks for offline native gripper prediction accounting."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT = (Path(__file__).resolve().parents[2] / "tools/analysis"
          / "analyze_velocity_gripper_rollout.py")
SPEC = importlib.util.spec_from_file_location("velocity_gripper_analysis", SCRIPT)
ANALYSIS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYSIS)


class VelocityGripperRolloutAnalysisTest(unittest.TestCase):
    def test_strict_threshold_and_discarded_only_classification(self) -> None:
        values = np.array([-0.2, 0.5, 0.4, 0.51, 1.2])
        classified = ANALYSIS.classify_gripper(values, executed_count=3)
        self.assertEqual(classified["first_close_index"], 3)
        self.assertEqual(classified["first_close_region"], "discarded")
        self.assertEqual(classified["close_count_executed"], 0)
        self.assertEqual(classified["close_count_discarded"], 2)
        np.testing.assert_array_equal(values, [-0.2, 0.5, 0.4, 0.51, 1.2])

    def test_overlap_uses_nominally_matching_future_slots(self) -> None:
        previous = np.array([0.1, 0.1, 0.1, 0.8, 0.9])
        current = np.array([0.2, 0.3, 0.9, 0.1, 0.1])
        mae, disagreements, count = ANALYSIS.overlap_metrics(
            previous, current, executed_count=3
        )
        self.assertEqual(count, 2)
        self.assertAlmostEqual(mae, (0.6 + 0.6) / 2)
        self.assertEqual(disagreements, 2)


if __name__ == "__main__":
    unittest.main()
