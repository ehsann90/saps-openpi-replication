"""Pure SIM-P5 action-selection and fresh-state safety checks."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim/src"))

from isaac_fr3.policy_execution import (
    arm_target, select_actions, terminal_hold_target,
)
from saps.physical.droid_gripper import droid_gripper_decision


class SimPolicyExecutionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.lower = np.full(7, -2.0)
        self.upper = np.full(7, 2.0)

    def test_first_eight_preserve_native_values(self) -> None:
        actions = np.arange(120, dtype=np.float64).reshape(15, 8) / 100
        selected, suffix = select_actions(actions)
        self.assertEqual(selected.shape, (8, 8))
        self.assertEqual(suffix, 7)
        np.testing.assert_array_equal(selected, actions[:8])
        with self.assertRaisesRegex(ValueError, "fewer than eight"):
            select_actions(actions[:7])

    def test_fresh_q_mapping_clipping_and_no_accumulation(self) -> None:
        action = np.asarray([2, -2, 0.5, 0, 0, 0, 0, 0.4], dtype=float)
        first = arm_target(action, np.zeros(7), self.lower, self.upper)
        np.testing.assert_array_equal(
            first["clipped_arm_action"], [1, -1, 0.5, 0, 0, 0, 0]
        )
        np.testing.assert_allclose(
            first["delta_q_rad"], [0.2, -0.2, 0.1, 0, 0, 0, 0]
        )
        measured = np.full(7, 0.05)
        second = arm_target(action, measured, self.lower, self.upper)
        np.testing.assert_allclose(
            second["q_target_rad"], measured + first["delta_q_rad"]
        )
        self.assertNotAlmostEqual(
            second["q_target_rad"][0],
            first["q_target_rad"][0] + first["delta_q_rad"][0],
        )

    def test_joint_limit_rejection_and_terminal_hold(self) -> None:
        action = np.asarray([1, 0, 0, 0, 0, 0, 0, 0.5], dtype=float)
        with self.assertRaisesRegex(ValueError, "outside FR3 joint limits"):
            arm_target(action, np.asarray([1.9, 0, 0, 0, 0, 0, 0]),
                       self.lower, self.upper)
        with self.assertRaisesRegex(ValueError, "finite action"):
            arm_target(np.full(8, np.nan), np.zeros(7), self.lower, self.upper)
        measured = np.asarray([0.3] * 7)
        np.testing.assert_array_equal(
            terminal_hold_target(measured, self.lower, self.upper), measured
        )
        with self.assertRaisesRegex(ValueError, "Cannot hold"):
            terminal_hold_target(np.full(7, np.nan), self.lower, self.upper)

    def test_binary_gripper_threshold(self) -> None:
        self.assertEqual(droid_gripper_decision(0.5).binary_closure, 0.0)
        self.assertEqual(droid_gripper_decision(0.500001).binary_closure, 1.0)


if __name__ == "__main__":
    unittest.main()
