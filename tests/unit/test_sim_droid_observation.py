"""Pure checks for the Isaac-to-DROID observation adapter."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim/src"))

from isaac_fr3.droid_observation import (
    ARM_JOINT_NAMES, build_observation, measured_arm, measured_gripper,
)
from saps.policies.openpi_droid import DROID_POLICY_INPUT_KEYS


class SimDroidObservationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.names = ["fr3_finger_joint1", *reversed(ARM_JOINT_NAMES),
                      "fr3_finger_joint2"]
        self.positions = np.arange(9, dtype=np.float64)
        self.external = np.zeros((180, 320, 3), dtype=np.uint8)
        self.wrist = np.ones((180, 320, 3), dtype=np.uint8)

    def build(self, **changes):
        values = {
            "exterior_image": self.external,
            "wrist_image": self.wrist,
            "joint_names": self.names,
            "joint_positions": self.positions,
            "finger_positions": np.asarray([0.02, 0.02]),
            "prompt": "Pick up the red object",
        }
        values.update(changes)
        return build_observation(**values)

    def test_joint_order_and_exact_keys_without_source_mutation(self) -> None:
        original = self.positions.copy()
        request, width = self.build()
        self.assertEqual(tuple(request), DROID_POLICY_INPUT_KEYS)
        np.testing.assert_array_equal(request["observation/joint_position"],
                                      [7, 6, 5, 4, 3, 2, 1])
        self.assertEqual(request["observation/joint_position"].shape, (7,))
        self.assertEqual(request["observation/gripper_position"].shape, (1,))
        self.assertEqual(width, 0.04)
        np.testing.assert_array_equal(self.positions, original)
        np.testing.assert_array_equal(self.external, 0)
        np.testing.assert_array_equal(self.wrist, 1)

    def test_gripper_width_and_clipping(self) -> None:
        for width, expected in ((0.08, 0.0), (0.04, 0.5),
                                (0.0, 1.0), (0.10, 0.0), (-0.02, 1.0)):
            with self.subTest(width=width):
                measured, scalar = measured_gripper(np.asarray([width / 2] * 2))
                self.assertAlmostEqual(measured, width)
                self.assertAlmostEqual(float(scalar[0]), expected)

    def test_reject_invalid_joint_image_and_prompt(self) -> None:
        with self.assertRaisesRegex(ValueError, "Missing canonical"):
            measured_arm(self.names[:1] + self.names[2:],
                         np.concatenate((self.positions[:1], self.positions[2:])))
        with self.assertRaisesRegex(ValueError, "Measured articulation"):
            measured_arm(self.names, np.zeros(7))
        with self.assertRaisesRegex(ValueError, "shape"):
            self.build(wrist_image=np.zeros((320, 180, 3), dtype=np.uint8))
        with self.assertRaisesRegex(TypeError, "uint8"):
            self.build(exterior_image=self.external.astype(np.float32))
        with self.assertRaisesRegex(ValueError, "prompt"):
            self.build(prompt=" ")


if __name__ == "__main__":
    unittest.main()
