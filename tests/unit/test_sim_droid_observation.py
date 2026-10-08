"""Pure checks for the Isaac-to-DROID observation adapter."""

from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim/src"))

from isaac_fr3.droid_observation import (
    ARM_JOINT_NAMES, build_observation, capture_observation,
    measured_arm, measured_gripper,
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

    def test_robotiq_capture_keeps_seven_arm_joints_and_pad_scalar(self) -> None:
        names = [*ARM_JOINT_NAMES, "finger_joint"]

        class Articulation:
            def get_dof_indices(self, selected):
                return np.asarray([names.index(name) for name in selected])

            def get_dof_positions(self):
                return np.asarray([0.1] * 7 + [0.4], dtype=np.float32)

        handles = SimpleNamespace(fr3=Articulation())
        rig = SimpleNamespace(external=object(), wrist=object())
        config = {
            "robot": {"arm_dof_names": list(ARM_JOINT_NAMES),
                      "finger_dof_names": ["finger_joint"]},
            "gripper": {"kind": "robotiq_2f85"},
            "droid": {"gripper_max_width_m": 0.08708},
            "cameras": {"external": {"resolution_wh": [320, 180]},
                        "wrist": {"resolution_wh": [320, 180]}},
        }
        camera_module = SimpleNamespace(
            capture_rgb=lambda camera, resolution: self.external
        )
        with patch.dict(sys.modules, {"isaac_fr3.cameras": camera_module}), \
                patch("isaac_fr3.robotiq_gripper.measured_opening",
                      return_value=(0.04, np.asarray([1 - 0.04 / 0.08708],
                                                     dtype=np.float32))):
            request, driver, width, _, _, _ = capture_observation(
                handles, rig, config, "Pick up the red object"
            )
        self.assertEqual(tuple(request), DROID_POLICY_INPUT_KEYS)
        np.testing.assert_allclose(
            request["observation/joint_position"], [0.1] * 7
        )
        self.assertEqual(request["observation/joint_position"].shape, (7,))
        np.testing.assert_allclose(driver, [0.4])
        self.assertAlmostEqual(width, 0.04)
        self.assertAlmostEqual(
            float(request["observation/gripper_position"][0]),
            1 - 0.04 / 0.08708, places=6,
        )
        with self.assertRaisesRegex(ValueError, "Invalid measured gripper"):
            self.build(
                measured_gripper_state=(0.04, np.asarray([0.0])),
            )


if __name__ == "__main__":
    unittest.main()
