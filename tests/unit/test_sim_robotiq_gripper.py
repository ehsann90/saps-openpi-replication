"""Pure width and binary-command checks for the separate Robotiq scene."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim/src"))

from isaac_fr3.robotiq_gripper import (
    RobotiqGripperController, opening_from_pad_origins,
)


class FakeArticulation:
    def __init__(self) -> None:
        self.commands: list[tuple[np.ndarray, np.ndarray]] = []

    def set_dof_position_targets(
        self, targets: np.ndarray, dof_indices: np.ndarray,
    ) -> None:
        self.commands.append((targets.copy(), dof_indices.copy()))


class FakeHandles:
    def __init__(self) -> None:
        self.fr3 = FakeArticulation()
        self.finger_indices = np.asarray([7])


class RobotiqGripperTest(unittest.TestCase):
    def test_pad_motion_maps_to_physical_width_and_scalar(self) -> None:
        axis = np.asarray([0.0, 1.0, 0.0])
        left = np.asarray([0.0, 0.0, 0.0])
        for travel, expected_width in (
            (0.0, 0.08708), (0.04708, 0.04), (0.1, 0.0),
        ):
            right = np.asarray([0.0, -travel, 0.0])
            width, closure = opening_from_pad_origins(
                left, right, axis, 0.08708
            )
            self.assertAlmostEqual(width, expected_width)
            self.assertEqual(closure.shape, (1,))
            self.assertEqual(closure.dtype, np.float32)
            self.assertAlmostEqual(
                float(closure[0]), 1.0 - expected_width / 0.08708,
                places=6,
            )

    def test_rejects_invalid_geometry(self) -> None:
        with self.assertRaisesRegex(ValueError, "geometry"):
            opening_from_pad_origins(
                np.zeros(3), np.zeros(3), np.ones(3), 0.08708
            )
        with self.assertRaisesRegex(ValueError, "geometry"):
            opening_from_pad_origins(
                np.zeros(3), np.zeros(3), np.asarray([0, 1, 0]), 0.0
            )

    def test_binary_intent_commands_one_driver_only_on_transition(self) -> None:
        handles = FakeHandles()
        config = {"gripper": {
            "open_joint_rad": 0.0,
            "closed_joint_rad": 0.8203047484373349,
        }}
        controller = RobotiqGripperController()
        self.assertFalse(controller.request(handles, config, 0.5))
        self.assertTrue(controller.request(handles, config, 0.500001))
        self.assertFalse(controller.request(handles, config, 1.0))
        self.assertTrue(controller.request(handles, config, 0.0))
        self.assertEqual(len(handles.fr3.commands), 2)
        np.testing.assert_allclose(handles.fr3.commands[0][0], [[0.82030475]])
        np.testing.assert_allclose(handles.fr3.commands[1][0], [[0.0]])
        for _, indices in handles.fr3.commands:
            np.testing.assert_array_equal(indices, [7])
        with self.assertRaisesRegex(ValueError, "Non-finite"):
            controller.request(handles, config, float("nan"))


if __name__ == "__main__":
    unittest.main()
