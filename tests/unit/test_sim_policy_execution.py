"""Pure SIM-P5 action-selection and fresh-state safety checks."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim/src"))

from isaac_fr3.policy_execution import (
    ArmVelocityDrive, arm_target, select_actions, terminal_hold_target,
    velocity_command,
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

    def test_explicit_velocity_scale_and_limits(self) -> None:
        action = np.array([2, -2, 0.5, 1, 0, 0, 0, 0.7])
        q = np.array([0, 0, 0, 1.99, 0, 0, 0], dtype=float)
        mapped = velocity_command(
            action, q, self.lower, self.upper,
            np.full(7, 0.8), 1.0,
        )
        np.testing.assert_array_equal(mapped["clipped_arm_action"][:4],
                                      [1, -1, 0.5, 1])
        np.testing.assert_allclose(mapped["requested_velocity_rad_s"][:4],
                                   [1, -1, 0.5, 1])
        np.testing.assert_allclose(mapped["commanded_velocity_rad_s"][:4],
                                   [0.8, -0.8, 0.5, 0.15])
        np.testing.assert_allclose(mapped["velocity_saturation_rad_s"][:4],
                                   [0.2, -0.2, 0, 0.85])
        with self.assertRaisesRegex(ValueError, "finite floating"):
            velocity_command(np.full(8, np.nan), q, self.lower,
                             self.upper, np.ones(7), 1.0)
        with self.assertRaisesRegex(ValueError, "Velocity scale"):
            velocity_command(action, q, self.lower, self.upper,
                             np.ones(7), 0)

    def test_velocity_drive_switches_arm_only_and_clears_stale_target(self) -> None:
        class FakeWorld:
            current_time_step_index = 0
            current_time = 0.0

        class FakeFr3:
            q = np.zeros(8)
            position_target = np.zeros(7)
            velocity_target = np.zeros(7)
            stiffness = np.full(7, 5.0)
            damping = np.full(7, 4.0)
            fail_after_gain = False

            def get_dof_positions(self):
                return self.q.reshape(1, 8)

            def get_dof_velocities(self):
                return np.zeros((1, 8))

            def get_dof_limits(self, *, dof_indices):
                return np.full((1, 7), -2.0), np.full((1, 7), 2.0)

            def get_dof_gains(self, *, dof_indices):
                return self.stiffness.reshape(1, 7), self.damping.reshape(1, 7)

            def get_dof_max_velocities(self, *, dof_indices):
                return np.full((1, 7), 2.0)

            def set_dof_gains(self, *, stiffnesses, dampings, dof_indices,
                              update_default_gains):
                np.testing.assert_array_equal(dof_indices, np.arange(7))
                self.stiffness = np.asarray(stiffnesses).reshape(-1).copy()
                self.damping = np.asarray(dampings).reshape(-1).copy()
                if self.fail_after_gain:
                    raise RuntimeError("gain update interrupted")

            def set_dof_position_targets(self, value, *, dof_indices):
                np.testing.assert_array_equal(dof_indices, np.arange(7))
                self.position_target = np.asarray(value).reshape(-1).copy()

            def get_dof_position_targets(self, *, dof_indices):
                return self.position_target.reshape(1, 7)

            def set_dof_velocity_targets(self, value, *, dof_indices):
                np.testing.assert_array_equal(dof_indices, np.arange(7))
                self.velocity_target = np.asarray(value).reshape(-1).copy()

            def get_dof_velocity_targets(self, *, dof_indices):
                return self.velocity_target.reshape(1, 7)

        class Handles:
            world = FakeWorld()
            fr3 = FakeFr3()
            arm_indices = np.arange(7)
            finger_indices = np.array([7])

        handles = Handles()
        drive = ArmVelocityDrive(handles)
        drive.enter_velocity()
        np.testing.assert_array_equal(handles.fr3.stiffness, 0)
        drive.command(np.ones(7))
        np.testing.assert_array_equal(handles.fr3.velocity_target, 1)
        drive.hold()
        np.testing.assert_array_equal(handles.fr3.velocity_target, 0)
        np.testing.assert_array_equal(handles.fr3.stiffness, 5)
        np.testing.assert_array_equal(handles.fr3.position_target, 0)
        self.assertEqual([row["mode"] for row in drive.transitions],
                         ["joint_velocity", "position_hold"])
        handles.fr3.fail_after_gain = True
        with self.assertRaisesRegex(RuntimeError, "gain update interrupted"):
            drive.enter_velocity()
        self.assertTrue(drive.in_velocity_mode)
        handles.fr3.fail_after_gain = False
        drive.hold()
        np.testing.assert_array_equal(handles.fr3.stiffness, 5)
        np.testing.assert_array_equal(handles.fr3.velocity_target, 0)
        handles.finger_indices = np.array([6])
        with self.assertRaisesRegex(ValueError, "non-gripper"):
            ArmVelocityDrive(handles)


if __name__ == "__main__":
    unittest.main()
