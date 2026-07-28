"""Tests for the SAPS keyboard-to-action mapping."""

from __future__ import annotations

import unittest

import numpy as np

from saps.human_input.keyboard import KeyboardActionMapper


class KeyboardActionMapperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.mapper = KeyboardActionMapper()

    def sample(
        self,
        keys: set[str],
        *,
        mode: str = "fine",
        armed: bool = True,
        gripper: float = -1.0,
    ) -> np.ndarray:
        return self.mapper.sample(
            pressed_keys=keys,
            gripper_command=gripper,
            speed_mode=mode,
            connected=True,
            armed=armed,
            abort_requested=False,
            last_event_monotonic_seconds=None,
        ).action

    def assert_action(
        self,
        keys: set[str],
        expected: list[float],
        *,
        mode: str = "fine",
    ) -> None:
        np.testing.assert_allclose(
            self.sample(keys, mode=mode),
            expected,
            atol=1e-6,
        )

    def test_camera_relative_translation(self) -> None:
        self.assert_action(
            {"w"},
            [0.07, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
        )
        self.assert_action(
            {"s"},
            [-0.07, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
        )
        self.assert_action(
            {"a"},
            [0.0, -0.07, 0.0, 0.0, 0.0, 0.0, -1.0],
        )
        self.assert_action(
            {"d"},
            [0.0, 0.07, 0.0, 0.0, 0.0, 0.0, -1.0],
        )
        self.assert_action(
            {"space"},
            [0.0, 0.0, 0.07, 0.0, 0.0, 0.0, -1.0],
        )
        self.assert_action(
            {"shift"},
            [0.0, 0.0, -0.07, 0.0, 0.0, 0.0, -1.0],
        )

    def test_rotation_mapping(self) -> None:
        self.assert_action(
            {"arrowright"},
            [0.0, 0.0, 0.0, 0.10, 0.0, 0.0, -1.0],
        )
        self.assert_action(
            {"arrowup"},
            [0.0, 0.0, 0.0, 0.0, 0.10, 0.0, -1.0],
        )
        self.assert_action(
            {"q"},
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.10, -1.0],
        )

    def test_speed_profiles(self) -> None:
        fine = self.sample({"w"}, mode="fine")
        normal = self.sample({"w"}, mode="normal")
        fast = self.sample({"w"}, mode="fast")

        self.assertAlmostEqual(float(fine[0]), 0.07)
        self.assertAlmostEqual(float(normal[0]), 0.14)
        self.assertAlmostEqual(float(fast[0]), 0.25)

    def test_diagonal_is_normalized(self) -> None:
        action = self.sample({"w", "d"})

        self.assertAlmostEqual(
            float(np.linalg.norm(action[:3])),
            0.07,
            places=6,
        )

    def test_disarmed_motion_is_zero(self) -> None:
        action = self.sample(
            {"w", "space", "q"},
            armed=False,
        )

        np.testing.assert_allclose(
            action[:6],
            np.zeros(6),
            atol=1e-6,
        )

    def test_gripper_command_is_preserved(self) -> None:
        closed = self.sample(
            set(),
            gripper=1.0,
        )

        self.assertEqual(float(closed[6]), 1.0)


if __name__ == "__main__":
    unittest.main()
