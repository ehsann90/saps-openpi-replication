"""Tests for the physical spnavd SAPS input backend."""

from __future__ import annotations

import unittest

import numpy as np

from saps.human_input.spnav import ButtonEvent
from saps.human_input.spnav import gripper_command_from_buttons
from saps.human_input.spnav import MotionEvent
from saps.human_input.spnav import process_spnav_motion
from saps.human_input.spnav import rotate_normalized_cartesian_motion
from saps.human_input.spnav import SpnavConfig
from saps.human_input.spnav import SpnavHumanInputBackend


class FakeSpnavBoundary:
    def __init__(self) -> None:
        self.events: list[MotionEvent | ButtonEvent] = []
        self.now = 10.0
        self.opened = False
        self.closed = False
        self.device_present = True
        self.poll_error = None

    def open(self) -> None:
        self.opened = True

    def poll_event(self) -> MotionEvent | ButtonEvent | None:
        if self.poll_error is not None:
            raise self.poll_error
        if not self.events:
            return None
        return self.events.pop(0)

    def close(self) -> None:
        self.closed = True

    def monotonic(self) -> float:
        return self.now

    def device_exists(self, path: str) -> bool:
        del path
        return self.device_present


class SpnavAxisProcessingTest(unittest.TestCase):
    def test_pinned_lab_axis_transformation_is_preserved(self) -> None:
        processed = process_spnav_motion(
            [100, 200, 300, 400, 500, 600],
            SpnavConfig(deadzone=0.0),
        )

        np.testing.assert_allclose(
            processed.tcp_frame_normalized,
            [-0.6, 0.2, 0.4, -1.2, 0.8, 1.0],
        )

    def test_deadzone_zeros_components_without_rescaling(self) -> None:
        processed = process_spnav_motion(
            [149, 150, -151, 0, 0, 0],
            SpnavConfig(deadzone=0.3),
        )

        np.testing.assert_allclose(
            processed.deadzoned_raw,
            [0.0, 0.3, -0.302, 0.0, 0.0, 0.0],
        )

    def test_values_above_nominal_maximum_are_not_clipped(self) -> None:
        processed = process_spnav_motion(
            [0, 0, 750, 0, 0, 0],
            SpnavConfig(deadzone=0.0),
        )

        self.assertEqual(processed.tcp_frame_normalized[0], -1.5)


class HumanFrameTransformationTest(unittest.TestCase):
    def test_rotation_resolves_both_blocks_and_preserves_norms(self) -> None:
        angle = np.pi / 3.0
        rotation = np.asarray(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        motion = np.asarray(
            [0.2, -0.4, 0.1, 0.8, 0.3, -0.2],
            dtype=np.float64,
        )

        transformed = rotate_normalized_cartesian_motion(motion, rotation)

        np.testing.assert_allclose(transformed[:3], rotation @ motion[:3])
        np.testing.assert_allclose(transformed[3:], rotation @ motion[3:])
        self.assertAlmostEqual(
            np.linalg.norm(transformed[:3]),
            np.linalg.norm(motion[:3]),
        )
        self.assertAlmostEqual(
            np.linalg.norm(transformed[3:]),
            np.linalg.norm(motion[3:]),
        )
        self.assertAlmostEqual(
            np.linalg.norm(transformed),
            np.linalg.norm(motion),
        )

    def test_common_rotation_preserves_cosine_geometry(self) -> None:
        rotation = np.asarray(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        first = np.asarray([1.0, 2.0, 3.0, -1.0, 0.5, 0.25])
        second = np.asarray([-2.0, 1.0, 0.5, 0.2, 1.5, -0.5])
        before = float(first @ second / np.linalg.norm(first) / np.linalg.norm(second))
        first_rotated = rotate_normalized_cartesian_motion(first, rotation)
        second_rotated = rotate_normalized_cartesian_motion(second, rotation)
        after = float(
            first_rotated
            @ second_rotated
            / np.linalg.norm(first_rotated)
            / np.linalg.norm(second_rotated)
        )

        self.assertAlmostEqual(before, after)


class SpnavBackendTest(unittest.TestCase):
    def setUp(self) -> None:
        self.boundary = FakeSpnavBoundary()
        self.backend = SpnavHumanInputBackend(
            SpnavConfig(deadzone=0.0, stale_timeout_seconds=0.25),
            boundary=self.boundary,
        )
        self.backend.start()

    def tearDown(self) -> None:
        self.backend.close()

    def test_motion_produces_human_input_in_common_frame(self) -> None:
        self.boundary.events.append(
            MotionEvent((100, 200, 300), (400, 500, 600))
        )
        rotation = np.asarray(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

        sample = self.backend.sample(base_rotation_tcp=rotation)

        self.assertEqual(sample.human_input.action.shape, (7,))
        np.testing.assert_allclose(
            sample.human_input.action[:6],
            sample.base_frame_normalized,
        )
        self.assertEqual(sample.human_input.input_source, "spnavd")
        self.assertTrue(sample.human_input.physical_device_connected)
        self.assertFalse(sample.human_input.stale_input)
        self.assertTrue(sample.human_input.motion_active)
        self.assertEqual(sample.human_input.gripper_command, 0.0)

    def test_stale_motion_is_zero_but_last_raw_event_is_preserved(self) -> None:
        self.boundary.events.append(MotionEvent((500, 0, 0), (0, 0, 0)))
        first = self.backend.sample(base_rotation_tcp=np.eye(3))
        self.boundary.now += 0.3

        stale = self.backend.sample(base_rotation_tcp=np.eye(3))

        self.assertTrue(np.any(first.human_input.action[:6] != 0.0))
        np.testing.assert_array_equal(stale.human_input.action[:6], 0.0)
        self.assertTrue(stale.human_input.stale_input)
        self.assertEqual(stale.last_raw_motion, (500, 0, 0, 0, 0, 0))
        self.assertAlmostEqual(stale.event_age_seconds, 0.3)

    def test_buttons_expose_explicit_intent_and_neutral_is_not_open(self) -> None:
        neutral = self.backend.sample(base_rotation_tcp=np.eye(3))
        self.assertEqual(neutral.human_input.gripper_command, 0.0)

        self.boundary.events.append(ButtonEvent(button=0, pressed=True))
        opened = self.backend.sample(base_rotation_tcp=np.eye(3))
        self.assertEqual(opened.human_input.gripper_command, -1.0)
        self.assertTrue(opened.human_input.open_button_pressed)
        self.assertEqual(len(opened.button_events), 1)

        self.boundary.events.append(ButtonEvent(button=1, pressed=True))
        closed = self.backend.sample(base_rotation_tcp=np.eye(3))
        self.assertEqual(closed.human_input.gripper_command, 1.0)
        self.assertTrue(closed.human_input.close_button_pressed)

    def test_close_has_deterministic_priority_if_both_are_pressed(self) -> None:
        self.assertEqual(
            gripper_command_from_buttons(
                open_pressed=True,
                close_pressed=True,
            ),
            1.0,
        )

    def test_input_error_disconnects_and_clears_motion_and_buttons(self) -> None:
        self.boundary.events.extend(
            [
                MotionEvent((500, 0, 0), (0, 0, 0)),
                ButtonEvent(button=1, pressed=True),
            ]
        )
        active = self.backend.sample(base_rotation_tcp=np.eye(3))
        self.assertTrue(active.human_input.close_button_pressed)
        self.boundary.poll_error = OSError("spnav socket lost")

        failed = self.backend.sample(base_rotation_tcp=np.eye(3))

        self.assertFalse(failed.human_input.connected)
        self.assertFalse(failed.human_input.close_button_pressed)
        self.assertEqual(failed.human_input.gripper_command, 0.0)
        np.testing.assert_array_equal(failed.human_input.action, 0.0)
        self.assertIn("spnav socket lost", failed.human_input.physical_device_error)


if __name__ == "__main__":
    unittest.main()
