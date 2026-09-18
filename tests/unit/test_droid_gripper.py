"""Pure DROID gripper intent and FR3 embodiment mapping; no ROS required."""

import math
import unittest
from unittest.mock import Mock

from saps.physical.droid_gripper import (
    DroidGripper,
    droid_gripper_decision,
)


class DroidGripperTests(unittest.TestCase):
    def test_strict_threshold_polarity(self):
        for raw, closure in (
            (-.2, 0.),
            (0., 0.),
            (.5, 0.),
            (.5000000001, 1.),
            (1., 1.),
            (2., 1.),
        ):
            with self.subTest(raw=raw):
                decision = droid_gripper_decision(raw)
                self.assertEqual(decision.raw_policy_value, raw)
                self.assertEqual(decision.binary_closure, closure)

    def test_nonfinite_policy_values_are_rejected(self):
        for raw in (
            float('nan'),
            float('inf'),
            -float('inf'),
        ):
            with self.assertRaises(ValueError):
                droid_gripper_decision(raw)

    def test_open_maps_to_move_maximum_width(self):
        hand = Mock()
        hand.request_move.return_value = {
            'request_id': 0,
            'command_type': 'move',
            'target_width_m': .08,
            'disposition': 'queued',
        }

        adapter = DroidGripper(
            hand,
            maximum_width_m=.08,
            measured=lambda: {'width_m': .04, 'closure': .5},
        )

        row = adapter.command(.2)

        hand.request_move.assert_called_once_with(.08)
        hand.request_grasp.assert_not_called()
        self.assertEqual(row['binary_closure'], 0.)
        self.assertEqual(row['command_type'], 'move')
        self.assertEqual(row['measured_before']['width_m'], .04)

    def test_closed_maps_to_explicit_grasp(self):
        hand = Mock()
        hand.request_grasp.return_value = {
            'request_id': 0,
            'command_type': 'grasp',
            'target_width_m': 0.,
            'disposition': 'queued',
        }

        adapter = DroidGripper(
            hand,
            maximum_width_m=.08,
            measured=lambda: {'width_m': .06, 'closure': .25},
            grasp_force_n=20.,
            grasp_epsilon_inner_m=0.,
            grasp_epsilon_outer_m=.08,
        )

        row = adapter.command(.8)

        hand.request_move.assert_not_called()
        hand.request_grasp.assert_called_once_with(
            0.,
            force=20.,
            epsilon_inner=0.,
            epsilon_outer=.08,
        )
        self.assertEqual(row['binary_closure'], 1.)
        self.assertEqual(row['command_type'], 'grasp')
        self.assertEqual(row['measured_before']['width_m'], .06)

    def test_closed_rejects_unconfigured_grasp_parameters(self):
        hand = Mock()

        adapter = DroidGripper(
            hand,
            maximum_width_m=.08,
            measured=lambda: {'width_m': .06, 'closure': .25},
        )

        with self.assertRaisesRegex(
            RuntimeError,
            'gripper_grasp_parameters_unconfigured',
        ):
            adapter.command(.8)

        hand.request_move.assert_not_called()
        hand.request_grasp.assert_not_called()

    def test_invalid_adapter_parameters(self):
        for width in (0., -1., float('nan'), float('inf')):
            with self.subTest(width=width):
                with self.assertRaises(ValueError):
                    DroidGripper(
                        Mock(),
                        maximum_width_m=width,
                        measured=lambda: {},
                    )

        for force in (0., -1., float('nan'), float('inf')):
            with self.subTest(force=force):
                with self.assertRaises(ValueError):
                    DroidGripper(
                        Mock(),
                        maximum_width_m=.08,
                        measured=lambda: {},
                        grasp_force_n=force,
                    )

    def test_inference_rejects_queued_replacement(self):
        hand = Mock()
        hand.evidence.return_value = {
            'requests': [{'disposition': 'queued'}]
        }

        adapter = DroidGripper(
            hand,
            maximum_width_m=.08,
            measured=lambda: {},
        )

        with self.assertRaisesRegex(
            RuntimeError,
            'pending_before_inference',
        ):
            adapter.check_inference()

        hand.evidence.return_value = {
            'requests': [{'disposition': 'issued'}]
        }
        adapter.check_inference()


if __name__ == '__main__':
    unittest.main()