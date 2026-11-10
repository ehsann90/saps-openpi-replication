from __future__ import annotations

import unittest

import numpy as np

from saps.physical.c4_target_audit import audit_targets
from saps.physical.embodiment import FR3_JOINT_NAMES


class SavedChunkAuditTests(unittest.TestCase):
    def setUp(self):
        lower = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, .5445, -3.0159]
        upper = [2.7437, 1.7837, 2.9007, -.1518, 2.8065, 4.5169, 3.0159]
        self.xml = '<robot name="fixture">' + ''.join(
            f'<joint name="{name}" type="revolute"><limit lower="{lo}" upper="{hi}"/></joint>'
            for name, lo, hi in zip(FR3_JOINT_NAMES, lower, upper)) + '</robot>'
        self.request = dict(joint_names=list(FR3_JOINT_NAMES),
                            joint_positions=[-.09, -.41, .085, -2.59, -.03, 2.11, .78])
        self.actions = np.zeros((15, 8), dtype=np.float64)

    def test_targets_use_saved_q_without_accumulation_or_motion_approval(self):
        self.actions[:, 0] = .1
        self.actions[0, 7] = .5
        self.actions[1, 7] = .5001
        original = self.actions.copy()
        report = audit_targets(self.xml, self.request, self.actions)
        self.assertTrue(report['first_chunk_snapshot_checks_passed'])
        self.assertFalse(report['safe_to_execute'])
        self.assertEqual(report['physical_rollout_readiness'], 'NO-GO')
        self.assertEqual(report['policy_actions_executed'], 0)
        self.assertEqual(report['gripper_commands_issued'], 0)
        for row in report['rows']:
            self.assertAlmostEqual(row['proposed_q_target_rad'][0], -.07)
            self.assertFalse(row['safe_to_execute'])
        self.assertEqual(report['rows'][0]['gripper_reference'], 'open')
        self.assertEqual(report['rows'][1]['gripper_reference'], 'closed')
        self.assertEqual(sum(row['inside_execution_window'] for row in report['rows']), 8)
        np.testing.assert_array_equal(self.actions, original)

    def test_existing_joint_margin_rejects_position_inside_urdf_limits(self):
        self.request['joint_positions'][3] = -3.0
        report = audit_targets(self.xml, self.request, self.actions)
        self.assertFalse(report['first_chunk_snapshot_checks_passed'])
        self.assertIn('target_inside_existing_joint_margin', report['rows'][0]['rejection_reasons'])

    def test_singularity_is_rejected_without_relaxing_threshold(self):
        self.request['joint_positions'] = [0.] * 7
        report = audit_targets(self.xml, self.request, self.actions)
        self.assertFalse(report['first_chunk_snapshot_checks_passed'])
        self.assertIn('measured_singularity_gate_rejected', report['rows'][0]['rejection_reasons'])

    def test_malformed_actions_and_joint_order_fail(self):
        for actions in (np.zeros((8, 8)), np.full((15, 8), np.nan), np.zeros((15, 8), dtype=int)):
            with self.subTest(shape=actions.shape, dtype=actions.dtype), self.assertRaises(ValueError):
                audit_targets(self.xml, self.request, actions)
        self.request['joint_names'].reverse()
        with self.assertRaises(ValueError):
            audit_targets(self.xml, self.request, self.actions)

    def test_later_prediction_failure_is_not_an_executed_chunk_failure(self):
        self.actions[14, 3] = -10.
        # Keep the observation far enough inside limits that zero actions pass.
        self.request['joint_positions'][3] = -2.85
        report = audit_targets(self.xml, self.request, self.actions)
        self.assertTrue(report['first_chunk_snapshot_checks_passed'])
        self.assertFalse(report['all_predictions_snapshot_checks_passed'])
        self.assertFalse(report['safe_to_execute'])


if __name__ == '__main__':
    unittest.main()
