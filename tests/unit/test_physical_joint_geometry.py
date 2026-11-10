"""Joint-space calculations need no hand TCP; physical/provenance gates remain."""
from __future__ import annotations

from contextlib import ExitStack
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from saps.physical import fr3_forward_kinematics as fk
from saps.physical.discrete_verifier import limits_from_urdf
from saps.physical.hardware_provenance import (
    arm_geometry_blockers, hardware_timing_helpers, require_supported_arm_geometry,
)
from saps.physical.shadow_config import load_shadow_config
from saps.physical.single_action import safety_gate, singularity_metric
from saps.physical.streaming_playback import MeasuredState, prepare

ROOT = Path(__file__).resolve().parents[2]
Q = np.array([-.09, -.41, .085, -2.59, -.03, 2.11, .78])


class JointGeometryTests(unittest.TestCase):
    def test_direct_flange_matches_previous_shift_at_many_poses(self):
        poses = [Q, np.zeros(7), *np.random.default_rng(10).uniform(-2, 2, (40, 7))]
        for q in poses:
            previous = fk.fr3_tcp_jacobian(q).copy()
            offset = fk.fr3_flange_fk(q)[:3, 3] - fk.fr3_tcp_fk(q)[:3, 3]
            previous[:3] += np.cross(previous[3:].T, offset).T
            direct = fk.fr3_flange_jacobian(q)
            np.testing.assert_allclose(direct, previous, atol=1e-14, rtol=1e-14)
            np.testing.assert_allclose(np.linalg.svd(direct, compute_uv=False),
                                       np.linalg.svd(previous, compute_uv=False), atol=1e-14)

    def test_flange_jacobian_matches_independent_finite_difference(self):
        for q in (Q, np.zeros(7)):
            numeric = np.zeros((6, 7))
            initial = fk.fr3_flange_fk(q)
            for j in range(7):
                delta = np.eye(7)[j] * 1e-6
                plus, minus = fk.fr3_flange_fk(q + delta), fk.fr3_flange_fk(q - delta)
                numeric[:3, j] = (plus[:3, 3] - minus[:3, 3]) / 2e-6
                derivative = (plus[:3, :3] - minus[:3, :3]) / 2e-6
                skew = derivative @ initial[:3, :3].T
                numeric[3:, j] = [skew[2, 1], skew[0, 2], skew[1, 0]]
            np.testing.assert_allclose(fk.fr3_flange_jacobian(q), numeric, atol=1e-8)

    def test_arm_only_limits_and_preparation_work_with_all_hand_functions_broken(self):
        config = load_shadow_config(ROOT / 'configs/physical_pi05_fr3_c4.json')
        self.assertIsNone(config['robot']['tcp_frame'])
        xml = '<robot name="arm_only">' + ''.join(
            f'<joint name="fr3_joint{i}" type="revolute">'
            '<limit lower="-4" upper="4"/></joint>' for i in range(1, 8)) + '</robot>'
        limits = limits_from_urdf(xml)
        nominal = singularity_metric(Q)
        with ExitStack() as stack:
            for name in ('fr3_tcp_fk', 'fr3_tcp_jacobian', 'fr3_tcp_finite_displacement'):
                stack.enter_context(patch.object(fk, name, side_effect=AssertionError('unused hand TCP')))
            stack.enter_context(patch.object(fk, 'FLANGE_TO_HAND_TCP_M', np.nan))
            metric = singularity_metric(Q)
            state = MeasuredState(Q, 1_000_000_000, 1_000_000_000, 1_000_000_000)
            action = np.array([.1, -.2, .3, 0, 0, 0, 0, .7])
            row = prepare(action, state, limits, now_ns=1_000_000_000,
                          start_ns=1_000_000_000, scheduled_ns=1_000_000_000,
                          kind='policy_action', action_index=0)
        self.assertEqual(metric['condition_number'], nominal['condition_number'])
        self.assertTrue(row['safety_gate']['accepted'])
        np.testing.assert_allclose(row['q_target'], Q + .2 * action[:7])

    def test_existing_joint_and_runtime_safety_checks_still_reject(self):
        result = safety_gate(np.zeros(8), Q, dict(lower_rad=Q + .01, upper_rad=Q + .2),
                             state_age=.2, receive_age=.2, action_age=2.,
                             readiness_reasons=['franka_collision_indicator_active_or_invalid'])
        self.assertFalse(result['accepted'])
        self.assertIn('measured_joint_position_outside_limits', result['rejection_reasons'])
        self.assertIn('franka_collision_indicator_active_or_invalid', result['rejection_reasons'])

    def test_joint_space_findings_are_informational_and_cartesian_use_still_rejects(self):
        config = load_shadow_config(ROOT / 'configs/physical_pi05_fr3_c4.json')
        for claimed in (False, True):
            config['geometry']['execution_ready'] = claimed
            blockers = arm_geometry_blockers(config, joint_space=True)
            self.assertEqual(len(blockers), 4)
            self.assertNotIn('TCP', ' '.join(blockers))
            for requirement in ('collision', 'workspace', 'clearance', 'payload', 'end-effector'):
                self.assertIn(requirement, ' '.join(blockers))
            require_supported_arm_geometry(config, joint_space=True)
            with self.assertRaisesRegex(ValueError, 'TCP'):
                require_supported_arm_geometry(config)
        self.assertEqual(arm_geometry_blockers({}, joint_space=True), [])

    def test_explicit_execution_is_still_required_before_startup(self):
        from saps.physical.live_inference_hold_ros import run_inference_hold
        with self.assertRaisesRegex(ValueError,'Explicit execution'):
            run_inference_hold(SimpleNamespace(execute=False),policy_execution=True)

    def test_ros_entry_point_reaches_runtime_startup_without_geometry_approval(self):
        from saps.physical.live_inference_hold_ros import run_inference_hold
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / 'must_not_exist'
            args = SimpleNamespace(execute=True, prompt='fixture', policy_episode_seed=1,
                                   observation_timeout=1, policy_timeout=1,
                                   application_confirmation_timeout=1,
                                   provenance_mode='development', max_replans=1,
                                   max_executed_policy_chunks=1, singularity_limit=17.,
                                   lab_stack_dir=ROOT, igd_control_dir=ROOT,
                                   config=ROOT / 'configs/physical_pi05_fr3_c4.json',
                                   output_dir=destination)
            evidence = dict(openpi={'commit':'15a9616a00943ada6c20a0f158e3adb39df2ccac','dirty':False},
                            qualification={'status':'unqualified_sources','blockers':['dirty fixture']},
                            effective_hardware={'status':'not_observed'})
            with patch('saps.physical.rollout_provenance.capture_provenance',return_value=evidence), \
                    patch('saps.physical.hardware_provenance.hardware_timing_helpers',
                          side_effect=RuntimeError('injected runtime startup failure')) as helper, \
                    patch('saps.physical.live_inference_hold_ros.StreamingBoundary') as boundary, \
                    patch('builtins.print') as console:
                self.assertEqual(run_inference_hold(args, policy_execution=True), 1)
            console.assert_called_once()
            summary = __import__('json').loads(console.call_args.args[0])
            self.assertEqual(summary['status'], 'failed')
            self.assertEqual(summary['policy_actions_executed'], 0)
            self.assertIn('injected runtime startup failure', summary['error'])
            self.assertEqual(summary['output'], str(destination))
            helper.assert_called_once_with(ROOT,strict=False)
            boundary.assert_not_called()
            import json
            record = json.loads((destination/'run.json').read_text())
            self.assertIn('injected runtime startup failure', record['error'])
            self.assertEqual(record['policy_actions_executed'], 0)
            self.assertEqual(record['readiness']['physical_runtime']['status'], 'unverified')

    def test_invalid_episode_budget_rejects_before_boundary_calls(self):
        from saps.physical.policy_execution import run_policy_episode
        boundary, policy = Mock(), Mock()
        with self.assertRaisesRegex(ValueError, 'Positive finite bounds'):
            run_policy_episode(
                boundary=boundary, collector=Mock(), policy=policy,
                config={'hardware_profile': 'zed_robotiq_2f85'}, output_dir=Path('/unused'),
                limits={}, analyzer=Mock(), expected_identity=('fixture', 1), analysis_start=0,
                application_timeout=1, observation_timeout=1, policy_episode_seed=1,
                warmup_policy_seed=2, max_replans=0, max_executed_policy_chunks=1,
                spin_once=Mock(), ros_now=Mock(), result={})
        self.assertEqual(boundary.mock_calls, [])
        self.assertEqual(policy.mock_calls, [])

    def test_dirty_checkout_and_manifest_mismatch_still_block_source_loading(self):
        module = 'saps.physical.hardware_provenance.'
        with patch(module + 'git_identity', return_value={'dirty': True}), \
                patch(module + 'runpy.run_path') as loader:
            with self.assertRaisesRegex(ValueError, 'clean lab checkout'):
                hardware_timing_helpers(Path('/unused'))
            loader.assert_not_called()
        with patch(module + 'git_identity', return_value={'dirty': False}), \
                patch(module + 'subprocess.run'), \
                patch(module + 'subprocess.check_output', return_value=b''), \
                patch(module + 'runpy.run_path') as loader:
            with self.assertRaisesRegex(ValueError, 'manifest'):
                hardware_timing_helpers(Path('/unused'))
            loader.assert_not_called()


if __name__ == '__main__':
    unittest.main()
