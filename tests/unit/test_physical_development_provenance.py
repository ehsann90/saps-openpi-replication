"""Development records source changes; qualification and physical safety are separate."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from saps.physical import rollout_provenance as provenance
from saps.physical.hardware_provenance import hardware_timing_helpers, require_supported_arm_geometry
from saps.physical.shadow_config import OPENPI_COMMIT


class DevelopmentProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.lab = self.root/'lab'
        self.lab.mkdir()
        (self.lab/'fr3_lab_stack_runtime').mkdir()
        self.helper = self.lab/'fr3_lab_stack_runtime/timing_evidence.py'
        self.helper.write_text('def clock_provenance():\n    return {"fixture": True}\n')
        self.config = dict(hardware_profile='zed_robotiq_2f85', integration_stage='c4',
                           geometry={'execution_ready':True})
        self.config_path = self.root/'hardware.json'
        self.config_path.write_text(json.dumps(self.config))
        (self.root/'configs').mkdir()
        self.files = {'fr3_lab_stack_runtime/timing_evidence.py':provenance.file_hash(self.helper)}
        (self.root/'configs/zed_robotiq_lab_sources.json').write_text(json.dumps(
            dict(baseline_commit='fixture-base', files_sha256=self.files)))

    def capture(self, *, dirty=True, mismatch=True):
        def snapshot(path, prefixes=()):
            files = dict(self.files)
            if mismatch:
                files['untracked_fixture.py'] = 'changed-content-hash'
            return dict(path=str(path), available=True, commit=OPENPI_COMMIT,
                        branch='fixture-branch', dirty=False if path.name=='openpi' else dirty,
                        diff_sha256={'head':'fixture-diff'}, source_sha256=files)
        with patch.object(provenance, 'repository_snapshot', side_effect=snapshot), \
                patch.object(provenance, 'ancestor', return_value=True):
            return provenance.capture_provenance(self.root,self.lab,self.lab,self.config_path,self.config)

    def test_capture_preserves_dirty_revision_diff_and_config_without_rejection(self):
        record = self.capture()
        self.assertTrue(record['repository']['dirty'])
        self.assertEqual(record['repository']['branch'], 'fixture-branch')
        self.assertEqual(record['repository']['diff_sha256']['head'], 'fixture-diff')
        self.assertIn('untracked_fixture.py', record['manifest']['added'])
        self.assertEqual(record['config']['sha256'], provenance.file_hash(self.config_path))
        self.assertEqual(record['requested_hardware_config'], self.config)
        self.assertEqual(record['effective_hardware']['status'], 'not_observed')

    def test_repository_snapshot_hashes_staged_unstaged_and_untracked_code(self):
        (self.lab/'untracked.py').write_text('fixture = 1\n')
        def git(args, **kwargs):
            if 'ls-files' in args:
                return b'fr3_lab_stack_runtime/timing_evidence.py\0untracked.py\0'
            return b'staged fixture' if '--cached' in args else b'unstaged fixture'
        with patch.object(provenance, 'git_identity', return_value=dict(available=True,dirty=True)), \
                patch.object(provenance.subprocess, 'check_output', side_effect=git):
            record = provenance.repository_snapshot(self.lab)
        self.assertEqual(record['diff_sha256']['staged'], hashlib.sha256(b'staged fixture').hexdigest())
        self.assertNotEqual(record['diff_sha256']['staged'], record['diff_sha256']['unstaged'])
        self.assertEqual(record['source_sha256']['untracked.py'], provenance.file_hash(self.lab/'untracked.py'))

    def test_strict_qualification_rejects_dirty_or_mismatched_sources(self):
        for kwargs in (dict(dirty=True,mismatch=False),dict(dirty=False,mismatch=True)):
            record = self.capture(**kwargs)
            with self.assertRaisesRegex(ValueError,'Strict experiment provenance'):
                provenance.require_qualified_sources(record)
        clean = self.capture(dirty=False,mismatch=False)
        provenance.require_qualified_sources(clean)
        self.assertEqual(clean['qualification']['status'], 'qualified_sources')

    def test_source_qualification_never_means_physical_approval(self):
        dirty = provenance.readiness_report(self.config,self.capture())
        clean = provenance.readiness_report(self.config,self.capture(dirty=False,mismatch=False))
        self.assertEqual(dirty['physical_runtime'],clean['physical_runtime'])
        self.assertEqual(clean['physical_runtime']['status'],'unverified')
        self.assertEqual(clean['reproducible_experiment_qualification']['status'],'not_qualified')
        self.assertEqual(clean['observation']['status'],'not_checked')
        for text in ('collision','workspace','velocity/acceleration','payload','emergency-stop'):
            self.assertIn(text,' '.join(clean['physical_runtime']['unverified_conditions']))
        self.assertEqual(clean['physical_runtime']['blockers'], [])
        for mode in ('development','strict'):
            with self.subTest(mode=mode):
                require_supported_arm_geometry(self.config,joint_space=True)

    def test_development_helper_loads_dirty_code_and_records_actual_hash(self):
        with patch('saps.physical.rollout_provenance.repository_snapshot',
                   return_value=dict(dirty=True,commit='fixture',branch='dev')):
            module, record = hardware_timing_helpers(self.lab,strict=False)
        self.assertTrue(module.clock_provenance()['fixture'])
        self.assertTrue(record['dirty'])
        self.assertEqual(record['timing_helper_sha256'],provenance.file_hash(self.helper))
        self.assertEqual(record['provenance_mode'],'development')

    def test_development_accepts_large_finite_budgets_but_not_invalid_or_unlimited(self):
        for replans,chunks in ((1,1),(100,1),(1,2),(1000,1000)):
            provenance.validate_mode('development',self.config,execution=True,
                                     max_replans=replans,max_chunks=chunks)
        for replans,chunks in ((1,None),(None,1),(True,1),(1,False),(0,1),
                              (1,0),(-1,1),(1,-1),(1.5,1),(1,float('inf'))):
            with self.assertRaisesRegex(ValueError,'positive finite integer'):
                provenance.validate_mode('development',self.config,execution=True,
                                         max_replans=replans,max_chunks=chunks)
        provenance.validate_mode('strict',{},execution=True,max_replans=100,max_chunks=2)
        with self.assertRaises(ValueError):
            provenance.validate_mode('ignore',self.config)

    def test_strict_non_actuating_run_records_qualification_failure_before_ros(self):
        from saps.physical import c4_observation
        from test_physical_c4 import profile
        config = profile()
        self.config_path.write_text(json.dumps(config))
        args = NS(config=self.config_path,mode='observations',samples=1,prompt='offline',
                  output_dir=self.root/'strict_run',provenance_mode='strict',
                  observation_timeout=1.,policy_timeout=1.,policy_episode_seed=1,warmup_policy_seed=2,
                  lab_stack_dir=self.lab,igd_control_dir=self.lab)
        with patch.object(c4_observation,'capture_provenance',return_value=self.capture()):
            self.assertEqual(c4_observation.run(args),1)
        result = json.loads((args.output_dir/'run.json').read_text())
        self.assertIn('Strict experiment provenance',result['error'])
        self.assertEqual(result['policy_actions_executed'],0)
        self.assertTrue((args.output_dir/'provenance.json').exists())

    def test_episode_rejects_unbounded_development_before_any_boundary_calls(self):
        from saps.physical.policy_execution import run_policy_episode
        boundary = Mock()
        with self.assertRaisesRegex(ValueError,'positive finite integer'):
            run_policy_episode(
                boundary=boundary,collector=Mock(),policy=Mock(),config=self.config,
                output_dir=self.root/'no_run',limits={},analyzer=Mock(),expected_identity=('fixture',1),
                analysis_start=0,application_timeout=1,observation_timeout=1,
                policy_episode_seed=1,warmup_policy_seed=2,max_replans=2,max_executed_policy_chunks=None,
                spin_once=Mock(),ros_now=Mock(),result={},provenance_mode='development')
        self.assertFalse(boundary.mock_calls)


if __name__ == '__main__':
    unittest.main()
