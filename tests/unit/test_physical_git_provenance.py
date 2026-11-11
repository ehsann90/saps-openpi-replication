"""Real temporary checkouts cover nested packages and absent Git metadata."""
from __future__ import annotations

from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest
from unittest.mock import patch

from saps.physical.shadow_ros import git_identity
from saps.physical.rollout_provenance import repository_snapshot


class GitProvenanceTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('git'), 'Git executable unavailable')
    def test_nested_package_resolves_outer_root_and_hashes_root_relative_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            subprocess.run(['git', 'init', '-q', str(root)], check=True)
            package = root / 'franka_droid_stack'
            package.mkdir()
            source = package / 'fixture.py'
            source.write_text('value = 1\n')
            subprocess.run(['git', '-C', str(root), 'add', '.'], check=True)
            subprocess.run(['git', '-C', str(root), '-c', 'user.name=Fixture',
                            '-c', 'user.email=fixture@example.invalid',
                            'commit', '-qm', 'Fixture'], check=True)
            record = repository_snapshot(package)
            self.assertEqual(record['repository_root'], str(root.resolve()))
            self.assertFalse(record['dirty'])
            self.assertIn('franka_droid_stack/fixture.py', record['source_sha256'])
            source.write_text('value = 2\n')
            self.assertTrue(git_identity(package)['dirty'])
            self.assertNotEqual(repository_snapshot(package)['source_sha256'],
                                record['source_sha256'])

    def test_non_git_directory_is_explicit_and_git_stderr_is_captured(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch('saps.physical.shadow_ros.subprocess.check_output',
                       wraps=subprocess.check_output) as call:
                record = git_identity(Path(tmp))
            self.assertFalse(record['available'])
            self.assertNotIn('commit', record)
            self.assertEqual(record['source_kind'], 'directory_without_git_identity')
            self.assertEqual(call.call_args.kwargs['stderr'], subprocess.PIPE)
            self.assertFalse(repository_snapshot(Path(tmp))['available'])
            self.assertEqual(git_identity(Path(tmp) / 'missing')['reason'],
                             'directory_missing')

    def test_installed_camera_packages_record_versions_without_false_git_identity(self):
        from types import ModuleType
        from saps.physical.camera_provenance import camera_software_provenance

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            package = root / 'franka_zed_cameras'
            package.mkdir()
            for name in ('camera_validation.py', 'camera_pairing.py', 'zed_cameras.py'):
                (package / name).write_text('# fixture\n')
            installed = root / 'installed'
            installed.mkdir()
            (installed / 'package.xml').write_text('<package><version>5.5.0</version></package>')
            validation = ModuleType('franka_zed_cameras.camera_validation')
            validation.__file__ = str(package / 'camera_validation.py')
            cameras = ModuleType('franka_zed_cameras')
            cameras.camera_validation = validation
            ament = ModuleType('ament_index_python.packages')
            ament.get_package_share_directory = lambda _: str(installed)
            with patch.dict('sys.modules', {
                    'franka_zed_cameras': cameras,
                    'ament_index_python': ModuleType('ament_index_python'),
                    'ament_index_python.packages': ament}):
                report = camera_software_provenance()
            self.assertFalse(report['lab_repository']['available'])
            for evidence in report['packages'].values():
                self.assertEqual(evidence['version'], '5.5.0')
                self.assertFalse(evidence['repository']['available'])
                self.assertEqual(evidence['repository']['source_kind'],
                                 'installed_package_without_git_identity')
                self.assertIn('package_xml_sha256', evidence)

    def test_root_resolution_is_used_for_commit_and_status_queries(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            child = root / 'package'
            child.mkdir()
            def git(args, **kwargs):
                operation = args[3:]
                if operation == ['rev-parse', '--show-toplevel']:
                    self.assertEqual(args[2], str(child))
                    return str(root) + '\n'
                self.assertEqual(args[2], str(root))
                return {('status', '--porcelain'): ' M fixture.py\n',
                        ('branch', '--show-current'): 'main\n',
                        ('rev-parse', 'HEAD'): 'a' * 40 + '\n'}[tuple(operation)]
            with patch('saps.physical.shadow_ros.subprocess.check_output',
                       side_effect=git):
                record = git_identity(child)
            self.assertTrue(record['available'])
            self.assertTrue(record['dirty'])
            self.assertEqual(record['commit'], 'a' * 40)
