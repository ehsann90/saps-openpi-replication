"""Migrated source/overlay identity; no ROS node or hardware access."""
from pathlib import Path
import os
import tempfile
from types import ModuleType
import unittest
from unittest.mock import patch

from saps.physical.c4_readiness import workspace_evidence
from saps.physical.ros_packages import (
    default_lab_stack_directory, lab_runtime_file,
)


class RosPackageTests(unittest.TestCase):
    def test_explicit_source_layouts_and_ambiguous_sources(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            old = root / 'fr3_lab_stack_runtime/timing_evidence.py'
            new = root / 'franka_droid_stack/franka_droid_stack_runtime/timing_evidence.py'
            with self.assertRaises(ValueError):
                lab_runtime_file(root, 'timing_evidence.py')
            new.parent.mkdir(parents=True)
            new.touch()
            self.assertEqual(lab_runtime_file(root, new.name), new)
            old.parent.mkdir()
            old.touch()
            with self.assertRaises(ValueError):
                lab_runtime_file(root, old.name)
            new.unlink()
            self.assertEqual(lab_runtime_file(root, old.name), old)

    def test_checkout_override_keeps_frozen_protocols_available(self) -> None:
        with patch.dict(os.environ, {'FRANKA_ROS2_WS': '/fixture/ws'}, clear=True):
            self.assertEqual(default_lab_stack_directory(),
                             Path('/fixture/ws/src/franka_droid_stack'))
            os.environ['FRANKA_LAB_STACK_DIR'] = '/fixture/frozen-lab'
            self.assertEqual(default_lab_stack_directory(),
                             Path('/fixture/frozen-lab'))

    def test_both_packages_must_resolve_to_the_recorded_repository(self) -> None:
        packages = ModuleType('ament_index_python.packages')
        index = ModuleType('ament_index_python')
        index.packages = packages
        root = Path('/fixture/personal-repo')
        shares = {name: str(root / name) for name in (
            'franka_droid_stack', 'franka_robotiq')}
        packages.get_package_share_directory = shares.__getitem__
        with patch.dict('sys.modules', {'ament_index_python': index,
                                       'ament_index_python.packages': packages}), \
                patch.dict(os.environ, {'ROS_DISTRO': 'jazzy'}):
            self.assertEqual(workspace_evidence(root)['source_directory'],
                             str(root / 'franka_droid_stack'))
            shares['franka_robotiq'] = '/fixture/another/franka_robotiq'
            with self.assertRaisesRegex(RuntimeError, 'gripper overlay mismatch'):
                workspace_evidence(root)
            shares['franka_droid_stack'] = '/fixture/legacy/fr3_lab_stack'
            with self.assertRaisesRegex(RuntimeError, 'lab overlay mismatch'):
                workspace_evidence(root)
