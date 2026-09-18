"""Pin the committed G1B hand runtime and verify its fixed embodiment defaults."""

from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest
from unittest.mock import Mock, patch

from saps.physical.gripper_ros import (
    DEFAULT_GRASP_EPSILON_INNER_M,
    DEFAULT_GRASP_FORCE_N,
    GRIPPER_LAB_COMMIT,
    create_gripper,
    gripper_timing_helpers,
)


class GripperPinTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

        (self.root / "fr3_lab_stack_runtime").mkdir()
        (self.root / "docs").mkdir()
        (self.root / "test").mkdir()

        (self.root / "fr3_lab_stack_runtime/timing_evidence.py").write_text(
            "VALUE = 1\n"
        )
        (self.root / "fr3_lab_stack_runtime/franka_hand.py").write_text(
            "# hand\n"
        )
        (self.root / "docs/franka_hand.md").write_text("# hand\n")
        (self.root / "test/test_franka_hand.py").write_text("# test\n")
        (self.root / "CMakeLists.txt").write_text("cmake\n")
        (self.root / "package.xml").write_text("<package/>\n")

        self.head = GRIPPER_LAB_COMMIT
        self.status = b""

        def git(command, **kwargs):
            args = command[3:]
            if args == ["rev-parse", "HEAD"]:
                return (self.head + "\n").encode()
            if args == ["status", "--porcelain"]:
                return self.status
            raise AssertionError("Unexpected git command: " + repr(args))

        self.git = patch(
            "saps.physical.gripper_ros.subprocess.check_output",
            side_effect=git,
        )
        self.git.start()
        self.addCleanup(self.git.stop)

        self.identity = patch(
            "saps.physical.gripper_ros.git_identity",
            return_value={},
        )
        self.identity.start()
        self.addCleanup(self.identity.stop)

    def test_exact_clean_gripper_commit_is_accepted(self):
        module, identity = gripper_timing_helpers(self.root)
        self.assertEqual(module.VALUE, 1)
        self.assertEqual(
            identity["gripper_commit"],
            GRIPPER_LAB_COMMIT,
        )
        self.assertIn("arm_baseline", identity)
        self.assertIn("extension_sha256", identity)

    def test_wrong_gripper_commit_is_rejected(self):
        self.head = "0" * 40
        with self.assertRaisesRegex(
            ValueError,
            "requires frozen fr3_lab_stack commit",
        ):
            gripper_timing_helpers(self.root)

    def test_dirty_gripper_checkout_is_rejected(self):
        self.status = b" M fr3_lab_stack_runtime/franka_hand.py\n"
        with self.assertRaisesRegex(
            ValueError,
            "requires a clean",
        ):
            gripper_timing_helpers(self.root)

    def test_create_gripper_uses_validated_fixed_grasp_defaults(self):
        node = Mock()
        collector = NS(latest_gripper=None)
        config = {
            "robot": {"maximum_finger_position_m": 0.04},
            "freshness": {"maximum_source_age_seconds": 0.2},
        }

        hand = Mock()
        factory = Mock(return_value=hand)

        with patch(
            "saps.physical.gripper_ros.runpy.run_path",
            return_value={"RosFrankaHand": factory},
        ):
            gripper = create_gripper(
                node,
                collector,
                config,
                self.root,
                speed=0.1,
                timeout=3.0,
            )

        factory.assert_called_once_with(
            node,
            maximum_width=0.08,
            speed=0.1,
            timeout=3.0,
        )
        self.assertEqual(
            gripper.grasp_force_n,
            DEFAULT_GRASP_FORCE_N,
        )
        self.assertEqual(
            gripper.grasp_epsilon_inner_m,
            DEFAULT_GRASP_EPSILON_INNER_M,
        )
        self.assertEqual(
            gripper.grasp_epsilon_outer_m,
            0.08,
        )


if __name__ == "__main__":
    unittest.main()
