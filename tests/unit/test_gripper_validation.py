"""Validation runner owns its executor and archives startup failures."""

from contextlib import ExitStack
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from saps.physical import gripper_validation as validation


class ValidationTests(unittest.TestCase):
    def test_grasp_release_owns_executor_and_cleans_up_on_failed_readiness(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            args = NS(
                execute=True,
                mode="grasp-release",
                output_dir=output,
                config=Path(
                    "configs/physical_pi05_fr3.json"
                ),
                lab_stack_dir=Path(directory),
                prior_validation=None,
                actions=None,
            )

            ros = Mock()
            executor = Mock()
            executors = NS(
                SingleThreadedExecutor=Mock(
                    return_value=executor
                )
            )

            boundary = Mock()
            boundary.snapshot.side_effect = (
                RuntimeError("not ready")
            )
            boundary.records = []
            boundary.lock = threading.RLock()

            gripper = Mock()
            gripper.hand.settled = True
            gripper.evidence.return_value = {
                "requests": [],
                "events": [],
                "error": None,
            }

            timing = Mock()
            clock = iter(range(0, 1000, 20))

            modules = {
                "rclpy": ros,
                "rclpy.executors": executors,
                "rclpy.qos": NS(
                    qos_profile_sensor_data=object()
                ),
                "sensor_msgs": NS(),
                "sensor_msgs.msg": NS(
                    JointState=object()
                ),
            }

            with ExitStack() as stack:
                stack.enter_context(
                    patch.dict("sys.modules", modules)
                )
                stack.enter_context(
                    patch.object(
                        validation,
                        "gripper_timing_helpers",
                        return_value=(timing, {}),
                    )
                )
                stack.enter_context(
                    patch.object(
                        validation,
                        "git_identity",
                        return_value={},
                    )
                )
                stack.enter_context(
                    patch.object(
                        validation,
                        "create_gripper",
                        return_value=gripper,
                    )
                )
                stack.enter_context(
                    patch.object(
                        validation,
                        "StreamingBoundary",
                        return_value=boundary,
                    )
                )
                stack.enter_context(
                    patch.object(
                        validation,
                        "limits_from_urdf",
                        return_value={},
                    )
                )
                stack.enter_context(
                    patch(
                        "subprocess.check_output",
                        return_value="<robot/>",
                    )
                )
                stack.enter_context(
                    patch.object(
                        validation.time,
                        "monotonic",
                        side_effect=lambda: next(clock),
                    )
                )

                self.assertEqual(
                    validation.run_validation(args),
                    1,
                )

            executor.add_node.assert_called_once_with(
                ros.create_node.return_value
            )
            executor.spin_once.assert_called_once_with(
                timeout_sec=0.01
            )
            executor.shutdown.assert_called_once()

            boundary.publish.assert_not_called()
            gripper.command.assert_not_called()
            gripper.stop.assert_called_once()

            saved = json.loads(
                (output / "run.json").read_text()
            )
            self.assertEqual(
                saved["status"],
                "failed",
            )
            self.assertTrue(
                (
                    output
                    / "gripper_feedback.json"
                ).is_file()
            )


if __name__ == "__main__":
    unittest.main()
