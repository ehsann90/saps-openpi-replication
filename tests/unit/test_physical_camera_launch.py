"""Camera selection shares execution identity without changing lab code."""

from pathlib import Path
import unittest

from saps.physical.camera_launch import realsense_arguments
from saps.physical.shadow_config import load_shadow_config


class CameraLaunchTests(unittest.TestCase):
    def setUp(self) -> None:
        root = Path(__file__).resolve().parents[2]
        self.config = load_shadow_config(root / "configs/physical_pi05_fr3.json")

    def test_replacement_serial_and_node_select_device_and_publisher(self) -> None:
        camera = self.config["wrist_camera"]
        camera.update(serial="243322070397", node="/new/wrist",
                      topic="/new/wrist/color/image_raw")
        args = realsense_arguments(camera)
        self.assertEqual(args["serial_no"], "_243322070397")
        self.assertEqual(args["camera_namespace"], "new")
        self.assertEqual(args["camera_name"], "wrist")
        self.assertEqual(args["rgb_camera.color_profile"], "1280x720x30")
        self.assertEqual(args["rgb_camera.color_format"], "RGB8")
        self.assertEqual(args["enable_depth"], "false")

    def test_incompatible_driver_or_topic_is_rejected(self) -> None:
        for change in ({"serial_parameter": "device.serial"},
                       {"serial_parameter_prefix": ""},
                       {"topic": "/other/image"}):
            with self.subTest(change=change):
                camera = dict(self.config["wrist_camera"], **change)
                with self.assertRaises(ValueError):
                    realsense_arguments(camera)
