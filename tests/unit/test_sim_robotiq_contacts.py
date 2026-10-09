"""Loaded pad contacts must be bilateral in one physics report."""

from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim/src"))

from isaac_fr3.robotiq_contacts import RobotiqContactMonitor


class RobotiqContactsTest(unittest.TestCase):
    def test_selected_mug_target_receives_contact_events(self) -> None:
        monitor = RobotiqContactMonitor.__new__(RobotiqContactMonitor)
        monitor.root = "/World/Robot/Gripper"
        monitor.cube = "/World/TargetMug"
        monitor.sides = {
            side: {"loaded_events": 0, "total_impulse_ns": 0.0,
                   "minimum_separation_m": None, "last_colliders": None}
            for side in ("left", "right")
        }
        monitor.bilateral_loaded_reports = 0
        point = SimpleNamespace(
            impulse=SimpleNamespace(x=1.0, y=0.0, z=0.0),
            separation=-0.001,
        )
        pxr = SimpleNamespace(PhysicsSchemaTools=SimpleNamespace(
            intToSdfPath=lambda value: value
        ))

        def header(target: str):
            return SimpleNamespace(
                collider0="/World/Robot/Gripper/left_fingertip/collision",
                collider1=target, contact_data_offset=0,
                num_contact_data=1,
            )

        with patch.dict(sys.modules, {"pxr": pxr}):
            monitor._on_events([header("/World/TargetCube")], [point], None)
            self.assertEqual(monitor.summary()["left"]["loaded_events"], 0)
            monitor._on_events([header("/World/TargetMug")], [point], None)
        self.assertEqual(monitor.summary()["left"]["loaded_events"], 1)

    def test_contacts_at_different_times_are_not_bilateral(self) -> None:
        monitor = RobotiqContactMonitor.__new__(RobotiqContactMonitor)
        monitor.root = "/World/Robot/Gripper"
        monitor.cube = "/World/TargetCube"
        monitor.sides = {
            side: {"loaded_events": 0, "total_impulse_ns": 0.0,
                   "minimum_separation_m": None, "last_colliders": None}
            for side in ("left", "right")
        }
        monitor.bilateral_loaded_reports = 0
        point = SimpleNamespace(
            impulse=SimpleNamespace(x=1.0, y=0.0, z=0.0),
            separation=-0.001,
        )

        def header(side: str, offset: int):
            return SimpleNamespace(
                collider0=f"/World/Robot/Gripper/{side}_fingertip/collision",
                collider1="/World/TargetCube", contact_data_offset=offset,
                num_contact_data=1,
            )

        pxr = SimpleNamespace(PhysicsSchemaTools=SimpleNamespace(
            intToSdfPath=lambda value: value
        ))
        with patch.dict(sys.modules, {"pxr": pxr}):
            monitor._on_events([header("left", 0)], [point], None)
            monitor._on_events([header("right", 0)], [point], None)
            self.assertEqual(monitor.summary()["bilateral_loaded_reports"], 0)
            monitor._on_events(
                [header("left", 0), header("right", 1)], [point, point], None
            )
        summary = monitor.summary()
        self.assertEqual(summary["bilateral_loaded_reports"], 1)
        self.assertEqual(summary["left"]["loaded_events"], 2)
        self.assertEqual(summary["right"]["loaded_events"], 2)


if __name__ == "__main__":
    unittest.main()
