"""Bounded PhysX contact evidence for the Robotiq policy condition."""

from __future__ import annotations

from copy import deepcopy
from typing import Any

import numpy as np


class RobotiqContactMonitor:
    """Accumulate loaded pad/cube contacts without storing every event."""

    def __init__(self, stage: Any, config: dict) -> None:
        from omni.physics.core import get_physics_simulation_interface
        from pxr import PhysxSchema

        spec = config["gripper"]
        self.root = spec["body_prim"].rsplit("/base_link", 1)[0]
        self.cube = "/World/TargetCube"
        self.sides = {
            side: {"loaded_events": 0, "total_impulse_ns": 0.0,
                   "minimum_separation_m": None, "last_colliders": None}
            for side in ("left", "right")
        }
        self.bilateral_loaded_reports = 0
        for path in (spec["left_pad_prim"], spec["right_pad_prim"], self.cube):
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid() or not prim.IsActive():
                raise RuntimeError(f"Missing active contact body: {path}")
            PhysxSchema.PhysxContactReportAPI.Apply(
                prim
            ).CreateThresholdAttr().Set(0.0)
        self.subscription = (
            get_physics_simulation_interface()
            .subscribe_physics_contact_report_events(self._on_events)
        )
        if self.subscription is None:
            raise RuntimeError("PhysX contact subscription failed")

    def _on_events(self, headers: Any, contacts: Any, _anchors: Any) -> None:
        from pxr import PhysicsSchemaTools

        loaded_sides = set()
        for header in headers:
            pair = [str(PhysicsSchemaTools.intToSdfPath(value))
                    for value in (header.collider0, header.collider1)]
            if not any(path.startswith(self.cube) for path in pair):
                continue
            points = contacts[
                header.contact_data_offset:
                header.contact_data_offset + header.num_contact_data
            ]
            impulse = sum(float(np.linalg.norm([
                point.impulse.x, point.impulse.y, point.impulse.z
            ])) for point in points)
            if impulse <= 1e-6:
                continue
            separation = min(
                (float(point.separation) for point in points), default=None
            )
            for side in ("left", "right"):
                if not any(path.startswith(self.root + "/" + side)
                           for path in pair):
                    continue
                record = self.sides[side]
                loaded_sides.add(side)
                record["loaded_events"] += 1
                record["total_impulse_ns"] += impulse
                record["last_colliders"] = pair
                if separation is not None:
                    previous = record["minimum_separation_m"]
                    record["minimum_separation_m"] = (
                        separation if previous is None
                        else min(previous, separation)
                    )
        if loaded_sides == {"left", "right"}:
            self.bilateral_loaded_reports += 1

    def summary(self) -> dict:
        return {
            **deepcopy(self.sides),
            "bilateral_loaded_reports": self.bilateral_loaded_reports,
        }
