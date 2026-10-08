"""Robotiq 2F-85 joint drive and pad-opening observation for the separate scene."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from saps.physical.droid_gripper import droid_gripper_decision


def opening_from_pad_origins(
    left_xyz_m: np.ndarray, right_xyz_m: np.ndarray,
    base_y_world: np.ndarray, open_width_m: float,
) -> tuple[float, np.ndarray]:
    """Project pad travel onto the gripper's lateral axis, then normalize."""
    left = np.asarray(left_xyz_m, dtype=np.float64)
    right = np.asarray(right_xyz_m, dtype=np.float64)
    lateral = np.asarray(base_y_world, dtype=np.float64)
    if (left.shape != (3,) or right.shape != (3,)
            or lateral.shape != (3,) or not np.isfinite(left).all()
            or not np.isfinite(right).all() or not np.isfinite(lateral).all()
            or not np.isfinite(open_width_m) or open_width_m <= 0
            or not np.isclose(np.linalg.norm(lateral), 1, atol=1e-4)):
        raise ValueError("Invalid Robotiq pad-opening geometry")
    travel_m = float(np.dot(left - right, lateral))
    width_m = float(np.clip(open_width_m - travel_m, 0.0, open_width_m))
    closure = np.asarray(
        [np.clip(1.0 - width_m / open_width_m, 0.0, 1.0)],
        dtype=np.float32,
    )
    return width_m, closure


def measured_opening(handles: Any, config: dict) -> tuple[float, np.ndarray]:
    """Read the passive fingertip poses without assuming linear finger joints."""
    from pxr import UsdGeom

    spec = config["gripper"]
    stage = handles.world.stage
    cache = UsdGeom.XformCache()
    points = []
    for key in ("body_prim", "left_pad_prim", "right_pad_prim"):
        prim = stage.GetPrimAtPath(spec[key])
        if not prim.IsValid() or not prim.IsActive():
            raise RuntimeError(f"Missing active Robotiq prim: {spec[key]}")
        points.append(cache.GetLocalToWorldTransform(prim))
    body, left, right = points
    lateral = np.asarray([body[1][i] for i in range(3)], dtype=np.float64)
    left_xyz = np.asarray([left[3][i] for i in range(3)], dtype=np.float64)
    right_xyz = np.asarray([right[3][i] for i in range(3)], dtype=np.float64)
    return opening_from_pad_origins(
        left_xyz, right_xyz, lateral, float(spec["open_width_m"])
    )


@dataclass
class RobotiqGripperController:
    """Drive the one 2F-85 joint only when binary policy intent changes."""

    intent: str = "OPEN"

    def request(self, handles: Any, config: dict, policy_value: float) -> bool:
        decision = droid_gripper_decision(policy_value)
        new_intent = "CLOSED" if decision.binary_closure else "OPEN"
        if new_intent == self.intent:
            return False
        spec = config["gripper"]
        target = (
            spec["closed_joint_rad"] if new_intent == "CLOSED"
            else spec["open_joint_rad"]
        )
        handles.fr3.set_dof_position_targets(
            np.asarray([[target]], dtype=np.float32),
            dof_indices=handles.finger_indices,
        )
        self.intent = new_intent
        return True
