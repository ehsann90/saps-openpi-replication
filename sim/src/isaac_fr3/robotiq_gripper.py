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
    target_joint_rad: float | None = None
    transition_steps: int = 0
    faults: tuple[str, ...] = ()

    def read(self, handles: Any, config: dict) -> dict:
        """Report the driver in radians and opening from the moving pads."""
        positions = handles.fr3.get_dof_positions()
        positions = np.asarray(
            positions.numpy() if hasattr(positions, "numpy") else positions
        ).reshape(-1)
        indices = handles.finger_indices
        indices = np.asarray(
            indices.numpy() if hasattr(indices, "numpy") else indices
        ).astype(int).reshape(-1)
        if len(indices) != 1 or indices[0] < 0 or indices[0] >= len(positions):
            self.faults += ("Robotiq must have exactly one driven joint",)
            raise RuntimeError("Robotiq must have exactly one driven joint")
        driver = float(positions[indices[0]])
        if not np.isfinite(driver):
            self.faults += ("Robotiq driver position is non-finite",)
            raise RuntimeError("Robotiq driver position is non-finite")
        try:
            width, scalar = measured_opening(handles, config)
        except (ValueError, RuntimeError) as error:
            self.faults += (f"Robotiq pad measurement failed: {error}",)
            raise
        return {
            "positions": np.asarray([driver], dtype=np.float64),
            "width_m": width,
            "scalar": scalar,
            "driver_position_rad": driver,
        }

    def request_intent(self, policy_value: float, width_m: float) -> dict:
        """Persist intent across actions; a duplicate does not restart motion."""
        decision = droid_gripper_decision(policy_value)
        new_intent = "CLOSED" if decision.binary_closure else "OPEN"
        previous = self.intent
        transition = new_intent != previous
        if transition:
            self.intent = new_intent
            self.transition_steps = 0
        return {"from": previous, "to": new_intent,
                "transition": transition}

    def apply_transition(self, handles: Any, config: dict) -> None:
        """Set one joint target once, after arm-target safety validation."""
        spec = config["gripper"]
        target = float(spec[
            "closed_joint_rad" if self.intent == "CLOSED" else "open_joint_rad"
        ])
        handles.fr3.set_dof_position_targets(
            np.asarray([[target]], dtype=np.float32),
            dof_indices=handles.finger_indices,
        )
        self.target_joint_rad = target

    def advance_step(self, handles: Any, config: dict,
                     physics_dt: float) -> None:
        """Keep the existing drive active and check for a lost target."""
        if self.target_joint_rad is None:
            return
        targets = handles.fr3.get_dof_position_targets(
            dof_indices=handles.finger_indices
        )
        targets = np.asarray(
            targets.numpy() if hasattr(targets, "numpy") else targets
        ).reshape(-1)
        if (targets.shape != (1,) or not np.isfinite(targets).all()
                or not np.isclose(targets[0], self.target_joint_rad,
                                  rtol=0, atol=1e-5)):
            self.faults += ("Robotiq active driver target changed",)
            raise RuntimeError(self.faults[-1])
        self.transition_steps += 1

    def freeze(self, handles: Any, config: dict) -> dict:
        """Hold measured driver position after a rejected arm target."""
        state = self.read(handles, config)
        driver = state["driver_position_rad"]
        handles.fr3.set_dof_position_targets(
            np.asarray([[driver]], dtype=np.float32),
            dof_indices=handles.finger_indices,
        )
        self.target_joint_rad = driver
        return {"gripper_driver_frozen_rad": driver,
                "gripper_width_frozen_m": state["width_m"]}

    def report(self, handles: Any, config: dict) -> dict:
        try:
            state = self.read(handles, config)
        except (ValueError, RuntimeError):
            return {
                "intent": self.intent, "driver_position_rad": None,
                "measured_width_m": None, "normalized_observation": None,
                "target_joint_rad": self.target_joint_rad,
                "transition_active": None,
                "transition_steps": self.transition_steps,
                "faults": list(self.faults),
            }
        target = self.target_joint_rad
        return {
            "intent": self.intent,
            "driver_position_rad": state["driver_position_rad"],
            "measured_width_m": state["width_m"],
            "normalized_observation": float(state["scalar"][0]),
            "target_joint_rad": target,
            "transition_active": target is not None and abs(
                state["driver_position_rad"] - target
            ) > 0.005,
            "transition_steps": self.transition_steps,
            "faults": list(self.faults),
        }

    def request(self, handles: Any, config: dict, policy_value: float) -> bool:
        """Retain the SIM-P7 manual-validation command entry point."""
        result = self.request_intent(policy_value, 0.0)
        if result["transition"]:
            self.apply_transition(handles, config)
        return result["transition"]
