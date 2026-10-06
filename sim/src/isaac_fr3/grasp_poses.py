"""Open-gripper approach targets matching SIM-P2 grasp validation."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.robot_motion.motion_generation.lula.kinematics import (
    LulaKinematicsSolver,
)

from .scene import SceneHandles, get_tcp_pose, settle


PREGRASP_HEIGHT_M = 0.10
POSE_STEPS = 180
MAX_POSITION_ERROR_M = 0.012
MAX_ORIENTATION_ERROR_RAD = 0.08


def make_solver(config: dict) -> LulaKinematicsSolver:
    """Load the same bundled FR3 Lula assets used by SIM-P2."""
    extension = get_extension_path_from_name(
        "isaacsim.robot_motion.motion_generation"
    )
    if extension is None:
        raise RuntimeError("Isaac's bundled FR3 Lula configuration is unavailable")
    root = Path(extension) / "motion_policy_configs" / "FR3"
    description = root / "rmpflow" / "fr3_robot_description.yaml"
    urdf = root / "fr3.urdf"
    if not description.is_file() or not urdf.is_file():
        raise RuntimeError(f"Missing bundled FR3 IK assets: {description}, {urdf}")
    solver = LulaKinematicsSolver(str(description), str(urdf))
    robot_cfg = config["robot"]
    if solver.get_joint_names() != robot_cfg["arm_dof_names"]:
        raise RuntimeError("Bundled FR3 IK joint order differs from scene config")
    if "fr3_hand_tcp" not in solver.get_all_frame_names():
        raise RuntimeError("Bundled FR3 IK lacks the configured TCP frame")
    return solver


def tip_center(stage: Usd.Stage, finger_path: str) -> np.ndarray:
    """Find the lowest enabled tip collider center as in SIM-P2."""
    root = stage.GetPrimAtPath(finger_path)
    if not root.IsValid():
        raise RuntimeError(f"Missing finger prim: {finger_path}")
    centers = []
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        cube = UsdGeom.Cube(prim)
        if not cube:
            raise RuntimeError(f"Expected a Cube collider at {prim.GetPath()}")
        half = float(cube.GetSizeAttr().Get()) / 2.0
        transform = cache.GetLocalToWorldTransform(prim)
        corners = np.array([
            list(transform.Transform(Gf.Vec3d(x, y, z)))
            for x in (-half, half)
            for y in (-half, half)
            for z in (-half, half)
        ])
        centers.append(corners.mean(axis=0))
    if not centers:
        raise RuntimeError(f"No enabled colliders beneath {finger_path}")
    return min(centers, key=lambda center: center[2])


def approach_targets(
    handles: SceneHandles, config: dict
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    """Use SIM-P2's tip-midpoint offset and settled cube to derive targets."""
    robot_cfg = config["robot"]
    left = tip_center(handles.world.stage, robot_cfg["left_finger_prim"])
    right = tip_center(handles.world.stage, robot_cfg["right_finger_prim"])
    home_tcp, home_orientation = get_tcp_pose(config)
    home_tcp = np.asarray(home_tcp).reshape(3)
    home_orientation = np.asarray(home_orientation).reshape(4)
    cube, _ = handles.target.get_world_pose()
    cube = np.asarray(cube).reshape(3)
    tip_midpoint = 0.5 * (left + right)
    grasp_target = cube - (tip_midpoint - home_tcp)
    pregrasp_target = grasp_target + np.array([0.0, 0.0, PREGRASP_HEIGHT_M])
    geometry = {
        "home_tip_midpoint_world_m": tip_midpoint.tolist(),
        "home_left_tip_center_world_m": left.tolist(),
        "home_right_tip_center_world_m": right.tolist(),
    }
    return pregrasp_target, grasp_target, home_orientation, geometry


def move_to_tcp_pose(
    handles: SceneHandles,
    config: dict,
    solver: LulaKinematicsSolver,
    target: np.ndarray,
    orientation: np.ndarray,
) -> None:
    """Command the same validated IK solution and 180-step settling rule."""
    current = np.asarray(handles.fr3.get_dof_positions()).reshape(-1)
    joints, solved = solver.compute_inverse_kinematics(
        "fr3_hand_tcp", target, orientation, current[handles.arm_indices]
    )
    lower, upper = solver.get_cspace_position_limits()
    if not (solved and np.all(np.isfinite(joints))
            and np.all(joints >= lower) and np.all(joints <= upper)):
        raise RuntimeError("FR3 IK failed or violated joint limits")
    handles.fr3.set_dof_position_targets(
        np.asarray([joints], dtype=np.float32),
        dof_indices=handles.arm_indices,
    )
    settle(handles, POSE_STEPS, render=True)
    measured_position, measured_orientation = get_tcp_pose(config)
    measured_position = np.asarray(measured_position).reshape(3)
    measured_orientation = np.asarray(measured_orientation).reshape(4)
    position_error = float(np.linalg.norm(measured_position - target))
    dot = abs(float(np.dot(measured_orientation, orientation)))
    orientation_error = float(2.0 * np.arccos(np.clip(dot, 0.0, 1.0)))
    if (position_error > MAX_POSITION_ERROR_M
            or orientation_error > MAX_ORIENTATION_ERROR_RAD):
        raise RuntimeError(
            f"FR3 pose error: {position_error:.6f} m, "
            f"{orientation_error:.6f} rad"
        )
