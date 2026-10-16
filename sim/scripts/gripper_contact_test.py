#!/usr/bin/env python3
"""Validate native FR3 approach, grasp contact, lift, and release in Isaac Sim."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--close-mode", choices=("abrupt", "rate-limited"),
        default="rate-limited",
        help="Use the prior baseline or the physical hand's speed-limited command.",
    )
    return parser.parse_args()


args = parse_args()
simulation_app = SimulationApp({"headless": args.headless})

import numpy as np
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics

from omni.physics.core import ContactEventType, get_physics_simulation_interface

from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.robot_motion.motion_generation.lula.kinematics import (
    LulaKinematicsSolver,
)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from isaac_fr3.scene import (
    CUBE_SETTLE_STEPS,
    HOME_SETTLE_STEPS,
    SceneHandles,
    command_home,
    create_scene,
    get_tcp_pose,
    load_config,
    settle,
)


# All distances are metres. These are validation thresholds, not physics tuning.
PREGRASP_HEIGHT = 0.10
LIFT_HEIGHT = 0.05
TABLE_CLEARANCE = 0.003
POSE_STEPS = 180
CLOSE_STEPS = 180
RELEASE_STEPS = 120
MAX_POSITION_ERROR = 0.012
MAX_ORIENTATION_ERROR_RAD = 0.08
MAX_CONTACT_SEPARATION_PENETRATION = 0.001
MAX_CLOSE_PENETRATION = 0.002
TOTAL_WIDTH_SPEED_M_S = 0.1
GRASP_FORCE_N = 20.0


class ContactReporter:
    """Track PhysX finger/cube contact events at each physics step."""

    def __init__(self, stage, robot_cfg: dict):
        self.finger_paths = {
            "left": robot_cfg["left_finger_prim"],
            "right": robot_cfg["right_finger_prim"],
        }
        self.cube_path = "/World/TargetCube"
        for path in (*self.finger_paths.values(), self.cube_path):
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"Contact-report body is missing: {path}")
            api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            api.CreateThresholdAttr().Set(0.0)
            print(f"Contact report enabled on {path}")
        self.active = {"left": set(), "right": set()}
        self.loaded = {"left": False, "right": False}
        self.first_width = {"left": None, "right": None}
        self.step = 0
        self.events = []
        self.current_events = []
        self.close_widths = []
        self.subscription = (
            get_physics_simulation_interface()
            .subscribe_physics_contact_report_events(self._on_events)
        )
        if self.subscription is None:
            raise RuntimeError("PhysX contact-report subscription failed")

    def _on_events(self, headers, contacts, _friction_anchors) -> None:
        for header in headers:
            path0 = str(PhysicsSchemaTools.intToSdfPath(header.collider0))
            path1 = str(PhysicsSchemaTools.intToSdfPath(header.collider1))
            sides = [
                side for side, path in self.finger_paths.items()
                if (path0.startswith(path + "/") and path1.startswith(self.cube_path))
                or (path1.startswith(path + "/") and path0.startswith(self.cube_path))
            ]
            for side in sides:
                pair = tuple(sorted((path0, path1)))
                data = contacts[
                    header.contact_data_offset:
                    header.contact_data_offset + header.num_contact_data
                ]
                separations = [float(item.separation) for item in data]
                impulses = [
                    float(np.linalg.norm([
                        item.impulse.x, item.impulse.y, item.impulse.z
                    ]))
                    for item in data
                ]
                loaded = [i for i, impulse in enumerate(impulses)
                          if impulse > 1e-6]
                event = {
                    "step": self.step,
                    "side": side,
                    "pair": pair,
                    "type": header.type,
                    "count": len(loaded),
                    "min_separation": min(separations, default=None),
                    "loaded_min_separation": min(
                        (separations[i] for i in loaded), default=None
                    ),
                    "total_impulse": sum(impulses),
                }
                self.events.append(event)
                if header.type == ContactEventType.CONTACT_LOST:
                    self.active[side].discard(pair)
                else:
                    self.active[side].add(pair)

    def report_step(
        self, handles: SceneHandles, target_width: float, phase: str
    ) -> None:
        width = float(np.sum(finger_positions(handles)))
        if phase == "CLOSE":
            self.close_widths.append(width)
        cube, _ = cube_pose(handles)
        events = [event for event in self.events if event["step"] == self.step]
        self.current_events = events
        elapsed = self.step * SimulationManager.get_physics_dt()
        for side in self.finger_paths:
            self.loaded[side] = any(
                event["side"] == side and event["count"] > 0
                for event in events
            )
            if self.loaded[side] and self.first_width[side] is None:
                self.first_width[side] = width
                print(f"First {side} contact: step={self.step}, "
                      f"time={elapsed:.3f} s, "
                      f"measured width={width:.6f} m")
        if self.step % 15 == 0 or self.step == 1:
            print(
                f"{phase} step {self.step} at {elapsed:.3f} s: "
                f"target width={target_width:.6f} m, "
                f"measured width={width:.6f} m, cube={cube}, "
                f"left/right contact="
                f"{self.loaded['left']}/{self.loaded['right']}, "
                f"contact count={sum(event['count'] for event in events)}"
            )
            for event in events:
                if event["count"]:
                    print(f"  PhysX contact {event}")

    def bilateral(self) -> bool:
        return bool(self.loaded["left"] and self.loaded["right"])

    def min_loaded_separation(self) -> float | None:
        values = [
            event["loaded_min_separation"]
            for event in self.current_events if event["count"]
        ]
        return min(values) if values else None


def print_finger_drives(stage, handles: SceneHandles, robot_cfg: dict) -> None:
    print("=== NATIVE FINGER DRIVES ===")
    for name in robot_cfg["finger_dof_names"]:
        paths = [
            prim for prim in stage.Traverse() if prim.GetName() == name
        ]
        if len(paths) != 1:
            raise RuntimeError(f"Expected one USD finger joint named {name}")
        prim = paths[0]
        drive = UsdPhysics.DriveAPI.Get(prim, "linear")
        joint = UsdPhysics.PrismaticJoint(prim)
        physx = PhysxSchema.PhysxJointAPI(prim)
        print(f"{name} prim: {prim.GetPath()}")
        print(f"  drive type: {drive.GetTypeAttr().Get()}")
        print(f"  stiffness: {drive.GetStiffnessAttr().Get()}")
        print(f"  damping: {drive.GetDampingAttr().Get()}")
        print(f"  maximum force: {drive.GetMaxForceAttr().Get()}")
        print(f"  position target: {drive.GetTargetPositionAttr().Get()}")
        print(f"  joint limits: "
              f"{joint.GetLowerLimitAttr().Get()}, "
              f"{joint.GetUpperLimitAttr().Get()}")
        print(f"  PhysX max joint velocity: "
              f"{physx.GetMaxJointVelocityAttr().Get()}")
        print(f"  applied schemas: {prim.GetAppliedSchemas()}")
        for attribute in prim.GetAttributes():
            if "MimicJoint" in attribute.GetName():
                print(f"  {attribute.GetName()}: {attribute.Get()}")
    indices = array(handles.finger_indices).astype(int).reshape(-1)
    print(f"Runtime max efforts: "
          f"{array(handles.fr3.get_dof_max_efforts()).reshape(-1)[indices]}")
    print(f"Runtime max velocities: "
          f"{array(handles.fr3.get_dof_max_velocities()).reshape(-1)[indices]}")


def print_contact_offsets(stage, robot_cfg: dict) -> None:
    print("=== NATIVE CONTACT OFFSETS ===")
    for path in (
        robot_cfg["left_finger_prim"] + "/collisions/mesh_3",
        robot_cfg["right_finger_prim"] + "/collisions/mesh_3",
        "/World/TargetCube",
    ):
        prim = stage.GetPrimAtPath(path)
        api = PhysxSchema.PhysxCollisionAPI(prim)
        print(
            f"{path}: contact offset="
            f"{api.GetContactOffsetAttr().Get()}, rest offset="
            f"{api.GetRestOffsetAttr().Get()}"
        )


def array(value) -> np.ndarray:
    return np.asarray(value.numpy() if hasattr(value, "numpy") else value)


def pose(prim: XformPrim) -> tuple[np.ndarray, np.ndarray]:
    position, orientation = prim.get_world_poses()
    return array(position).reshape(-1, 3)[0], array(orientation).reshape(-1, 4)[0]


def cube_pose(handles: SceneHandles) -> tuple[np.ndarray, np.ndarray]:
    position, orientation = handles.target.get_world_pose()
    return array(position).reshape(3), array(orientation).reshape(4)


def arm_positions(handles: SceneHandles) -> np.ndarray:
    indices = array(handles.arm_indices).astype(int).reshape(-1)
    return array(handles.fr3.get_dof_positions()).reshape(-1)[indices]


def finger_positions(handles: SceneHandles) -> np.ndarray:
    indices = array(handles.finger_indices).astype(int).reshape(-1)
    return array(handles.fr3.get_dof_positions()).reshape(-1)[indices]


def angle_error(actual: np.ndarray, desired: np.ndarray) -> float:
    dot = abs(float(np.dot(actual, desired)))
    return float(2 * np.arccos(np.clip(dot, 0.0, 1.0)))


def make_solver() -> LulaKinematicsSolver:
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
    print(f"FR3 IK robot description: {description}")
    print(f"FR3 IK URDF: {urdf}")
    return LulaKinematicsSolver(str(description), str(urdf))


def collider_corners(prim: Usd.Prim) -> np.ndarray:
    """Transform the eight corners of a collision Cube; invisible USD bounds are empty."""
    cube = UsdGeom.Cube(prim)
    if not cube:
        raise RuntimeError(f"Expected a Cube collider at {prim.GetPath()}")
    half = float(cube.GetSizeAttr().Get()) / 2
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
        prim
    )
    return np.array(
        [
            array(matrix.Transform(Gf.Vec3d(x, y, z)))
            for x in (-half, half)
            for y in (-half, half)
            for z in (-half, half)
        ]
    )


def finger_geometry(stage, path: str) -> list[dict]:
    root = stage.GetPrimAtPath(path)
    if not root.IsValid():
        raise RuntimeError(f"Missing finger prim: {path}")
    colliders = []
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        corners = collider_corners(prim)
        info = {
            "path": str(prim.GetPath()),
            "corners": corners,
            "center": corners.mean(axis=0),
            "minimum": corners.min(axis=0),
            "maximum": corners.max(axis=0),
        }
        colliders.append(info)
    if not colliders:
        raise RuntimeError(f"No enabled colliders beneath {path}")
    return colliders


def tip_geometry(stage, robot_cfg: dict, *, report: bool) -> tuple[dict, dict, np.ndarray, float]:
    left = finger_geometry(stage, robot_cfg["left_finger_prim"])
    right = finger_geometry(stage, robot_cfg["right_finger_prim"])
    if report:
        for side, colliders in (("left", left), ("right", right)):
            for item in colliders:
                print(
                    f"{side} collider {item['path']}: "
                    f"world AABB {item['minimum']} .. {item['maximum']}"
                )
    left_tip = min(left, key=lambda item: item["center"][2])
    right_tip = min(right, key=lambda item: item["center"][2])
    axis = left_tip["center"] - right_tip["center"]
    axis /= np.linalg.norm(axis)
    left_inner = np.min(left_tip["corners"] @ axis)
    right_inner = np.max(right_tip["corners"] @ axis)
    gap = float(left_inner - right_inner)
    if report:
        print(f"Opposing tip colliders: {left_tip['path']}, {right_tip['path']}")
        print(f"Closing axis right-to-left: {axis}")
        print(f"Tip contact-region midpoint: "
              f"{0.5 * (left_tip['center'] + right_tip['center'])}")
        print(f"Open collision-surface gap [m]: {gap:.6f}")
    return left_tip, right_tip, axis, gap


def cube_half_span(
    axis: np.ndarray, size: np.ndarray, orientation: np.ndarray
) -> float:
    """Project the oriented cube's half extents onto the closing axis."""
    w, x, y, z = orientation
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
             2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
             2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x),
             1 - 2 * (x * x + y * y)],
        ]
    )
    return float(0.5 * np.sum(np.abs(axis @ rotation) * size))


def move_to_pose(
    handles: SceneHandles,
    config: dict,
    solver: LulaKinematicsSolver,
    label: str,
    desired_position: np.ndarray,
    desired_orientation: np.ndarray,
    reporter: ContactReporter | None = None,
) -> bool:
    print(f"Desired TCP position [m]: {desired_position}")
    print(f"Desired TCP orientation [w,x,y,z]: {desired_orientation}")
    joints, solved = solver.compute_inverse_kinematics(
        "fr3_hand_tcp",
        desired_position,
        desired_orientation,
        arm_positions(handles),
    )
    print(f"{label} IK success: {solved}")
    print(f"{label} solved q[7] [rad]: {joints}")
    lower, upper = solver.get_cspace_position_limits()
    valid = bool(
        solved
        and np.all(np.isfinite(joints))
        and np.all(joints >= lower)
        and np.all(joints <= upper)
    )
    print(f"{label} joint limits valid: {valid}")
    if not valid:
        return False
    handles.fr3.set_dof_position_targets(
        np.asarray([joints], dtype=np.float32),
        dof_indices=handles.arm_indices,
    )
    if reporter is None:
        settle(handles, POSE_STEPS, render=not args.headless)
    else:
        for _ in range(POSE_STEPS):
            reporter.step += 1
            handles.world.step(render=not args.headless)
            reporter.report_step(handles, 0.0, label)
    measured_position, measured_orientation = get_tcp_pose(config)
    measured_position = array(measured_position).reshape(3)
    measured_orientation = array(measured_orientation).reshape(4)
    position_error = float(np.linalg.norm(measured_position - desired_position))
    orientation_error = angle_error(measured_orientation, desired_orientation)
    print(f"Measured TCP position [m]: {measured_position}")
    print(f"Measured TCP orientation [w,x,y,z]: {measured_orientation}")
    print(f"TCP position error [m]: {position_error:.6f}")
    print(f"TCP orientation error [rad]: {orientation_error:.6f}")
    return (
        position_error <= MAX_POSITION_ERROR
        and orientation_error <= MAX_ORIENTATION_ERROR_RAD
    )


def run() -> dict[str, bool]:
    flags = dict.fromkeys(
        ("PRE_CLOSE_GEOMETRY", "CONTACT_LIMITED_CLOSE", "RETENTION", "RELEASE"),
        False,
    )
    config = load_config(args.config)
    robot_cfg = config["robot"]
    cube_size = np.asarray(config["target_object"]["size_m"], dtype=float)
    table_top = (
        config["table"]["position_m"][2]
        + 0.5 * config["table"]["size_m"][2]
    )

    print("=== RESET ===")
    handles = create_scene(config)
    print_finger_drives(handles.world.stage, handles, robot_cfg)
    print_contact_offsets(handles.world.stage, robot_cfg)
    contacts = ContactReporter(handles.world.stage, robot_cfg)
    command_home(handles, config)
    settle(
        handles, HOME_SETTLE_STEPS + CUBE_SETTLE_STEPS,
        render=not args.headless,
    )
    print(f"Home measured arm [rad]: {arm_positions(handles)}")
    print(f"Home measured fingers [m]: {finger_positions(handles)}")
    cube_start, _ = cube_pose(handles)
    print(f"Settled cube position [m]: {cube_start}")

    print("=== COLLISION GEOMETRY ===")
    stage = handles.world.stage
    left_tip, right_tip, axis, open_gap = tip_geometry(
        stage, robot_cfg, report=True
    )
    if not np.isfinite(open_gap) or open_gap <= 0:
        return flags
    home_tcp_position, home_orientation = get_tcp_pose(config)
    home_tcp_position = array(home_tcp_position).reshape(3)
    home_orientation = array(home_orientation).reshape(4)
    tip_midpoint = 0.5 * (left_tip["center"] + right_tip["center"])
    tip_offset = tip_midpoint - home_tcp_position
    grasp_target = cube_start - tip_offset
    pregrasp_target = grasp_target + np.array([0.0, 0.0, PREGRASP_HEIGHT])
    predicted_tip_bottom = min(
        left_tip["minimum"][2], right_tip["minimum"][2]
    ) + grasp_target[2] - home_tcp_position[2]
    print(f"PREGRASP vertical clearance above grasp [m]: {PREGRASP_HEIGHT}")
    print(f"GRASP predicted tip-table clearance [m]: "
          f"{predicted_tip_bottom - table_top:.6f}")
    print(f"Required tip-table clearance [m]: {TABLE_CLEARANCE}")
    if predicted_tip_bottom <= table_top + TABLE_CLEARANCE:
        print("Predicted finger-table clearance is insufficient")
        return flags

    solver = make_solver()
    if solver.get_joint_names() != robot_cfg["arm_dof_names"]:
        raise RuntimeError("Bundled FR3 IK joint order differs from scene config")
    if "fr3_hand_tcp" not in solver.get_all_frame_names():
        raise RuntimeError("Bundled FR3 IK lacks the configured TCP frame")

    print("=== PREGRASP ===")
    if not move_to_pose(
        handles, config, solver, "PREGRASP", pregrasp_target, home_orientation
    ):
        print("PREGRASP positioning failed; no descent or close requested")
        return flags

    print("=== GRASP POSE ===")
    if not move_to_pose(
        handles, config, solver, "GRASP", grasp_target, home_orientation
    ):
        print("GRASP positioning failed; no close requested")
        return flags
    cube_before, cube_orientation = cube_pose(handles)
    tcp_before, _ = get_tcp_pose(config)
    tcp_before = array(tcp_before).reshape(3)
    left_pose = pose(XformPrim(robot_cfg["left_finger_prim"]))[0]
    right_pose = pose(XformPrim(robot_cfg["right_finger_prim"]))[0]
    left_tip, right_tip, axis, gap_before = tip_geometry(
        stage, robot_cfg, report=False
    )
    tip_midpoint = 0.5 * (left_tip["center"] + right_tip["center"])
    cube_span = 2 * cube_half_span(axis, cube_size, cube_orientation)
    left_inner = np.min(left_tip["corners"] @ axis)
    right_inner = np.max(right_tip["corners"] @ axis)
    cube_coordinate = float(cube_before @ axis)
    margin_left = left_inner - (cube_coordinate + cube_span / 2)
    margin_right = (cube_coordinate - cube_span / 2) - right_inner
    tip_z_min = max(left_tip["minimum"][2], right_tip["minimum"][2])
    tip_z_max = min(left_tip["maximum"][2], right_tip["maximum"][2])
    cube_z_min = cube_before[2] - cube_size[2] / 2
    cube_z_max = cube_before[2] + cube_size[2] / 2
    print(f"Cube world pose: {cube_before}, {cube_orientation}")
    print(f"TCP world position [m]: {tcp_before}")
    print(f"Left/right finger world positions [m]: {left_pose}, {right_pose}")
    print(f"Open finger collision gap [m]: {gap_before:.6f}")
    print(f"Cube relative to tip midpoint [m]: {cube_before - tip_midpoint}")
    print(f"Cube projected width along closing axis [m]: {cube_span:.6f}")
    print(f"Cube-to-left/right inner surface margins [m]: "
          f"{margin_left:.6f}, {margin_right:.6f}")
    print(f"Tip/cube vertical overlap [m]: "
          f"{min(tip_z_max, cube_z_max) - max(tip_z_min, cube_z_min):.6f}")
    preclose_ok = bool(
        np.all(np.isfinite(cube_before))
        and margin_left > 0.002
        and margin_right > 0.002
        and gap_before > cube_span
        and tip_z_min < cube_z_max
        and tip_z_max > cube_z_min
        and min(left_tip["minimum"][2], right_tip["minimum"][2])
        > table_top + TABLE_CLEARANCE
        and np.linalg.norm(cube_before[:2] - cube_start[:2]) < 0.01
    )
    flags["PRE_CLOSE_GEOMETRY"] = preclose_ok
    print(f"PRE_CLOSE_GEOMETRY: {'PASS' if preclose_ok else 'FAIL'}")
    if not preclose_ok:
        return flags

    print("=== CLOSE ===")
    width_before = float(np.sum(finger_positions(handles)))
    dt = SimulationManager.get_physics_dt()
    print(f"CLOSE mode: {args.close_mode}")
    print(f"Physics timestep [s]: {dt}")
    print(f"Reference total-width closing speed [m/s]: "
          f"{TOTAL_WIDTH_SPEED_M_S}")
    print(f"Per-finger target speed [m/s]: "
          f"{TOTAL_WIDTH_SPEED_M_S / 2}")
    driven_index = int(array(handles.finger_indices).reshape(-1)[0])
    native_effort = float(
        array(handles.fr3.get_dof_max_efforts()).reshape(-1)[driven_index]
    )
    if args.close_mode == "rate-limited":
        print(f"BASELINE VALUE active finger drive max effort [N]: "
              f"{native_effort}")
        print(f"NEW VALUE active finger drive max effort [N]: "
              f"{GRASP_FORCE_N}")
        print("SOURCE / JUSTIFICATION: physical G1B Franka Grasp command "
              "uses 20 N; right finger is a native mimic joint")
        handles.fr3.set_dof_max_efforts(
            np.asarray([[GRASP_FORCE_N]], dtype=np.float32),
            dof_indices=[driven_index],
        )
    print(f"Applied active finger max effort [N]: "
          f"{array(handles.fr3.get_dof_max_efforts()).reshape(-1)[driven_index]}")
    for step in range(1, CLOSE_STEPS + 1):
        target_width = (
            0.0 if args.close_mode == "abrupt"
            else max(0.0, width_before - TOTAL_WIDTH_SPEED_M_S * step * dt)
        )
        handles.fr3.set_dof_position_targets(
            np.full((1, 2), target_width / 2, dtype=np.float32),
            dof_indices=handles.finger_indices,
        )
        contacts.step = step
        handles.world.step(render=not args.headless)
        contacts.report_step(handles, target_width, "CLOSE")
    width_after = float(np.sum(finger_positions(handles)))
    cube_closed, cube_closed_orientation = cube_pose(handles)
    left_tip, right_tip, axis, closed_gap = tip_geometry(
        stage, robot_cfg, report=False
    )
    cube_span = 2 * cube_half_span(
        axis, cube_size, cube_closed_orientation
    )
    cube_center_projection = float(cube_closed @ axis)
    left_inner = float(np.min(left_tip["corners"] @ axis))
    right_inner = float(np.max(right_tip["corners"] @ axis))
    penetration = cube_span - closed_gap
    observation = float(np.clip(1 - width_after / 0.08, 0.0, 1.0))
    print(f"Finger width before/after CLOSE [m]: "
          f"{width_before:.6f}, {width_after:.6f}")
    print(f"Closed collision-surface gap [m]: {closed_gap:.6f}")
    print(f"Cube projected width [m]: {cube_span:.6f}")
    print(f"Estimated surface penetration [m]: {penetration:.6f}")
    print(f"Cube after CLOSE [m]: {cube_closed}")
    print(f"Cube orientation after CLOSE [w,x,y,z]: "
          f"{cube_closed_orientation}")
    print(f"DROID gripper observation after CLOSE: {observation:.6f}")
    print(f"First left/right contact widths [m]: "
          f"{contacts.first_width['left']}, {contacts.first_width['right']}")
    print(f"Final bilateral PhysX contact: {contacts.bilateral()}")
    final_separation = contacts.min_loaded_separation()
    print(f"Final minimum loaded PhysX separation [m]: "
          f"{final_separation}")
    print("Contact acceptance: <=1 mm negative PhysX separation per "
          "contact and <=2 mm combined projected overlap; both limits "
          "are small relative to the 40 mm cube and its zero rest offset")
    close_width_range = float(np.ptp(contacts.close_widths[-30:]))
    print(f"Measured width range over final 30 CLOSE steps [m]: "
          f"{close_width_range:.6f}")
    close_ok = bool(
        np.all(np.isfinite(cube_closed))
        and width_before >= 0.075
        and width_after < width_before - 0.005
        and width_after > 0.02
        and abs(closed_gap - cube_span) <= 0.012
        and penetration <= MAX_CLOSE_PENETRATION
        and contacts.bilateral()
        and all(value is not None for value in contacts.first_width.values())
        and close_width_range < 0.001
        and final_separation is not None
        and final_separation >= -MAX_CONTACT_SEPARATION_PENETRATION
        and right_inner < cube_center_projection < left_inner
        and np.linalg.norm(cube_closed - cube_before) < 0.02
    )
    flags["CONTACT_LIMITED_CLOSE"] = close_ok
    print(f"CONTACT_LIMITED_CLOSE: {'PASS' if close_ok else 'FAIL'}")
    if not close_ok:
        print("Native grasp baseline failed; no lift or physics tuning requested")
        return flags

    print("=== LIFT ===")
    tcp_lift_start = array(get_tcp_pose(config)[0]).reshape(3)
    cube_lift_start, _ = cube_pose(handles)
    width_lift_start = float(np.sum(finger_positions(handles)))
    lift_target = grasp_target + np.array([0.0, 0.0, LIFT_HEIGHT])
    lift_positioned = move_to_pose(
        handles, config, solver, "LIFT", lift_target, home_orientation,
        reporter=contacts,
    )
    tcp_lift_end = array(get_tcp_pose(config)[0]).reshape(3)
    cube_lift_end, _ = cube_pose(handles)
    width_lift_end = float(np.sum(finger_positions(handles)))
    delta_tcp = tcp_lift_end - tcp_lift_start
    delta_cube = cube_lift_end - cube_lift_start
    print(f"TCP before/after lift [m]: {tcp_lift_start}, {tcp_lift_end}")
    print(f"TCP lift displacement [m]: {delta_tcp}")
    print(f"Cube before/after lift [m]: {cube_lift_start}, {cube_lift_end}")
    print(f"Cube lift displacement [m]: {delta_cube}")
    print(f"Finger width before/after lift [m]: "
          f"{width_lift_start:.6f}, {width_lift_end:.6f}")
    print(f"Bilateral loaded contact after lift: {contacts.bilateral()}")
    print(f"Minimum loaded separation after lift [m]: "
          f"{contacts.min_loaded_separation()}")
    retained = bool(
        lift_positioned
        and np.all(np.isfinite(cube_lift_end))
        and delta_tcp[2] > 0.035
        and delta_cube[2] > 0.025
        and cube_lift_end[2] > table_top + cube_size[2] / 2 + 0.015
        and abs(delta_cube[2] - delta_tcp[2]) < 0.02
        and np.linalg.norm(delta_cube[:2] - delta_tcp[:2]) < 0.025
        and width_lift_end > 0.02
        and contacts.bilateral()
    )
    flags["RETENTION"] = retained
    print(f"RETENTION: {'PASS' if retained else 'FAIL'}")
    if not retained:
        return flags

    print("=== RELEASE ===")
    open_width = float(sum(robot_cfg["home_fingers_m"]))
    opening_steps = int(np.ceil(
        (open_width - width_lift_end) / (TOTAL_WIDTH_SPEED_M_S * dt)
    ))
    for step in range(1, opening_steps + 1):
        target_width = min(
            open_width, width_lift_end + TOTAL_WIDTH_SPEED_M_S * step * dt
        )
        handles.fr3.set_dof_position_targets(
            np.full((1, 2), target_width / 2, dtype=np.float32),
            dof_indices=handles.finger_indices,
        )
        contacts.step += 1
        handles.world.step(render=not args.headless)
        contacts.report_step(handles, target_width, "OPEN")
        if step == 1 or step % 15 == 0 or step == opening_steps:
            print(f"OPEN step {step}: target width={target_width:.6f} m, "
                  f"measured width={np.sum(finger_positions(handles)):.6f} m")
    cube_just_opened, _ = cube_pose(handles)
    for _ in range(RELEASE_STEPS):
        contacts.step += 1
        handles.world.step(render=not args.headless)
        contacts.report_step(handles, open_width, "RELEASE")
    cube_released, _ = cube_pose(handles)
    width_released = float(np.sum(finger_positions(handles)))
    print(f"Finger positions after OPEN [m]: {finger_positions(handles)}")
    print(f"Cube immediately after OPEN [m]: {cube_just_opened}")
    print(f"Cube after settling [m]: {cube_released}")
    print(f"Loaded finger-cube contact after release: "
          f"{contacts.loaded['left']}, {contacts.loaded['right']}")
    released = bool(
        np.all(np.isfinite(cube_released))
        and width_released > 0.075
        and cube_released[2] < cube_lift_end[2] - 0.025
        and cube_released[2] <= table_top + cube_size[2] / 2 + 0.01
        and not any(contacts.loaded.values())
    )
    flags["RELEASE"] = released
    print(f"RELEASE: {'PASS' if released else 'FAIL'}")
    return flags


try:
    results = run()
    print("=== FINAL RESULT ===")
    for name, passed in results.items():
        print(f"{name}: {'PASS' if passed else 'FAIL'}")
    print(f"SIM_P2: {'PASS' if all(results.values()) else 'FAIL'}")
    while not args.headless and simulation_app.is_running():
        simulation_app.update()
finally:
    simulation_app.close()
