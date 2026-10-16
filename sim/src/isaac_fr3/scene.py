"""Shared construction and settling for the Isaac Sim FR3 scene."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path


HOME_SETTLE_STEPS = 180
CUBE_SETTLE_STEPS = 120


@dataclass
class SceneHandles:
    world: World
    fr3: Articulation
    table: FixedCuboid
    target: DynamicCuboid
    arm_indices: np.ndarray
    finger_indices: np.ndarray


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as config_file:
        return json.load(config_file)


def create_scene(config: dict) -> SceneHandles:
    """Build and reset the baseline scene before commanding the FR3."""
    robot_cfg = config["robot"]
    table_cfg = config["table"]
    object_cfg = config["target_object"]

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("Isaac Sim asset root could not be resolved.")
    robot_usd = assets_root + robot_cfg["asset"]
    print(f"Isaac asset root: {assets_root}")
    print(f"FR3 asset: {robot_usd}")
    add_reference_to_stage(
        usd_path=robot_usd,
        prim_path=robot_cfg["prim_path"],
    )

    table = world.scene.add(
        FixedCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.asarray(table_cfg["position_m"], dtype=np.float64),
            scale=np.asarray(table_cfg["size_m"], dtype=np.float64),
            color=np.asarray(
                table_cfg.get("color_rgb", [0.5, 0.5, 0.5]),
                dtype=np.float64,
            ),
        )
    )

    target = world.scene.add(
        DynamicCuboid(
            prim_path="/World/TargetCube",
            name="target_cube",
            position=np.asarray(object_cfg["position_m"], dtype=np.float64),
            scale=np.asarray(object_cfg["size_m"], dtype=np.float64),
            color=np.asarray(
                object_cfg.get("color_rgb", [1.0, 0.0, 0.0]),
                dtype=np.float64,
            ),
            mass=float(object_cfg["mass_kg"]),
        )
    )

    world.reset()
    fr3 = Articulation(robot_cfg["prim_path"])
    arm_indices = fr3.get_dof_indices(robot_cfg["arm_dof_names"])
    finger_indices = fr3.get_dof_indices(robot_cfg["finger_dof_names"])
    return SceneHandles(world, fr3, table, target, arm_indices, finger_indices)


def command_home(handles: SceneHandles, config: dict) -> None:
    """Command the configured arm and fully open finger positions."""
    robot_cfg = config["robot"]
    home_arm = np.asarray([robot_cfg["home_arm_rad"]], dtype=np.float32)
    home_fingers = np.asarray(
        [robot_cfg["home_fingers_m"]], dtype=np.float32
    )
    handles.fr3.set_dof_position_targets(
        home_arm, dof_indices=handles.arm_indices
    )
    handles.fr3.set_dof_position_targets(
        home_fingers, dof_indices=handles.finger_indices
    )


def settle(handles: SceneHandles, steps: int, *, render: bool) -> None:
    """Advance physics with the launcher's original rendering behavior."""
    for _ in range(steps):
        handles.world.step(render=render)


def get_tcp_pose(config: dict) -> tuple[np.ndarray, np.ndarray]:
    """Return the configured TCP's batched world position and quaternion."""
    tcp = XformPrim(config["robot"]["tcp_prim"])
    return tcp.get_world_poses()
