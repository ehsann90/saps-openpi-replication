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
    cubes: list[DynamicCuboid]


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as config_file:
        return json.load(config_file)


def create_scene(config: dict) -> SceneHandles:
    """Build and reset the configured scene before commanding the FR3."""
    robot_cfg = config["robot"]
    table_cfg = config["table"]
    object_cfgs = (
        config["objects"] if "objects" in config
        else [config["target_object"]]
    )
    if not object_cfgs:
        raise ValueError("Scene must contain at least one cube")

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    if "floor" in config:
        floor_cfg = config["floor"]
        world.scene.add(
            FixedCuboid(
                prim_path="/World/PresentationFloor",
                name="presentation_floor",
                position=np.asarray(floor_cfg["position_m"], dtype=np.float64),
                scale=np.asarray(floor_cfg["size_m"], dtype=np.float64),
                color=np.asarray(floor_cfg["color_rgb"], dtype=np.float64),
            )
        )

    if robot_cfg.get("asset_source") == "local":
        robot_usd = str((Path(__file__).resolve().parents[2]
                         / robot_cfg["asset"]).resolve())
        if not Path(robot_usd).is_file():
            raise FileNotFoundError(f"Missing local robot assembly: {robot_usd}")
    else:
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

    cubes = []
    for index, object_cfg in enumerate(object_cfgs):
        if object_cfg["type"] != "cube":
            raise ValueError("Only cube scene objects are supported")
        cubes.append(
            world.scene.add(
                DynamicCuboid(
                    prim_path=(
                        "/World/TargetCube" if index == 0
                        else f"/World/Cube{index}"
                    ),
                    name=("target_cube" if index == 0 else f"cube_{index}"),
                    position=np.asarray(
                        object_cfg["position_m"], dtype=np.float64
                    ),
                    scale=np.asarray(object_cfg["size_m"], dtype=np.float64),
                    color=np.asarray(
                        object_cfg.get("color_rgb", [1.0, 0.0, 0.0]),
                        dtype=np.float64,
                    ),
                    mass=float(object_cfg["mass_kg"]),
                )
            )
        )

    if "basket" in config:
        basket_cfg = config["basket"]
        center = np.asarray(
            [*basket_cfg["center_xy_m"], 0.0], dtype=np.float64
        )
        width, depth = map(float, basket_cfg["size_xy_m"])
        wall = float(basket_cfg["wall_thickness_m"])
        bottom = float(basket_cfg["bottom_thickness_m"])
        height = float(basket_cfg["wall_height_m"])
        if (
            min(width, depth, wall, bottom, height) <= 0
            or 2 * wall >= min(width, depth)
        ):
            raise ValueError("Basket dimensions must form an open interior")
        color = np.asarray(basket_cfg["color_rgb"], dtype=np.float64)
        table_top = (
            float(table_cfg["position_m"][2])
            + float(table_cfg["size_m"][2]) / 2
        )
        parts = (
            ("Bottom", [0, 0, bottom / 2], [width, depth, bottom]),
            ("Left", [-width / 2 + wall / 2, 0, bottom + height / 2],
             [wall, depth, height]),
            ("Right", [width / 2 - wall / 2, 0, bottom + height / 2],
             [wall, depth, height]),
            ("Front", [0, -depth / 2 + wall / 2, bottom + height / 2],
             [width - 2 * wall, wall, height]),
            ("Back", [0, depth / 2 - wall / 2, bottom + height / 2],
             [width - 2 * wall, wall, height]),
        )
        for name, offset, size in parts:
            position = center + np.asarray(offset, dtype=np.float64)
            position[2] += table_top
            world.scene.add(
                FixedCuboid(
                    prim_path=f"/World/Basket{name}",
                    name=f"basket_{name.lower()}",
                    position=position,
                    scale=np.asarray(size, dtype=np.float64),
                    color=color,
                )
            )

    world.reset()
    fr3 = Articulation(robot_cfg["prim_path"])
    arm_indices = fr3.get_dof_indices(robot_cfg["arm_dof_names"])
    finger_indices = fr3.get_dof_indices(robot_cfg["finger_dof_names"])
    return SceneHandles(
        world, fr3, table, cubes[0], arm_indices, finger_indices, cubes
    )


def command_home(handles: SceneHandles, config: dict) -> None:
    """Command the configured arm and fully open finger positions."""
    robot_cfg = config["robot"]
    home_arm = np.asarray([robot_cfg["home_arm_rad"]], dtype=np.float32)
    if config.get("gripper", {}).get("kind") == "robotiq_2f85":
        home_fingers = np.asarray(
            [[config["gripper"]["open_joint_rad"]]], dtype=np.float32
        )
    else:
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
