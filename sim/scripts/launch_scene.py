#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path

from isaacsim import SimulationApp


def parse_args():
    parser = argparse.ArgumentParser(
        description="Launch the reproducible Isaac Sim FR3/DROID scene."
    )
    parser.add_argument(
        "--config",
        type=Path,
        required=True,
        help="Path to the FR3/DROID scene JSON configuration.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run Isaac Sim without the GUI.",
    )
    return parser.parse_args()


args = parse_args()

simulation_app = SimulationApp(
    {
        "headless": args.headless,
    }
)

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


config = load_config(args.config)

robot_cfg = config["robot"]
table_cfg = config["table"]
object_cfg = config["target_object"]

world = World(
    stage_units_in_meters=1.0,
)

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
        position=np.asarray(
            object_cfg["position_m"],
            dtype=np.float64,
        ),
        scale=np.asarray(
            object_cfg["size_m"],
            dtype=np.float64,
        ),
        color=np.asarray(
            object_cfg.get("color_rgb", [1.0, 0.0, 0.0]),
            dtype=np.float64,
        ),
        mass=float(object_cfg["mass_kg"]),
    )
)

world.reset()

fr3 = Articulation(robot_cfg["prim_path"])

arm_indices = fr3.get_dof_indices(
    robot_cfg["arm_dof_names"]
)
finger_indices = fr3.get_dof_indices(
    robot_cfg["finger_dof_names"]
)

home_arm = np.asarray(
    [robot_cfg["home_arm_rad"]],
    dtype=np.float32,
)

home_fingers = np.asarray(
    [robot_cfg["home_fingers_m"]],
    dtype=np.float32,
)

fr3.set_dof_position_targets(
    home_arm,
    dof_indices=arm_indices,
)

fr3.set_dof_position_targets(
    home_fingers,
    dof_indices=finger_indices,
)

# Let the controller settle.
for _ in range(180):
    world.step(render=not args.headless)

print("Measured FR3 DOFs:")
print(fr3.get_dof_positions())

tcp = XformPrim(robot_cfg["tcp_prim"])
tcp_position, tcp_orientation = tcp.get_world_poses()

print("TCP world position [m]:")
print(tcp_position)

print("TCP world orientation [w,x,y,z]:")
print(tcp_orientation)

print("Scene ready.")

for _ in range(120):
    world.step(render=not args.headless)

cube_position, cube_orientation = target.get_world_pose()

print("Target cube settled position [m]:")
print(cube_position)

print("Target cube orientation [w,x,y,z]:")
print(cube_orientation)

while simulation_app.is_running():
    world.step(render=not args.headless)

simulation_app.close()
