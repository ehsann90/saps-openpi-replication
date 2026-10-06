#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from isaac_fr3.scene import (
    CUBE_SETTLE_STEPS,
    HOME_SETTLE_STEPS,
    command_home,
    create_scene,
    get_tcp_pose,
    load_config,
    settle,
)


config = load_config(args.config)
handles = create_scene(config)
command_home(handles, config)
settle(handles, HOME_SETTLE_STEPS, render=not args.headless)

print("Measured FR3 DOFs:")
print(handles.fr3.get_dof_positions())

tcp_position, tcp_orientation = get_tcp_pose(config)

print("TCP world position [m]:")
print(tcp_position)

print("TCP world orientation [w,x,y,z]:")
print(tcp_orientation)

settle(handles, CUBE_SETTLE_STEPS, render=not args.headless)

cube_position, cube_orientation = handles.target.get_world_pose()

print("Target cube settled position [m]:")
print(cube_position)

print("Target cube orientation [w,x,y,z]:")
print(cube_orientation)

print("Scene ready.")

while simulation_app.is_running():
    handles.world.step(render=not args.headless)

simulation_app.close()
