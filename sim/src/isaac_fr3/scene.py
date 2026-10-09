"""Shared construction and settling for the Isaac Sim FR3 scene."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, DynamicCylinder, FixedCuboid
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
    target: DynamicCuboid | DynamicCylinder
    arm_indices: np.ndarray
    finger_indices: np.ndarray
    cubes: list[DynamicCuboid | DynamicCylinder]


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as config_file:
        return json.load(config_file)


def select_active_target(config: dict) -> dict:
    """Choose the one active package or mug in a single-target scene."""
    package = config["target_object"]
    mug = config.get("optional_mug", {"enabled": False})
    package_enabled = package.get("enabled", True)
    mug_enabled = mug["enabled"]
    if not isinstance(package_enabled, bool) or not isinstance(mug_enabled, bool):
        raise ValueError("Target enabled flags must be booleans")
    if package_enabled == mug_enabled:
        raise ValueError("Enable exactly one package or mug target")
    if mug_enabled:
        return {**mug, "type": "usd_visual_mug"}
    return package


def create_scene(config: dict) -> SceneHandles:
    """Build and reset the configured scene before commanding the FR3."""
    robot_cfg = config["robot"]
    table_cfg = config["table"]
    object_cfgs = (
        config["objects"] if "objects" in config
        else [select_active_target(config)]
    )
    if not object_cfgs:
        raise ValueError("Scene must contain at least one object")

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    if "lighting" in config:
        from pxr import UsdLux

        light_config = config["lighting"]["default_ground_sphere"]
        radius = float(light_config["radius_m"])
        intensity = float(light_config["intensity"])
        specular = float(light_config["specular"])
        if (not np.isfinite([radius, intensity, specular]).all()
                or radius <= 0 or intensity <= 0
                or not 0 <= specular <= 1):
            raise ValueError("Invalid default ground sphere light settings")
        light_prim = world.stage.GetPrimAtPath(
            "/World/defaultGroundPlane/SphereLight"
        )
        if not light_prim.IsA(UsdLux.SphereLight):
            raise RuntimeError("Default ground sphere light is missing")
        light = UsdLux.SphereLight(light_prim)
        light.GetRadiusAttr().Set(radius)
        light.GetIntensityAttr().Set(intensity)
        light.GetSpecularAttr().Set(specular)
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
        object_type = object_cfg["type"]
        if object_type not in ("cube", "usd_visual_box", "usd_visual_mug"):
            raise ValueError(f"Unsupported scene object: {object_type}")
        prim_path = (
            ("/World/TargetMug" if object_type == "usd_visual_mug"
             else "/World/TargetCube")
            if index == 0 else f"/World/Cube{index}"
        )
        if object_type == "usd_visual_mug":
            target = DynamicCylinder(
                prim_path=prim_path,
                name="target_mug",
                position=np.asarray(object_cfg["position_m"], dtype=np.float64),
                radius=float(object_cfg["body_radius_m"]),
                height=float(object_cfg["height_m"]),
                mass=float(object_cfg["mass_kg"]),
            )
        else:
            target = DynamicCuboid(
                prim_path=prim_path,
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
        cubes.append(world.scene.add(target))
        if object_type in ("usd_visual_box", "usd_visual_mug"):
            from pxr import UsdGeom

            visual_path = (
                Path(__file__).resolve().parents[2]
                / object_cfg["visual_asset"]
            ).resolve()
            if not visual_path.is_file():
                raise FileNotFoundError(f"Missing target visual: {visual_path}")
            # Keep the configured primitive as the sole collision body.
            collision_prim = world.stage.GetPrimAtPath(prim_path)
            # Isaac's primitive material overrides descendant bindings.
            # The referenced asset must supply its own visible material.
            collision_prim.RemoveProperty("material:binding")
            UsdGeom.Imageable(collision_prim).CreatePurposeAttr(
                UsdGeom.Tokens.guide
            )
            child_path = prim_path + "/Visual"
            add_reference_to_stage(str(visual_path), child_path)
            visual_prim = world.stage.GetPrimAtPath(child_path)
            UsdGeom.Imageable(visual_prim).CreatePurposeAttr(
                UsdGeom.Tokens.render
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
        if "visual_asset" in basket_cfg:
            from pxr import Gf, UsdGeom

            visual_path = (
                Path(__file__).resolve().parents[2]
                / basket_cfg["visual_asset"]
            ).resolve()
            if not visual_path.is_file():
                raise FileNotFoundError(f"Missing basket visual: {visual_path}")
            for name, _, _ in parts:
                collision_prim = world.stage.GetPrimAtPath(
                    f"/World/Basket{name}"
                )
                UsdGeom.Imageable(collision_prim).CreatePurposeAttr(
                    UsdGeom.Tokens.guide
                )
            basket_visual = "/World/BasketVisual"
            add_reference_to_stage(str(visual_path), basket_visual)
            visual_prim = world.stage.GetPrimAtPath(basket_visual)
            visual_prim.GetAttribute("xformOp:translate").Set(
                Gf.Vec3d(float(center[0]), float(center[1]), table_top)
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
