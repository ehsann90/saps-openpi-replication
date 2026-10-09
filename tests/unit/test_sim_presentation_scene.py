"""Check optional Isaac scene construction without requiring Isaac Sim."""

from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SCENE_PATH = ROOT / "sim/src/isaac_fr3/scene.py"
CONFIG_DIR = ROOT / "sim/configs"


class FakePrim:
    def __init__(self, **kwargs: object) -> None:
        self.__dict__.update(kwargs)


class FakeArticulation:
    def __init__(self, prim_path: str) -> None:
        self.prim_path = prim_path

    def get_dof_indices(self, names: list[str]) -> np.ndarray:
        return np.arange(len(names))


class FakeScene:
    def __init__(self) -> None:
        self.objects: list[FakePrim] = []

    def add_default_ground_plane(self) -> None:
        pass

    def add(self, obj: FakePrim) -> FakePrim:
        self.objects.append(obj)
        return obj


class FakeWorld:
    def __init__(self, stage_units_in_meters: float) -> None:
        self.scene = FakeScene()

    def reset(self) -> None:
        pass


def load_scene_module() -> types.ModuleType:
    modules = {
        name: types.ModuleType(name)
        for name in (
            "isaacsim", "isaacsim.core", "isaacsim.core.api",
            "isaacsim.core.api.objects", "isaacsim.core.experimental",
            "isaacsim.core.experimental.prims", "isaacsim.core.utils",
            "isaacsim.core.utils.stage", "isaacsim.storage",
            "isaacsim.storage.native",
        )
    }
    modules["isaacsim.core.api"].World = FakeWorld
    modules["isaacsim.core.api.objects"].DynamicCuboid = FakePrim
    modules["isaacsim.core.api.objects"].DynamicCylinder = FakePrim
    modules["isaacsim.core.api.objects"].FixedCuboid = FakePrim
    modules["isaacsim.core.experimental.prims"].Articulation = FakeArticulation
    modules["isaacsim.core.experimental.prims"].XformPrim = FakePrim
    modules["isaacsim.core.utils.stage"].add_reference_to_stage = lambda **_: None
    modules["isaacsim.storage.native"].get_assets_root_path = lambda: "/Assets"
    spec = importlib.util.spec_from_file_location("test_isolated_scene", SCENE_PATH)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {**modules, spec.name: module}):
        spec.loader.exec_module(module)
    return module


class PresentationSceneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.scene = load_scene_module()
        cls.baseline = cls.scene.load_config(CONFIG_DIR / "fr3_droid_scene.json")
        cls.presentation = cls.scene.load_config(
            CONFIG_DIR / "fr3_presentation_scene.json"
        )

    def test_baseline_keeps_single_target_without_basket(self) -> None:
        handles = self.scene.create_scene(self.baseline)
        self.assertEqual(len(handles.cubes), 1)
        self.assertEqual(handles.target.prim_path, "/World/TargetCube")
        self.assertEqual(len(handles.world.scene.objects), 2)

    def test_presentation_geometry_and_unchanged_robot_camera(self) -> None:
        config = self.presentation
        for key in ("robot", "table", "cameras", "droid"):
            self.assertEqual(config[key], self.baseline[key])
        handles = self.scene.create_scene(config)
        self.assertEqual(len(handles.cubes), 4)
        self.assertIs(handles.target, handles.cubes[0])
        self.assertEqual(len(handles.world.scene.objects), 11)
        floor = handles.world.scene.objects[0]
        self.assertEqual(floor.prim_path, "/World/PresentationFloor")
        self.assertAlmostEqual(floor.position[2] + floor.scale[2] / 2, 0.002)

        table_center = np.asarray(config["table"]["position_m"])
        table_size = np.asarray(config["table"]["size_m"])
        top = table_center[2] + table_size[2] / 2
        for cube in handles.cubes:
            self.assertAlmostEqual(cube.position[2] - cube.scale[2] / 2, top)
            self.assertAlmostEqual(cube.mass, 0.05)
            for axis in (0, 1):
                self.assertLess(
                    abs(cube.position[axis] - table_center[axis])
                    + cube.scale[axis] / 2,
                    table_size[axis] / 2,
                )
        for first, second in zip(handles.cubes, handles.cubes[1:]):
            self.assertGreater(
                second.position[0] - first.position[0],
                (first.scale[0] + second.scale[0]) / 2,
            )
        basket = handles.world.scene.objects[6:]
        self.assertEqual(len(basket), 5)
        self.assertAlmostEqual(basket[0].position[2] - basket[0].scale[2] / 2, top)
        for axis in (0, 1):
            self.assertLess(
                abs(basket[0].position[axis] - table_center[axis])
                + basket[0].scale[axis] / 2,
                table_size[axis] / 2,
            )
        self.assertLess(
            max(cube.position[1] + cube.scale[1] / 2
                for cube in handles.cubes),
            basket[0].position[1] - basket[0].scale[1] / 2,
        )
        self.assertGreater(
            config["basket"]["size_xy_m"][0]
            - 2 * config["basket"]["wall_thickness_m"],
            2 * handles.cubes[0].scale[0],
        )


class CatalogTargetSelectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.scene = load_scene_module()
        cls.package = cls.scene.load_config(
            CONFIG_DIR / "fr3_droid_robotiq_pick_place_catalog_scene.json"
        )
        cls.mug = cls.scene.load_config(
            CONFIG_DIR / "fr3_droid_robotiq_mug_target_scene.json"
        )

    def test_package_only_remains_selected(self) -> None:
        self.assertIs(
            self.scene.select_active_target(self.package),
            self.package["target_object"],
        )

    def test_disabled_package_selects_only_mug(self) -> None:
        target = self.scene.select_active_target(self.mug)
        self.assertEqual(target["type"], "usd_visual_mug")
        self.assertEqual(target["visual_asset"],
                         self.mug["optional_mug"]["visual_asset"])
        self.assertEqual(target["position_m"],
                         self.mug["optional_mug"]["position_m"])

    def test_rejects_both_enabled_or_disabled(self) -> None:
        for enabled in (True, False):
            with self.subTest(mug_enabled=enabled):
                config = {
                    "target_object": {"enabled": enabled},
                    "optional_mug": {"enabled": enabled},
                }
                with self.assertRaisesRegex(ValueError, "exactly one"):
                    self.scene.select_active_target(config)


if __name__ == "__main__":
    unittest.main()
