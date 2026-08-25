"""Tests for SpaceMouse calibration profiles and safe live adjustment."""

from __future__ import annotations

import ast
from pathlib import Path
import tempfile
import unittest

from saps.evaluation.calibration import reset_nominal_calibration_scene
from saps.human_input.spacemouse import AXIS_NAMES
from saps.human_input.spacemouse import process_spacemouse_axes
from saps.human_input.spacemouse import SpaceMouseBackend
from saps.human_input.spacemouse import SpaceMouseConfig
from saps.human_input.spacemouse_profile import load_spacemouse_profile
from saps.human_input.spacemouse_profile import save_spacemouse_profile
from saps.human_input.spacemouse_profile import SpaceMouseProfile
from saps.human_input.web_operator import _build_operator_page
from saps.human_input.web_operator import BrowserOperatorServer

from tests.unit.test_spacemouse_input import FakeLinuxBoundary


class FakeCalibrationEnvironment:
    def __init__(self) -> None:
        self.reset_count = 0
        self.initial_state = None
        self.actions: list[list[float]] = []

    def reset(self) -> None:
        self.reset_count += 1

    def set_init_state(self, state: object) -> dict[str, object]:
        self.initial_state = state
        return {"frame": 0}

    def step(
        self,
        action: list[float],
    ) -> tuple[dict[str, object], float, bool, dict[str, object]]:
        self.actions.append(action)
        return {"frame": len(self.actions)}, 0.0, False, {}


class CalibrationSceneResetTest(unittest.TestCase):
    def test_reset_restores_and_settles_nominal_state(self) -> None:
        env = FakeCalibrationEnvironment()
        observation, steps = reset_nominal_calibration_scene(
            env=env,
            initial_states=["state0", "state1"],
            initial_state_index=1,
            num_steps_wait=3,
        )
        self.assertEqual(env.reset_count, 1)
        self.assertEqual(env.initial_state, "state1")
        self.assertEqual(steps, 3)
        self.assertEqual(observation, {"frame": 3})
        self.assertEqual(len(env.actions), 3)
        self.assertTrue(all(action[6] == -1.0 for action in env.actions))


class SpaceMouseProfileTest(unittest.TestCase):
    def make_profile(self) -> SpaceMouseProfile:
        config = SpaceMouseConfig(
            axis_signs=(1.0, 1.0, -1.0, 1.0, -1.0, 1.0),
            axis_scales=(1.0, 0.8, 1.2, 0.7, 0.9, 1.1),
            axis_enabled=(True, True, True, True, False, True),
        )
        return SpaceMouseProfile.from_config(
            config,
            device_type="3Dconnexion SpaceMouse Wireless",
        )

    def test_round_trip_load_and_save(self) -> None:
        profile = self.make_profile()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.json"
            save_spacemouse_profile(path, profile)
            loaded = load_spacemouse_profile(path)
        self.assertEqual(loaded, profile)
        self.assertNotIn("device_path", loaded.as_dict())

    def test_profile_rejects_unknown_and_invalid_values(self) -> None:
        data = self.make_profile().as_dict()
        data["device_path"] = "/dev/input/event12"
        with self.assertRaisesRegex(ValueError, "unknown fields"):
            SpaceMouseProfile.from_dict(data)

        data = self.make_profile().as_dict()
        data["axis_enabled"] = [True, True, True]
        with self.assertRaisesRegex(ValueError, "six boolean"):
            SpaceMouseProfile.from_dict(data)

    def test_runtime_device_path_is_separate(self) -> None:
        config = self.make_profile().to_config(
            device_path="/dev/input/by-id/device"
        )
        self.assertEqual(
            config.device_path,
            "/dev/input/by-id/device",
        )
        self.assertEqual(config.axis_scales[2], 1.2)


class BrowserCalibrationTest(unittest.TestCase):
    def make_operator(
        self,
        *,
        profile_path: str | None = None,
    ) -> tuple[BrowserOperatorServer, SpaceMouseBackend]:
        backend = SpaceMouseBackend(
            SpaceMouseConfig(),
            boundary=FakeLinuxBoundary(),
        )
        operator = BrowserOperatorServer(
            input_source="spacemouse",
            spacemouse_backend=backend,
            calibration_mode=True,
            calibration_profile_path=profile_path,
        )
        return operator, backend

    def calibration_message(self) -> dict[str, object]:
        return {
            "type": "calibration_apply",
            "axis_mapping": [
                "ABS_Y",
                "ABS_X",
                "ABS_Z",
                "ABS_RY",
                "ABS_RX",
                "ABS_RZ",
            ],
            "axis_signs": [1, -1, -1, 1, 1, -1],
            "translation_gain": 0.09,
            "rotation_gain": 0.12,
            "deadzone": 0.1,
            "axis_scales": [1.0, 0.8, 1.2, 0.5, 0.6, 0.7],
            "axis_enabled": [True, True, True, False, False, True],
        }

    def test_live_values_reach_mapper_only_after_disarm(self) -> None:
        operator, backend = self.make_operator()
        operator._armed = True
        operator._apply_message(self.calibration_message())
        self.assertFalse(operator._armed)
        self.assertEqual(
            backend.config.axis_mapping[0],
            "ABS_Y",
        )
        self.assertEqual(backend.config.axis_signs[2], -1.0)
        self.assertEqual(backend.config.translation_gain, 0.09)
        self.assertEqual(backend.config.rotation_gain, 0.12)
        self.assertEqual(backend.config.deadzone, 0.1)
        self.assertEqual(backend.config.axis_scales[2], 1.2)
        self.assertFalse(backend.config.axis_enabled[3])
        processed = process_spacemouse_axes(
            [350, 350, 350, 350, 350, 350],
            backend.config,
        )
        self.assertLess(processed.final_motion[1], 0.0)
        self.assertLess(processed.final_motion[2], 0.0)
        self.assertEqual(processed.final_motion[3], 0.0)

    def test_reset_is_calibration_only_and_disarms(self) -> None:
        operator, backend = self.make_operator()
        del backend
        operator._armed = True
        operator._apply_message({"type": "calibration_reset"})
        self.assertFalse(operator._armed)
        self.assertTrue(operator.consume_calibration_reset_request())
        self.assertFalse(operator.consume_calibration_reset_request())

        normal = BrowserOperatorServer(input_source="keyboard")
        normal._apply_message({"type": "calibration_reset"})
        self.assertFalse(normal.consume_calibration_reset_request())

    def test_save_uses_current_values_and_no_event_path(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.json"
            operator, backend = self.make_operator(
                profile_path=str(path)
            )
            operator._apply_message(self.calibration_message())
            operator._armed = True
            operator._apply_message({"type": "calibration_save"})
            self.assertFalse(operator._armed)
            loaded = load_spacemouse_profile(path)
        self.assertEqual(
            loaded.axis_mapping,
            backend.config.axis_mapping,
        )
        self.assertEqual(loaded.axis_enabled, backend.config.axis_enabled)
        self.assertNotIn("event", str(loaded.as_dict()))

    def test_calibration_page_has_graphical_controls(self) -> None:
        page = _build_operator_page(
            websocket_port=8765,
            calibration_mode=True,
        )
        for label in (*AXIS_NAMES, "Translation only", "Save profile"):
            self.assertIn(label, page)
        self.assertIn("mapped-axis-rows", page)
        self.assertIn("Reset nominal scene", page)
        self.assertIn("Stage 1: Translation only", page)
        self.assertIn("click Apply to activate", page)
        self.assertIn("+dx is screen forward", page)
        self.assertIn("Rotation-only test order", page)
        self.assertIn("Twist the puck clockwise", page)
        self.assertIn("central 8% in either", page)
        self.assertIn("direction produces zero", page)
        self.assertIn("applies to", page)
        self.assertIn("all six raw axes", page)
        self.assertIn("Applied values", page)
        self.assertIn("editable", page)
        self.assertIn("fields below are drafts", page)


class NormalRunnerDefaultsTest(unittest.TestCase):
    def test_committed_profile_matches_physically_validated_candidate(
        self,
    ) -> None:
        repository_root = Path(__file__).resolve().parents[2]
        profile = load_spacemouse_profile(
            repository_root / "configs" / "spacemouse_profile.json"
        )
        self.assertEqual(
            profile.axis_mapping,
            (
                "ABS_Y",
                "ABS_X",
                "ABS_Z",
                "ABS_RY",
                "ABS_RX",
                "ABS_RZ",
            ),
        )
        self.assertEqual(
            profile.axis_signs,
            (-1.0, 1.0, -1.0, -1.0, 1.0, 1.0),
        )
        self.assertEqual(profile.translation_gain, 0.30)
        self.assertEqual(profile.rotation_gain, 0.08)
        self.assertEqual(profile.deadzone, 0.08)
        self.assertEqual(profile.axis_enabled, (True,) * 6)

    def test_no_profile_retains_existing_normal_runner_defaults(self) -> None:
        repository_root = Path(__file__).resolve().parents[2]
        for relative_path in {
            "scripts/run_teleoperation_episode.py",
            "scripts/run_shared_autonomy_episode.py",
        }:
            module = ast.parse(
                (repository_root / relative_path).read_text(
                    encoding="utf-8"
                )
            )
            args_class = next(
                node
                for node in module.body
                if isinstance(node, ast.ClassDef) and node.name == "Args"
            )
            defaults = {
                node.target.id: ast.literal_eval(node.value)
                for node in args_class.body
                if isinstance(node, ast.AnnAssign)
                and isinstance(node.target, ast.Name)
                and node.value is not None
            }
            self.assertEqual(defaults["spacemouse_profile_path"], "")
            self.assertEqual(
                defaults["spacemouse_axis_signs"],
                "1,1,1,1,1,1",
            )
            self.assertEqual(defaults["translation_gain"], 0.14)
            self.assertEqual(defaults["rotation_gain"], 0.18)

    def test_calibration_starts_with_translation_only_candidate(self) -> None:
        source = (
            Path(__file__).resolve().parents[2]
            / "scripts"
            / "run_spacemouse_calibration.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            '"ABS_Y",\n            "ABS_X",\n'
            '            "ABS_Z",\n            "ABS_RY",\n'
            '            "ABS_RX",\n            "ABS_RZ",',
            source,
        )
        self.assertIn(
            "axis_signs=(-1.0, 1.0, -1.0, -1.0, 1.0, 1.0)",
            source,
        )
        self.assertIn(
            "axis_enabled=(True, True, True, False, False, False)",
            source,
        )
        self.assertIn("translation_gain: float = 0.30", source)
        self.assertIn("rotation_gain: float = 0.08", source)


if __name__ == "__main__":
    unittest.main()
