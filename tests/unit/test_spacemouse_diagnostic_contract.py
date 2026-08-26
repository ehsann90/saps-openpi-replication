"""Keep the SpaceMouse Make target aligned with its diagnostic CLI."""

from __future__ import annotations

import ast
import importlib.util
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile
import unittest

from saps.human_input.spacemouse_profile import load_spacemouse_profile
from saps.human_input.spacemouse_profile import save_spacemouse_profile


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DIAGNOSTIC_PATH = (
    REPOSITORY_ROOT
    / "tools"
    / "diagnostics"
    / "inspect_spacemouse_input.py"
)
DIAGNOSTIC_SPEC = importlib.util.spec_from_file_location(
    "saps_spacemouse_diagnostic",
    DIAGNOSTIC_PATH,
)
assert DIAGNOSTIC_SPEC is not None and DIAGNOSTIC_SPEC.loader is not None
spacemouse_diagnostic = importlib.util.module_from_spec(DIAGNOSTIC_SPEC)
sys.modules[DIAGNOSTIC_SPEC.name] = spacemouse_diagnostic
DIAGNOSTIC_SPEC.loader.exec_module(spacemouse_diagnostic)


class SpaceMouseDiagnosticContractTest(unittest.TestCase):
    def test_make_arguments_exist_in_diagnostic_parser(self) -> None:
        result = subprocess.run(
            [
                "make",
                "-n",
                "spacemouse-diagnostic",
                (
                    "SPACEMOUSE_DEVICE=/dev/input/by-id/"
                    "usb-3Dconnexion_SpaceMouse_Wireless-"
                    "event-joystick"
                ),
            ],
            cwd=REPOSITORY_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        match = re.search(
            r'SAPS_RUNTIME_ARGS="([^"]*)"',
            result.stdout,
        )
        self.assertIsNotNone(match)
        tokens = shlex.split(match.group(1))
        passed_options = {
            token[2:].replace("-", "_")
            for token in tokens
            if token.startswith("--")
        }
        module = ast.parse(
            DIAGNOSTIC_PATH.read_text(encoding="utf-8")
        )
        args_class = next(
            node
            for node in module.body
            if isinstance(node, ast.ClassDef) and node.name == "Args"
        )
        parser_fields = {
            node.target.id
            for node in args_class.body
            if isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
        }
        self.assertTrue(passed_options.issubset(parser_fields))
        self.assertEqual(
            passed_options,
            {
                "device_path",
                "profile_path",
                "refresh_frequency_hz",
            },
        )

    def test_calibration_make_arguments_exist_in_parser(self) -> None:
        device_path = (
            "/dev/input/by-id/"
            "usb-3Dconnexion_SpaceMouse_Wireless-event-joystick"
        )
        result = subprocess.run(
            [
                "make",
                "-n",
                "spacemouse-calibrate",
                f"SPACEMOUSE_DEVICE={device_path}",
            ],
            cwd=REPOSITORY_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        match = re.search(
            r'SAPS_RUNTIME_ARGS="([^"]*)"',
            result.stdout,
        )
        self.assertIsNotNone(match)
        tokens = shlex.split(match.group(1))
        passed_options = {
            token[2:].replace("-", "_")
            for token in tokens
            if token.startswith("--")
        }
        calibration_path = (
            REPOSITORY_ROOT
            / "scripts"
            / "run_spacemouse_calibration.py"
        )
        module = ast.parse(
            calibration_path.read_text(encoding="utf-8")
        )
        args_class = next(
            node
            for node in module.body
            if isinstance(node, ast.ClassDef) and node.name == "Args"
        )
        parser_fields = {
            node.target.id
            for node in args_class.body
            if isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
        }
        self.assertTrue(passed_options.issubset(parser_fields))
        self.assertEqual(
            passed_options,
            {
                "device_path",
                "profile_path",
            },
        )

    def test_diagnostic_profile_overrides_raw_fallback_values(self) -> None:
        calibrated = load_spacemouse_profile(
            REPOSITORY_ROOT / "configs" / "spacemouse_profile.json"
        )
        with tempfile.TemporaryDirectory() as directory:
            profile_path = Path(directory) / "profile.json"
            save_spacemouse_profile(profile_path, calibrated)
            config = spacemouse_diagnostic.make_config(
                spacemouse_diagnostic.Args(
                    device_path="/dev/input/by-id/test-device",
                    profile_path=str(profile_path),
                    translation_gain=0.99,
                    rotation_gain=0.99,
                    axis_signs="1,1,1,1,1,1",
                )
            )

        self.assertEqual(config.device_path, "/dev/input/by-id/test-device")
        self.assertEqual(config.translation_gain, 0.4)
        self.assertEqual(config.rotation_gain, 0.08)
        self.assertEqual(config.axis_mapping, calibrated.axis_mapping)
        self.assertEqual(config.axis_signs, calibrated.axis_signs)


if __name__ == "__main__":
    unittest.main()
