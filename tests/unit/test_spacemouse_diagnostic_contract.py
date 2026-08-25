"""Keep the SpaceMouse Make target aligned with its diagnostic CLI."""

from __future__ import annotations

import ast
from pathlib import Path
import re
import shlex
import subprocess
import unittest

class SpaceMouseDiagnosticContractTest(unittest.TestCase):
    def test_make_arguments_exist_in_diagnostic_parser(self) -> None:
        repository_root = Path(__file__).resolve().parents[2]
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
            cwd=repository_root,
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
        diagnostic_path = (
            repository_root
            / "tools"
            / "diagnostics"
            / "inspect_spacemouse_input.py"
        )
        module = ast.parse(
            diagnostic_path.read_text(encoding="utf-8")
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
                "translation_gain",
                "rotation_gain",
                "deadzone",
                "axis_mapping",
                "axis_signs",
                "axis_maxima",
                "stale_input_timeout_seconds",
                "open_button",
                "close_button",
                "refresh_frequency_hz",
            },
        )

    def test_calibration_make_arguments_exist_in_parser(self) -> None:
        repository_root = Path(__file__).resolve().parents[2]
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
            cwd=repository_root,
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
            repository_root
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
                "translation_gain",
                "rotation_gain",
                "deadzone",
                "stale_input_timeout_seconds",
                "open_button",
                "close_button",
            },
        )


if __name__ == "__main__":
    unittest.main()
