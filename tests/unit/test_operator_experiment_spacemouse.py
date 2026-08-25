"""SpaceMouse profile integration for manifest-driven sessions."""

from __future__ import annotations

import ast
import dataclasses
import importlib.util
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from saps.evaluation.experiment_session import ExperimentManifest
from saps.human_input.spacemouse_profile import load_spacemouse_profile
from saps.human_input.spacemouse_profile import save_spacemouse_profile
from saps.human_input.web_operator import BrowserOperatorServer


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
CALIBRATED_PROFILE_PATH = (
    REPOSITORY_ROOT / "configs" / "spacemouse_profile.json"
)
RUNNER_PATH = REPOSITORY_ROOT / "scripts" / "run_operator_experiment.py"
RUNNER_SPEC = importlib.util.spec_from_file_location(
    "saps_run_operator_experiment",
    RUNNER_PATH,
)
assert RUNNER_SPEC is not None and RUNNER_SPEC.loader is not None
run_operator_experiment = importlib.util.module_from_spec(RUNNER_SPEC)
sys.modules[RUNNER_SPEC.name] = run_operator_experiment
RUNNER_SPEC.loader.exec_module(run_operator_experiment)


def make_manifest(
    *,
    config_path: str = "configs/test.json",
) -> ExperimentManifest:
    """Return one compact valid operator manifest."""

    return ExperimentManifest(
        schema_version=3,
        experiment_id="test_spacemouse_session",
        config_path=config_path,
        conditions=("nominal",),
        modes=("teleoperation",),
        trials_per_condition=1,
        initial_state_index=0,
        environment_seed=7,
        policy_base_seed=20260724,
        fixed_autonomy_weight=0.5,
        cosine_gain=6.0,
        control_frequency_hz=20.0,
        operator_max_steps=1200,
        fine_translation_gain=0.25,
        fine_rotation_gain=0.25,
        normal_translation_gain=0.5,
        normal_rotation_gain=0.5,
        fast_translation_gain=1.0,
        fast_rotation_gain=1.0,
        default_speed_mode="normal",
        ordering_seed=42,
    )


def command_value(command: list[str], option: str) -> str:
    """Return the value immediately following one command option."""

    return command[command.index(option) + 1]


class OperatorExperimentSpaceMouseTest(unittest.TestCase):
    def make_args(self, **overrides: object) -> run_operator_experiment.Args:
        values: dict[str, object] = {
            "manifest_path": "manifest.json",
            "repository_commit": "a" * 40,
        }
        values.update(overrides)
        return run_operator_experiment.Args(**values)

    def test_spacemouse_session_requires_profile(self) -> None:
        args = self.make_args(input_source="spacemouse")

        with self.assertRaisesRegex(ValueError, "explicit.*profile"):
            run_operator_experiment._human_input_configuration(
                args=args,
                manifest=make_manifest(),
            )

    def test_keyboard_session_does_not_require_profile(self) -> None:
        configuration = run_operator_experiment._human_input_configuration(
            args=self.make_args(input_source="keyboard"),
            manifest=make_manifest(),
        )

        self.assertEqual(configuration["input_source"], "keyboard")
        self.assertNotIn("spacemouse_profile", configuration)
        self.assertEqual(
            configuration["spacemouse"]["translation_gain"],
            0.5,
        )

    def test_session_uses_validated_profile_loader(self) -> None:
        args = self.make_args(
            input_source="spacemouse",
            spacemouse_profile_path=str(CALIBRATED_PROFILE_PATH),
        )

        with mock.patch.object(
            run_operator_experiment,
            "load_spacemouse_profile",
            wraps=load_spacemouse_profile,
        ) as loader:
            configuration = (
                run_operator_experiment._human_input_configuration(
                    args=args,
                    manifest=make_manifest(),
                )
            )

        loader.assert_called_once_with(CALIBRATED_PROFILE_PATH)
        self.assertEqual(
            configuration["spacemouse_profile"]["contents"][
                "translation_gain"
            ],
            0.3,
        )

    def test_profile_path_reaches_all_child_modes(self) -> None:
        profile_path = str(CALIBRATED_PROFILE_PATH)
        args = self.make_args(
            input_source="spacemouse",
            spacemouse_profile_path=profile_path,
        )
        manifest = make_manifest()

        for mode in (
            "teleoperation",
            "takeover",
            "fixed_blend",
            "cosine_blend",
        ):
            with self.subTest(mode=mode):
                command = run_operator_experiment._episode_command(
                    manifest=manifest,
                    episode={
                        "mode": mode,
                        "condition_id": "nominal",
                        "trial_index": 0,
                        "initial_state_index": 0,
                    },
                    attempt_root=Path("outputs/attempt"),
                    args=args,
                )
                passed_path = command_value(
                    command,
                    "--spacemouse-profile-path",
                )
                loaded = load_spacemouse_profile(Path(passed_path))
                self.assertEqual(passed_path, profile_path)
                self.assertEqual(loaded.translation_gain, 0.3)
                self.assertEqual(loaded.rotation_gain, 0.08)
                for reconstructed_option in (
                    "--spacemouse-deadzone",
                    "--spacemouse-axis-mapping",
                    "--spacemouse-axis-signs",
                    "--spacemouse-axis-maxima",
                    "--spacemouse-stale-input-timeout-seconds",
                    "--spacemouse-open-button",
                    "--spacemouse-close-button",
                ):
                    self.assertNotIn(reconstructed_option, command)
                expected_script = (
                    "scripts/run_teleoperation_episode.py"
                    if mode == "teleoperation"
                    else "scripts/run_shared_autonomy_episode.py"
                )
                self.assertEqual(command[1], expected_script)

    def test_profile_config_overrides_manifest_keyboard_gains(self) -> None:
        manifest = make_manifest()
        profile = load_spacemouse_profile(CALIBRATED_PROFILE_PATH)
        operator = BrowserOperatorServer(
            input_source="spacemouse",
            normal_translation_gain=manifest.normal_translation_gain,
            normal_rotation_gain=manifest.normal_rotation_gain,
            spacemouse_config=profile.to_config(),
        )

        self.assertIsNotNone(operator._spacemouse)
        assert operator._spacemouse is not None
        self.assertEqual(operator._spacemouse.config.translation_gain, 0.3)
        self.assertEqual(operator._spacemouse.config.rotation_gain, 0.08)
        self.assertNotEqual(
            operator._spacemouse.config.translation_gain,
            manifest.normal_translation_gain,
        )

    def test_profile_provenance_is_frozen_across_resume(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            task_config_path = root / "task.json"
            task_config_path.write_text(
                json.dumps(
                    {
                        "task_id": 1,
                        "offsets": [
                            {"id": "nominal", "dx": 0.0, "dy": 0.0}
                        ],
                    }
                ),
                encoding="utf-8",
            )
            manifest = make_manifest(config_path=str(task_config_path))
            manifest_path = root / "manifest.json"
            manifest_path.write_text(
                json.dumps(manifest.as_dict()),
                encoding="utf-8",
            )
            profile_path = root / "profile.json"
            profile = load_spacemouse_profile(CALIBRATED_PROFILE_PATH)
            save_spacemouse_profile(profile_path, profile)
            output_path = root / "session"
            args = self.make_args(
                manifest_path=str(manifest_path),
                output_dir=str(output_path),
                dry_run=True,
                input_source="spacemouse",
                spacemouse_device_path="/dev/input/by-id/test-device",
                spacemouse_profile_path=str(profile_path),
            )

            run_operator_experiment.main(args)
            run_operator_experiment.main(args)

            frozen = json.loads(
                (output_path / "human_input.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(frozen["input_source"], "spacemouse")
            self.assertEqual(
                frozen["spacemouse_device_path"],
                "/dev/input/by-id/test-device",
            )
            self.assertEqual(
                frozen["spacemouse_profile"]["path"],
                str(profile_path),
            )
            self.assertEqual(
                len(frozen["spacemouse_profile"]["sha256"]),
                64,
            )
            self.assertEqual(
                frozen["spacemouse_profile"]["schema_version"],
                profile.schema_version,
            )
            self.assertEqual(
                frozen["spacemouse_profile"]["contents"],
                profile.as_dict(),
            )
            provenance = json.loads(
                (output_path / "repository_provenance.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(provenance["repository_commit"], "a" * 40)

            changed = dataclasses.replace(profile, translation_gain=0.31)
            save_spacemouse_profile(profile_path, changed)
            with self.assertRaisesRegex(
                ValueError,
                "different human-input configuration",
            ):
                run_operator_experiment.main(args)

    def test_child_profile_must_match_frozen_profile(self) -> None:
        configuration = run_operator_experiment._human_input_configuration(
            args=self.make_args(
                input_source="spacemouse",
                spacemouse_profile_path=str(CALIBRATED_PROFILE_PATH),
            ),
            manifest=make_manifest(),
        )
        identity = {
            key: configuration["spacemouse_profile"][key]
            for key in ("path", "schema_version", "sha256")
        }
        run_operator_experiment._validate_child_human_input(
            summary={"spacemouse_profile": identity},
            input_configuration=configuration,
        )

        with self.assertRaisesRegex(ValueError, "does not match"):
            run_operator_experiment._validate_child_human_input(
                summary={
                    "spacemouse_profile": {**identity, "sha256": "0" * 64}
                },
                input_configuration=configuration,
            )

    def test_keyboard_child_command_retains_manifest_gains(self) -> None:
        command = run_operator_experiment._episode_command(
            manifest=make_manifest(),
            episode={
                "mode": "teleoperation",
                "condition_id": "nominal",
                "trial_index": 0,
                "initial_state_index": 0,
            },
            attempt_root=Path("outputs/attempt"),
            args=self.make_args(input_source="keyboard"),
        )

        self.assertNotIn("--spacemouse-profile-path", command)
        self.assertEqual(command_value(command, "--translation-gain"), "0.5")
        self.assertEqual(command_value(command, "--rotation-gain"), "0.5")


class OperatorSessionMakeContractTest(unittest.TestCase):
    def test_make_defaults_profile_only_for_spacemouse_sessions(self) -> None:
        profile_path = "configs/spacemouse_profile.json"
        result = subprocess.run(
            [
                "make",
                "-n",
                "operator-session",
                "INPUT_SOURCE=spacemouse",
                "SPACEMOUSE_DEVICE=/dev/input/by-id/test-device",
            ],
            cwd=REPOSITORY_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        match = re.search(r'SAPS_RUNTIME_ARGS="([^"]*)"', result.stdout)
        self.assertIsNotNone(match)
        assert match is not None
        tokens = shlex.split(match.group(1))
        self.assertEqual(
            command_value(tokens, "--spacemouse-profile-path"),
            profile_path,
        )

        keyboard_result = subprocess.run(
            [
                "make",
                "-n",
                "operator-session",
                "INPUT_SOURCE=keyboard",
                "SPACEMOUSE_DEVICE=/dev/input/by-id/unused-device",
            ],
            cwd=REPOSITORY_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        keyboard_match = re.search(
            r'SAPS_RUNTIME_ARGS="([^"]*)"',
            keyboard_result.stdout,
        )
        self.assertIsNotNone(keyboard_match)
        assert keyboard_match is not None
        keyboard_tokens = shlex.split(keyboard_match.group(1))
        self.assertNotIn("--spacemouse-profile-path", keyboard_tokens)
        self.assertNotIn("--spacemouse-device-path", keyboard_tokens)

        module = ast.parse(
            (
                REPOSITORY_ROOT
                / "scripts"
                / "run_operator_experiment.py"
            ).read_text(encoding="utf-8")
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
        self.assertIn("spacemouse_profile_path", parser_fields)

    def test_make_application_targets_do_not_rebuild_profile(self) -> None:
        for target in ("teleop", "takeover", "fixed-blend", "cosine-blend"):
            with self.subTest(target=target):
                result = subprocess.run(
                    [
                        "make",
                        "-n",
                        target,
                        "INPUT_SOURCE=spacemouse",
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
                assert match is not None
                tokens = shlex.split(match.group(1))
                self.assertEqual(
                    command_value(tokens, "--spacemouse-profile-path"),
                    "configs/spacemouse_profile.json",
                )
                for raw_option in (
                    "--translation-gain",
                    "--rotation-gain",
                    "--spacemouse-deadzone",
                    "--spacemouse-axis-mapping",
                    "--spacemouse-axis-signs",
                    "--spacemouse-axis-maxima",
                    "--spacemouse-stale-input-timeout-seconds",
                    "--spacemouse-open-button",
                    "--spacemouse-close-button",
                ):
                    self.assertNotIn(raw_option, tokens)


if __name__ == "__main__":
    unittest.main()
