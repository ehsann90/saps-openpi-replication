"""Gate-2 operator-pilot protocol and command contracts."""

from __future__ import annotations

from collections import Counter
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

from saps.evaluation.experiment_session import build_schedule
from saps.evaluation.experiment_session import ExperimentManifest
from saps.evaluation.experiment_session import load_manifest
from saps.evaluation.experiment_session import write_json_atomic
from saps.evaluation.gate2_protocol import GATE2_CONDITIONS
from saps.evaluation.gate2_protocol import GATE2_EXPECTED_MANIFEST
from saps.evaluation.gate2_protocol import GATE2_EXPERIMENT_ID
from saps.evaluation.gate2_protocol import GATE2_MANIFEST_PATH
from saps.evaluation.gate2_protocol import GATE2_MODES
from saps.evaluation.gate2_protocol import GATE2_OUTPUT_ROOT
from saps.evaluation.gate2_protocol import GATE2_PROFILE_PATH
from saps.evaluation.gate2_protocol import GATE2_TRIALS
from saps.evaluation.gate2_protocol import validate_gate2_manifest
from saps.evaluation.gate2_protocol import validate_gate2_protocol
from saps.human_input.spacemouse_profile import load_spacemouse_profile


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPOSITORY_ROOT / "scripts" / "run_operator_experiment.py"
RUNNER_SPEC = importlib.util.spec_from_file_location(
    "saps_gate2_operator_runner",
    RUNNER_PATH,
)
assert RUNNER_SPEC is not None and RUNNER_SPEC.loader is not None
operator_runner = importlib.util.module_from_spec(RUNNER_SPEC)
sys.modules[RUNNER_SPEC.name] = operator_runner
RUNNER_SPEC.loader.exec_module(operator_runner)

AUTONOMOUS_GATE2_SEEDS = {
    ("nominal", 0): 1594108130,
    ("nominal", 1): 252540981,
    ("nominal", 2): 846374469,
    ("nominal", 3): 1367343007,
    ("nominal", 4): 1611811172,
    ("p02", 0): 1805589632,
    ("p02", 1): 1280121721,
    ("p02", 2): 762959635,
    ("p02", 3): 1694273979,
    ("p02", 4): 1327508485,
    ("p06", 0): 531065419,
    ("p06", 1): 1630501309,
    ("p06", 2): 328150321,
    ("p06", 3): 427257404,
    ("p06", 4): 1893540928,
    ("p09", 0): 820753697,
    ("p09", 1): 279031833,
    ("p09", 2): 842708994,
    ("p09", 3): 409407624,
    ("p09", 4): 185183832,
}


def make_contract_args(target: str) -> list[str]:
    """Return parsed SAPS runtime arguments from a Make dry run."""

    result = subprocess.run(
        [
            "make",
            "-n",
            target,
            "SPACEMOUSE_DEVICE=/dev/input/by-id/test-device",
            "GATE2_MANIFEST=/tmp/wrong-manifest.json",
            "GATE2_PROFILE=/tmp/wrong-profile.json",
            "GATE2_OUTPUT=/tmp/wrong-output",
            "INPUT_SOURCE=keyboard",
            "SPACEMOUSE_PROFILE=/tmp/wrong-generic-profile.json",
        ],
        cwd=REPOSITORY_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    match = re.search(r'SAPS_RUNTIME_ARGS="([^"]*)"', result.stdout)
    if match is None:
        raise AssertionError(result.stdout)
    return shlex.split(match.group(1))


def command_value(command: list[str], option: str) -> str:
    return command[command.index(option) + 1]


class Gate2ProtocolTest(unittest.TestCase):
    def setUp(self) -> None:
        self.manifest = load_manifest(REPOSITORY_ROOT / GATE2_MANIFEST_PATH)

    def validate(self) -> dict[str, object]:
        return validate_gate2_protocol(
            manifest=self.manifest,
            input_source="spacemouse",
            spacemouse_profile_path=GATE2_PROFILE_PATH,
            spacemouse_device_path="/dev/input/by-id/test-device",
            output_root=Path(GATE2_OUTPUT_ROOT),
        )

    def test_manifest_is_the_exact_fixed_gate2_protocol(self) -> None:
        self.assertEqual(self.manifest.as_dict(), GATE2_EXPECTED_MANIFEST)
        self.assertEqual(self.manifest.experiment_id, GATE2_EXPERIMENT_ID)
        self.assertEqual(self.manifest.operator_max_steps, 280)
        self.assertEqual(self.manifest.fixed_autonomy_weight, 0.5)
        self.assertEqual(self.manifest.cosine_gain, 6.0)

    def test_schedule_has_exact_coverage_and_historical_seeds(self) -> None:
        result = self.validate()
        schedule = result["schedule"]
        assert isinstance(schedule, dict)
        episodes = schedule["episodes"]

        self.assertEqual(len(episodes), 60)
        self.assertEqual(
            Counter(episode["mode"] for episode in episodes),
            {mode: 20 for mode in GATE2_MODES},
        )
        self.assertEqual(
            Counter(episode["condition_id"] for episode in episodes),
            {condition: 15 for condition in GATE2_CONDITIONS},
        )
        self.assertEqual(
            Counter(
                (episode["mode"], episode["condition_id"])
                for episode in episodes
            ),
            {
                (mode, condition): 5
                for mode in GATE2_MODES
                for condition in GATE2_CONDITIONS
            },
        )
        self.assertEqual(
            {episode["trial_index"] for episode in episodes},
            set(GATE2_TRIALS),
        )
        episode_ids = [episode["episode_id"] for episode in episodes]
        self.assertEqual(len(episode_ids), len(set(episode_ids)))

        for identity, autonomous_seed in AUTONOMOUS_GATE2_SEEDS.items():
            condition_id, trial_index = identity
            matched = [
                episode
                for episode in episodes
                if episode["condition_id"] == condition_id
                and episode["trial_index"] == trial_index
            ]
            self.assertEqual(
                {episode["mode"] for episode in matched},
                set(GATE2_MODES),
            )
            self.assertEqual(
                {episode["policy_episode_seed"] for episode in matched},
                {autonomous_seed},
            )

    def test_schedule_regeneration_is_deterministic(self) -> None:
        first = build_schedule(
            manifest=self.manifest,
            task_id=1,
            output_root=Path(GATE2_OUTPUT_ROOT),
        )
        second = build_schedule(
            manifest=self.manifest,
            task_id=1,
            output_root=Path(GATE2_OUTPUT_ROOT),
        )
        self.assertEqual(first, second)
        units_per_trial = len(GATE2_MODES) * len(GATE2_CONDITIONS)
        starts = [
            first["episodes"][trial * units_per_trial]["episode_id"]
            for trial in GATE2_TRIALS
        ]
        self.assertEqual(len(starts), len(set(starts)))

    def test_gate2_rejects_protocol_and_input_drift(self) -> None:
        changed = dataclasses.replace(
            self.manifest,
            modes=(*self.manifest.modes, "takeover"),
        )
        with self.assertRaisesRegex(ValueError, "fixed.*protocol"):
            validate_gate2_manifest(changed)

        cases = (
            {
                "input_source": "keyboard",
                "error": "input_source='spacemouse'",
            },
            {
                "spacemouse_device_path": "",
                "error": "device path",
            },
            {
                "spacemouse_profile_path": "/tmp/another-profile.json",
                "error": "configs/spacemouse_profile.json",
            },
            {
                "output_root": Path("outputs/another-study"),
                "error": "output root",
            },
        )
        defaults = {
            "manifest": self.manifest,
            "input_source": "spacemouse",
            "spacemouse_profile_path": GATE2_PROFILE_PATH,
            "spacemouse_device_path": "/dev/input/by-id/test-device",
            "output_root": Path(GATE2_OUTPUT_ROOT),
        }
        for case in cases:
            error = str(case["error"])
            values = {**defaults, **case}
            del values["error"]
            with self.subTest(error=error):
                with self.assertRaisesRegex(ValueError, error):
                    validate_gate2_protocol(**values)

        profile = load_spacemouse_profile(
            REPOSITORY_ROOT / GATE2_PROFILE_PATH
        )
        changed_profile = dataclasses.replace(
            profile,
            translation_gain=0.31,
        )
        with mock.patch(
            "saps.evaluation.gate2_protocol.load_spacemouse_profile",
            return_value=changed_profile,
        ):
            with self.assertRaisesRegex(ValueError, "profile hash"):
                self.validate()

    def test_horizon_and_blend_parameters_reach_child_commands(self) -> None:
        args = operator_runner.Args(
            manifest_path=GATE2_MANIFEST_PATH,
            repository_commit="a" * 40,
            input_source="spacemouse",
            spacemouse_device_path="/dev/input/by-id/test-device",
            spacemouse_profile_path=GATE2_PROFILE_PATH,
        )
        schedule = self.validate()["schedule"]
        assert isinstance(schedule, dict)
        for episode in schedule["episodes"]:
            command = operator_runner._episode_command(
                manifest=self.manifest,
                episode=episode,
                attempt_root=Path("outputs/attempt"),
                args=args,
            )
            self.assertEqual(command_value(command, "--max-steps"), "280")
            if episode["mode"] != "teleoperation":
                self.assertEqual(
                    command_value(command, "--fixed-autonomy-weight"),
                    "0.5",
                )
                self.assertEqual(
                    command_value(command, "--cosine-gain"),
                    "6.0",
                )

    def test_generic_runner_cannot_bypass_gate2_guard(self) -> None:
        args = operator_runner.Args(
            manifest_path=GATE2_MANIFEST_PATH,
            repository_commit="a" * 40,
            input_source="keyboard",
        )
        with self.assertRaisesRegex(ValueError, "explicit protocol guard"):
            operator_runner.main(args)


class Gate2ResumeAndMakeContractTest(unittest.TestCase):
    def test_resume_preserves_state_and_rejects_schedule_drift(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output_root = Path(directory) / "session"
            manifest, schedule = operator_runner._initialize_experiment(
                manifest_path=REPOSITORY_ROOT / GATE2_MANIFEST_PATH,
                output_root=output_root,
            )
            schedule["episodes"][0]["status"] = "completed"
            schedule["episodes"][0]["attempt_count"] = 1
            schedule["episodes"][0]["attempts"] = [
                {"attempt_number": 1, "valid": True}
            ]
            write_json_atomic(output_root / "schedule.json", schedule)

            _, resumed = operator_runner._initialize_experiment(
                manifest_path=REPOSITORY_ROOT / GATE2_MANIFEST_PATH,
                output_root=output_root,
            )
            self.assertEqual(resumed, schedule)
            self.assertEqual(manifest.experiment_id, GATE2_EXPERIMENT_ID)

            schedule["episodes"][0]["mode"] = "takeover"
            write_json_atomic(output_root / "schedule.json", schedule)
            with self.assertRaisesRegex(ValueError, "immutable field"):
                operator_runner._initialize_experiment(
                    manifest_path=REPOSITORY_ROOT / GATE2_MANIFEST_PATH,
                    output_root=output_root,
                )

    def test_perturbation_provenance_rejects_changed_config(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "task.json"
            config = {
                "task_id": 1,
                "offsets": [{"id": "nominal", "dx": 0.0, "dy": 0.0}],
            }
            config_path.write_text(json.dumps(config), encoding="utf-8")
            manifest = ExperimentManifest(
                **{
                    **load_manifest(
                        REPOSITORY_ROOT / GATE2_MANIFEST_PATH
                    ).__dict__,
                    "experiment_id": "provenance_test",
                    "config_path": str(config_path),
                    "conditions": ("nominal",),
                    "modes": ("teleoperation",),
                    "trials_per_condition": 1,
                }
            )
            manifest_path = root / "manifest.json"
            write_json_atomic(manifest_path, manifest.as_dict())
            output_root = root / "output"
            args = operator_runner.Args(
                manifest_path=str(manifest_path),
                repository_commit="a" * 40,
                output_dir=str(output_root),
                input_source="keyboard",
                dry_run=True,
            )
            operator_runner.main(args)
            self.assertTrue(
                (output_root / "perturbation_config.json").is_file()
            )
            self.assertTrue((output_root / "session_protocol.json").is_file())

            config["note"] = "changed"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            with self.assertRaisesRegex(
                ValueError,
                "different perturbation configuration",
            ):
                operator_runner.main(args)

    def test_gate2_make_targets_are_fixed_and_profile_backed(self) -> None:
        preflight = make_contract_args("gate2-preflight")
        self.assertEqual(
            command_value(preflight, "--manifest-path"),
            GATE2_MANIFEST_PATH,
        )
        self.assertEqual(
            command_value(preflight, "--spacemouse-profile-path"),
            GATE2_PROFILE_PATH,
        )
        self.assertEqual(
            command_value(preflight, "--output-dir"),
            GATE2_OUTPUT_ROOT,
        )

        session = make_contract_args("gate2-session")
        self.assertEqual(
            command_value(session, "--manifest-path"),
            GATE2_MANIFEST_PATH,
        )
        self.assertEqual(
            command_value(session, "--required-protocol-id"),
            GATE2_EXPERIMENT_ID,
        )
        self.assertEqual(command_value(session, "--input-source"), "spacemouse")
        self.assertEqual(
            command_value(session, "--spacemouse-profile-path"),
            GATE2_PROFILE_PATH,
        )
        self.assertEqual(
            command_value(session, "--spacemouse-device-path"),
            "/dev/input/by-id/test-device",
        )
        self.assertEqual(
            command_value(session, "--output-dir"),
            GATE2_OUTPUT_ROOT,
        )

    def test_gate2_make_targets_require_device_path_before_docker(self) -> None:
        for target in ("gate2-preflight", "gate2-session"):
            with self.subTest(target=target):
                result = subprocess.run(
                    ["make", target, "SPACEMOUSE_DEVICE="],
                    cwd=REPOSITORY_ROOT,
                    check=False,
                    capture_output=True,
                    text=True,
                )
                self.assertEqual(result.returncode, 2)
                self.assertIn("requires SPACEMOUSE_DEVICE", result.stdout)


if __name__ == "__main__":
    unittest.main()
