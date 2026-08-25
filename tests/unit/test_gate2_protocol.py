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
from saps.evaluation.experiment_session import validate_summary
from saps.evaluation.experiment_session import write_json_atomic
from saps.evaluation.gate2_protocol import GATE2_CONDITIONS
from saps.evaluation.gate2_protocol import GATE2_EXPECTED_MANIFEST
from saps.evaluation.gate2_protocol import GATE2_EXPERIMENT_ID
from saps.evaluation.gate2_protocol import GATE2_MANIFEST_PATH
from saps.evaluation.gate2_protocol import GATE2_ORDERING_METHOD
from saps.evaluation.gate2_protocol import GATE2_MODES
from saps.evaluation.gate2_protocol import GATE2_OUTPUT_ROOT
from saps.evaluation.gate2_protocol import GATE2_PROFILE_PATH
from saps.evaluation.gate2_protocol import (
    GATE2_REDO_REQUIRED_TERMINATION_REASONS,
)
from saps.evaluation.gate2_protocol import GATE2_TRIALS
from saps.evaluation.gate2_protocol import GATE2_UNITS_PER_TRIAL
from saps.evaluation.gate2_protocol import build_gate2_schedule
from saps.evaluation.gate2_protocol import gate2_ordering_metrics
from saps.evaluation.gate2_protocol import validate_gate2_attempt_completion
from saps.evaluation.gate2_protocol import validate_gate2_manifest
from saps.evaluation.gate2_protocol import validate_gate2_protocol
from saps.human_input.spacemouse_profile import load_spacemouse_profile
from saps.policies.seeding import make_policy_episode_seed


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


def gate2_human_input(
    *,
    connected: bool = True,
    physical_device_connected: bool = True,
    armed: bool = True,
    stale_input: bool = False,
) -> dict[str, object]:
    """Return one synthetic SpaceMouse integrity record."""

    return {
        "input_source": "spacemouse",
        "connected": connected,
        "physical_device_connected": physical_device_connected,
        "armed": armed,
        "stale_input": stale_input,
    }


def write_gate2_attempt(
    root: Path,
    *,
    termination_reason: str = "success",
    success: bool = True,
    control_steps: int = 1,
    human_inputs: list[dict[str, object]] | None = None,
    summary_values: dict[str, object] | None = None,
) -> tuple[Path, dict[str, object]]:
    """Write the raw files needed by Gate-2 completion validation."""

    if human_inputs is None:
        human_inputs = [gate2_human_input() for _ in range(control_steps)]
    if len(human_inputs) != control_steps:
        raise ValueError("human_inputs must match control_steps")
    root.mkdir(parents=True, exist_ok=True)
    summary: dict[str, object] = {
        "arbitration_mode": "teleoperation",
        "success": success,
        "termination_reason": termination_reason,
        "control_steps": control_steps,
        **(summary_values or {}),
    }
    summary_path = root / "summary.json"
    write_json_atomic(summary_path, summary)
    (root / "steps.jsonl").write_text(
        "".join(
            json.dumps({"human_input": value}) + "\n"
            for value in human_inputs
        ),
        encoding="utf-8",
    )
    return summary_path, summary


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

    def test_schedule_has_exact_coverage_and_matched_seeds(self) -> None:
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

        for condition_id in GATE2_CONDITIONS:
            for trial_index in GATE2_TRIALS:
                matched = [
                    episode
                    for episode in episodes
                    if episode["condition_id"] == condition_id
                    and episode["trial_index"] == trial_index
                ]
                expected_seed = make_policy_episode_seed(
                    base_seed=self.manifest.policy_base_seed,
                    condition_id=condition_id,
                    trial_index=trial_index,
                    task_id=1,
                    initial_state_index=self.manifest.initial_state_index,
                )
                self.assertEqual(
                    {episode["mode"] for episode in matched},
                    set(GATE2_MODES),
                )
                self.assertEqual(
                    {
                        episode["policy_episode_seed"]
                        for episode in matched
                    },
                    {expected_seed},
                )

    def test_constrained_schedule_is_deterministic_and_seeded(self) -> None:
        first = build_gate2_schedule(
            manifest=self.manifest,
            task_id=1,
            output_root=Path(GATE2_OUTPUT_ROOT),
        )
        second = build_gate2_schedule(
            manifest=self.manifest,
            task_id=1,
            output_root=Path(GATE2_OUTPUT_ROOT),
        )
        self.assertEqual(first, second)
        changed_manifest = dataclasses.replace(
            self.manifest,
            ordering_seed=self.manifest.ordering_seed + 1,
        )
        changed = build_gate2_schedule(
            manifest=changed_manifest,
            task_id=1,
            output_root=Path(GATE2_OUTPUT_ROOT),
        )
        first_order = [
            (episode["mode"], episode["condition_id"])
            for episode in first["episodes"]
        ]
        changed_order = [
            (episode["mode"], episode["condition_id"])
            for episode in changed["episodes"]
        ]
        self.assertNotEqual(first_order, changed_order)
        self.assertEqual(
            {
                (
                    episode["mode"],
                    episode["condition_id"],
                    episode["trial_index"],
                    episode["policy_episode_seed"],
                )
                for episode in first["episodes"]
            },
            {
                (
                    episode["mode"],
                    episode["condition_id"],
                    episode["trial_index"],
                    episode["policy_episode_seed"],
                )
                for episode in changed["episodes"]
            },
        )

    def test_each_trial_round_contains_all_twelve_units(self) -> None:
        episodes = self.validate()["schedule"]["episodes"]
        for trial_index in GATE2_TRIALS:
            start = trial_index * GATE2_UNITS_PER_TRIAL
            trial_episodes = episodes[
                start:start + GATE2_UNITS_PER_TRIAL
            ]
            self.assertEqual(
                {episode["trial_index"] for episode in trial_episodes},
                {trial_index},
            )
            self.assertEqual(
                Counter(episode["mode"] for episode in trial_episodes),
                {mode: 4 for mode in GATE2_MODES},
            )
            self.assertEqual(
                Counter(
                    episode["condition_id"] for episode in trial_episodes
                ),
                {condition_id: 3 for condition_id in GATE2_CONDITIONS},
            )
            self.assertEqual(
                {
                    (episode["mode"], episode["condition_id"])
                    for episode in trial_episodes
                },
                {
                    (mode, condition_id)
                    for mode in GATE2_MODES
                    for condition_id in GATE2_CONDITIONS
                },
            )

    def test_all_ordering_constraints_are_satisfied(self) -> None:
        schedule = self.validate()["schedule"]
        ordering = gate2_ordering_metrics(schedule)
        self.assertEqual(
            schedule["ordering_method"],
            GATE2_ORDERING_METHOD,
        )
        self.assertEqual(ordering["ordering_method"], GATE2_ORDERING_METHOD)
        self.assertEqual(ordering["maximum_same_condition_run_length"], 1)
        self.assertLessEqual(ordering["maximum_same_mode_run_length"], 2)
        self.assertGreaterEqual(
            ordering["minimum_same_identity_intervening_episodes"],
            1,
        )
        for precedence in ordering["pairwise_mode_precedence"].values():
            self.assertTrue(
                all(count in {2, 3} for count in precedence.values())
            )

    def test_legacy_scheduler_retains_previous_cyclic_order(self) -> None:
        legacy = build_schedule(
            manifest=self.manifest,
            task_id=1,
            output_root=Path(GATE2_OUTPUT_ROOT),
        )
        self.assertNotIn("ordering_method", legacy)
        self.assertEqual(
            [
                (episode["mode"], episode["condition_id"])
                for episode in legacy["episodes"][:12]
            ],
            [
                ("cosine_blend", "p06"),
                ("cosine_blend", "p02"),
                ("teleoperation", "p09"),
                ("cosine_blend", "p09"),
                ("cosine_blend", "nominal"),
                ("teleoperation", "nominal"),
                ("fixed_blend", "p09"),
                ("fixed_blend", "p06"),
                ("fixed_blend", "p02"),
                ("teleoperation", "p06"),
                ("fixed_blend", "nominal"),
                ("teleoperation", "p02"),
            ],
        )

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


class Gate2AttemptCompletionTest(unittest.TestCase):
    def validate_attempt(
        self,
        root: Path,
        **values: object,
    ) -> None:
        summary_path, summary = write_gate2_attempt(root, **values)
        validate_gate2_attempt_completion(
            summary_path=summary_path,
            summary=summary,
        )

    def test_success_is_accepted(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            self.validate_attempt(Path(directory))

    def test_timeout_is_accepted(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            self.validate_attempt(
                Path(directory),
                termination_reason="timeout",
                success=False,
                control_steps=280,
            )

    def test_operator_abort_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "requires redo"):
                self.validate_attempt(
                    Path(directory),
                    termination_reason="operator_abort",
                    success=False,
                )

    def test_browser_disconnect_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            inputs = [gate2_human_input(connected=False)]
            with self.assertRaisesRegex(ValueError, "operator_disconnected"):
                self.validate_attempt(
                    Path(directory),
                    human_inputs=inputs,
                )

    def test_spacemouse_disconnect_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            inputs = [
                gate2_human_input(physical_device_connected=False)
            ]
            with self.assertRaisesRegex(
                ValueError,
                "input_device_disconnected",
            ):
                self.validate_attempt(
                    Path(directory),
                    human_inputs=inputs,
                )

    def test_mid_episode_disarm_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            inputs = [
                gate2_human_input(),
                gate2_human_input(armed=False),
            ]
            with self.assertRaisesRegex(ValueError, "operator_disarmed"):
                self.validate_attempt(
                    Path(directory),
                    control_steps=2,
                    human_inputs=inputs,
                )

    def test_disarm_during_policy_wait_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            summary_path, summary = write_gate2_attempt(root)
            summary["arbitration_mode"] = "fixed_blend"
            write_json_atomic(summary_path, summary)
            (root / "scheduler_waits.jsonl").write_text(
                json.dumps(
                    {
                        "human_input": gate2_human_input(armed=False),
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "operator_disarmed"):
                validate_gate2_attempt_completion(
                    summary_path=summary_path,
                    summary=summary,
                )

    def test_environment_termination_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "requires redo"):
                self.validate_attempt(
                    Path(directory),
                    termination_reason="environment_terminated",
                    success=False,
                )

    def test_neutral_stale_spacemouse_is_accepted(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            inputs = [gate2_human_input(stale_input=True)]
            self.validate_attempt(
                Path(directory),
                human_inputs=inputs,
            )

    def test_all_predeclared_redo_reasons_are_rejected(self) -> None:
        for reason in GATE2_REDO_REQUIRED_TERMINATION_REASONS:
            with self.subTest(reason=reason):
                with tempfile.TemporaryDirectory() as directory:
                    with self.assertRaisesRegex(ValueError, "requires redo"):
                        self.validate_attempt(
                            Path(directory),
                            termination_reason=reason,
                            success=False,
                        )

    def test_invalid_redo_preserves_previous_valid_selection(self) -> None:
        previous = {
            "valid": True,
            "selected_for_analysis": True,
        }
        invalid = {
            "valid": False,
            "selected_for_analysis": False,
            "error": None,
        }
        episode = {
            "status": "running",
            "attempts": [previous, invalid],
        }

        operator_runner._mark_attempt_invalid(
            episode=episode,
            attempt=invalid,
            redo_requested=True,
            previous_selected_valid=True,
            error=ValueError("operator_disarmed"),
        )

        self.assertEqual(episode["status"], "completed")
        self.assertTrue(previous["selected_for_analysis"])
        self.assertFalse(invalid["selected_for_analysis"])
        self.assertEqual(invalid["error"], "operator_disarmed")

    def test_legacy_summary_acceptance_remains_unchanged(self) -> None:
        manifest = load_manifest(REPOSITORY_ROOT / GATE2_MANIFEST_PATH)
        schedule = build_schedule(
            manifest=manifest,
            task_id=1,
            output_root=Path("outputs/legacy-test"),
        )
        episode = schedule["episodes"][0]
        with tempfile.TemporaryDirectory() as directory:
            summary_path, _ = write_gate2_attempt(
                Path(directory),
                termination_reason="operator_abort",
                success=False,
                summary_values={
                    "arbitration_mode": episode["mode"],
                    "condition_id": episode["condition_id"],
                    "trial_index": episode["trial_index"],
                    "initial_state_index": episode["initial_state_index"],
                    "policy_episode_seed": episode["policy_episode_seed"],
                    "policy_seed_protocol": episode[
                        "policy_seed_protocol"
                    ],
                },
            )

            summary = validate_summary(
                summary_path=summary_path,
                episode=episode,
            )
            self.assertEqual(
                summary["termination_reason"],
                "operator_abort",
            )
            with self.assertRaisesRegex(ValueError, "requires redo"):
                validate_gate2_attempt_completion(
                    summary_path=summary_path,
                    summary=summary,
                )


class Gate2ResumeAndMakeContractTest(unittest.TestCase):
    def test_preflight_and_session_use_the_same_gate2_order(self) -> None:
        manifest = load_manifest(REPOSITORY_ROOT / GATE2_MANIFEST_PATH)
        preflight = validate_gate2_protocol(
            manifest=manifest,
            input_source="spacemouse",
            spacemouse_profile_path=GATE2_PROFILE_PATH,
            spacemouse_device_path="/dev/input/by-id/test-device",
            output_root=Path(GATE2_OUTPUT_ROOT),
        )["schedule"]

        with tempfile.TemporaryDirectory() as directory:
            _, session = operator_runner._initialize_experiment(
                manifest_path=REPOSITORY_ROOT / GATE2_MANIFEST_PATH,
                output_root=Path(directory) / "session",
                required_protocol_id=GATE2_EXPERIMENT_ID,
            )

        def ordering_identity(schedule: dict[str, object]) -> tuple:
            episodes = schedule["episodes"]
            assert isinstance(episodes, list)
            return (
                schedule["ordering_method"],
                tuple(
                    (
                        episode["order_index"],
                        episode["mode"],
                        episode["condition_id"],
                        episode["trial_index"],
                        episode["policy_episode_seed"],
                    )
                    for episode in episodes
                ),
            )

        self.assertEqual(
            ordering_identity(preflight),
            ordering_identity(session),
        )

    def test_resume_preserves_state_and_rejects_schedule_drift(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output_root = Path(directory) / "session"
            manifest, schedule = operator_runner._initialize_experiment(
                manifest_path=REPOSITORY_ROOT / GATE2_MANIFEST_PATH,
                output_root=output_root,
                required_protocol_id=GATE2_EXPERIMENT_ID,
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
                required_protocol_id=GATE2_EXPERIMENT_ID,
            )
            self.assertEqual(resumed, schedule)
            self.assertEqual(manifest.experiment_id, GATE2_EXPERIMENT_ID)

            schedule["ordering_method"] = "changed_ordering_method"
            write_json_atomic(output_root / "schedule.json", schedule)
            with self.assertRaisesRegex(ValueError, "ordering_method"):
                operator_runner._initialize_experiment(
                    manifest_path=REPOSITORY_ROOT / GATE2_MANIFEST_PATH,
                    output_root=output_root,
                    required_protocol_id=GATE2_EXPERIMENT_ID,
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
