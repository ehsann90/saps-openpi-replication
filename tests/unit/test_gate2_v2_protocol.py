"""Deterministic contracts for the matched Gate-2 v2 design."""

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
import unittest

from saps.evaluation.experiment_session import load_manifest
from saps.evaluation.gate2_v2_protocol import (
    build_gate2_v2_autonomous_schedule,
)
from saps.evaluation.gate2_v2_protocol import (
    build_gate2_v2_shared_schedule,
)
from saps.evaluation.gate2_v2_protocol import GATE2_V2_ALL_MODES
from saps.evaluation.gate2_v2_protocol import (
    GATE2_V2_AUTONOMOUS_EXPERIMENT_ID,
)
from saps.evaluation.gate2_v2_protocol import (
    GATE2_V2_AUTONOMOUS_PROTOCOL_PATH,
)
from saps.evaluation.gate2_v2_protocol import GATE2_V2_CONDITIONS
from saps.evaluation.gate2_v2_protocol import GATE2_V2_MANIFEST_PATH
from saps.evaluation.gate2_v2_protocol import GATE2_V2_MAX_STEPS
from saps.evaluation.gate2_v2_protocol import GATE2_V2_ORDERING_METHOD
from saps.evaluation.gate2_v2_protocol import GATE2_V2_SHARED_MODES
from saps.evaluation.gate2_v2_protocol import GATE2_V2_SHARED_OUTPUT_ROOT
from saps.evaluation.gate2_v2_protocol import GATE2_V2_TRIALS
from saps.evaluation.gate2_v2_protocol import gate2_v2_ordering_metrics
from saps.evaluation.gate2_v2_protocol import (
    load_gate2_v2_autonomous_protocol,
)
from saps.evaluation.gate2_v2_protocol import (
    validate_gate2_v2_matched_design,
)
from saps.policies.seeding import make_policy_episode_seed
from saps.policies.seeding import SEED_PROTOCOL


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
OPERATOR_RUNNER_PATH = REPOSITORY_ROOT / "scripts" / "run_operator_experiment.py"
OPERATOR_RUNNER_SPEC = importlib.util.spec_from_file_location(
    "saps_gate2_v2_operator_runner",
    OPERATOR_RUNNER_PATH,
)
assert (
    OPERATOR_RUNNER_SPEC is not None
    and OPERATOR_RUNNER_SPEC.loader is not None
)
operator_runner = importlib.util.module_from_spec(OPERATOR_RUNNER_SPEC)
sys.modules[OPERATOR_RUNNER_SPEC.name] = operator_runner
OPERATOR_RUNNER_SPEC.loader.exec_module(operator_runner)


class Gate2V2ProtocolTest(unittest.TestCase):
    def setUp(self) -> None:
        self.manifest = load_manifest(REPOSITORY_ROOT / GATE2_V2_MANIFEST_PATH)
        self.protocol = load_gate2_v2_autonomous_protocol(
            REPOSITORY_ROOT / GATE2_V2_AUTONOMOUS_PROTOCOL_PATH
        )
        self.shared = build_gate2_v2_shared_schedule(
            manifest=self.manifest,
            task_id=1,
            output_root=Path(GATE2_V2_SHARED_OUTPUT_ROOT),
        )
        self.autonomous = build_gate2_v2_autonomous_schedule(self.protocol)

    def test_shared_schedule_has_exact_two_mode_coverage(self) -> None:
        episodes = self.shared["episodes"]
        self.assertEqual(len(episodes), 40)
        self.assertNotIn("teleoperation", {row["mode"] for row in episodes})
        self.assertEqual(
            Counter(row["mode"] for row in episodes),
            {mode: 20 for mode in GATE2_V2_SHARED_MODES},
        )
        self.assertEqual(
            Counter(row["condition_id"] for row in episodes),
            {condition: 10 for condition in GATE2_V2_CONDITIONS},
        )
        self.assertEqual(
            set(
                Counter(
                    (row["mode"], row["condition_id"])
                    for row in episodes
                ).values()
            ),
            {5},
        )

    def test_shared_schedule_has_five_complete_constrained_rounds(self) -> None:
        expected = {
            (mode, condition)
            for mode in GATE2_V2_SHARED_MODES
            for condition in GATE2_V2_CONDITIONS
        }
        for trial in GATE2_V2_TRIALS:
            rows = self.shared["episodes"][trial * 8:(trial + 1) * 8]
            self.assertEqual(len(rows), 8)
            self.assertEqual({row["trial_index"] for row in rows}, {trial})
            self.assertEqual(
                {(row["mode"], row["condition_id"]) for row in rows},
                expected,
            )
        metrics = gate2_v2_ordering_metrics(self.shared)
        self.assertEqual(metrics["ordering_method"], GATE2_V2_ORDERING_METHOD)
        self.assertEqual(metrics["maximum_same_condition_run_length"], 1)
        self.assertLessEqual(metrics["maximum_same_mode_run_length"], 2)
        self.assertGreaterEqual(metrics["minimum_pair_intervening_episodes"], 1)
        self.assertTrue(
            all(
                count in {2, 3}
                for count in metrics[
                    "fixed_before_cosine_by_condition"
                ].values()
            )
        )

    def test_schedule_is_deterministic_from_ordering_seed(self) -> None:
        regenerated = build_gate2_v2_shared_schedule(
            manifest=self.manifest,
            task_id=1,
            output_root=Path(GATE2_V2_SHARED_OUTPUT_ROOT),
        )
        self.assertEqual(self.shared, regenerated)
        with self.assertRaisesRegex(ValueError, "manifest"):
            build_gate2_v2_shared_schedule(
                manifest=dataclasses.replace(
                    self.manifest,
                    ordering_seed=self.manifest.ordering_seed + 1,
                ),
                task_id=1,
                output_root=Path(GATE2_V2_SHARED_OUTPUT_ROOT),
            )

    def test_shared_child_uses_matched_horizon_replan_and_checkpoint(self) -> None:
        episode = self.shared["episodes"][0]
        args = operator_runner.Args(
            manifest_path=GATE2_V2_MANIFEST_PATH,
            repository_commit="a" * 40,
            input_source="spacemouse",
        )
        command = operator_runner._episode_command(
            manifest=self.manifest,
            episode=episode,
            attempt_root=Path("outputs/test-attempt"),
            args=args,
        )
        self.assertEqual(
            command[command.index("--max-steps") + 1],
            str(self.protocol["max_steps"]),
        )
        self.assertEqual(
            command[command.index("--replan-steps") + 1],
            str(self.protocol["replan_steps"]),
        )
        self.assertEqual(
            command[command.index("--required-policy-config-name") + 1],
            self.protocol["policy_config_name"],
        )
        self.assertEqual(
            command[command.index("--required-policy-checkpoint") + 1],
            self.protocol["policy_checkpoint"],
        )

    def test_autonomous_schedule_is_exact_and_output_independent(self) -> None:
        episodes = self.autonomous["episodes"]
        self.assertEqual(len(episodes), 20)
        self.assertEqual(
            {row["condition_id"] for row in episodes},
            set(GATE2_V2_CONDITIONS),
        )
        self.assertEqual(
            {row["trial_index"] for row in episodes},
            set(GATE2_V2_TRIALS),
        )
        self.assertEqual({row["mode"] for row in episodes}, {"autonomous"})
        self.assertEqual(self.protocol["max_steps"], GATE2_V2_MAX_STEPS)
        self.assertEqual(self.protocol["replan_steps"], 5)
        self.assertEqual(self.protocol["settle_steps"], 10)
        self.assertEqual(self.protocol["control_frequency_hz"], 20.0)

    def test_all_three_modes_form_twenty_exact_seeded_triplets(self) -> None:
        result = validate_gate2_v2_matched_design(
            shared_schedule=self.shared,
            autonomous_schedule=self.autonomous,
        )
        self.assertEqual(result["matched_triplets"], 20)
        self.assertEqual(result["total_episodes"], 60)
        self.assertEqual(
            result["episodes_by_mode"],
            {mode: 20 for mode in GATE2_V2_ALL_MODES},
        )
        rows = [*self.shared["episodes"], *self.autonomous["episodes"]]
        for condition in GATE2_V2_CONDITIONS:
            for trial in GATE2_V2_TRIALS:
                matched = [
                    row
                    for row in rows
                    if row["condition_id"] == condition
                    and row["trial_index"] == trial
                ]
                expected = make_policy_episode_seed(
                    base_seed=20260724,
                    condition_id=condition,
                    trial_index=trial,
                    task_id=1,
                    initial_state_index=0,
                )
                self.assertEqual(
                    {row["mode"] for row in matched},
                    set(GATE2_V2_ALL_MODES),
                )
                self.assertEqual(
                    {row["policy_episode_seed"] for row in matched},
                    {expected},
                )
                self.assertEqual(
                    {row["policy_seed_protocol"] for row in matched},
                    {SEED_PROTOCOL},
                )

    def test_matched_design_rejects_a_mode_specific_seed(self) -> None:
        changed = {
            **self.autonomous,
            "episodes": [dict(row) for row in self.autonomous["episodes"]],
        }
        changed["episodes"][0]["policy_episode_seed"] += 1
        with self.assertRaisesRegex(ValueError, "triplets"):
            validate_gate2_v2_matched_design(
                shared_schedule=self.shared,
                autonomous_schedule=changed,
            )

    def test_v2_manifest_has_one_keyboard_gain_pair(self) -> None:
        manifest_data = self.manifest.as_dict()
        self.assertEqual(manifest_data["keyboard_translation_gain"], 0.5)
        self.assertEqual(manifest_data["keyboard_rotation_gain"], 0.2)
        self.assertFalse(any("fine_" in key for key in manifest_data))
        self.assertFalse(any("fast_" in key for key in manifest_data))

    def test_v2_manifest_rejects_legacy_speed_gain_fields(self) -> None:
        manifest_data = self.manifest.as_dict()
        manifest_data["fine_translation_gain"] = 0.25
        with self.assertRaisesRegex(ValueError, "unsupported fields"):
            from saps.evaluation.experiment_session import ExperimentManifest

            ExperimentManifest.from_dict(json.loads(json.dumps(manifest_data)))

    def test_autonomous_make_contract_is_guarded_and_exact(self) -> None:
        result = subprocess.run(
            ["make", "-n", "gate2-autonomous"],
            cwd=REPOSITORY_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        match = re.search(r'SAPS_RUNTIME_ARGS="([^"]*)"', result.stdout)
        self.assertIsNotNone(match)
        assert match is not None
        command = shlex.split(match.group(1))
        self.assertEqual(
            command[command.index("--condition-ids") + 1],
            "nominal,p02,p06,p09",
        )
        self.assertEqual(command[command.index("--max-steps") + 1], "280")
        self.assertEqual(command[command.index("--replan-steps") + 1], "5")
        self.assertEqual(
            command[command.index("--required-protocol-id") + 1],
            GATE2_V2_AUTONOMOUS_EXPERIMENT_ID,
        )


if __name__ == "__main__":
    unittest.main()
