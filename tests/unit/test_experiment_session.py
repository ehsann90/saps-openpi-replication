"""Tests for manifest-driven operator experiment scheduling."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from saps.evaluation.experiment_session import build_schedule
from saps.evaluation.experiment_session import ExperimentManifest
from saps.evaluation.experiment_session import manifest_sha256
from saps.evaluation.experiment_session import validate_summary
from saps.policies.seeding import make_policy_episode_seed
from saps.policies.seeding import SEED_PROTOCOL


def make_manifest() -> ExperimentManifest:
    """Return one compact valid test manifest."""

    return ExperimentManifest(
        schema_version=3,
        experiment_id="test_operator_v1",
        config_path="configs/test.json",
        conditions=("nominal", "p01"),
        modes=("teleoperation", "takeover", "cosine_blend"),
        trials_per_condition=3,
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


class ExperimentSessionTest(unittest.TestCase):
    def test_schedule_is_deterministic_and_counterbalanced(
        self,
    ) -> None:
        manifest = make_manifest()
        first = build_schedule(
            manifest=manifest,
            task_id=1,
            output_root=Path("outputs/test"),
        )
        second = build_schedule(
            manifest=manifest,
            task_id=1,
            output_root=Path("outputs/test"),
        )

        self.assertEqual(first, second)
        units_per_trial = len(manifest.conditions) * len(
            manifest.modes
        )
        self.assertEqual(
            len(first["episodes"]),
            units_per_trial * manifest.trials_per_condition,
        )

        starts = [
            first["episodes"][index * units_per_trial][
                "episode_id"
            ]
            for index in range(manifest.trials_per_condition)
        ]
        self.assertEqual(len(starts), len(set(starts)))

    def test_modes_share_the_autonomous_seed(self) -> None:
        manifest = make_manifest()
        schedule = build_schedule(
            manifest=manifest,
            task_id=1,
            output_root=Path("outputs/test"),
        )

        for condition_id in manifest.conditions:
            for trial_index in range(
                manifest.trials_per_condition
            ):
                matched = [
                    episode
                    for episode in schedule["episodes"]
                    if episode["condition_id"] == condition_id
                    and episode["trial_index"] == trial_index
                ]
                seeds = {
                    episode["policy_episode_seed"]
                    for episode in matched
                }
                expected = make_policy_episode_seed(
                    base_seed=manifest.policy_base_seed,
                    condition_id=condition_id,
                    trial_index=trial_index,
                    task_id=1,
                    initial_state_index=(
                        manifest.initial_state_index
                    ),
                )

                self.assertEqual(seeds, {expected})

    def test_manifest_hash_ignores_no_fields(self) -> None:
        manifest = make_manifest()
        changed = ExperimentManifest(
            **{
                **manifest.__dict__,
                "ordering_seed": manifest.ordering_seed + 1,
            }
        )

        self.assertNotEqual(
            manifest_sha256(manifest),
            manifest_sha256(changed),
        )

    def test_completed_summary_must_match_seed_and_identity(
        self,
    ) -> None:
        manifest = make_manifest()
        episode = build_schedule(
            manifest=manifest,
            task_id=1,
            output_root=Path("outputs/test"),
        )["episodes"][0]

        with tempfile.TemporaryDirectory() as directory:
            summary_path = Path(directory) / "summary.json"
            summary = {
                "arbitration_mode": episode["mode"],
                "condition_id": episode["condition_id"],
                "trial_index": episode["trial_index"],
                "initial_state_index": episode[
                    "initial_state_index"
                ],
                "policy_episode_seed": episode[
                    "policy_episode_seed"
                ],
                "policy_seed_protocol": SEED_PROTOCOL,
                "success": False,
                "termination_reason": "timeout",
                "control_steps": 1200,
            }
            summary_path.write_text(
                json.dumps(summary),
                encoding="utf-8",
            )
            summary_path.with_name("steps.jsonl").write_text(
                "{}\n",
                encoding="utf-8",
            )

            validated = validate_summary(
                summary_path=summary_path,
                episode=episode,
            )
            self.assertEqual(validated, summary)

            summary["policy_episode_seed"] += 1
            summary_path.write_text(
                json.dumps(summary),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(
                ValueError,
                "autonomous-matched",
            ):
                validate_summary(
                    summary_path=summary_path,
                    episode=episode,
                )

    def test_invalid_mode_is_rejected(self) -> None:
        manifest = make_manifest()
        invalid = ExperimentManifest(
            **{
                **manifest.__dict__,
                "modes": ("autonomous",),
            }
        )

        with self.assertRaisesRegex(ValueError, "Operator session modes"):
            invalid.validate()

    def test_invalid_or_unsorted_operator_gains_are_rejected(self) -> None:
        manifest = make_manifest()
        too_large = ExperimentManifest(
            **{**manifest.__dict__, "fast_translation_gain": 1.1}
        )
        unsorted = ExperimentManifest(
            **{**manifest.__dict__, "normal_rotation_gain": 0.1}
        )

        with self.assertRaisesRegex(ValueError, "within"):
            too_large.validate()

        with self.assertRaisesRegex(ValueError, "non-decreasing"):
            unsorted.validate()


if __name__ == "__main__":
    unittest.main()
