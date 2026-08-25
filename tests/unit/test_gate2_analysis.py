"""Synthetic, hardware-independent Gate-2 analysis tests."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import tempfile
import unittest

from saps.evaluation.experiment_session import load_manifest
from saps.evaluation.gate2_analysis import analyze_gate2_collection
from saps.evaluation.gate2_protocol import build_gate2_schedule
from saps.evaluation.gate2_protocol import GATE2_CONFIG_SHA256
from saps.evaluation.gate2_protocol import GATE2_EXPERIMENT_ID
from saps.evaluation.gate2_protocol import GATE2_MANIFEST_PATH
from saps.evaluation.gate2_protocol import GATE2_PROFILE_SHA256


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def write_jsonl(path: Path, values: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(value) + "\n" for value in values),
        encoding="utf-8",
    )


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as file:
        return list(csv.DictReader(file))


class Gate2AnalysisFixture:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.session = root / "session"
        self.autonomous = root / "autonomous"
        self.output = root / "analysis"
        self.manifest = load_manifest(REPOSITORY_ROOT / GATE2_MANIFEST_PATH)
        self.schedule = build_gate2_schedule(
            manifest=self.manifest,
            task_id=1,
            output_root=self.session,
        )
        write_json(self.session / "manifest.json", self.manifest.as_dict())
        write_json(self.session / "schedule.json", self.schedule)
        write_json(
            self.session / "human_input.json",
            {
                "input_source": "spacemouse",
                "spacemouse_profile": {
                    "sha256": GATE2_PROFILE_SHA256,
                },
            },
        )
        write_json(
            self.session / "repository_provenance.json",
            {
                "repository_commit": "a" * 40,
                "manifest_sha256": self.schedule["manifest_sha256"],
            },
        )
        write_json(
            self.session / "session_protocol.json",
            {"required_protocol_id": GATE2_EXPERIMENT_ID},
        )
        write_json(
            self.session / "perturbation_config.json",
            {"sha256": GATE2_CONFIG_SHA256},
        )
        self.autonomous.mkdir(parents=True)

    def episode(
        self,
        mode: str,
        condition_id: str = "nominal",
        trial_index: int = 0,
    ) -> dict[str, object]:
        return next(
            episode
            for episode in self.schedule["episodes"]
            if episode["mode"] == mode
            and episode["condition_id"] == condition_id
            and episode["trial_index"] == trial_index
        )

    def add_attempt(
        self,
        episode: dict[str, object],
        *,
        attempt_number: int = 1,
        selected: bool = True,
        success: bool = True,
        termination_reason: str = "success",
        steps: list[dict[str, object]] | None = None,
        waits: list[dict[str, object]] | None = None,
        summary_seed: int | None = None,
        profile_sha256: str = GATE2_PROFILE_SHA256,
    ) -> Path:
        mode = str(episode["mode"])
        if steps is None:
            steps = [self.step(mode=mode, control_step=0)]
        attempt_root = (
            self.session
            / "synthetic_attempts"
            / str(episode["episode_id"])
            / f"attempt_{attempt_number:03d}"
        )
        summary_path = attempt_root / "summary.json"
        control_steps = len(steps)
        summary = {
            "condition_id": episode["condition_id"],
            "arbitration_mode": mode,
            "trial_index": episode["trial_index"],
            "initial_state_index": episode["initial_state_index"],
            "policy_episode_seed": (
                episode["policy_episode_seed"]
                if summary_seed is None
                else summary_seed
            ),
            "policy_seed_protocol": episode["policy_seed_protocol"],
            "success": success,
            "termination_reason": termination_reason,
            "control_steps": control_steps,
            "control_frequency_hz": 20.0,
            "simulated_control_seconds": control_steps / 20.0,
            "control_elapsed_seconds": control_steps / 20.0 + 0.2,
            "total_elapsed_seconds": control_steps / 20.0 + 1.0,
            "spacemouse_profile": {
                "sha256": profile_sha256,
            },
        }
        write_json(summary_path, summary)
        write_jsonl(attempt_root / "steps.jsonl", steps)
        if mode != "teleoperation":
            write_jsonl(attempt_root / "scheduler_waits.jsonl", waits or [])
        attempts = episode["attempts"]
        assert isinstance(attempts, list)
        attempts.append(
            {
                "attempt_number": attempt_number,
                "summary_path": str(summary_path),
                "valid": True,
                "selected_for_analysis": selected,
            }
        )
        episode["attempt_count"] = max(
            int(episode["attempt_count"]),
            attempt_number,
        )
        if selected:
            episode["status"] = "completed"
            episode["success"] = success
            episode["termination_reason"] = termination_reason
        write_json(self.session / "schedule.json", self.schedule)
        return summary_path

    @staticmethod
    def step(
        *,
        mode: str,
        control_step: int,
        weight: float | None = None,
        active: bool = False,
        inference_latency: float | None = None,
    ) -> dict[str, object]:
        action = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0] if active else [0.0] * 6 + [-1.0]
        return {
            "control_step": control_step,
            "human_action": action,
            "human_active": active,
            "operator_motion_active": active,
            "effective_autonomy_weight": weight,
            "cosine_similarity": 0.0 if weight is not None else None,
            "inference_latency_seconds": inference_latency,
        }

    def add_autonomous(
        self,
        episode: dict[str, object],
        *,
        seed_offset: int = 0,
        success: bool = False,
    ) -> None:
        path = (
            self.autonomous
            / f"seed_{int(episode['policy_episode_seed']) + seed_offset}"
            / "summary.json"
        )
        write_json(
            path,
            {
                "condition_id": episode["condition_id"],
                "trial_index": episode["trial_index"],
                "initial_state_index": episode["initial_state_index"],
                "policy_episode_seed": (
                    int(episode["policy_episode_seed"]) + seed_offset
                ),
                "success": success,
                "control_steps": 280,
            },
        )

    def analyze(self, *, write_plots: bool = False) -> dict[str, object]:
        return analyze_gate2_collection(
            session_root=self.session,
            autonomous_root=self.autonomous,
            output_dir=self.output,
            write_plots=write_plots,
        )


class Gate2AnalysisTest(unittest.TestCase):
    def test_incomplete_collection_preserves_all_sixty_rows(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = Gate2AnalysisFixture(Path(directory))
            report = fixture.analyze(write_plots=True)

            self.assertTrue(report["analysis_valid"])
            self.assertFalse(report["collection_complete"])
            self.assertEqual(report["selected_analyzable_episode_count"], 0)
            self.assertEqual(
                len(read_csv(fixture.output / "episode_metrics.csv")),
                60,
            )
            self.assertEqual(
                len(read_csv(fixture.output / "mode_summary.csv")),
                3,
            )
            self.assertEqual(
                len(read_csv(fixture.output / "condition_mode_summary.csv")),
                12,
            )
            for name in (
                "matched_autonomous_comparisons.csv",
                "fixed_blend_diagnostics.csv",
                "cosine_blend_diagnostics.csv",
                "policy_wait_summary.csv",
                "validation_report.json",
                "REPORT.md",
            ):
                self.assertTrue((fixture.output / name).is_file())
            for name in (
                "observed_success_by_cell.png",
                "human_active_fraction_by_mode.png",
                "policy_wait_fraction_by_mode.png",
                "cosine_weight_diagnostic.png",
            ):
                self.assertTrue((fixture.output / "plots" / name).is_file())

    def test_failure_timeout_uses_full_horizon_and_denominator(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = Gate2AnalysisFixture(Path(directory))
            episode = fixture.episode("teleoperation")
            steps = [
                fixture.step(mode="teleoperation", control_step=index)
                for index in range(280)
            ]
            fixture.add_attempt(
                episode,
                success=False,
                termination_reason="timeout",
                steps=steps,
            )
            report = fixture.analyze(write_plots=True)

            self.assertTrue(report["analysis_valid"])
            row = next(
                row
                for row in read_csv(fixture.output / "episode_metrics.csv")
                if row["episode_id"] == episode["episode_id"]
            )
            self.assertEqual(float(row["simulated_duration_seconds"]), 14.0)
            mode = next(
                row
                for row in read_csv(fixture.output / "mode_summary.csv")
                if row["mode"] == "teleoperation"
            )
            self.assertEqual(mode["n_failure"], "1")
            self.assertEqual(mode["n_selected_valid"], "1")
            self.assertEqual(float(mode["success_rate_observed"]), 0.0)

    def test_selected_redo_attempt_is_used(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = Gate2AnalysisFixture(Path(directory))
            episode = fixture.episode("teleoperation")
            fixture.add_attempt(episode, attempt_number=1, selected=False)
            fixture.add_attempt(
                episode,
                attempt_number=2,
                selected=True,
                success=False,
                termination_reason="operator_abort",
            )
            report = fixture.analyze()

            self.assertEqual(
                report["selected_attempts"][episode["episode_id"]],
                2,
            )
            row = next(
                row
                for row in read_csv(fixture.output / "episode_metrics.csv")
                if row["episode_id"] == episode["episode_id"]
            )
            self.assertEqual(row["selected_attempt_number"], "2")
            self.assertEqual(row["success"], "0")

    def test_seed_mismatch_is_blocking(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = Gate2AnalysisFixture(Path(directory))
            episode = fixture.episode("teleoperation")
            fixture.add_attempt(
                episode,
                summary_seed=int(episode["policy_episode_seed"]) + 1,
            )
            report = fixture.analyze()

            self.assertFalse(report["analysis_valid"])
            self.assertTrue(
                any(
                    "policy_episode_seed" in error
                    for error in report["blocking_errors"]
                )
            )

    def test_fixed_and_cosine_diagnostics_use_predefined_thresholds(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = Gate2AnalysisFixture(Path(directory))
            fixed = fixture.episode("fixed_blend")
            fixed_steps = [
                fixture.step(
                    mode="fixed_blend",
                    control_step=0,
                    weight=0.5,
                    active=True,
                ),
                fixture.step(
                    mode="fixed_blend",
                    control_step=1,
                    weight=0.6,
                    active=True,
                ),
            ]
            fixture.add_attempt(fixed, steps=fixed_steps)
            cosine = fixture.episode("cosine_blend")
            weights = [0.05, 0.2, None, 0.8, 0.95]
            cosine_steps = [
                fixture.step(
                    mode="cosine_blend",
                    control_step=index,
                    weight=weight,
                    active=True,
                )
                for index, weight in enumerate(weights)
            ]
            fixture.add_attempt(cosine, steps=cosine_steps)
            report = fixture.analyze()

            self.assertFalse(report["analysis_valid"])
            fixed_row = next(
                row
                for row in read_csv(
                    fixture.output / "fixed_blend_diagnostics.csv"
                )
                if row["episode_id"] == fixed["episode_id"]
            )
            self.assertEqual(fixed_row["deviation_count"], "1")
            self.assertEqual(fixed_row["within_tolerance"], "0")
            cosine_row = next(
                row
                for row in read_csv(
                    fixture.output / "cosine_blend_diagnostics.csv"
                )
                if row["episode_id"] == cosine["episode_id"]
            )
            self.assertEqual(cosine_row["weight_count"], "4")
            self.assertEqual(cosine_row["undefined_weight_count"], "1")
            self.assertAlmostEqual(float(cosine_row["near_zero_fraction"]), 0.25)
            self.assertAlmostEqual(float(cosine_row["near_one_fraction"]), 0.25)
            self.assertAlmostEqual(float(cosine_row["intermediate_fraction"]), 0.5)
            self.assertAlmostEqual(float(cosine_row["material_change_fraction"]), 1.0)

    def test_policy_wait_metrics_include_events_latency_and_overlap(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = Gate2AnalysisFixture(Path(directory))
            episode = fixture.episode("fixed_blend")
            steps = [
                fixture.step(
                    mode="fixed_blend",
                    control_step=index,
                    weight=0.5,
                    inference_latency=0.4 if index == 0 else None,
                )
                for index in range(5)
            ]
            waits = [
                {
                    "scheduler_tick": 0,
                    "control_steps": 0,
                    "autonomy_wait_ticks": 1,
                    "human_active": True,
                    "wall_time_unix_seconds": 10.00,
                    "inference_latency_seconds": None,
                },
                {
                    "scheduler_tick": 1,
                    "control_steps": 0,
                    "autonomy_wait_ticks": 2,
                    "human_active": False,
                    "wall_time_unix_seconds": 10.05,
                    "inference_latency_seconds": None,
                },
                {
                    "scheduler_tick": 4,
                    "control_steps": 2,
                    "autonomy_wait_ticks": 1,
                    "human_active": False,
                    "wall_time_unix_seconds": 10.20,
                    "inference_latency_seconds": 0.5,
                },
            ]
            fixture.add_attempt(episode, steps=steps, waits=waits)
            report = fixture.analyze()

            self.assertTrue(report["analysis_valid"])
            row = next(
                row
                for row in read_csv(fixture.output / "policy_wait_summary.csv")
                if row["episode_id"] == episode["episode_id"]
            )
            self.assertEqual(row["policy_wait_ticks"], "3")
            self.assertEqual(row["policy_wait_events"], "2")
            self.assertAlmostEqual(float(row["policy_wait_duration_seconds"]), 0.15)
            self.assertAlmostEqual(float(row["policy_wait_fraction"]), 3 / 8)
            self.assertEqual(row["human_active_policy_wait_ticks"], "1")
            self.assertEqual(row["inference_count"], "2")

    def test_profile_mismatch_is_blocking(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = Gate2AnalysisFixture(Path(directory))
            episode = fixture.episode("teleoperation")
            fixture.add_attempt(episode, profile_sha256="wrong")
            report = fixture.analyze()

            self.assertFalse(report["analysis_valid"])
            self.assertTrue(
                any(
                    "profile hash" in error.lower()
                    for error in report["blocking_errors"]
                )
            )

    def test_autonomous_pairing_requires_the_exact_four_field_identity(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = Gate2AnalysisFixture(Path(directory))
            episode = fixture.episode("teleoperation", "p06")
            fixture.add_attempt(episode)
            fixture.add_autonomous(episode, seed_offset=0, success=False)
            fixture.add_autonomous(episode, seed_offset=1, success=True)
            report = fixture.analyze()

            self.assertTrue(report["analysis_valid"])
            comparisons = read_csv(
                fixture.output / "matched_autonomous_comparisons.csv"
            )
            self.assertEqual(len(comparisons), 1)
            self.assertEqual(comparisons[0]["descriptive_recovery"], "1")
            self.assertEqual(
                comparisons[0]["policy_episode_seed"],
                str(episode["policy_episode_seed"]),
            )


if __name__ == "__main__":
    unittest.main()
