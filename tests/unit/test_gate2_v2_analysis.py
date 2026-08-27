"""Partial-data and exact-matching tests for Gate-2 v2 analysis."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from saps.evaluation.experiment_session import json_file_identity
from saps.evaluation.gate2_v2_analysis import _shared_policy_accounting
from saps.evaluation.gate2_v2_analysis import analyze_gate2_v2_collection
from saps.evaluation.gate2_v2_protocol import (
    build_gate2_v2_autonomous_schedule,
)
from saps.evaluation.gate2_v2_protocol import (
    GATE2_V2_AUTONOMOUS_PROTOCOL_PATH,
)
from saps.evaluation.gate2_v2_protocol import (
    load_gate2_v2_autonomous_protocol,
)
from saps.policies.seeding import SEED_PROTOCOL


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


class Gate2V2AnalysisTest(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = load_gate2_v2_autonomous_protocol(
            REPOSITORY_ROOT / GATE2_V2_AUTONOMOUS_PROTOCOL_PATH
        )
        self.schedule = build_gate2_v2_autonomous_schedule(self.protocol)

    def initialize_autonomous_root(self, root: Path) -> None:
        protocol_identity = json_file_identity(
            Path(GATE2_V2_AUTONOMOUS_PROTOCOL_PATH)
        )
        write_json(root / "protocol.json", self.protocol)
        write_json(root / "schedule.json", self.schedule)
        write_json(
            root / "perturbation_config.json",
            json_file_identity(
                REPOSITORY_ROOT / self.protocol["config_path"]
            ),
        )
        write_json(
            root / "repository_provenance.json",
            {
                "repository_commit": "a" * 40,
                "protocol_path": protocol_identity["path"],
                "protocol_sha256": protocol_identity["sha256"],
            },
        )

    def write_autonomous_episode(
        self,
        root: Path,
        *,
        policy_seed_delta: int = 0,
    ) -> None:
        episode = self.schedule["episodes"][0]
        episode_root = (
            root
            / episode["condition_id"]
            / "task_01"
            / "init_000"
            / f"trial_{episode['trial_index']:03d}"
        )
        summary = {
            "condition_id": episode["condition_id"],
            "task_id": 1,
            "task_description": "pick up the cream cheese",
            "trial_index": episode["trial_index"],
            "initial_state_index": 0,
            "policy_replan_count": 1,
            "delta_x": 0.0,
            "delta_y": 0.0,
            "offset_distance": 0.0,
            "success": True,
            "simulation_steps": 11,
            "control_steps": 1,
            "control_elapsed_seconds": 0.4,
            "total_elapsed_seconds": 1.2,
            "object_position_before": [0.0, 0.0, 0.0],
            "object_position_after_settle": [0.0, 0.0, 0.0],
            "output_directory": str(episode_root),
            "arbitration_mode": "autonomous",
            "policy_episode_seed": (
                episode["policy_episode_seed"] + policy_seed_delta
            ),
            "policy_seed_protocol": SEED_PROTOCOL,
            "policy_replans": 1,
            "sampling_protocol_version": 1,
        }
        write_json(episode_root / "summary.json", summary)
        (episode_root / "steps.jsonl").write_text(
            json.dumps(
                {
                    "replanned": True,
                    "policy_replan_index": 0,
                    "inference_latency_seconds": 0.35,
                }
            )
            + "\n",
            encoding="utf-8",
        )

    @staticmethod
    def shared_summary(policy_replan_count: int) -> dict[str, object]:
        return {
            "arbitration_mode": "fixed_blend",
            "condition_id": "nominal",
            "trial_index": 0,
            "success": True,
            "termination_reason": "success",
            "control_steps": policy_replan_count,
            "policy_replan_count": policy_replan_count,
        }

    @staticmethod
    def shared_record(
        *,
        scheduler_tick: int,
        submitted: int | None = None,
        result: int | None = None,
        latency: float | None = None,
        pending: bool = False,
        action_replan_index: int | None = None,
    ) -> dict[str, object]:
        return {
            "scheduler_tick": scheduler_tick,
            "policy_request_replan_index": submitted,
            "policy_result_replan_index": result,
            "inference_latency_seconds": latency,
            "policy_worker_pending": pending,
            "policy_replan_index": action_replan_index,
        }

    def account_shared(
        self,
        *,
        submitted_count: int,
        steps: list[dict[str, object]],
    ) -> tuple[dict[str, object], list[str]]:
        errors: list[str] = []
        diagnostic = _shared_policy_accounting(
            episode_id="synthetic",
            summary=self.shared_summary(submitted_count),
            steps=steps,
            waits=[],
            errors=errors,
        )
        return diagnostic, errors

    def test_complete_shared_policy_accounting_is_valid(self) -> None:
        steps = [
            self.shared_record(
                scheduler_tick=index,
                submitted=index + 1 if index < 2 else None,
                result=index,
                latency=0.1,
                action_replan_index=index,
            )
            for index in range(3)
        ]
        diagnostic, errors = self.account_shared(
            submitted_count=3,
            steps=steps,
        )

        self.assertEqual(errors, [])
        self.assertEqual(diagnostic["accounting_status"], "complete")
        self.assertEqual(diagnostic["policy_requests_submitted"], 3)
        self.assertEqual(diagnostic["policy_results_completed_and_logged"], 3)
        self.assertEqual(diagnostic["inference_latency_count"], 3)

    def test_terminal_outstanding_shared_request_is_valid(self) -> None:
        steps = [
            self.shared_record(
                scheduler_tick=0,
                submitted=1,
                result=0,
                latency=0.1,
                action_replan_index=0,
            ),
            self.shared_record(
                scheduler_tick=1,
                submitted=2,
                result=1,
                latency=0.1,
                pending=True,
                action_replan_index=1,
            ),
        ]
        diagnostic, errors = self.account_shared(
            submitted_count=3,
            steps=steps,
        )

        self.assertEqual(errors, [])
        self.assertEqual(
            diagnostic["accounting_status"],
            "terminal_request_unobserved",
        )
        self.assertEqual(diagnostic["terminal_unobserved_request_count"], 1)
        self.assertEqual(diagnostic["terminal_unobserved_replan_index"], 2)

    def test_missing_shared_latency_is_invalid(self) -> None:
        steps = [
            self.shared_record(
                scheduler_tick=index,
                submitted=index + 1 if index < 2 else None,
                result=index,
                latency=0.1 if index < 2 else None,
                action_replan_index=index,
            )
            for index in range(3)
        ]
        diagnostic, errors = self.account_shared(
            submitted_count=3,
            steps=steps,
        )

        self.assertEqual(diagnostic["accounting_status"], "invalid")
        self.assertTrue(any("latency" in error for error in errors))

    def test_middle_shared_result_gap_is_invalid(self) -> None:
        steps = [
            self.shared_record(
                scheduler_tick=0,
                submitted=1,
                result=0,
                latency=0.1,
                action_replan_index=0,
            ),
            self.shared_record(
                scheduler_tick=1,
                submitted=2,
                result=2,
                latency=0.1,
                action_replan_index=2,
            ),
        ]
        diagnostic, errors = self.account_shared(
            submitted_count=3,
            steps=steps,
        )

        self.assertEqual(diagnostic["accounting_status"], "invalid")
        self.assertTrue(any("contiguous prefix" in error for error in errors))

    def test_more_than_one_outstanding_shared_request_is_invalid(self) -> None:
        steps = [
            self.shared_record(
                scheduler_tick=0,
                submitted=1,
                result=0,
                latency=0.1,
                action_replan_index=0,
            ),
            self.shared_record(
                scheduler_tick=1,
                submitted=2,
                pending=True,
                action_replan_index=0,
            ),
        ]
        diagnostic, errors = self.account_shared(
            submitted_count=3,
            steps=steps,
        )

        self.assertEqual(diagnostic["accounting_status"], "invalid")
        self.assertTrue(any("outside [0, 1]" in error for error in errors))

    def test_analysis_handles_both_collections_not_started(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report = analyze_gate2_v2_collection(
                session_root=root / "shared",
                autonomous_root=root / "autonomous",
                output_dir=root / "analysis",
            )
            self.assertTrue(report["analysis_valid"])
            self.assertEqual(report["scheduled_episode_count"], 60)
            self.assertEqual(report["selected_analyzable_episode_count"], 0)
            self.assertEqual(report["matched_triplet_count"], 0)
            self.assertFalse(report["collection_complete"])
            self.assertEqual(
                report["timing"]["environment_time_definition"],
                "control_steps / 20 Hz",
            )
            self.assertFalse(
                report["timing"]["shared_wait_ticks_advance_environment"]
            )
            generated_report = (
                root / "analysis" / "REPORT.md"
            ).read_text(encoding="utf-8")
            self.assertIn(
                "exclude human input during policy-wait ticks",
                generated_report,
            )
            self.assertIn(
                "physical external environment and wall clock continue",
                generated_report,
            )

    def test_autonomous_data_are_analyzed_without_shared_data(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            autonomous = root / "autonomous"
            self.initialize_autonomous_root(autonomous)
            self.write_autonomous_episode(autonomous)
            report = analyze_gate2_v2_collection(
                session_root=root / "shared",
                autonomous_root=autonomous,
                output_dir=root / "analysis",
            )
            self.assertTrue(report["analysis_valid"])
            self.assertEqual(report["selected_analyzable_episode_count"], 1)
            self.assertEqual(report["analyzable_by_mode"], {"autonomous": 1})
            self.assertEqual(report["matched_triplet_count"], 0)
            metrics = (root / "analysis" / "episode_metrics.csv").read_text(
                encoding="utf-8"
            )
            self.assertIn("0.05", metrics)
            self.assertIn("0.4", metrics)
            self.assertIn("0.35", metrics)

    def test_mismatched_autonomous_seed_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            autonomous = root / "autonomous"
            self.initialize_autonomous_root(autonomous)
            self.write_autonomous_episode(autonomous, policy_seed_delta=1)
            report = analyze_gate2_v2_collection(
                session_root=root / "shared",
                autonomous_root=autonomous,
                output_dir=root / "analysis",
            )
            self.assertFalse(report["analysis_valid"])
            self.assertEqual(report["selected_analyzable_episode_count"], 0)
            self.assertTrue(
                any(
                    "identity mismatch" in error
                    for error in report["blocking_errors"]
                )
            )


if __name__ == "__main__":
    unittest.main()
