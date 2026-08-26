"""Strict collection and resume guards for Gate-2 v2 autonomous data."""

from __future__ import annotations

import dataclasses
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

from saps.evaluation.gate2_v2_protocol import (
    GATE2_V2_AUTONOMOUS_EXPERIMENT_ID,
)
from saps.evaluation.gate2_v2_protocol import (
    GATE2_V2_AUTONOMOUS_PROTOCOL_PATH,
)
from saps.evaluation.gate2_v2_protocol import (
    load_gate2_v2_autonomous_protocol,
)
from saps.evaluation.runner import EpisodeResult
from saps.policies.seeding import SEED_PROTOCOL


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPOSITORY_ROOT / "scripts" / "run_autonomous_sweep.py"
RUNNER_SPEC = importlib.util.spec_from_file_location(
    "saps_gate2_v2_autonomous_runner",
    RUNNER_PATH,
)
assert RUNNER_SPEC is not None and RUNNER_SPEC.loader is not None
runner = importlib.util.module_from_spec(RUNNER_SPEC)
sys.modules[RUNNER_SPEC.name] = runner
RUNNER_SPEC.loader.exec_module(runner)


class Gate2V2AutonomousRunnerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = load_gate2_v2_autonomous_protocol(
            REPOSITORY_ROOT / GATE2_V2_AUTONOMOUS_PROTOCOL_PATH
        )

    def exact_args(self) -> object:
        return runner.Args(
            config_path=self.protocol["config_path"],
            condition_ids=",".join(self.protocol["conditions"]),
            num_trials=5,
            initial_state_index=0,
            resume=True,
            required_protocol_id=GATE2_V2_AUTONOMOUS_EXPERIMENT_ID,
            protocol_path=GATE2_V2_AUTONOMOUS_PROTOCOL_PATH,
            repository_commit="a" * 40,
            deterministic_policy=True,
            policy_base_seed=20260724,
            seed=7,
            resolution=256,
            resize_size=224,
            replan_steps=5,
            num_steps_wait=10,
            max_steps=280,
            control_frequency_hz=20.0,
            video_fps=10,
            output_dir=self.protocol["output_root"],
        )

    def test_exact_args_pass_and_extra_condition_is_rejected(self) -> None:
        args = self.exact_args()
        runner.validate_gate2_v2_args(args=args, protocol=self.protocol)
        changed = dataclasses.replace(
            args,
            condition_ids="nominal,p01,p02,p06,p09",
        )
        with self.assertRaisesRegex(ValueError, "arguments drifted"):
            runner.validate_gate2_v2_args(
                args=changed,
                protocol=self.protocol,
            )

    def test_frozen_provenance_cannot_change(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "protocol.json"
            runner.freeze_json(path, {"value": 1})
            runner.freeze_json(path, {"value": 1})
            with self.assertRaisesRegex(ValueError, "Frozen autonomous"):
                runner.freeze_json(path, {"value": 2})

    def test_resume_requires_complete_steps_and_matching_seeds(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            summary_path = root / "summary.json"
            result = EpisodeResult(
                condition_id="nominal",
                task_id=1,
                task_description="cream cheese",
                trial_index=0,
                initial_state_index=0,
                policy_replan_count=1,
                delta_x=0.0,
                delta_y=0.0,
                offset_distance=0.0,
                success=True,
                simulation_steps=11,
                control_steps=1,
                control_elapsed_seconds=0.2,
                total_elapsed_seconds=1.0,
                object_position_before=[0.0, 0.0, 0.0],
                object_position_after_settle=[0.0, 0.0, 0.0],
                output_directory=str(root),
                arbitration_mode="autonomous",
                policy_episode_seed=123,
                policy_seed_protocol=SEED_PROTOCOL,
                policy_replans=1,
                sampling_protocol_version=1,
            )
            summary_path.write_text("{}", encoding="utf-8")
            steps_path = root / "steps.jsonl"
            steps_path.write_text(
                json.dumps({"policy_episode_seed": 123}) + "\n",
                encoding="utf-8",
            )
            runner.validate_gate2_v2_completed_result(
                result=result,
                summary_path=summary_path,
                protocol=self.protocol,
            )
            steps_path.write_text(
                json.dumps({"policy_episode_seed": 124}) + "\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "step seed"):
                runner.validate_gate2_v2_completed_result(
                    result=result,
                    summary_path=summary_path,
                    protocol=self.protocol,
                )


if __name__ == "__main__":
    unittest.main()
