"""Focused tests for shared-autonomy scheduler behavior."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest import mock
from typing import Any

import numpy as np

from saps.evaluation.shared_episode_loop import run_shared_episode_loop
from saps.human_input.keyboard import HumanInputSample


def make_sample(*, abort_requested: bool) -> HumanInputSample:
    return HumanInputSample(
        action=np.asarray(
            [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            dtype=np.float32,
        ),
        motion_active=True,
        connected=True,
        armed=True,
        abort_requested=abort_requested,
        pressed_keys=("w",),
        gripper_command=-1.0,
        speed_mode="fine",
        translation_gain=0.07,
        rotation_gain=0.10,
        sample_monotonic_seconds=1.0,
        last_event_monotonic_seconds=1.0,
    )


class FakeOperator:
    def __init__(self) -> None:
        self.samples = [
            make_sample(abort_requested=False),
            make_sample(abort_requested=True),
        ]
        self.statuses: list[dict[str, Any]] = []

    def sample(self) -> HumanInputSample:
        return self.samples.pop(0)

    def publish_frame_rgb(
        self,
        image_rgb: np.ndarray,
        runtime_status: dict[str, Any],
    ) -> None:
        self.statuses.append(runtime_status)


class FakePolicyWorker:
    def __init__(self) -> None:
        self._pending = False
        self.requests: list[dict[str, Any]] = []
        self.replan_count = 0
        self.sampling_protocol_version = None

    @property
    def pending(self) -> bool:
        return self._pending

    def poll(self) -> None:
        return None

    def submit(self, **kwargs: Any) -> Any:
        self.requests.append(dict(kwargs))
        self.replan_count += 1
        self._pending = True
        return SimpleNamespace(
            reason=kwargs["reason"],
            replan_index=self.replan_count - 1,
        )


class SharedEpisodeLoopSchedulerTest(unittest.TestCase):
    def test_fixed_blend_requests_policy_during_human_motion(
        self,
    ) -> None:
        operator = FakeOperator()
        worker = FakePolicyWorker()
        observation = {
            "agentview_image": np.zeros((8, 8, 3), dtype=np.uint8),
            "robot0_eye_in_hand_image": np.zeros(
                (8, 8, 3),
                dtype=np.uint8,
            ),
        }

        with tempfile.TemporaryDirectory() as directory:
            steps_path = Path(directory) / "steps.jsonl"
            with mock.patch(
                "saps.evaluation.shared_episode_loop._sleep_to_deadline",
                return_value=0.0,
            ):
                result = run_shared_episode_loop(
                    env=object(),
                    operator=operator,
                    policy_worker=worker,
                    initial_observation=observation,
                    task_description="test task",
                    object_body_name="unused",
                    arbitration_mode="fixed_blend",
                    fixed_autonomy_weight=0.5,
                    replan_steps=5,
                    policy_episode_seed=123,
                    environment_seed=7,
                    max_steps=1,
                    control_frequency_hz=20.0,
                    steps_path=steps_path,
                )
            waits = [
                json.loads(line)
                for line in steps_path.with_name(
                    "scheduler_waits.jsonl"
                ).read_text(encoding="utf-8").splitlines()
            ]

        self.assertEqual(result.termination_reason, "operator_abort")
        self.assertEqual(result.control_steps, 0)
        self.assertEqual(len(worker.requests), 1)
        self.assertEqual(worker.requests[0]["reason"], "periodic")
        self.assertEqual(
            worker.requests[0]["request_control_step"],
            0,
        )
        self.assertEqual(len(waits), 1)
        self.assertEqual(
            waits[0]["shared_control_state"],
            "fixed_blend_policy_wait",
        )
        self.assertTrue(waits[0]["human_active"])
        self.assertEqual(waits[0]["configured_autonomy_weight"], 0.5)
        self.assertEqual(waits[0]["effective_autonomy_weight"], 0.5)


if __name__ == "__main__":
    unittest.main()
