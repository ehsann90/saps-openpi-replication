"""Focused tests for shared-autonomy scheduler behavior."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest import mock
from typing import Any

import numpy as np

from saps.evaluation.shared_episode_loop import run_shared_episode_loop
from saps.human_input.keyboard import HumanInputSample
from saps.policies.async_worker import AsyncPolicyChunk


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


def make_complete_observation(
    position: tuple[float, float, float] = (0.4, 0.1, 0.2),
) -> dict[str, Any]:
    return {
        "agentview_image": np.zeros((8, 8, 3), dtype=np.uint8),
        "robot0_eye_in_hand_image": np.zeros(
            (8, 8, 3),
            dtype=np.uint8,
        ),
        "robot0_eef_pos": np.asarray(position, dtype=np.float32),
        "robot0_eef_quat": np.asarray(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        ),
        "robot0_gripper_qpos": np.asarray(
            [-1.0, -1.0],
            dtype=np.float32,
        ),
    }


class ContinuousOperator:
    def __init__(self) -> None:
        self.statuses: list[dict[str, Any]] = []

    def sample(self) -> HumanInputSample:
        return make_sample(abort_requested=False)

    def publish_frame_rgb(
        self,
        image_rgb: np.ndarray,
        runtime_status: dict[str, Any],
    ) -> None:
        self.statuses.append(runtime_status)


class FakeSimData:
    def get_body_xpos(self, body_name: str) -> np.ndarray:
        return np.asarray([0.5, 0.6, 0.7], dtype=np.float64)


class FakeEnvironment:
    def __init__(self) -> None:
        self.sim = SimpleNamespace(data=FakeSimData())
        self.actions: list[np.ndarray] = []

    def step(
        self,
        action: list[float],
    ) -> tuple[dict[str, Any], float, bool, dict[str, Any]]:
        self.actions.append(np.asarray(action, dtype=np.float32))
        return make_complete_observation(), 0.0, False, {}


def make_chunk(
    *,
    replan_index: int,
    request_position: tuple[float, float, float],
    action_value: float,
) -> AsyncPolicyChunk:
    now = time.monotonic()
    request = SimpleNamespace(
        replan_index=replan_index,
        request_control_step=0,
        generation=0,
        reason="test",
        submitted_monotonic_seconds=now,
        observation_eef_position=request_position,
        observation_eef_quaternion=(0.0, 0.0, 0.0, 1.0),
        observation_gripper_qpos=(-1.0, -1.0),
    )
    actions = np.zeros((4, 7), dtype=np.float32)
    actions[:, 0] = action_value
    actions[:, 6] = -1.0
    return AsyncPolicyChunk(
        request=request,
        actions=actions,
        inference_latency_seconds=0.1,
        completed_monotonic_seconds=now,
        sampling_protocol_version=1,
        noise_sha256="noise",
    )


class SequencedPolicyWorker:
    def __init__(self, chunks: list[AsyncPolicyChunk]) -> None:
        self.chunks = list(chunks)
        self.requests: list[dict[str, Any]] = []
        self.replan_count = len(chunks)
        self.sampling_protocol_version = 1
        self._pending = False

    @property
    def pending(self) -> bool:
        return self._pending

    def poll(self) -> AsyncPolicyChunk | None:
        if not self.chunks:
            return None

        self._pending = False
        return self.chunks.pop(0)

    def submit(self, **kwargs: Any) -> Any:
        self.requests.append(dict(kwargs))
        self._pending = True
        return SimpleNamespace(
            reason=kwargs["reason"],
            replan_index=self.replan_count,
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
        self.assertEqual(
            waits[0]["human_action"],
            make_sample(
                abort_requested=False
            ).action.tolist(),
        )

    def test_cosine_blend_requests_policy_during_human_motion(
        self,
    ) -> None:
        operator = FakeOperator()
        worker = FakePolicyWorker()
        observation = {
            "agentview_image": np.zeros(
                (8, 8, 3),
                dtype=np.uint8,
            ),
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
                    arbitration_mode="cosine_blend",
                    cosine_gain=6.0,
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
        self.assertEqual(len(waits), 1)
        self.assertEqual(
            waits[0]["shared_control_state"],
            "cosine_blend_policy_wait",
        )
        self.assertTrue(waits[0]["human_active"])
        self.assertIsNone(
            waits[0]["effective_autonomy_weight"]
        )
        self.assertEqual(waits[0]["cosine_gain"], 6.0)
        self.assertIsNone(waits[0]["cosine_similarity"])
        self.assertEqual(
            waits[0]["cosine_similarity_status"],
            "waiting_for_policy",
        )

    def test_latency_aware_prefetch_continues_stepping(
        self,
    ) -> None:
        operator = ContinuousOperator()
        environment = FakeEnvironment()
        worker = SequencedPolicyWorker(
            [
                make_chunk(
                    replan_index=0,
                    request_position=(0.4, 0.1, 0.2),
                    action_value=0.4,
                )
            ]
        )

        with tempfile.TemporaryDirectory() as directory:
            steps_path = Path(directory) / "steps.jsonl"
            with mock.patch(
                "saps.evaluation.shared_episode_loop._sleep_to_deadline",
                return_value=0.0,
            ):
                result = run_shared_episode_loop(
                    env=environment,
                    operator=operator,
                    policy_worker=worker,
                    initial_observation=make_complete_observation(),
                    task_description="test task",
                    object_body_name="object",
                    arbitration_mode="fixed_blend",
                    fixed_autonomy_weight=0.5,
                    replan_steps=4,
                    policy_episode_seed=123,
                    environment_seed=7,
                    max_steps=2,
                    control_frequency_hz=20.0,
                    steps_path=steps_path,
                    scheduler_mode="latency_aware",
                    prefetch_remaining_actions=3,
                )
            records = [
                json.loads(line)
                for line in steps_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            waits = steps_path.with_name(
                "scheduler_waits.jsonl"
            ).read_text(encoding="utf-8")

        self.assertEqual(result.control_steps, 2)
        self.assertEqual(len(environment.actions), 2)
        self.assertEqual(waits, "")
        self.assertEqual(len(worker.requests), 1)
        self.assertEqual(
            worker.requests[0]["reason"],
            "early_prefetch",
        )
        self.assertTrue(records[1]["policy_worker_pending"])

    def test_latency_aware_rejects_diverged_plan(
        self,
    ) -> None:
        operator = ContinuousOperator()
        environment = FakeEnvironment()
        worker = SequencedPolicyWorker(
            [
                make_chunk(
                    replan_index=0,
                    request_position=(0.4, 0.1, 0.2),
                    action_value=0.4,
                ),
                make_chunk(
                    replan_index=1,
                    request_position=(1.4, 0.1, 0.2),
                    action_value=-0.8,
                ),
            ]
        )

        with tempfile.TemporaryDirectory() as directory:
            steps_path = Path(directory) / "steps.jsonl"
            with mock.patch(
                "saps.evaluation.shared_episode_loop._sleep_to_deadline",
                return_value=0.0,
            ):
                result = run_shared_episode_loop(
                    env=environment,
                    operator=operator,
                    policy_worker=worker,
                    initial_observation=make_complete_observation(),
                    task_description="test task",
                    object_body_name="object",
                    arbitration_mode="fixed_blend",
                    fixed_autonomy_weight=0.5,
                    replan_steps=4,
                    policy_episode_seed=123,
                    environment_seed=7,
                    max_steps=2,
                    control_frequency_hz=20.0,
                    steps_path=steps_path,
                    scheduler_mode="latency_aware",
                    prefetch_remaining_actions=3,
                    max_plan_translation_m=0.15,
                )
            records = [
                json.loads(line)
                for line in steps_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]

        self.assertEqual(result.accepted_policy_results, 1)
        self.assertEqual(result.rejected_policy_results, 1)
        self.assertEqual(
            records[1]["plan_validation_status"],
            "rejected_translation_divergence",
        )
        self.assertGreater(
            records[1]["policy_plan_translation_m"],
            0.9,
        )
        self.assertGreater(environment.actions[1][0], 0.0)


if __name__ == "__main__":
    unittest.main()
