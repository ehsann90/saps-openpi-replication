"""Tests for combined policy and human control decisions."""

from __future__ import annotations

import unittest
from typing import Any

import numpy as np

from saps.evaluation.shared_control import (
    SharedAutonomyController,
)


class FakeSharedPolicy:
    """Return a deterministic two-action policy chunk."""

    def __init__(self) -> None:
        self.last_sampling_metadata: (
            dict[str, Any] | None
        ) = None
        self.inference_calls: list[dict[str, Any]] = []

        self.first_action = np.asarray(
            [
                0.10,
                0.20,
                0.30,
                0.40,
                0.50,
                0.60,
                -1.0,
            ],
            dtype=np.float32,
        )

        self.second_action = np.asarray(
            [
                -0.10,
                -0.20,
                -0.30,
                -0.40,
                -0.50,
                -0.60,
                1.0,
            ],
            dtype=np.float32,
        )

    def prepare_observation(
        self,
        observation: dict[str, Any],
        task_description: str,
    ) -> tuple[dict[str, Any], np.ndarray]:
        return (
            {
                "observation_id": observation["id"],
                "task": task_description,
            },
            np.full(
                (4, 4, 3),
                observation["id"],
                dtype=np.uint8,
            ),
        )

    def infer(
        self,
        policy_input: dict[str, Any],
        *,
        policy_episode_seed: int | None = None,
        replan_index: int | None = None,
    ) -> np.ndarray:
        self.inference_calls.append(
            {
                "policy_input": policy_input,
                "policy_episode_seed": (
                    policy_episode_seed
                ),
                "replan_index": replan_index,
            }
        )

        self.last_sampling_metadata = {
            "policy_episode_seed": policy_episode_seed,
            "replan_index": replan_index,
            "protocol_version": 1,
            "noise_sha256": "shared-noise-0",
        }

        return np.stack(
            (
                self.first_action,
                self.second_action,
            )
        )


class SharedAutonomyControllerTest(
    unittest.TestCase
):
    def test_takeover_switches_execution_source(
        self,
    ) -> None:
        policy = FakeSharedPolicy()
        controller = SharedAutonomyController(
            policy=policy,
            arbitration_mode="takeover",
            replan_steps=2,
            policy_episode_seed=55,
        )

        idle_human = np.asarray(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            dtype=np.float32,
        )

        active_human = np.asarray(
            [
                0.25,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -1.0,
            ],
            dtype=np.float32,
        )

        idle_decision = controller.decide(
            observation={"id": 1},
            task_description="test task",
            human_action=idle_human,
        )

        active_decision = controller.decide(
            observation={"id": 2},
            task_description="test task",
            human_action=active_human,
        )

        np.testing.assert_array_equal(
            idle_decision.executed_action,
            policy.first_action,
        )

        np.testing.assert_array_equal(
            active_decision.executed_action[:6],
            active_human[:6],
        )

        # The second autonomous action requested closing, so the
        # independent SAPS gripper rule preserves +1.
        self.assertEqual(
            float(active_decision.executed_action[6]),
            1.0,
        )

        self.assertEqual(
            idle_decision.arbitration_result.autonomy_weight,
            1.0,
        )
        self.assertEqual(
            active_decision.arbitration_result.autonomy_weight,
            0.0,
        )

        self.assertFalse(
            idle_decision.arbitration_result.human_active
        )
        self.assertTrue(
            active_decision.arbitration_result.human_active
        )

        self.assertEqual(
            idle_decision.policy_sample.policy_chunk_action_index,
            0,
        )
        self.assertEqual(
            active_decision.policy_sample.policy_chunk_action_index,
            1,
        )

        # Both control steps consumed one shared policy chunk.
        self.assertEqual(
            controller.policy_replan_count,
            1,
        )
        self.assertEqual(
            len(policy.inference_calls),
            1,
        )
        self.assertEqual(
            policy.inference_calls[0][
                "policy_episode_seed"
            ],
            55,
        )
        self.assertEqual(
            policy.inference_calls[0]["replan_index"],
            0,
        )

    def test_autonomous_mode_ignores_active_human_motion(
        self,
    ) -> None:
        policy = FakeSharedPolicy()
        controller = SharedAutonomyController(
            policy=policy,
            arbitration_mode="autonomous",
            replan_steps=2,
            policy_episode_seed=77,
        )

        active_human = np.asarray(
            [
                -0.25,
                0.10,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
            dtype=np.float32,
        )

        decision = controller.decide(
            observation={"id": 3},
            task_description="test task",
            human_action=active_human,
        )

        np.testing.assert_array_equal(
            decision.executed_action,
            policy.first_action,
        )

        log_record = decision.as_log_dict()

        self.assertEqual(
            log_record["arbitration_mode"],
            "autonomous",
        )
        self.assertTrue(log_record["human_active"])
        self.assertEqual(
            log_record["autonomy_weight"],
            1.0,
        )
        self.assertEqual(
            log_record["policy_replan_index"],
            0,
        )
        self.assertEqual(
            log_record["policy_chunk_action_index"],
            0,
        )

    def test_fixed_blend_combines_policy_and_human_motion(
        self,
    ) -> None:
        policy = FakeSharedPolicy()
        controller = SharedAutonomyController(
            policy=policy,
            arbitration_mode="fixed_blend",
            replan_steps=2,
            policy_episode_seed=88,
            fixed_autonomy_weight=0.5,
        )
        active_human = np.asarray(
            [-0.20, 0.10, 0.00, -0.10, 0.20, 0.30, 1.0],
            dtype=np.float32,
        )
        decision = controller.decide(
            observation={"id": 4},
            task_description="test task",
            human_action=active_human,
        )
        expected_motion = (
            0.5 * policy.first_action[:6]
            + 0.5 * active_human[:6]
        )
        np.testing.assert_allclose(
            decision.executed_action[:6],
            expected_motion,
        )
        self.assertEqual(float(decision.executed_action[6]), 1.0)
        record = decision.as_log_dict()
        self.assertEqual(record["configured_autonomy_weight"], 0.5)
        self.assertEqual(record["effective_autonomy_weight"], 0.5)
        self.assertEqual(record["autonomy_weight"], 0.5)


    def test_cosine_blend_logs_dynamic_weight(
        self,
    ) -> None:
        policy = FakeSharedPolicy()
        controller = SharedAutonomyController(
            policy=policy,
            arbitration_mode="cosine_blend",
            replan_steps=2,
            policy_episode_seed=99,
            cosine_gain=6.0,
        )
        human = policy.first_action.copy()
        human[6] = 1.0
        decision = controller.decide(
            observation={"id": 5},
            task_description="test task",
            human_action=human,
        )
        record = decision.as_log_dict()
        self.assertEqual(record["cosine_gain"], 6.0)
        self.assertAlmostEqual(record["cosine_similarity"], 1.0)
        self.assertEqual(record["cosine_similarity_status"], "computed")
        self.assertGreater(record["effective_autonomy_weight"], 0.99)


if __name__ == "__main__":
    unittest.main()
