"""Integration tests for the autonomous episode runner wiring."""

from __future__ import annotations

import contextlib
import dataclasses
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
from typing import Any

import numpy as np

from saps.evaluation.runner import run_episode


def make_observation() -> dict[str, Any]:
    """Create the observation fields used by the runner."""

    return {
        "agentview_image": np.zeros(
            (8, 8, 3),
            dtype=np.uint8,
        ),
        "robot0_eef_pos": np.asarray(
            [0.4, 0.1, 0.2],
            dtype=np.float32,
        ),
    }


@dataclasses.dataclass(frozen=True)
class FakePerturbation:
    """Minimal perturbation result required by run_episode."""

    body_position_before: list[float]


class FakeSimData:
    def get_body_xpos(
        self,
        body_name: str,
    ) -> np.ndarray:
        if body_name != "object_body":
            raise ValueError(
                f"Unexpected body name {body_name!r}."
            )

        return np.asarray(
            [0.5, 0.6, 0.7],
            dtype=np.float64,
        )


class FakeSim:
    def __init__(self) -> None:
        self.data = FakeSimData()


class FakeEnvironment:
    """Record the action passed to the simulated environment."""

    def __init__(self) -> None:
        self.sim = FakeSim()
        self.executed_actions: list[np.ndarray] = []
        self.reset_count = 0

    def reset(self) -> None:
        self.reset_count += 1

    def set_init_state(
        self,
        initial_state: np.ndarray,
    ) -> dict[str, Any]:
        np.testing.assert_array_equal(
            initial_state,
            np.asarray([9.0], dtype=np.float32),
        )
        return make_observation()

    def step(
        self,
        action: list[float],
    ) -> tuple[
        dict[str, Any],
        float,
        bool,
        dict[str, Any],
    ]:
        self.executed_actions.append(
            np.asarray(action, dtype=np.float32)
        )

        # End successfully after one control step.
        return make_observation(), 1.0, True, {}


class FakePolicy:
    """Return one deterministic action chunk."""

    def __init__(self) -> None:
        self.last_sampling_metadata: (
            dict[str, Any] | None
        ) = None
        self.inference_calls: list[dict[str, Any]] = []

        self.expected_action = np.asarray(
            [
                0.10,
                -0.20,
                0.30,
                -0.40,
                0.50,
                -0.60,
                1.0,
            ],
            dtype=np.float32,
        )

    def prepare_observation(
        self,
        obs: dict[str, Any],
        prompt: str,
    ) -> tuple[dict[str, Any], np.ndarray]:
        self.assert_valid_observation(obs)

        if prompt != "put the cream cheese in the basket":
            raise ValueError(
                f"Unexpected prompt {prompt!r}."
            )

        return (
            {"prepared": True},
            np.zeros((8, 8, 3), dtype=np.uint8),
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
            "noise_sha256": "fake-noise-hash",
        }

        second_action = self.expected_action.copy()
        second_action[0] = 0.25

        return np.stack(
            (
                self.expected_action,
                second_action,
            )
        )

    @staticmethod
    def assert_valid_observation(
        obs: dict[str, Any],
    ) -> None:
        if "robot0_eef_pos" not in obs:
            raise ValueError(
                "Observation is missing robot0_eef_pos."
            )


class AutonomousRunnerIntegrationTest(
    unittest.TestCase
):
    def test_one_step_autonomous_episode(
        self,
    ) -> None:
        env = FakeEnvironment()
        policy = FakePolicy()

        perturbation = FakePerturbation(
            body_position_before=[0.1, 0.2, 0.3],
        )

        settled_qpos = np.asarray(
            [0.0, 0.0, 0.0],
            dtype=np.float64,
        )
        settled_position = np.asarray(
            [0.11, 0.22, 0.33],
            dtype=np.float64,
        )

        with tempfile.TemporaryDirectory() as directory:
            output_root = Path(directory)

            with contextlib.ExitStack() as stack:
                stack.enter_context(
                    mock.patch(
                        "saps.evaluation.runner."
                        "_save_agent_image"
                    )
                )
                stack.enter_context(
                    mock.patch(
                        "saps.evaluation.runner."
                        "apply_planar_object_offset",
                        return_value=(
                            make_observation(),
                            perturbation,
                        ),
                    )
                )
                stack.enter_context(
                    mock.patch(
                        "saps.evaluation.runner."
                        "get_object_pose",
                        return_value=(
                            settled_qpos,
                            settled_position,
                        ),
                    )
                )
                stack.enter_context(
                    mock.patch(
                        "saps.evaluation.runner."
                        "imageio.mimwrite"
                    )
                )

                result = run_episode(
                    env=env,
                    policy=policy,
                    condition_id="nominal",
                    task_id=1,
                    task_description=(
                        "put the cream cheese in the basket"
                    ),
                    initial_state=np.asarray(
                        [9.0],
                        dtype=np.float32,
                    ),
                    initial_state_index=0,
                    trial_index=0,
                    output_root=output_root,
                    object_joint_name="object_joint",
                    object_body_name="object_body",
                    delta_x=0.0,
                    delta_y=0.0,
                    replan_steps=2,
                    num_steps_wait=0,
                    max_steps=1,
                    policy_episode_seed=1234,
                    arbitration_mode="autonomous",
                )

            self.assertTrue(result.success)
            self.assertEqual(result.control_steps, 1)
            self.assertEqual(result.simulation_steps, 1)
            self.assertEqual(
                result.policy_replan_count,
                1,
            )
            self.assertEqual(result.policy_replans, 1)
            self.assertEqual(
                result.sampling_protocol_version,
                1,
            )
            self.assertEqual(
                result.arbitration_mode,
                "autonomous",
            )

            self.assertEqual(env.reset_count, 1)
            self.assertEqual(
                len(env.executed_actions),
                1,
            )
            np.testing.assert_array_equal(
                env.executed_actions[0],
                policy.expected_action,
            )

            self.assertEqual(
                len(policy.inference_calls),
                1,
            )
            self.assertEqual(
                policy.inference_calls[0][
                    "policy_episode_seed"
                ],
                1234,
            )
            self.assertEqual(
                policy.inference_calls[0][
                    "replan_index"
                ],
                0,
            )

            episode_directory = (
                output_root
                / "nominal"
                / "task_01"
                / "init_000"
                / "trial_000"
            )

            steps_path = (
                episode_directory / "steps.jsonl"
            )
            summary_path = (
                episode_directory / "summary.json"
            )

            self.assertTrue(steps_path.is_file())
            self.assertTrue(summary_path.is_file())

            records = [
                json.loads(line)
                for line in steps_path.read_text(
                    encoding="utf-8"
                ).splitlines()
            ]

            self.assertEqual(len(records), 1)
            record = records[0]

            self.assertEqual(
                record["arbitration_mode"],
                "autonomous",
            )
            self.assertEqual(
                record["activity_state"],
                "idle",
            )
            self.assertFalse(record["human_active"])
            self.assertEqual(
                record["human_motion_norm"],
                0.0,
            )
            self.assertEqual(
                record["autonomy_weight"],
                1.0,
            )

            np.testing.assert_array_equal(
                record["human_action"],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            )
            np.testing.assert_array_equal(
                record["autonomous_action"],
                policy.expected_action,
            )
            np.testing.assert_array_equal(
                record["executed_action"],
                policy.expected_action,
            )
            np.testing.assert_array_equal(
                record["policy_action"],
                policy.expected_action,
            )

            self.assertTrue(record["replanned"])
            self.assertEqual(
                record["policy_replan_index"],
                0,
            )
            self.assertEqual(
                record["policy_chunk_action_index"],
                0,
            )
            self.assertEqual(
                record["policy_episode_seed"],
                1234,
            )
            self.assertEqual(
                record["sampling_protocol_version"],
                1,
            )
            self.assertEqual(
                record["policy_noise_sha256"],
                "fake-noise-hash",
            )


if __name__ == "__main__":
    unittest.main()
