"""Network-free tests for the G2 paired-prompt diagnostic."""

from __future__ import annotations

import unittest

import numpy as np

from saps.policies.model_input_audit import array_evidence
from saps.policies.openpi_droid import DroidPolicyResponse, prepare_droid_observation
from tools.diagnostics.compare_archived_droid_prompts import (
    ALTERNATE_PROMPT, ARCHIVED_PROMPT, compare, run_gate, with_prompt,
)


NOISE = "a" * 64


def fixture() -> tuple[dict, np.ndarray]:
    observation = prepare_droid_observation(
        exterior_image=np.ones((180, 320, 3), dtype=np.uint8),
        wrist_image=np.full((180, 320, 3), 2, dtype=np.uint8),
        joint_position=np.array([0, -.7, 0, -1.7, 0, 1.4, .7], dtype=np.float32),
        gripper_position=np.array([.000028], dtype=np.float32),
        prompt=ARCHIVED_PROMPT,
    )
    actions = np.zeros((15, 8), dtype=np.float64)
    actions[:8, 7] = .7
    return observation, actions


class FakePolicy:
    def __init__(self, archived: np.ndarray, *, alter_baseline: bool = False,
                 alter_repeat: bool = False, noise_mismatch: bool = False):
        self.archived = archived
        self.alter_baseline = alter_baseline
        self.alter_repeat = alter_repeat
        self.noise_mismatch = noise_mismatch
        self.prompts: list[str] = []

    def infer(self, observation: dict, *, policy_episode_seed: int,
              replan_index: int) -> DroidPolicyResponse:
        prompt = observation["prompt"]
        self.prompts.append(prompt)
        actions = self.archived.copy()
        if prompt == ALTERNATE_PROMPT:
            actions[0, 0] += .1
            actions[0, 7] = .4
            if self.alter_repeat and len(self.prompts) == 3:
                actions[0, 0] += .001
        elif self.alter_baseline:
            actions[0, 0] += .001
        metadata = {"policy_episode_seed": policy_episode_seed,
                    "replan_index": replan_index,
                    "noise_sha256": ("b" if self.noise_mismatch else "a") * 64}
        return DroidPolicyResponse(actions, 0., None, None, metadata, ())


class PairedPromptTest(unittest.TestCase):
    def test_only_prompt_changes_and_archived_closure_is_preserved(self) -> None:
        observation, _ = fixture()
        alternate = with_prompt(observation, ALTERNATE_PROMPT)
        self.assertEqual(alternate["prompt"], ALTERNATE_PROMPT)
        self.assertEqual(observation["prompt"], ARCHIVED_PROMPT)
        for key, value in observation.items():
            if isinstance(value, np.ndarray):
                self.assertEqual(array_evidence(value), array_evidence(alternate[key]))
        self.assertAlmostEqual(float(alternate["observation/gripper_position"][0]), .000028)

    def test_identical_baseline_then_three_repeatable_alternatives(self) -> None:
        observation, archived = fixture()
        policy = FakePolicy(archived)
        report, arrays = {}, {}
        run_gate(policy, observation, archived, 17, 6, NOISE, report, arrays)
        self.assertEqual(policy.prompts, [ARCHIVED_PROMPT] + [ALTERNATE_PROMPT] * 3)
        self.assertTrue(report["baseline_gate"]["passed"])
        self.assertTrue(report["alternate_repeatability"]["passed"])
        self.assertEqual(report["comparison"]["first_eight_gripper_decision_changed_indices"], [0])
        self.assertEqual(report["baseline"]["first_eight_close_count"], 8)
        self.assertEqual(report["alternate"]["first_eight_close_count"], 7)
        self.assertEqual(set(arrays), {"archived_actions", "baseline_replay",
                                       "alternate_replay_0", "alternate_replay_1", "alternate_replay_2"})

    def test_baseline_action_difference_skips_alternative(self) -> None:
        observation, archived = fixture()
        policy = FakePolicy(archived, alter_baseline=True)
        report = {}
        with self.assertRaisesRegex(RuntimeError, "not submitted"):
            run_gate(policy, observation, archived, 17, 6, NOISE, report, {})
        self.assertEqual(policy.prompts, [ARCHIVED_PROMPT])
        self.assertFalse(report["baseline_gate"]["passed"])

    def test_baseline_noise_difference_skips_alternative(self) -> None:
        observation, archived = fixture()
        policy = FakePolicy(archived, noise_mismatch=True)
        with self.assertRaisesRegex(RuntimeError, "not submitted"):
            run_gate(policy, observation, archived, 17, 6, NOISE, {}, {})
        self.assertEqual(policy.prompts, [ARCHIVED_PROMPT])

    def test_alternate_nonrepeatability_is_detected(self) -> None:
        observation, archived = fixture()
        policy = FakePolicy(archived, alter_repeat=True)
        report = {}
        with self.assertRaisesRegex(RuntimeError, "not repeatable"):
            run_gate(policy, observation, archived, 17, 6, NOISE, report, {})
        self.assertEqual(len(policy.prompts), 4)
        self.assertFalse(report["alternate_repeatability"]["passed"])

    def test_zero_displacement_has_undefined_direction_cosine(self) -> None:
        _, archived = fixture()
        comparison = compare(archived, archived, np.zeros(7, dtype=np.float32))
        self.assertIsNone(comparison["ideal_cumulative_translation_direction_cosine"])
        self.assertTrue(comparison["exactly_equal"])


if __name__ == "__main__":
    unittest.main()
