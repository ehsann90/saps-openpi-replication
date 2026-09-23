"""Software-only checks for the G2 deterministic replay and gripper sweep."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from saps.physical.droid_gripper import droid_gripper_decision
from saps.physical.fr3_forward_kinematics import fr3_tcp_fk
from saps.policies.model_input_audit import array_evidence
from saps.policies.openpi_droid import DroidPolicyResponse
from saps.policies.openpi_droid import prepare_droid_observation
from tools.diagnostics.sweep_archived_droid_gripper_state import (
    ACTION_SHAPE, GRIPPER_KEY, JOINT_KEY, action_evidence,
    changed_gripper_only, file_sha256, ideal_rollout,
    load_archive, run_gate, select_request,
)


NOISE = "a" * 64


def observation() -> dict:
    return prepare_droid_observation(
        exterior_image=np.full((180, 320, 3), 55, dtype=np.uint8),
        wrist_image=np.full((180, 320, 3), 155, dtype=np.uint8),
        joint_position=np.array([0, -.7, 0, -1.7, 0, 1.4, .7], dtype=np.float32),
        gripper_position=np.array([.000028], dtype=np.float32),
        prompt="Pick up the red object",
    )


def archived_actions() -> np.ndarray:
    actions = np.zeros(ACTION_SHAPE, dtype=np.float64)
    actions[:8, 7] = .85
    return actions


def write_archive(directory: Path, obs: dict, actions: np.ndarray) -> None:
    directory.mkdir()
    np.savez_compressed(directory / "observation.npz", **obs)
    np.savez_compressed(directory / "actions.npz", actions=actions)
    index = int(directory.name.removeprefix("request_"))
    request = {
        "observation_bundle": {"path": "observation.npz", "sha256": file_sha256(directory / "observation.npz")},
        "canonical_schema": {key: array_evidence(value) if isinstance(value, np.ndarray)
                             else {"type": "str", "value": value} for key, value in obs.items()},
        "prompt": obs["prompt"], "policy_episode_seed": 17,
        "request_index": index, "replan_index": index,
    }
    response = {
        "action_bundle": {"path": "actions.npz", "sha256": file_sha256(directory / "actions.npz")},
        "action": action_evidence(actions), "replan_index": index,
        "sampling_metadata": {"policy_episode_seed": 17, "replan_index": index,
                              "protocol_version": 1, "noise_sha256": NOISE},
    }
    (directory / "request.json").write_text(json.dumps(request), encoding="utf-8")
    (directory / "response.json").write_text(json.dumps(response), encoding="utf-8")


class FakePolicy:
    def __init__(self, actions: np.ndarray, *, change_call: int | None = None,
                 wrong_noise: bool = False):
        self.actions = actions
        self.change_call = change_call
        self.wrong_noise = wrong_noise
        self.calls: list[dict] = []

    def infer(self, obs: dict, *, policy_episode_seed: int, replan_index: int) -> DroidPolicyResponse:
        self.calls.append(obs)
        actions = self.actions.copy()
        if self.change_call == len(self.calls):
            actions[0, 0] += .01
        metadata = {"policy_episode_seed": policy_episode_seed, "replan_index": replan_index,
                    "noise_sha256": "b" * 64 if self.wrong_noise else NOISE}
        return DroidPolicyResponse(actions, 0., None, None, metadata, ())


class ArchivedG2SweepTest(unittest.TestCase):
    def test_selects_earliest_strong_close_with_open_gripper(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            early = archived_actions()
            early[:8, 7] = .7789
            write_archive(run / "request_0001", observation(), early)
            write_archive(run / "request_0006", observation(), archived_actions())
            self.assertEqual(select_request(run).name, "request_0006")

    def test_archive_loader_checks_exact_canonical_arrays_and_hashes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / "request_0006"
            original = observation()
            write_archive(directory, original, archived_actions())
            loaded, actions, seed, index, noise, provenance = load_archive(directory)
            self.assertEqual((seed, index, noise), (17, 6, NOISE))
            self.assertTrue(np.array_equal(actions, archived_actions()))
            self.assertEqual(provenance["prompt"], original["prompt"])
            for key in original:
                if isinstance(original[key], np.ndarray):
                    self.assertEqual(array_evidence(loaded[key]), array_evidence(original[key]))
            original_request = json.loads((directory / "request.json").read_text())
            original_request["canonical_schema"][GRIPPER_KEY]["sha256"] = "0" * 64
            (directory / "request.json").write_text(json.dumps(original_request))
            with self.assertRaisesRegex(ValueError, "canonical field disagrees"):
                load_archive(directory)

    def test_only_gripper_changes_with_valid_float32_closure(self) -> None:
        original = observation()
        hashes = {key: array_evidence(value) for key, value in original.items()
                  if isinstance(value, np.ndarray)}
        for closure in (0, .25, .5, .75, 1):
            candidate = changed_gripper_only(original, closure)
            self.assertEqual(candidate[GRIPPER_KEY].shape, (1,))
            self.assertEqual(candidate[GRIPPER_KEY].dtype, np.float32)
            for key in hashes:
                if key != GRIPPER_KEY:
                    self.assertEqual(array_evidence(candidate[key]), hashes[key])
            self.assertEqual(candidate["prompt"], original["prompt"])
        self.assertEqual(array_evidence(original[GRIPPER_KEY]), hashes[GRIPPER_KEY])
        for invalid in (-.1, 1.1, np.nan, np.inf):
            with self.assertRaises(ValueError):
                changed_gripper_only(original, invalid)
        self.assertEqual(droid_gripper_decision(.5).binary_closure, 0)
        self.assertEqual(droid_gripper_decision(.50001).binary_closure, 1)

    def test_ideal_rollout_uses_frozen_clip_and_fk(self) -> None:
        actions = archived_actions()
        actions[0, :7] = [2, -2, 1, 0, 0, 0, 0]
        q0 = observation()[JOINT_KEY]
        result = ideal_rollout(q0, actions)
        expected_q1 = q0 + np.array([.2, -.2, .2, 0, 0, 0, 0])
        np.testing.assert_allclose(result["ideal_q_rad"][1], expected_q1)
        np.testing.assert_allclose(result["ideal_tcp_xyz_m"][1], fr3_tcp_fk(expected_q1)[:3, 3])
        self.assertEqual(len(result["ideal_tcp_steps_m"]), 8)

    def test_exact_replay_then_five_conditions(self) -> None:
        policy = FakePolicy(archived_actions())
        report, arrays = {}, {}
        run_gate(policy, observation(), archived_actions(), 17, 6, NOISE, report, arrays)
        self.assertTrue(report["gate"]["passed"])
        self.assertEqual(len(policy.calls), 8)
        self.assertEqual(len(report["sweep"]), 5)
        self.assertEqual(set(arrays), {"archived_actions", "replay_0", "replay_1", "replay_2",
                                       "closures", *(f"sweep_{i}" for i in range(5))})
        self.assertEqual(report["sweep"][0]["first_eight_close_count"], 8)

    def test_nonidentical_repeat_stops_before_sweep(self) -> None:
        policy = FakePolicy(archived_actions(), change_call=2)
        report, arrays = {}, {}
        with self.assertRaisesRegex(RuntimeError, "sweep skipped"):
            run_gate(policy, observation(), archived_actions(), 17, 6, NOISE, report, arrays)
        self.assertEqual(len(policy.calls), 3)
        self.assertFalse(report["gate"]["passed"])
        self.assertNotIn("sweep", report)
        self.assertGreater(report["replays"][1]["max_abs_action_difference"], 0)

    def test_archive_action_difference_stops_before_sweep(self) -> None:
        policy = FakePolicy(archived_actions() + .01)
        with self.assertRaisesRegex(RuntimeError, "sweep skipped"):
            run_gate(policy, observation(), archived_actions(), 17, 6, NOISE, {}, {})
        self.assertEqual(len(policy.calls), 3)

    def test_noise_difference_stops_after_first_replay(self) -> None:
        policy = FakePolicy(archived_actions(), wrong_noise=True)
        with self.assertRaisesRegex(RuntimeError, "noise differs"):
            run_gate(policy, observation(), archived_actions(), 17, 6, NOISE, {}, {})
        self.assertEqual(len(policy.calls), 1)


if __name__ == "__main__":
    unittest.main()
