"""Tests for the offline OpenPI DROID policy contract."""

from __future__ import annotations

import json
import unittest
from typing import Any

import numpy as np

from saps.policies.openpi_droid import DroidRunProvenance
from saps.policies.openpi_droid import DROID_POLICY_INPUT_KEYS
from saps.policies.openpi_droid import OpenPiDroidPolicy
from saps.policies.openpi_droid import prepare_droid_observation
from saps.policies.openpi_droid import validate_droid_action_response


def make_valid_observation() -> dict[str, Any]:
    return {
        "exterior_image": np.zeros((180, 320, 3), dtype=np.uint8),
        "wrist_image": np.ones((180, 320, 3), dtype=np.uint8),
        "joint_position": np.arange(7, dtype=np.float64),
        "gripper_position": np.asarray([0.25], dtype=np.float64),
        "prompt": "Remove the lid",
    }


class FakeClient:
    def __init__(self, result: dict[str, Any]) -> None:
        self.result = result
        self.requests: list[dict[str, Any]] = []

    def get_server_metadata(self) -> dict[str, Any]:
        return {
            "saps_seeded_sampling": {
                "policy_config_name": "pi05_droid",
                "policy_checkpoint": (
                    "gs://openpi-assets/checkpoints/pi05_droid"
                ),
                "action_horizon": 15,
                "latent_action_dim": 32,
            }
        }

    def infer(self, request: dict[str, Any]) -> dict[str, Any]:
        self.requests.append(request)
        return self.result


class DroidObservationContractTest(unittest.TestCase):
    def test_valid_droid_observation_construction(self) -> None:
        policy_input = prepare_droid_observation(
            **make_valid_observation()
        )

        self.assertEqual(tuple(policy_input), DROID_POLICY_INPUT_KEYS)
        self.assertEqual(
            policy_input[
                "observation/exterior_image_1_left"
            ].shape,
            (180, 320, 3),
        )
        self.assertEqual(
            policy_input["observation/joint_position"].dtype,
            np.float32,
        )
        self.assertEqual(
            policy_input["observation/gripper_position"].shape,
            (1,),
        )

    def test_incorrect_image_shape_is_rejected(self) -> None:
        observation = make_valid_observation()
        observation["exterior_image"] = np.zeros(
            (180, 320),
            dtype=np.uint8,
        )

        with self.assertRaisesRegex(ValueError, "height, width, 3"):
            prepare_droid_observation(**observation)

    def test_incorrect_image_type_is_rejected(self) -> None:
        observation = make_valid_observation()
        observation["wrist_image"] = np.zeros(
            (180, 320, 3),
            dtype=np.float32,
        )

        with self.assertRaisesRegex(TypeError, "dtype uint8"):
            prepare_droid_observation(**observation)

    def test_incorrect_joint_dimension_is_rejected(self) -> None:
        observation = make_valid_observation()
        observation["joint_position"] = np.zeros(6, dtype=np.float32)

        with self.assertRaisesRegex(ValueError, r"shape \(7,\)"):
            prepare_droid_observation(**observation)

    def test_incorrect_gripper_shape_is_rejected(self) -> None:
        for invalid in (
            np.asarray(0.5, dtype=np.float32),
            np.asarray([0.5, 0.6], dtype=np.float32),
        ):
            with self.subTest(shape=invalid.shape):
                observation = make_valid_observation()
                observation["gripper_position"] = invalid
                with self.assertRaisesRegex(ValueError, r"shape \(1,\)"):
                    prepare_droid_observation(**observation)


class DroidActionContractTest(unittest.TestCase):
    def test_action_horizon_is_observed_not_assumed(self) -> None:
        for horizon in (1, 10, 15, 23):
            with self.subTest(horizon=horizon):
                actions = np.zeros((horizon, 8), dtype=np.float32)
                validated = validate_droid_action_response(
                    {"actions": actions}
                )
                self.assertEqual(validated.shape, (horizon, 8))

    def test_missing_actions_are_rejected(self) -> None:
        with self.assertRaisesRegex(KeyError, "actions"):
            validate_droid_action_response({})

    def test_malformed_action_shapes_are_rejected(self) -> None:
        malformed = (
            np.zeros(8, dtype=np.float32),
            np.zeros((0, 8), dtype=np.float32),
            np.zeros((15, 7), dtype=np.float32),
            np.zeros((1, 15, 8), dtype=np.float32),
        )
        for actions in malformed:
            with self.subTest(shape=actions.shape):
                with self.assertRaisesRegex(ValueError, "shape"):
                    validate_droid_action_response(
                        {"actions": actions}
                    )

    def test_non_floating_actions_are_rejected(self) -> None:
        with self.assertRaisesRegex(TypeError, "floating dtype"):
            validate_droid_action_response(
                {"actions": np.zeros((15, 8), dtype=np.int32)}
            )

    def test_non_finite_actions_are_rejected(self) -> None:
        for invalid in (np.nan, np.inf, -np.inf):
            with self.subTest(invalid=invalid):
                actions = np.zeros((15, 8), dtype=np.float32)
                actions[3, 2] = invalid
                with self.assertRaisesRegex(ValueError, "finite"):
                    validate_droid_action_response(
                        {"actions": actions}
                    )

    def test_seeded_client_preserves_runtime_horizon(self) -> None:
        result = {
            "actions": np.zeros((15, 8), dtype=np.float32),
            "saps_sampling": {
                "policy_episode_seed": 123,
                "replan_index": 4,
                "protocol_version": 1,
                "noise_sha256": "abc",
            },
            "policy_timing": {"infer_ms": 50.0},
            "server_timing": {"infer_ms": 51.0},
        }
        client = FakeClient(result)
        policy = OpenPiDroidPolicy(client=client)

        response = policy.infer(
            {"observation": "synthetic"},
            policy_episode_seed=123,
            replan_index=4,
        )

        self.assertEqual(response.actions.shape, (15, 8))
        self.assertEqual(
            client.requests[0]["__saps_protocol_version__"],
            1,
        )
        self.assertEqual(
            response.sampling_metadata["noise_sha256"],
            "abc",
        )


class DroidProvenanceTest(unittest.TestCase):
    def test_provenance_serialization_is_json_compatible(self) -> None:
        provenance = DroidRunProvenance(
            repository_commit="a" * 40,
            repository_dirty=True,
            openpi_commit="b" * 40,
            checkpoint="gs://checkpoint",
            policy_config="pi05_droid",
            dataset_source="DROID raw 1.0.1",
            sample_identities=("episode:step:0",),
            runtime={"count": np.int64(1)},
            server_metadata={
                "shape": np.asarray([15, 32], dtype=np.int32)
            },
        )

        serialized = provenance.as_dict()
        encoded = json.dumps(serialized)

        self.assertIn("pi05_droid", encoded)
        self.assertEqual(serialized["runtime"]["count"], 1)
        self.assertEqual(
            serialized["server_metadata"]["shape"],
            [15, 32],
        )


if __name__ == "__main__":
    unittest.main()
