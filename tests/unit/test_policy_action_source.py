"""Tests for deterministic buffered policy actions."""

from __future__ import annotations

import unittest
from typing import Any

import numpy as np

from saps.policies.action_source import (
    ChunkedPolicyActionSource,
)


def make_chunk(
    base: float,
    *,
    length: int = 3,
) -> np.ndarray:
    rows = []

    for index in range(length):
        rows.append(
            np.full(
                7,
                base + index,
                dtype=np.float32,
            )
        )

    return np.stack(rows)


class FakePolicy:
    def __init__(
        self,
        chunks: list[np.ndarray],
        *,
        protocol_versions: list[int] | None = None,
        returned_seed_offset: int = 0,
        returned_replan_offset: int = 0,
    ) -> None:
        self._chunks = list(chunks)
        self._protocol_versions = (
            list(protocol_versions)
            if protocol_versions is not None
            else [1] * len(chunks)
        )
        self._returned_seed_offset = (
            returned_seed_offset
        )
        self._returned_replan_offset = (
            returned_replan_offset
        )

        self.calls: list[dict[str, Any]] = []
        self.last_sampling_metadata: (
            dict[str, Any] | None
        ) = None

    def infer(
        self,
        policy_input: dict[str, Any],
        *,
        policy_episode_seed: int | None = None,
        replan_index: int | None = None,
    ) -> np.ndarray:
        self.calls.append(
            {
                "policy_input": policy_input,
                "policy_episode_seed": (
                    policy_episode_seed
                ),
                "replan_index": replan_index,
            }
        )

        if not self._chunks:
            raise RuntimeError("No fake chunks remain.")

        chunk_index = len(self.calls) - 1
        chunk = self._chunks.pop(0)

        if policy_episode_seed is None:
            self.last_sampling_metadata = None
        else:
            self.last_sampling_metadata = {
                "policy_episode_seed": (
                    policy_episode_seed
                    + self._returned_seed_offset
                ),
                "replan_index": (
                    int(replan_index)
                    + self._returned_replan_offset
                ),
                "protocol_version": (
                    self._protocol_versions[
                        chunk_index
                    ]
                ),
                "noise_sha256": (
                    f"noise-{chunk_index}"
                ),
            }

        return chunk


class ChunkedPolicyActionSourceTest(
    unittest.TestCase
):
    def test_seeded_actions_preserve_replan_sequence(
        self,
    ) -> None:
        policy = FakePolicy(
            [
                make_chunk(10.0),
                make_chunk(20.0),
            ]
        )
        source = ChunkedPolicyActionSource(
            policy=policy,
            replan_steps=2,
            policy_episode_seed=1234,
        )

        first = source.next_action({"observation": 1})
        second = source.next_action({"observation": 2})
        third = source.next_action({"observation": 3})

        np.testing.assert_array_equal(
            first.action,
            np.full(7, 10.0, dtype=np.float32),
        )
        np.testing.assert_array_equal(
            second.action,
            np.full(7, 11.0, dtype=np.float32),
        )
        np.testing.assert_array_equal(
            third.action,
            np.full(7, 20.0, dtype=np.float32),
        )

        self.assertTrue(first.replanned)
        self.assertFalse(second.replanned)
        self.assertTrue(third.replanned)

        self.assertEqual(
            first.policy_replan_index,
            0,
        )
        self.assertEqual(
            second.policy_replan_index,
            0,
        )
        self.assertEqual(
            third.policy_replan_index,
            1,
        )

        self.assertEqual(
            first.policy_chunk_action_index,
            0,
        )
        self.assertEqual(
            second.policy_chunk_action_index,
            1,
        )
        self.assertEqual(
            third.policy_chunk_action_index,
            0,
        )

        self.assertEqual(source.replan_count, 2)
        self.assertEqual(
            source.sampling_protocol_version,
            1,
        )
        self.assertEqual(
            policy.calls[0]["policy_episode_seed"],
            1234,
        )
        self.assertEqual(
            policy.calls[0]["replan_index"],
            0,
        )
        self.assertEqual(
            policy.calls[1]["replan_index"],
            1,
        )

    def test_unseeded_policy_omits_sampling_arguments(
        self,
    ) -> None:
        policy = FakePolicy([make_chunk(1.0)])
        source = ChunkedPolicyActionSource(
            policy=policy,
            replan_steps=1,
            policy_episode_seed=None,
        )

        sample = source.next_action({"frame": 1})

        self.assertIsNone(
            policy.calls[0]["policy_episode_seed"]
        )
        self.assertIsNone(
            policy.calls[0]["replan_index"]
        )
        self.assertIsNone(
            sample.sampling_metadata
        )
        self.assertIsNone(
            sample.sampling_protocol_version
        )

    def test_seed_mismatch_is_rejected(self) -> None:
        policy = FakePolicy(
            [make_chunk(1.0)],
            returned_seed_offset=1,
        )
        source = ChunkedPolicyActionSource(
            policy=policy,
            replan_steps=1,
            policy_episode_seed=10,
        )

        with self.assertRaisesRegex(
            RuntimeError,
            "different episode seed",
        ):
            source.next_action({})

    def test_replan_mismatch_is_rejected(self) -> None:
        policy = FakePolicy(
            [make_chunk(1.0)],
            returned_replan_offset=1,
        )
        source = ChunkedPolicyActionSource(
            policy=policy,
            replan_steps=1,
            policy_episode_seed=10,
        )

        with self.assertRaisesRegex(
            RuntimeError,
            "different replan index",
        ):
            source.next_action({})

    def test_protocol_change_is_rejected(self) -> None:
        policy = FakePolicy(
            [
                make_chunk(1.0),
                make_chunk(2.0),
            ],
            protocol_versions=[1, 2],
        )
        source = ChunkedPolicyActionSource(
            policy=policy,
            replan_steps=1,
            policy_episode_seed=10,
        )

        source.next_action({})

        with self.assertRaisesRegex(
            RuntimeError,
            "protocol changed",
        ):
            source.next_action({})

    def test_short_chunk_is_rejected(self) -> None:
        policy = FakePolicy(
            [make_chunk(1.0, length=2)]
        )
        source = ChunkedPolicyActionSource(
            policy=policy,
            replan_steps=3,
            policy_episode_seed=10,
        )

        with self.assertRaisesRegex(
            ValueError,
            "replan_steps=3",
        ):
            source.next_action({})

    def test_invalid_chunk_shape_is_rejected(
        self,
    ) -> None:
        policy = FakePolicy(
            [
                np.zeros(
                    (3, 6),
                    dtype=np.float32,
                )
            ]
        )
        source = ChunkedPolicyActionSource(
            policy=policy,
            replan_steps=1,
            policy_episode_seed=10,
        )

        with self.assertRaisesRegex(
            ValueError,
            "shape",
        ):
            source.next_action({})


if __name__ == "__main__":
    unittest.main()
