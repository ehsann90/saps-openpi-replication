"""Buffered policy actions with deterministic SAPS sampling metadata."""

from __future__ import annotations

import collections
import dataclasses
import time
from typing import Any
from typing import Protocol

import numpy as np


ACTION_DIMENSION = 7


class ChunkPolicy(Protocol):
    """Minimal policy interface required by the action source."""

    last_sampling_metadata: dict[str, Any] | None

    def infer(
        self,
        policy_input: dict[str, Any],
        *,
        policy_episode_seed: int | None = None,
        replan_index: int | None = None,
    ) -> np.ndarray:
        """Return one action chunk."""


@dataclasses.dataclass(frozen=True)
class PolicyActionSample:
    """One action selected from the active policy chunk."""

    action: np.ndarray
    replanned: bool
    policy_replan_index: int
    policy_chunk_action_index: int
    inference_latency_seconds: float | None
    sampling_metadata: dict[str, Any] | None
    sampling_protocol_version: int | None

    def as_log_dict(self) -> dict[str, Any]:
        """Return JSON-compatible policy fields for step logging."""

        return {
            "replanned": self.replanned,
            "policy_replan_index": self.policy_replan_index,
            "policy_chunk_action_index": (
                self.policy_chunk_action_index
            ),
            "sampling_protocol_version": (
                self.sampling_protocol_version
            ),
            "policy_noise_sha256": (
                self.sampling_metadata.get("noise_sha256")
                if self.sampling_metadata is not None
                else None
            ),
            "inference_latency_seconds": (
                self.inference_latency_seconds
            ),
            # Retained for compatibility with Phase 1 logs.
            "policy_action": self.action.tolist(),
        }


class ChunkedPolicyActionSource:
    """Buffer a fixed number of actions from each policy inference."""

    def __init__(
        self,
        *,
        policy: ChunkPolicy,
        replan_steps: int,
        policy_episode_seed: int | None,
    ) -> None:
        if replan_steps <= 0:
            raise ValueError(
                "replan_steps must be greater than zero."
            )

        if (
            policy_episode_seed is not None
            and policy_episode_seed < 0
        ):
            raise ValueError(
                "policy_episode_seed must be non-negative."
            )

        self._policy = policy
        self._replan_steps = int(replan_steps)
        self._policy_episode_seed = policy_episode_seed

        self._action_plan: collections.deque[
            np.ndarray
        ] = collections.deque()

        self._next_replan_index = 0
        self._active_replan_index: int | None = None
        self._active_chunk_action_index = 0
        self._active_sampling_metadata: (
            dict[str, Any] | None
        ) = None
        self._sampling_protocol_version: int | None = None

    @property
    def replan_count(self) -> int:
        """Return the number of policy chunks requested so far."""

        return self._next_replan_index

    @property
    def sampling_protocol_version(self) -> int | None:
        """Return the seeded-sampling protocol used in this episode."""

        return self._sampling_protocol_version

    def next_action(
        self,
        policy_input: dict[str, Any],
    ) -> PolicyActionSample:
        """Return the next action, requesting a new chunk if necessary."""

        replanned = False
        inference_latency_seconds: float | None = None

        if not self._action_plan:
            (
                inference_latency_seconds,
                sampling_metadata,
                action_chunk,
            ) = self._request_chunk(policy_input)

            active_replan_index = self._next_replan_index

            self._action_plan.extend(
                _readonly_action(action)
                for action in action_chunk[
                    : self._replan_steps
                ]
            )
            self._active_replan_index = active_replan_index
            self._active_chunk_action_index = 0
            self._active_sampling_metadata = (
                sampling_metadata
            )
            self._next_replan_index += 1
            replanned = True

        if self._active_replan_index is None:
            raise RuntimeError(
                "No active policy action chunk is available."
            )

        action = self._action_plan.popleft()
        chunk_action_index = (
            self._active_chunk_action_index
        )
        self._active_chunk_action_index += 1

        return PolicyActionSample(
            action=action,
            replanned=replanned,
            policy_replan_index=(
                self._active_replan_index
            ),
            policy_chunk_action_index=(
                chunk_action_index
            ),
            inference_latency_seconds=(
                inference_latency_seconds
            ),
            sampling_metadata=(
                dict(self._active_sampling_metadata)
                if self._active_sampling_metadata
                is not None
                else None
            ),
            sampling_protocol_version=(
                self._sampling_protocol_version
            ),
        )

    def _request_chunk(
        self,
        policy_input: dict[str, Any],
    ) -> tuple[
        float,
        dict[str, Any] | None,
        np.ndarray,
    ]:
        replan_index = self._next_replan_index
        inference_start = time.perf_counter()

        if self._policy_episode_seed is None:
            action_chunk = self._policy.infer(
                policy_input
            )
        else:
            action_chunk = self._policy.infer(
                policy_input,
                policy_episode_seed=(
                    self._policy_episode_seed
                ),
                replan_index=replan_index,
            )

        inference_latency_seconds = (
            time.perf_counter() - inference_start
        )

        raw_metadata = getattr(
            self._policy,
            "last_sampling_metadata",
            None,
        )
        sampling_metadata = (
            dict(raw_metadata)
            if raw_metadata is not None
            else None
        )

        if self._policy_episode_seed is not None:
            self._validate_sampling_metadata(
                sampling_metadata,
                expected_replan_index=replan_index,
            )

        chunk = np.asarray(
            action_chunk,
            dtype=np.float32,
        )

        if (
            chunk.ndim != 2
            or chunk.shape[1] != ACTION_DIMENSION
        ):
            raise ValueError(
                "Policy action chunk must have shape "
                f"[horizon, {ACTION_DIMENSION}], "
                f"received {chunk.shape}."
            )

        if len(chunk) < self._replan_steps:
            raise ValueError(
                f"Policy returned {len(chunk)} actions, "
                f"but replan_steps={self._replan_steps}."
            )

        if not np.all(np.isfinite(chunk)):
            raise ValueError(
                "Policy action chunk must contain only "
                "finite values."
            )

        return (
            inference_latency_seconds,
            sampling_metadata,
            chunk,
        )

    def _validate_sampling_metadata(
        self,
        metadata: dict[str, Any] | None,
        *,
        expected_replan_index: int,
    ) -> None:
        if metadata is None:
            raise RuntimeError(
                "The seeded policy server did not return "
                "sampling metadata."
            )

        required = {
            "policy_episode_seed",
            "replan_index",
            "protocol_version",
        }
        missing = required.difference(metadata)

        if missing:
            raise RuntimeError(
                "Seeded policy metadata is missing fields: "
                f"{sorted(missing)}."
            )

        returned_seed = int(
            metadata["policy_episode_seed"]
        )
        returned_replan = int(
            metadata["replan_index"]
        )
        returned_protocol = int(
            metadata["protocol_version"]
        )

        if returned_seed != self._policy_episode_seed:
            raise RuntimeError(
                "The policy server returned a different "
                "episode seed."
            )

        if returned_replan != expected_replan_index:
            raise RuntimeError(
                "The policy server returned a different "
                "replan index."
            )

        if self._sampling_protocol_version is None:
            self._sampling_protocol_version = (
                returned_protocol
            )
        elif (
            self._sampling_protocol_version
            != returned_protocol
        ):
            raise RuntimeError(
                "Sampling protocol changed during an episode."
            )


def _readonly_action(action: np.ndarray) -> np.ndarray:
    """Copy and freeze one seven-dimensional policy action."""

    result = np.asarray(
        action,
        dtype=np.float32,
    ).copy()
    result.setflags(write=False)
    return result
