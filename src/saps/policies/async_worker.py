"""Single-threaded asynchronous OpenPI inference worker."""

from __future__ import annotations

import concurrent.futures
import dataclasses
import time
from typing import Any

import numpy as np


@dataclasses.dataclass(frozen=True)
class AsyncPolicyRequest:
    """Metadata describing one policy request."""

    replan_index: int
    request_control_step: int
    generation: int
    reason: str
    submitted_monotonic_seconds: float
    observation_eef_position: tuple[float, ...] | None
    observation_eef_quaternion: tuple[float, ...] | None
    observation_gripper_qpos: tuple[float, ...] | None


@dataclasses.dataclass(frozen=True)
class AsyncPolicyChunk:
    """One complete policy chunk returned by the worker."""

    request: AsyncPolicyRequest
    actions: np.ndarray
    inference_latency_seconds: float
    completed_monotonic_seconds: float
    sampling_protocol_version: int
    noise_sha256: str | None

    @property
    def horizon(self) -> int:
        """Return the number of available actions."""

        return int(self.actions.shape[0])


class AsyncPolicyWorker:
    """Run at most one policy inference at a time."""

    def __init__(
        self,
        *,
        policy: Any,
        policy_episode_seed: int,
    ) -> None:
        if policy_episode_seed < 0:
            raise ValueError(
                "policy_episode_seed must be non-negative."
            )

        self._policy = policy
        self._policy_episode_seed = (
            int(policy_episode_seed)
        )

        self._executor = (
            concurrent.futures.ThreadPoolExecutor(
                max_workers=1,
                thread_name_prefix="saps-policy",
            )
        )

        self._future: Any | None = None
        self._pending_request: (
            AsyncPolicyRequest | None
        ) = None

        self._next_replan_index = 0
        self._replan_count = 0
        self._sampling_protocol_version: (
            int | None
        ) = None
        self._last_request_control_step: (
            int | None
        ) = None

    @property
    def pending(self) -> bool:
        """Return whether an inference request is in flight."""

        return self._future is not None

    @property
    def pending_request(
        self,
    ) -> AsyncPolicyRequest | None:
        """Return metadata for the in-flight request."""

        return self._pending_request

    @property
    def replan_count(self) -> int:
        """Return the number of submitted policy requests."""

        return self._replan_count

    @property
    def sampling_protocol_version(
        self,
    ) -> int | None:
        """Return the sampling protocol observed so far."""

        return self._sampling_protocol_version

    @property
    def last_request_control_step(
        self,
    ) -> int | None:
        """Return the control step of the latest request."""

        return self._last_request_control_step

    def submit(
        self,
        *,
        observation: dict[str, Any],
        task_description: str,
        request_control_step: int,
        generation: int,
        reason: str,
    ) -> AsyncPolicyRequest | None:
        """Submit one request if the worker is currently idle."""

        if self.pending:
            return None

        policy_input, _ = (
            self._policy.prepare_observation(
                observation,
                task_description,
            )
        )

        request = AsyncPolicyRequest(
            replan_index=self._next_replan_index,
            request_control_step=(
                int(request_control_step)
            ),
            generation=int(generation),
            reason=str(reason),
            submitted_monotonic_seconds=(
                time.monotonic()
            ),
            observation_eef_position=_observation_tuple(
                observation,
                "robot0_eef_pos",
            ),
            observation_eef_quaternion=_observation_tuple(
                observation,
                "robot0_eef_quat",
            ),
            observation_gripper_qpos=_observation_tuple(
                observation,
                "robot0_gripper_qpos",
            ),
        )

        self._next_replan_index += 1
        self._replan_count += 1
        self._last_request_control_step = (
            request.request_control_step
        )

        self._pending_request = request
        self._future = self._executor.submit(
            self._infer,
            request,
            policy_input,
        )

        return request

    def poll(self) -> AsyncPolicyChunk | None:
        """Return a completed chunk without blocking."""

        if self._future is None:
            return None

        if not self._future.done():
            return None

        future = self._future

        self._future = None
        self._pending_request = None

        chunk = future.result()

        if self._sampling_protocol_version is None:
            self._sampling_protocol_version = (
                chunk.sampling_protocol_version
            )
        elif (
            self._sampling_protocol_version
            != chunk.sampling_protocol_version
        ):
            raise RuntimeError(
                "Policy sampling protocol changed from "
                f"{self._sampling_protocol_version} to "
                f"{chunk.sampling_protocol_version}."
            )

        return chunk

    def close(self) -> None:
        """Wait for an in-flight request and stop the worker."""

        self._executor.shutdown(wait=True)

    def _infer(
        self,
        request: AsyncPolicyRequest,
        policy_input: dict[str, Any],
    ) -> AsyncPolicyChunk:
        start = time.perf_counter()

        actions = self._policy.infer(
            policy_input,
            policy_episode_seed=(
                self._policy_episode_seed
            ),
            replan_index=request.replan_index,
        )

        latency = time.perf_counter() - start
        completed = time.monotonic()

        metadata = dict(
            self._policy.last_sampling_metadata
            or {}
        )

        returned_seed = metadata.get(
            "policy_episode_seed"
        )
        returned_replan = metadata.get(
            "replan_index"
        )
        protocol_version = metadata.get(
            "protocol_version"
        )

        if returned_seed != self._policy_episode_seed:
            raise RuntimeError(
                "Policy server returned seed "
                f"{returned_seed!r}; expected "
                f"{self._policy_episode_seed}."
            )

        if returned_replan != request.replan_index:
            raise RuntimeError(
                "Policy server returned replan index "
                f"{returned_replan!r}; expected "
                f"{request.replan_index}."
            )

        if protocol_version is None:
            raise RuntimeError(
                "Policy server did not return a "
                "sampling protocol version."
            )

        action_array = np.asarray(
            actions,
            dtype=np.float32,
        )

        if (
            action_array.ndim != 2
            or action_array.shape[1] != 7
            or action_array.shape[0] == 0
        ):
            raise ValueError(
                "Expected policy chunk shape [horizon, 7], "
                f"received {action_array.shape}."
            )

        if not np.all(np.isfinite(action_array)):
            raise ValueError(
                "Policy chunk contains non-finite values."
            )

        frozen_actions = action_array.copy()
        frozen_actions.setflags(write=False)

        noise_sha256 = metadata.get(
            "noise_sha256"
        )

        if noise_sha256 is not None:
            noise_sha256 = str(noise_sha256)

        return AsyncPolicyChunk(
            request=request,
            actions=frozen_actions,
            inference_latency_seconds=latency,
            completed_monotonic_seconds=completed,
            sampling_protocol_version=int(
                protocol_version
            ),
            noise_sha256=noise_sha256,
        )


def _observation_tuple(
    observation: dict[str, Any],
    key: str,
) -> tuple[float, ...] | None:
    """Copy one finite observation vector into request metadata."""

    value = observation.get(key)

    if value is None:
        return None

    array = np.asarray(value, dtype=np.float64).reshape(-1)

    if not np.all(np.isfinite(array)):
        raise ValueError(
            f"Observation field {key!r} contains non-finite values."
        )

    return tuple(float(item) for item in array)
