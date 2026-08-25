"""Responsive shared autonomy with asynchronous policy inference."""

from __future__ import annotations

import contextlib
import dataclasses
import json
from pathlib import Path
import time
from typing import Any
from typing import Protocol

import numpy as np

from saps.arbitration import ActionArbitrator
from saps.arbitration import ArbitrationMode
from saps.arbitration import SAPS_ACTIVITY_THRESHOLD
from saps.evaluation.operator_episode import operator_view_rgb
from saps.human_input.sample import HumanInputSample
from saps.policies.async_worker import AsyncPolicyChunk
from saps.policies.async_worker import AsyncPolicyRequest
from saps.policies.async_worker import AsyncPolicyWorker
from saps.policies.seeding import SEED_PROTOCOL


class OperatorInterface(Protocol):
    """Browser interface required by the shared-control loop."""

    def sample(self) -> HumanInputSample:
        """Return the latest operator input."""

    def publish_frame_rgb(
        self,
        image_rgb: np.ndarray,
        runtime_status: dict[str, Any],
    ) -> None:
        """Publish one browser frame."""


@dataclasses.dataclass(frozen=True)
class SharedEpisodeLoopResult:
    """Result of one shared-control episode."""

    success: bool
    termination_reason: str
    simulation_steps: int
    control_steps: int
    control_elapsed_seconds: float

    policy_replan_count: int
    sampling_protocol_version: int | None

    final_observation: dict[str, Any]
    replay_images: tuple[np.ndarray, ...]



def _policy_execution_state(
    mode: ArbitrationMode,
) -> str:
    """Return the state used while consuming policy actions."""

    if mode is ArbitrationMode.FIXED_BLEND:
        return "fixed_blend"

    if mode is ArbitrationMode.COSINE_BLEND:
        return "cosine_blend"

    return "autonomous"


def _policy_wait_state(
    mode: ArbitrationMode,
) -> str:
    """Return the normal policy-wait state for one mode."""

    if mode is ArbitrationMode.FIXED_BLEND:
        return "fixed_blend_policy_wait"

    if mode is ArbitrationMode.COSINE_BLEND:
        return "cosine_blend_policy_wait"

    return "policy_wait"


def _configured_weight(
    mode: ArbitrationMode,
    fixed_autonomy_weight: float,
) -> float | None:
    """Return the configured blend weight when applicable."""

    if mode is ArbitrationMode.FIXED_BLEND:
        return fixed_autonomy_weight

    return None


def _effective_weight_without_step(
    *,
    mode: ArbitrationMode,
    human_active: bool,
    fixed_autonomy_weight: float,
) -> float | None:
    """Report the weight that would apply once policy data exists."""

    if mode is ArbitrationMode.FIXED_BLEND and human_active:
        return fixed_autonomy_weight

    if mode is ArbitrationMode.COSINE_BLEND and human_active:
        return None

    if mode is ArbitrationMode.TAKEOVER and human_active:
        return 0.0

    return 1.0


def _reported_cosine_gain(
    mode: ArbitrationMode,
    cosine_gain: float,
) -> float | None:
    """Return the configured cosine gain when applicable."""

    if mode is ArbitrationMode.COSINE_BLEND:
        return cosine_gain

    return None


def _cosine_wait_status(
    *,
    mode: ArbitrationMode,
    human_active: bool,
) -> str:
    """Describe cosine metadata while no policy action exists."""

    if mode is not ArbitrationMode.COSINE_BLEND:
        return "not_applicable"

    if not human_active:
        return "human_idle"

    return "waiting_for_policy"


def run_shared_episode_loop(
    *,
    env: Any,
    operator: OperatorInterface,
    policy_worker: AsyncPolicyWorker,
    initial_observation: dict[str, Any],
    task_description: str,
    object_body_name: str,
    arbitration_mode: str,
    fixed_autonomy_weight: float = 0.5,
    cosine_gain: float = 6.0,
    replan_steps: int,
    policy_episode_seed: int,
    environment_seed: int,
    max_steps: int,
    control_frequency_hz: float,
    steps_path: Path,
    initial_generation: int = 0,
) -> SharedEpisodeLoopResult:
    """Run responsive shared autonomy with asynchronous inference.

    The operator scheduler runs continuously at the requested rate.
    LIBERO advances only when a valid arbitration decision exists.
    """

    if replan_steps <= 0:
        raise ValueError("replan_steps must be positive.")

    if max_steps <= 0:
        raise ValueError("max_steps must be positive.")

    if control_frequency_hz <= 0.0:
        raise ValueError(
            "control_frequency_hz must be positive."
        )

    mode = ArbitrationMode(arbitration_mode)
    arbitrator = ActionArbitrator(
        mode=mode,
        fixed_autonomy_weight=fixed_autonomy_weight,
        cosine_gain=cosine_gain,
    )

    steps_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )
    waits_path = steps_path.with_name(
        "scheduler_waits.jsonl"
    )

    observation = initial_observation
    replay_images: list[np.ndarray] = []

    simulation_steps = 0
    control_steps = 0
    scheduler_ticks = 0

    success = False
    termination_reason = "timeout"

    generation = int(initial_generation)
    shared_control_state = _policy_wait_state(
        mode
    )

    active_chunk: AsyncPolicyChunk | None = None
    active_chunk_index = 0

    last_autonomous_action = np.asarray(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
        dtype=np.float32,
    )
    takeover_action = last_autonomous_action.copy()

    last_policy_replan_index: int | None = None
    last_policy_chunk_index: int | None = None
    last_protocol_version: int | None = None
    last_noise_sha256: str | None = None

    takeover_replan_index: int | None = None
    takeover_chunk_index: int | None = None
    takeover_protocol_version: int | None = None
    takeover_noise_sha256: str | None = None

    autonomy_wait_ticks = 0
    last_executed_gripper = -1.0

    control_period = 1.0 / control_frequency_hz
    next_deadline = time.monotonic()
    control_start = time.perf_counter()

    with contextlib.ExitStack() as stack:
        steps_file = stack.enter_context(
            steps_path.open(
                "w",
                encoding="utf-8",
            )
        )
        waits_file = stack.enter_context(
            waits_path.open(
                "w",
                encoding="utf-8",
            )
        )

        while control_steps < max_steps:
            tick_start = time.monotonic()

            completed_chunk = policy_worker.poll()
            result_disposition: str | None = None
            result_applied = False

            if completed_chunk is not None:
                if (
                    completed_chunk.request.generation
                    != generation
                ):
                    result_disposition = (
                        "discarded_stale_generation"
                    )
                elif (
                    mode is ArbitrationMode.TAKEOVER
                    and shared_control_state
                    == "human_takeover"
                ):
                    result_disposition = (
                        "discarded_human_takeover"
                    )
                else:
                    active_chunk = completed_chunk
                    active_chunk_index = 0
                    shared_control_state = (
                        _policy_execution_state(mode)
                    )

                    result_disposition = "applied"
                    result_applied = True

            sample = operator.sample()

            if sample.abort_requested:
                termination_reason = "operator_abort"
                break

            if not sample.connected:
                termination_reason = (
                    "operator_disconnected"
                )
                break

            human_action = _validated_human_action(
                sample.action
            )
            human_motion_norm = float(
                np.linalg.norm(human_action[:6])
            )
            human_active = (
                human_motion_norm
                > SAPS_ACTIVITY_THRESHOLD
            )

            transition: str | None = None
            submitted_request: (
                AsyncPolicyRequest | None
            ) = None

            if mode is ArbitrationMode.TAKEOVER:
                if (
                    human_active
                    and shared_control_state
                    != "human_takeover"
                ):
                    transition = "takeover_started"
                    generation += 1

                    (
                        takeover_action,
                        takeover_replan_index,
                        takeover_chunk_index,
                        takeover_protocol_version,
                        takeover_noise_sha256,
                    ) = _capture_counterfactual(
                        active_chunk=active_chunk,
                        active_chunk_index=(
                            active_chunk_index
                        ),
                        fallback_action=(
                            last_autonomous_action
                        ),
                        fallback_replan_index=(
                            last_policy_replan_index
                        ),
                        fallback_chunk_index=(
                            last_policy_chunk_index
                        ),
                        fallback_protocol_version=(
                            last_protocol_version
                        ),
                        fallback_noise_sha256=(
                            last_noise_sha256
                        ),
                    )

                    active_chunk = None
                    active_chunk_index = 0
                    shared_control_state = (
                        "human_takeover"
                    )

                    if result_applied:
                        result_disposition = (
                            "superseded_by_takeover"
                        )
                        result_applied = False

                elif (
                    not human_active
                    and shared_control_state
                    == "human_takeover"
                ):
                    transition = "takeover_released"
                    generation += 1

                    active_chunk = None
                    active_chunk_index = 0
                    shared_control_state = (
                        "takeover_resync"
                    )
                    autonomy_wait_ticks = 0

            if (
                shared_control_state
                in {
                    "policy_wait",
                    "fixed_blend_policy_wait",
                    "cosine_blend_policy_wait",
                    "takeover_resync",
                }
                and not (
                    mode is ArbitrationMode.TAKEOVER
                    and human_active
                )
                and not policy_worker.pending
            ):
                submitted_request = (
                    policy_worker.submit(
                        observation=observation,
                        task_description=(
                            task_description
                        ),
                        request_control_step=(
                            control_steps
                        ),
                        generation=generation,
                        reason=(
                            "takeover_resync"
                            if shared_control_state
                            == "takeover_resync"
                            else "periodic"
                        ),
                    )
                )

            can_step_environment = False
            autonomous_action: np.ndarray
            autonomous_action_status: str
            autonomous_action_fresh: bool

            policy_replan_index: int | None
            policy_chunk_action_index: int | None
            protocol_version: int | None
            noise_sha256: str | None
            chunk_remaining: int | None

            if (
                shared_control_state
                == "human_takeover"
            ):
                autonomous_action = (
                    takeover_action.copy()
                )
                autonomous_action_status = (
                    "buffered_pre_takeover"
                )
                autonomous_action_fresh = False

                policy_replan_index = (
                    takeover_replan_index
                )
                policy_chunk_action_index = (
                    takeover_chunk_index
                )
                protocol_version = (
                    takeover_protocol_version
                )
                noise_sha256 = (
                    takeover_noise_sha256
                )
                chunk_remaining = None

                can_step_environment = True

            elif (
                shared_control_state
                == _policy_execution_state(mode)
                and active_chunk is not None
                and active_chunk_index
                < min(
                    replan_steps,
                    active_chunk.horizon,
                )
            ):
                autonomous_action = (
                    active_chunk.actions[
                        active_chunk_index
                    ].copy()
                )
                autonomous_action_status = (
                    "buffered_policy"
                )
                autonomous_action_fresh = True

                policy_replan_index = (
                    active_chunk.request.replan_index
                )
                policy_chunk_action_index = (
                    active_chunk_index
                )
                protocol_version = (
                    active_chunk
                    .sampling_protocol_version
                )
                noise_sha256 = (
                    active_chunk.noise_sha256
                )
                chunk_remaining = (
                    min(
                        replan_steps,
                        active_chunk.horizon,
                    )
                    - active_chunk_index
                    - 1
                )

                can_step_environment = True

            else:
                autonomous_action = np.asarray(
                    [
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        last_executed_gripper,
                    ],
                    dtype=np.float32,
                )
                autonomous_action_status = (
                    "waiting_for_fresh_policy"
                )
                autonomous_action_fresh = False

                policy_replan_index = None
                policy_chunk_action_index = None
                protocol_version = (
                    policy_worker
                    .sampling_protocol_version
                )
                noise_sha256 = None
                chunk_remaining = 0

                if shared_control_state not in {
                    "policy_wait",
                    "fixed_blend_policy_wait",
                    "cosine_blend_policy_wait",
                    "takeover_resync",
                }:
                    shared_control_state = (
                        _policy_wait_state(mode)
                    )

            if not can_step_environment:
                autonomy_wait_ticks += 1

                wait_record = {
                    "scheduler_tick": scheduler_ticks,
                    "control_steps": control_steps,
                    "shared_control_state": (
                        shared_control_state
                    ),
                    "shared_control_transition": (
                        transition
                    ),
                    "control_generation": generation,
                    "arbitration_mode": mode.value,
                    "configured_autonomy_weight": (
                        _configured_weight(
                            mode,
                            fixed_autonomy_weight,
                        )
                    ),
                    "effective_autonomy_weight": (
                        _effective_weight_without_step(
                            mode=mode,
                            human_active=human_active,
                            fixed_autonomy_weight=(
                                fixed_autonomy_weight
                            ),
                        )
                    ),
                    "cosine_gain": (
                        _reported_cosine_gain(
                            mode,
                            cosine_gain,
                        )
                    ),
                    "cosine_similarity": None,
                    "cosine_similarity_status": (
                        _cosine_wait_status(
                            mode=mode,
                            human_active=human_active,
                        )
                    ),
                    "policy_worker_pending": (
                        policy_worker.pending
                    ),
                    "policy_result_disposition": (
                        result_disposition
                    ),
                    "inference_latency_seconds": (
                        completed_chunk
                        .inference_latency_seconds
                        if completed_chunk is not None
                        else None
                    ),
                    "autonomy_wait_ticks": (
                        autonomy_wait_ticks
                    ),
                    "human_active": human_active,
                    "operator_pressed_keys": list(
                        sample.pressed_keys
                    ),
                    "human_input": sample.as_dict(),
                    "wall_time_unix_seconds": (
                        time.time()
                    ),
                }

                waits_file.write(
                    json.dumps(wait_record) + "\n"
                )
                waits_file.flush()

                operator.publish_frame_rgb(
                    operator_view_rgb(observation),
                    runtime_status={
                        "phase": "shared_autonomy",
                        "arbitration_mode": mode.value,
                        "shared_control_state": (
                            shared_control_state
                        ),
                        "control_step": control_steps,
                        "max_steps": max_steps,
                        "human_active": human_active,
                        "configured_autonomy_weight": (
                            _configured_weight(
                                mode,
                                fixed_autonomy_weight,
                            )
                        ),
                        "effective_autonomy_weight": (
                            _effective_weight_without_step(
                                mode=mode,
                                human_active=human_active,
                                fixed_autonomy_weight=(
                                    fixed_autonomy_weight
                                ),
                            )
                        ),
                        "cosine_gain": (
                            _reported_cosine_gain(
                                mode,
                                cosine_gain,
                            )
                        ),
                        "cosine_similarity": None,
                        "cosine_similarity_status": (
                            _cosine_wait_status(
                                mode=mode,
                                human_active=human_active,
                            )
                        ),
                        "policy_worker_pending": (
                            policy_worker.pending
                        ),
                        "autonomous_action_status": (
                            autonomous_action_status
                        ),
                        "autonomy_wait_ticks": (
                            autonomy_wait_ticks
                        ),
                    },
                )

                scheduler_ticks += 1
                next_deadline = _sleep_to_deadline(
                    next_deadline=next_deadline,
                    control_period=control_period,
                )
                continue

            state_used_for_step = shared_control_state

            arbitration_result = arbitrator.arbitrate(
                autonomous_action=autonomous_action,
                human_action=human_action,
            )

            try:
                (
                    observation,
                    reward,
                    done,
                    info,
                ) = env.step(
                    arbitration_result
                    .executed_action
                    .tolist()
                )
            except ValueError as error:
                if (
                    "executing action in terminated episode"
                    not in str(error)
                ):
                    raise

                termination_reason = (
                    "environment_terminated"
                )
                break

            simulation_steps += 1
            control_steps += 1

            last_executed_gripper = float(
                arbitration_result
                .executed_action[6]
            )

            wait_ticks_before_step = (
                autonomy_wait_ticks
            )
            autonomy_wait_ticks = 0

            if state_used_for_step in {
                "autonomous",
                "fixed_blend",
                "cosine_blend",
            }:
                active_chunk_index += 1

                last_autonomous_action = (
                    autonomous_action.copy()
                )
                last_policy_replan_index = (
                    policy_replan_index
                )
                last_policy_chunk_index = (
                    policy_chunk_action_index
                )
                last_protocol_version = (
                    protocol_version
                )
                last_noise_sha256 = noise_sha256

                action_limit = min(
                    replan_steps,
                    (
                        active_chunk.horizon
                        if active_chunk is not None
                        else replan_steps
                    ),
                )

                if active_chunk_index >= action_limit:
                    active_chunk = None
                    active_chunk_index = 0
                    shared_control_state = (
                        _policy_wait_state(mode)
                    )

                    if not policy_worker.pending:
                        submitted_request = (
                            policy_worker.submit(
                                observation=observation,
                                task_description=(
                                    task_description
                                ),
                                request_control_step=(
                                    control_steps
                                ),
                                generation=generation,
                                reason="periodic",
                            )
                        )

            frame = operator_view_rgb(observation)
            replay_images.append(frame)

            eef_position = np.asarray(
                observation["robot0_eef_pos"],
                dtype=np.float32,
            )
            gripper_qpos = np.asarray(
                observation[
                    "robot0_gripper_qpos"
                ],
                dtype=np.float32,
            )
            object_position = np.asarray(
                env.sim.data.get_body_xpos(
                    object_body_name
                ),
                dtype=np.float64,
            )

            loop_seconds = (
                time.monotonic() - tick_start
            )

            completed_request = (
                completed_chunk.request
                if completed_chunk is not None
                else None
            )

            record = {
                "scheduler_tick": scheduler_ticks,
                "simulation_step": simulation_steps,
                "control_step": control_steps - 1,
                **arbitration_result.as_dict(),
                "shared_control_state": (
                    state_used_for_step
                ),
                "shared_control_state_after_step": (
                    shared_control_state
                ),
                "shared_control_transition": (
                    transition
                ),
                "control_generation": generation,
                "autonomous_action_status": (
                    autonomous_action_status
                ),
                "autonomous_action_fresh": (
                    autonomous_action_fresh
                ),
                "policy_action": (
                    autonomous_action.tolist()
                ),
                "policy_replan_index": (
                    policy_replan_index
                ),
                "policy_chunk_action_index": (
                    policy_chunk_action_index
                ),
                "policy_chunk_remaining_actions": (
                    chunk_remaining
                ),
                "policy_chunk_exhausted": False,
                "replanned": result_applied,
                "inference_latency_seconds": (
                    completed_chunk
                    .inference_latency_seconds
                    if completed_chunk is not None
                    else None
                ),
                "sampling_protocol_version": (
                    protocol_version
                ),
                "policy_noise_sha256": (
                    noise_sha256
                ),
                "policy_worker_pending": (
                    policy_worker.pending
                ),
                "policy_request_paused_for_human": (
                    mode is ArbitrationMode.TAKEOVER
                    and state_used_for_step
                    == "human_takeover"
                ),
                "policy_request_submitted_this_step": (
                    submitted_request is not None
                ),
                "policy_request_reason": (
                    submitted_request.reason
                    if submitted_request is not None
                    else None
                ),
                "policy_request_replan_index": (
                    submitted_request.replan_index
                    if submitted_request is not None
                    else None
                ),
                "policy_result_replan_index": (
                    completed_request.replan_index
                    if completed_request is not None
                    else None
                ),
                "policy_result_disposition": (
                    result_disposition
                ),
                "autonomy_wait_ticks_before_step": (
                    wait_ticks_before_step
                ),
                "environment_seed": environment_seed,
                "policy_episode_seed": (
                    policy_episode_seed
                ),
                "policy_seed_protocol": (
                    SEED_PROTOCOL
                ),
                "operator_connected": (
                    sample.connected
                ),
                "operator_armed": sample.armed,
                "operator_abort_requested": (
                    sample.abort_requested
                ),
                "operator_motion_active": (
                    sample.motion_active
                ),
                "operator_speed_mode": (
                    sample.speed_mode
                ),
                "operator_translation_gain": (
                    sample.translation_gain
                ),
                "operator_rotation_gain": (
                    sample.rotation_gain
                ),
                "operator_gripper_command": (
                    sample.gripper_command
                ),
                "operator_pressed_keys": list(
                    sample.pressed_keys
                ),
                "operator_sample_monotonic_seconds": (
                    sample.sample_monotonic_seconds
                ),
                "operator_last_event_monotonic_seconds": (
                    sample
                    .last_event_monotonic_seconds
                ),
                "human_input": sample.as_dict(),
                "reward": float(reward),
                "done": bool(done),
                "eef_position": eef_position.tolist(),
                "object_position": (
                    object_position.tolist()
                ),
                "gripper_qpos": (
                    gripper_qpos.tolist()
                ),
                "step_wall_time_unix_seconds": (
                    time.time()
                ),
                "step_loop_seconds": loop_seconds,
                "control_deadline_missed": (
                    loop_seconds > control_period
                ),
            }

            steps_file.write(
                json.dumps(record) + "\n"
            )
            steps_file.flush()

            operator.publish_frame_rgb(
                frame,
                runtime_status={
                    "phase": "shared_autonomy",
                    "arbitration_mode": mode.value,
                    "shared_control_state": (
                        state_used_for_step
                    ),
                    "control_step": control_steps,
                    "max_steps": max_steps,
                    "success": bool(done),
                    "reward": float(reward),
                    "human_active": (
                        arbitration_result.human_active
                    ),
                    "configured_autonomy_weight": (
                        arbitration_result
                        .configured_autonomy_weight
                    ),
                    "effective_autonomy_weight": (
                        arbitration_result
                        .autonomy_weight
                    ),
                    "autonomy_weight": (
                        arbitration_result
                        .autonomy_weight
                    ),
                    "cosine_gain": (
                        arbitration_result.cosine_gain
                    ),
                    "cosine_similarity": (
                        arbitration_result.cosine_similarity
                    ),
                    "cosine_similarity_status": (
                        arbitration_result
                        .cosine_similarity_status
                    ),
                    "policy_worker_pending": (
                        policy_worker.pending
                    ),
                    "autonomous_action_status": (
                        autonomous_action_status
                    ),
                    "autonomy_wait_ticks_before_step": (
                        wait_ticks_before_step
                    ),
                    "eef_position": (
                        eef_position.tolist()
                    ),
                    "object_position": (
                        object_position.tolist()
                    ),
                },
            )

            if done:
                success = True
                termination_reason = "success"
                break

            scheduler_ticks += 1
            next_deadline = _sleep_to_deadline(
                next_deadline=next_deadline,
                control_period=control_period,
            )

    return SharedEpisodeLoopResult(
        success=success,
        termination_reason=termination_reason,
        simulation_steps=simulation_steps,
        control_steps=control_steps,
        control_elapsed_seconds=(
            time.perf_counter() - control_start
        ),
        policy_replan_count=(
            policy_worker.replan_count
        ),
        sampling_protocol_version=(
            policy_worker.sampling_protocol_version
        ),
        final_observation=observation,
        replay_images=tuple(replay_images),
    )


def _capture_counterfactual(
    *,
    active_chunk: AsyncPolicyChunk | None,
    active_chunk_index: int,
    fallback_action: np.ndarray,
    fallback_replan_index: int | None,
    fallback_chunk_index: int | None,
    fallback_protocol_version: int | None,
    fallback_noise_sha256: str | None,
) -> tuple[
    np.ndarray,
    int | None,
    int | None,
    int | None,
    str | None,
]:
    """Capture the next autonomous action at takeover."""

    if (
        active_chunk is not None
        and active_chunk_index < active_chunk.horizon
    ):
        return (
            active_chunk.actions[
                active_chunk_index
            ].copy(),
            active_chunk.request.replan_index,
            active_chunk_index,
            active_chunk.sampling_protocol_version,
            active_chunk.noise_sha256,
        )

    return (
        fallback_action.copy(),
        fallback_replan_index,
        fallback_chunk_index,
        fallback_protocol_version,
        fallback_noise_sha256,
    )


def _validated_human_action(
    action: np.ndarray,
) -> np.ndarray:
    """Validate and clip one browser action."""

    human_action = np.asarray(
        action,
        dtype=np.float32,
    )

    if human_action.shape != (7,):
        raise ValueError(
            "Expected human action shape (7,), "
            f"received {human_action.shape}."
        )

    if not np.all(np.isfinite(human_action)):
        raise ValueError(
            "Human action must contain only finite values."
        )

    return np.clip(
        human_action,
        -1.0,
        1.0,
    ).astype(np.float32)


def _sleep_to_deadline(
    *,
    next_deadline: float,
    control_period: float,
) -> float:
    """Sleep until the next operator-scheduler deadline."""

    next_deadline += control_period
    remaining = next_deadline - time.monotonic()

    if remaining > 0.0:
        time.sleep(remaining)
    else:
        next_deadline = time.monotonic()

    return next_deadline
