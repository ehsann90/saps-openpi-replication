"""Episode execution and logging for the SAPS LIBERO replication."""

from __future__ import annotations

import collections
import dataclasses
import json
from pathlib import Path
import time
from typing import Any

import imageio.v2 as imageio
import numpy as np

from saps.arbitration import ActionArbitrator
from saps.arbitration import ArbitrationMode
from saps.environments.perturbations import apply_planar_object_offset
from saps.environments.perturbations import get_object_pose
from saps.policies.openpi_client import OpenPiLiberoPolicy
from saps.policies.seeding import SEED_PROTOCOL


LIBERO_DUMMY_ACTION = np.asarray(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
    dtype=np.float32,
)

IDLE_HUMAN_ACTION = np.asarray(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
    dtype=np.float32,
)


@dataclasses.dataclass(frozen=True)
class EpisodeResult:
    condition_id: str
    task_id: int
    task_description: str
    trial_index: int
    initial_state_index: int
    policy_replan_count: int
    delta_x: float
    delta_y: float
    offset_distance: float
    success: bool
    simulation_steps: int
    control_steps: int
    control_elapsed_seconds: float
    total_elapsed_seconds: float
    object_position_before: list[float]
    object_position_after_settle: list[float]
    output_directory: str

    # Added for matched SAPS experiments. Defaults preserve compatibility
    # with summaries produced before deterministic sampling was introduced.
    arbitration_mode: str = "autonomous"
    policy_episode_seed: int | None = None
    policy_seed_protocol: str | None = None
    policy_replans: int = 0
    sampling_protocol_version: int | None = None


def _save_agent_image(
    path: Path,
    obs: dict[str, Any],
) -> None:
    """Save agent-view image in the orientation used by OpenPI."""

    image = np.ascontiguousarray(
        obs["agentview_image"][::-1, ::-1]
    )
    imageio.imwrite(path, image)


def run_episode(
    *,
    env: Any,
    policy: OpenPiLiberoPolicy,
    condition_id: str,
    task_id: int,
    task_description: str,
    initial_state: np.ndarray,
    initial_state_index: int,
    trial_index: int,
    output_root: Path,
    object_joint_name: str,
    object_body_name: str,
    delta_x: float,
    delta_y: float,
    replan_steps: int,
    num_steps_wait: int,
    max_steps: int,
    video_fps: int = 10,
    policy_episode_seed: int | None = None,
    arbitration_mode: str = "autonomous",
) -> EpisodeResult:
    """Run one perturbed OpenPI episode.

    When policy_episode_seed is provided, every newly sampled action chunk
    receives a deterministic replan index. The same episode seed can therefore
    be reused across matched autonomous and shared-autonomy conditions.
    """

    if replan_steps <= 0:
        raise ValueError("replan_steps must be greater than zero.")

    if max_steps <= 0:
        raise ValueError("max_steps must be greater than zero.")

    if policy_episode_seed is not None and policy_episode_seed < 0:
        raise ValueError(
            "policy_episode_seed must be non-negative."
        )

    try:
        parsed_arbitration_mode = ArbitrationMode(
            arbitration_mode
        )
    except ValueError as error:
        supported = ", ".join(
            mode.value for mode in ArbitrationMode
        )
        raise ValueError(
            f"Unsupported arbitration mode "
            f"{arbitration_mode!r}. Supported modes: "
            f"{supported}."
        ) from error

    if parsed_arbitration_mode is not ArbitrationMode.AUTONOMOUS:
        raise ValueError(
            "The autonomous episode runner currently supports "
            "only arbitration_mode='autonomous'. Takeover "
            "requires the human-input runner."
        )

    arbitrator = ActionArbitrator(
        parsed_arbitration_mode
    )

    episode_directory = (
        output_root
        / condition_id
        / f"task_{task_id:02d}"
        / f"init_{initial_state_index:03d}"
        / f"trial_{trial_index:03d}"
    )
    episode_directory.mkdir(parents=True, exist_ok=True)

    total_start = time.perf_counter()

    env.reset()
    nominal_obs = env.set_init_state(initial_state)

    _save_agent_image(
        episode_directory / "01_nominal_initial.png",
        nominal_obs,
    )

    obs, perturbation = apply_planar_object_offset(
        env,
        joint_name=object_joint_name,
        body_name=object_body_name,
        delta_x=delta_x,
        delta_y=delta_y,
    )

    _save_agent_image(
        episode_directory / "02_perturbed_before_settle.png",
        obs,
    )

    done = False
    simulation_steps = 0

    # Match the upstream LIBERO evaluation: allow objects to settle before
    # requesting actions from the policy.
    for _ in range(num_steps_wait):
        obs, reward, done, info = env.step(
            LIBERO_DUMMY_ACTION.tolist()
        )
        simulation_steps += 1

    _save_agent_image(
        episode_directory / "03_perturbed_after_settle.png",
        obs,
    )

    settled_qpos, settled_body_position = get_object_pose(
        env,
        joint_name=object_joint_name,
        body_name=object_body_name,
    )

    perturbation_report = {
        "condition_id": condition_id,
        "requested_offset": {
            "delta_x": float(delta_x),
            "delta_y": float(delta_y),
            "distance": float(np.hypot(delta_x, delta_y)),
        },
        "immediate_result": dataclasses.asdict(perturbation),
        "settled_joint_qpos": settled_qpos.tolist(),
        "settled_body_position": settled_body_position.tolist(),
    }

    with (episode_directory / "perturbation.json").open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(perturbation_report, file, indent=2)

    action_plan: collections.deque[np.ndarray] = (
        collections.deque()
    )
    replay_images: list[np.ndarray] = []
    step_records: list[dict[str, Any]] = []

    next_replan_index = 0
    active_replan_index: int | None = None
    active_chunk_action_index = 0
    active_sampling_metadata: dict[str, Any] | None = None
    sampling_protocol_version: int | None = None

    control_start = time.perf_counter()

    while len(step_records) < max_steps and not done:
        policy_input, replay_image = policy.prepare_observation(
            obs,
            task_description,
        )
        replay_images.append(replay_image)

        replanned = False
        inference_latency_seconds: float | None = None

        if not action_plan:
            active_replan_index = next_replan_index
            active_chunk_action_index = 0

            inference_start = time.perf_counter()

            if policy_episode_seed is None:
                action_chunk = policy.infer(policy_input)
            else:
                action_chunk = policy.infer(
                    policy_input,
                    policy_episode_seed=policy_episode_seed,
                    replan_index=active_replan_index,
                )

            inference_latency_seconds = (
                time.perf_counter() - inference_start
            )
            replanned = True

            active_sampling_metadata = (
                dict(policy.last_sampling_metadata)
                if policy.last_sampling_metadata is not None
                else None
            )

            if policy_episode_seed is not None:
                if active_sampling_metadata is None:
                    raise RuntimeError(
                        "The seeded policy server did not return "
                        "sampling metadata."
                    )

                returned_seed = int(
                    active_sampling_metadata[
                        "policy_episode_seed"
                    ]
                )
                returned_replan = int(
                    active_sampling_metadata["replan_index"]
                )

                if returned_seed != policy_episode_seed:
                    raise RuntimeError(
                        "The policy server returned a different "
                        "episode seed."
                    )

                if returned_replan != active_replan_index:
                    raise RuntimeError(
                        "The policy server returned a different "
                        "replan index."
                    )

                returned_protocol = int(
                    active_sampling_metadata[
                        "protocol_version"
                    ]
                )

                if sampling_protocol_version is None:
                    sampling_protocol_version = (
                        returned_protocol
                    )
                elif (
                    sampling_protocol_version
                    != returned_protocol
                ):
                    raise RuntimeError(
                        "Sampling protocol changed during an episode."
                    )

            if len(action_chunk) < replan_steps:
                raise ValueError(
                    f"Policy returned {len(action_chunk)} actions, "
                    f"but replan_steps={replan_steps}."
                )

            action_plan.extend(
                action_chunk[:replan_steps]
            )
            next_replan_index += 1

        if active_replan_index is None:
            raise RuntimeError(
                "No active policy action chunk is available."
            )

        policy_action = np.asarray(
            action_plan.popleft(),
            dtype=np.float32,
        )

        policy_chunk_action_index = (
            active_chunk_action_index
        )
        active_chunk_action_index += 1

        arbitration_result = arbitrator.arbitrate(
            autonomous_action=policy_action,
            human_action=IDLE_HUMAN_ACTION,
        )
        executed_action = (
            arbitration_result.executed_action
        )

        obs, reward, done, info = env.step(
            executed_action.tolist()
        )
        simulation_steps += 1

        step_records.append(
            {
                "simulation_step": simulation_steps,
                "control_step": len(step_records),
                **arbitration_result.as_dict(),
                "replanned": replanned,
                "policy_episode_seed": policy_episode_seed,
                "policy_replan_index": active_replan_index,
                "policy_chunk_action_index": (
                    policy_chunk_action_index
                ),
                "sampling_protocol_version": (
                    active_sampling_metadata.get(
                        "protocol_version"
                    )
                    if active_sampling_metadata
                    else None
                ),
                "policy_noise_sha256": (
                    active_sampling_metadata.get(
                        "noise_sha256"
                    )
                    if active_sampling_metadata
                    else None
                ),
                "inference_latency_seconds": (
                    inference_latency_seconds
                ),
                "policy_action": policy_action.tolist(),
                "reward": float(reward),
                "done": bool(done),
                "eef_position": np.asarray(
                    obs["robot0_eef_pos"],
                    dtype=np.float32,
                ).tolist(),
                "object_position": np.asarray(
                    env.sim.data.get_body_xpos(
                        object_body_name
                    ),
                    dtype=np.float64,
                ).tolist(),
            }
        )

    control_elapsed_seconds = (
        time.perf_counter() - control_start
    )
    total_elapsed_seconds = (
        time.perf_counter() - total_start
    )

    success = bool(done)
    suffix = "success" if success else "failure"

    if replay_images:
        imageio.mimwrite(
            episode_directory / f"rollout_{suffix}.mp4",
            replay_images,
            fps=video_fps,
        )

    with (episode_directory / "steps.jsonl").open(
        "w",
        encoding="utf-8",
    ) as file:
        for record in step_records:
            file.write(json.dumps(record) + "\n")

    result = EpisodeResult(
        condition_id=condition_id,
        task_id=task_id,
        task_description=task_description,
        trial_index=trial_index,
        initial_state_index=initial_state_index,
        policy_replan_count=next_replan_index,
        delta_x=float(delta_x),
        delta_y=float(delta_y),
        offset_distance=float(
            np.hypot(delta_x, delta_y)
        ),
        success=success,
        simulation_steps=simulation_steps,
        control_steps=len(step_records),
        control_elapsed_seconds=control_elapsed_seconds,
        total_elapsed_seconds=total_elapsed_seconds,
        object_position_before=(
            perturbation.body_position_before
        ),
        object_position_after_settle=(
            settled_body_position.tolist()
        ),
        output_directory=str(episode_directory),
        arbitration_mode=(
            parsed_arbitration_mode.value
        ),
        policy_episode_seed=policy_episode_seed,
        policy_seed_protocol=(
            SEED_PROTOCOL
            if policy_episode_seed is not None
            else None
        ),
        policy_replans=next_replan_index,
        sampling_protocol_version=(
            sampling_protocol_version
        ),
    )

    with (episode_directory / "summary.json").open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            dataclasses.asdict(result),
            file,
            indent=2,
        )

    return result
