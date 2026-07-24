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

from saps.policies.openpi_client import OpenPiLiberoPolicy


LIBERO_DUMMY_ACTION = np.asarray(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
    dtype=np.float32,
)


@dataclasses.dataclass(frozen=True)
class EpisodeResult:
    task_id: int
    task_description: str
    trial_index: int
    initial_state_index: int
    success: bool
    simulation_steps: int
    control_steps: int
    elapsed_seconds: float
    output_directory: str


def run_episode(
    *,
    env: Any,
    policy: OpenPiLiberoPolicy,
    task_id: int,
    task_description: str,
    initial_state: np.ndarray,
    initial_state_index: int,
    trial_index: int,
    output_root: Path,
    replan_steps: int,
    num_steps_wait: int,
    max_steps: int,
    video_fps: int = 10,
) -> EpisodeResult:
    """Run one autonomous OpenPI episode and write reproducibility logs."""

    if replan_steps <= 0:
        raise ValueError("replan_steps must be greater than zero.")

    episode_directory = (
        output_root
        / f"task_{task_id:02d}"
        / f"init_{initial_state_index:03d}"
        / f"trial_{trial_index:03d}"
    )
    episode_directory.mkdir(parents=True, exist_ok=True)

    env.reset()
    obs = env.set_init_state(initial_state)

    action_plan: collections.deque[np.ndarray] = collections.deque()
    replay_images: list[np.ndarray] = []
    step_records: list[dict[str, Any]] = []

    done = False
    simulation_step = 0
    start_time = time.perf_counter()

    while simulation_step < max_steps + num_steps_wait:
        if simulation_step < num_steps_wait:
            obs, reward, done, info = env.step(
                LIBERO_DUMMY_ACTION.tolist()
            )
            simulation_step += 1
            continue

        policy_input, replay_image = policy.prepare_observation(
            obs,
            task_description,
        )
        replay_images.append(replay_image)

        inference_latency_seconds: float | None = None
        replanned = False

        if not action_plan:
            inference_start = time.perf_counter()
            action_chunk = policy.infer(policy_input)
            inference_latency_seconds = (
                time.perf_counter() - inference_start
            )
            replanned = True

            if len(action_chunk) < replan_steps:
                raise ValueError(
                    f"Policy returned {len(action_chunk)} actions, but "
                    f"replan_steps={replan_steps}."
                )

            action_plan.extend(action_chunk[:replan_steps])

        policy_action = np.asarray(action_plan.popleft(), dtype=np.float32)

        obs, reward, done, info = env.step(policy_action.tolist())
        simulation_step += 1

        step_records.append(
            {
                "simulation_step": simulation_step,
                "control_step": len(step_records),
                "replanned": replanned,
                "inference_latency_seconds": inference_latency_seconds,
                "policy_action": policy_action.tolist(),
                "reward": float(reward),
                "done": bool(done),
                "eef_position": np.asarray(
                    obs["robot0_eef_pos"],
                    dtype=np.float32,
                ).tolist(),
            }
        )

        if done:
            break

    elapsed_seconds = time.perf_counter() - start_time
    success = bool(done)

    suffix = "success" if success else "failure"
    video_path = episode_directory / f"rollout_{suffix}.mp4"

    if replay_images:
        imageio.mimwrite(
            video_path,
            [np.asarray(frame) for frame in replay_images],
            fps=video_fps,
        )

    step_log_path = episode_directory / "steps.jsonl"
    with step_log_path.open("w", encoding="utf-8") as file:
        for record in step_records:
            file.write(json.dumps(record) + "\n")

    result = EpisodeResult(
        task_id=task_id,
        task_description=task_description,
        trial_index=trial_index,
        initial_state_index=initial_state_index,
        success=success,
        simulation_steps=simulation_step,
        control_steps=len(step_records),
        elapsed_seconds=elapsed_seconds,
        output_directory=str(episode_directory),
    )

    with (episode_directory / "summary.json").open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(dataclasses.asdict(result), file, indent=2)

    return result
