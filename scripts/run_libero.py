#!/usr/bin/env python3
"""Run the SAPS LIBERO cream-cheese task with autonomous OpenPI."""

from __future__ import annotations

import dataclasses
import json
import logging
from pathlib import Path
from typing import Any

import tyro

from saps.environments.libero_env import create_libero_task
from saps.evaluation.runner import EpisodeResult
from saps.evaluation.runner import run_episode
from saps.policies.openpi_client import OpenPiLiberoPolicy


@dataclasses.dataclass
class Args:
    # OpenPI server
    host: str = "0.0.0.0"
    port: int = 8000

    # LIBERO task
    task_suite_name: str = "libero_object"
    task_id: int = 1
    initial_state_index: int = 0
    num_trials: int = 1

    # Perturbation
    condition_id: str = "nominal"
    object_joint_name: str = "cream_cheese_1_joint0"
    object_body_name: str = "cream_cheese_1_main"
    delta_x: float = 0.0
    delta_y: float = 0.0

    # Upstream-compatible settings
    seed: int = 7
    resolution: int = 256
    resize_size: int = 224
    replan_steps: int = 5
    num_steps_wait: int = 10
    max_steps: int = 280
    video_fps: int = 10

    # Output
    output_dir: str = "outputs/phase1_conditions"


def main(args: Args) -> None:
    output_root = Path(args.output_dir)
    output_root.mkdir(parents=True, exist_ok=True)

    env: Any | None = None
    results: list[EpisodeResult] = []

    try:
        env, task_description, initial_states = create_libero_task(
            task_suite_name=args.task_suite_name,
            task_id=args.task_id,
            resolution=args.resolution,
            seed=args.seed,
        )

        logging.info("Task suite: %s", args.task_suite_name)
        logging.info("Task ID: %d", args.task_id)
        logging.info("Task: %s", task_description)
        logging.info(
            "Condition: %s, dx=%.3f, dy=%.3f",
            args.condition_id,
            args.delta_x,
            args.delta_y,
        )

        if "cream cheese" not in task_description.lower():
            raise ValueError(
                "Selected task is not the expected cream-cheese task: "
                f"{task_description!r}"
            )

        final_state_index = (
            args.initial_state_index + args.num_trials - 1
        )

        if final_state_index >= len(initial_states):
            raise ValueError(
                f"Requested initial states "
                f"{args.initial_state_index}..{final_state_index}, but only "
                f"{len(initial_states)} states are available."
            )

        policy = OpenPiLiberoPolicy(
            host=args.host,
            port=args.port,
            resize_size=args.resize_size,
        )

        for trial_index in range(args.num_trials):
            state_index = (
                args.initial_state_index + trial_index
            )

            logging.info(
                "Starting trial %d with initial state %d",
                trial_index,
                state_index,
            )

            result = run_episode(
                env=env,
                policy=policy,
                condition_id=args.condition_id,
                task_id=args.task_id,
                task_description=task_description,
                initial_state=initial_states[state_index],
                initial_state_index=state_index,
                trial_index=trial_index,
                output_root=output_root,
                object_joint_name=args.object_joint_name,
                object_body_name=args.object_body_name,
                delta_x=args.delta_x,
                delta_y=args.delta_y,
                replan_steps=args.replan_steps,
                num_steps_wait=args.num_steps_wait,
                max_steps=args.max_steps,
                video_fps=args.video_fps,
            )

            results.append(result)

            logging.info(
                "Trial %d: success=%s, control_steps=%d, "
                "control_time=%.2fs",
                trial_index,
                result.success,
                result.control_steps,
                result.control_elapsed_seconds,
            )

    finally:
        if env is not None:
            close = getattr(env, "close", None)
            if callable(close):
                close()

    successes = sum(result.success for result in results)

    run_summary = {
        "arguments": dataclasses.asdict(args),
        "episodes": len(results),
        "successes": successes,
        "success_rate": (
            successes / len(results) if results else 0.0
        ),
        "results": [
            dataclasses.asdict(result)
            for result in results
        ],
    }

    condition_directory = (
        output_root / args.condition_id
    )
    condition_directory.mkdir(parents=True, exist_ok=True)

    with (condition_directory / "run_summary.json").open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(run_summary, file, indent=2)

    logging.info(
        "Completed %d episode(s): %d success(es), success rate %.1f%%",
        len(results),
        successes,
        100.0 * run_summary["success_rate"],
    )


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )
    main(tyro.cli(Args))
