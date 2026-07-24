#!/usr/bin/env python3
"""Run an autonomous π0.5 sweep over cream-cheese perturbations."""

from __future__ import annotations

import dataclasses
import json
import logging
from pathlib import Path
from typing import Any

import numpy as np
import tyro

from saps.environments.libero_env import create_libero_task
from saps.evaluation.runner import EpisodeResult
from saps.evaluation.runner import run_episode
from saps.policies.openpi_client import OpenPiLiberoPolicy


@dataclasses.dataclass
class Args:
    # Experiment configuration
    config_path: str = "configs/libero_cream_cheese_offsets.json"
    condition_ids: str = ""
    num_trials: int = 1
    initial_state_index: int = 0

    # OpenPI server
    host: str = "0.0.0.0"
    port: int = 8000

    # Environment and policy settings
    seed: int = 7
    resolution: int = 256
    resize_size: int = 224
    replan_steps: int = 5
    num_steps_wait: int = 10
    max_steps: int = 280
    control_frequency_hz: float = 20.0
    video_fps: int = 10

    # Outputs
    output_dir: str = "outputs/autonomous_sweep"


def load_config(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Configuration file does not exist: {path}")

    with path.open("r", encoding="utf-8") as file:
        config = json.load(file)

    required = {
        "task_suite_name",
        "task_id",
        "joint_name",
        "body_name",
        "offsets",
    }
    missing = required.difference(config)

    if missing:
        raise ValueError(
            f"Configuration is missing required fields: {sorted(missing)}"
        )

    if not isinstance(config["offsets"], list) or not config["offsets"]:
        raise ValueError("Configuration must contain at least one offset.")

    return config


def select_conditions(
    offsets: list[dict[str, Any]],
    requested_ids: str,
) -> list[dict[str, Any]]:
    if not requested_ids.strip():
        return offsets

    selected_ids = [
        item.strip()
        for item in requested_ids.split(",")
        if item.strip()
    ]

    offset_by_id = {str(offset["id"]): offset for offset in offsets}

    unknown = [
        condition_id
        for condition_id in selected_ids
        if condition_id not in offset_by_id
    ]
    if unknown:
        raise ValueError(
            f"Unknown condition IDs: {unknown}. "
            f"Available conditions: {sorted(offset_by_id)}"
        )

    return [offset_by_id[condition_id] for condition_id in selected_ids]


def main(args: Args) -> None:
    if args.num_trials <= 0:
        raise ValueError("num_trials must be greater than zero.")

    if args.control_frequency_hz <= 0:
        raise ValueError("control_frequency_hz must be greater than zero.")

    config = load_config(Path(args.config_path))
    conditions = select_conditions(
        config["offsets"],
        args.condition_ids,
    )

    output_root = Path(args.output_dir)
    output_root.mkdir(parents=True, exist_ok=True)

    env: Any | None = None
    all_condition_summaries: list[dict[str, Any]] = []

    try:
        env, task_description, initial_states = create_libero_task(
            task_suite_name=str(config["task_suite_name"]),
            task_id=int(config["task_id"]),
            resolution=args.resolution,
            seed=args.seed,
        )

        if not 0 <= args.initial_state_index < len(initial_states):
            raise ValueError(
                f"Initial-state index {args.initial_state_index} is invalid; "
                f"{len(initial_states)} states are available."
            )

        if "cream cheese" not in task_description.lower():
            raise ValueError(
                "Selected task is not the expected cream-cheese task: "
                f"{task_description!r}"
            )

        policy = OpenPiLiberoPolicy(
            host=args.host,
            port=args.port,
            resize_size=args.resize_size,
        )

        logging.info("Task: %s", task_description)
        logging.info(
            "Using fixed LIBERO initial state %d for every trial.",
            args.initial_state_index,
        )
        logging.info(
            "Selected conditions: %s",
            ", ".join(str(condition["id"]) for condition in conditions),
        )

        for condition_index, condition in enumerate(conditions):
            condition_id = str(condition["id"])
            delta_x = float(condition["dx"])
            delta_y = float(condition["dy"])
            distance = float(np.hypot(delta_x, delta_y))

            logging.info(
                "Condition %d/%d: %s, dx=%.3f, dy=%.3f, distance=%.3f",
                condition_index + 1,
                len(conditions),
                condition_id,
                delta_x,
                delta_y,
                distance,
            )

            results: list[EpisodeResult] = []

            for trial_index in range(args.num_trials):
                logging.info(
                    "Condition %s: starting trial %d/%d",
                    condition_id,
                    trial_index + 1,
                    args.num_trials,
                )

                result = run_episode(
                    env=env,
                    policy=policy,
                    condition_id=condition_id,
                    task_id=int(config["task_id"]),
                    task_description=task_description,
                    initial_state=initial_states[
                        args.initial_state_index
                    ],
                    initial_state_index=args.initial_state_index,
                    trial_index=trial_index,
                    output_root=output_root,
                    object_joint_name=str(config["joint_name"]),
                    object_body_name=str(config["body_name"]),
                    delta_x=delta_x,
                    delta_y=delta_y,
                    replan_steps=args.replan_steps,
                    num_steps_wait=args.num_steps_wait,
                    max_steps=args.max_steps,
                    video_fps=args.video_fps,
                )
                results.append(result)

                logging.info(
                    "Condition %s, trial %d: success=%s, steps=%d",
                    condition_id,
                    trial_index,
                    result.success,
                    result.control_steps,
                )

            successes = sum(result.success for result in results)
            control_steps = [
                result.control_steps for result in results
            ]

            # Use simulated control time for experiment comparison.
            # Wall-clock time depends on GPU and compilation overhead.
            simulated_times = [
                steps / args.control_frequency_hz
                for steps in control_steps
            ]

            condition_summary = {
                "condition_id": condition_id,
                "delta_x": delta_x,
                "delta_y": delta_y,
                "offset_distance": distance,
                "initial_state_index": args.initial_state_index,
                "episodes": len(results),
                "successes": successes,
                "success_rate": successes / len(results),
                "mean_control_steps": float(np.mean(control_steps)),
                "std_control_steps": float(np.std(control_steps)),
                "mean_simulated_completion_seconds": float(
                    np.mean(simulated_times)
                ),
                "std_simulated_completion_seconds": float(
                    np.std(simulated_times)
                ),
                "results": [
                    dataclasses.asdict(result)
                    for result in results
                ],
            }

            condition_directory = output_root / condition_id
            condition_directory.mkdir(parents=True, exist_ok=True)

            with (condition_directory / "run_summary.json").open(
                "w",
                encoding="utf-8",
            ) as file:
                json.dump(condition_summary, file, indent=2)

            all_condition_summaries.append(condition_summary)

            logging.info(
                "Condition %s complete: %d/%d successful",
                condition_id,
                successes,
                len(results),
            )

    finally:
        if env is not None:
            close = getattr(env, "close", None)
            if callable(close):
                close()

    sweep_summary = {
        "arguments": dataclasses.asdict(args),
        "config": config,
        "task_description": (
            all_condition_summaries[0]["results"][0][
                "task_description"
            ]
            if all_condition_summaries
            else None
        ),
        "conditions": all_condition_summaries,
    }

    with (output_root / "sweep_summary.json").open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(sweep_summary, file, indent=2)

    logging.info(
        "Sweep completed: %d condition(s), %d episode(s)",
        len(all_condition_summaries),
        sum(
            summary["episodes"]
            for summary in all_condition_summaries
        ),
    )


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )
    main(tyro.cli(Args))
