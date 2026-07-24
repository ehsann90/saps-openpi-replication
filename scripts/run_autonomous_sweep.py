#!/usr/bin/env python3
"""Run a resumable autonomous π0.5 cream-cheese perturbation sweep."""

from __future__ import annotations

import dataclasses
import json
import logging
import math
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
    resume: bool = True

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
        raise FileNotFoundError(
            f"Configuration file does not exist: {path}"
        )

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
            f"Configuration is missing fields: {sorted(missing)}"
        )

    offsets = config["offsets"]

    if not isinstance(offsets, list) or not offsets:
        raise ValueError(
            "Configuration must contain at least one offset."
        )

    condition_ids = [str(offset["id"]) for offset in offsets]

    if len(condition_ids) != len(set(condition_ids)):
        raise ValueError(
            "Condition IDs in the configuration must be unique."
        )

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

    offset_by_id = {
        str(offset["id"]): offset
        for offset in offsets
    }

    unknown = [
        condition_id
        for condition_id in selected_ids
        if condition_id not in offset_by_id
    ]

    if unknown:
        raise ValueError(
            f"Unknown conditions: {unknown}. "
            f"Available: {sorted(offset_by_id)}"
        )

    return [
        offset_by_id[condition_id]
        for condition_id in selected_ids
    ]


def cyclic_condition_order(
    conditions: list[dict[str, Any]],
    trial_index: int,
) -> list[dict[str, Any]]:
    """Rotate the starting condition for each round.

    With 10 conditions and 20 trials, each condition occupies every
    execution-order position exactly twice.
    """

    if not conditions:
        return []

    shift = trial_index % len(conditions)

    return (
        conditions[shift:]
        + conditions[:shift]
    )


def episode_summary_path(
    *,
    output_root: Path,
    condition_id: str,
    task_id: int,
    initial_state_index: int,
    trial_index: int,
) -> Path:
    return (
        output_root
        / condition_id
        / f"task_{task_id:02d}"
        / f"init_{initial_state_index:03d}"
        / f"trial_{trial_index:03d}"
        / "summary.json"
    )


def load_completed_result(
    path: Path,
    *,
    condition_id: str,
    trial_index: int,
    initial_state_index: int,
    delta_x: float,
    delta_y: float,
) -> EpisodeResult | None:
    """Load a completed compatible episode for resume support."""

    if not path.is_file():
        return None

    try:
        with path.open("r", encoding="utf-8") as file:
            data = json.load(file)

        if data["condition_id"] != condition_id:
            return None

        if int(data["trial_index"]) != trial_index:
            return None

        if int(data["initial_state_index"]) != initial_state_index:
            return None

        if not math.isclose(
            float(data["delta_x"]),
            delta_x,
            abs_tol=1e-12,
        ):
            return None

        if not math.isclose(
            float(data["delta_y"]),
            delta_y,
            abs_tol=1e-12,
        ):
            return None

        result_fields = {
            field.name
            for field in dataclasses.fields(EpisodeResult)
        }

        if not result_fields.issubset(data):
            return None

        return EpisodeResult(
            **{
                field_name: data[field_name]
                for field_name in result_fields
            }
        )

    except (
        OSError,
        KeyError,
        TypeError,
        ValueError,
        json.JSONDecodeError,
    ) as error:
        logging.warning(
            "Could not resume from %s: %s",
            path,
            error,
        )
        return None


def summarize_condition(
    *,
    condition: dict[str, Any],
    results: list[EpisodeResult],
    initial_state_index: int,
    expected_episodes: int,
    control_frequency_hz: float,
) -> dict[str, Any]:
    ordered_results = sorted(
        results,
        key=lambda result: result.trial_index,
    )

    successes = [
        result
        for result in ordered_results
        if result.success
    ]
    failures = [
        result
        for result in ordered_results
        if not result.success
    ]

    all_steps = np.asarray(
        [result.control_steps for result in ordered_results],
        dtype=np.float64,
    )
    successful_steps = np.asarray(
        [result.control_steps for result in successes],
        dtype=np.float64,
    )

    all_durations = (
        all_steps / control_frequency_hz
        if len(all_steps)
        else np.asarray([], dtype=np.float64)
    )
    successful_durations = (
        successful_steps / control_frequency_hz
        if len(successful_steps)
        else np.asarray([], dtype=np.float64)
    )

    def mean_or_none(values: np.ndarray) -> float | None:
        if not len(values):
            return None
        return float(np.mean(values))

    def std_or_none(values: np.ndarray) -> float | None:
        if not len(values):
            return None
        return float(np.std(values))

    delta_x = float(condition["dx"])
    delta_y = float(condition["dy"])

    return {
        "condition_id": str(condition["id"]),
        "delta_x": delta_x,
        "delta_y": delta_y,
        "offset_distance": float(
            np.hypot(delta_x, delta_y)
        ),
        "initial_state_index": initial_state_index,
        "expected_episodes": expected_episodes,
        "completed_episodes": len(ordered_results),
        "successes": len(successes),
        "timeouts": len(failures),
        "success_rate": (
            len(successes) / len(ordered_results)
            if ordered_results
            else None
        ),
        "mean_control_steps_all_episodes": mean_or_none(
            all_steps
        ),
        "std_control_steps_all_episodes": std_or_none(
            all_steps
        ),
        "mean_episode_duration_seconds_all": mean_or_none(
            all_durations
        ),
        "std_episode_duration_seconds_all": std_or_none(
            all_durations
        ),
        "mean_successful_completion_steps": mean_or_none(
            successful_steps
        ),
        "std_successful_completion_steps": std_or_none(
            successful_steps
        ),
        "mean_successful_completion_seconds": mean_or_none(
            successful_durations
        ),
        "std_successful_completion_seconds": std_or_none(
            successful_durations
        ),
        "results": [
            dataclasses.asdict(result)
            for result in ordered_results
        ],
    }


def write_progress(
    *,
    output_root: Path,
    args: Args,
    config: dict[str, Any],
    conditions: list[dict[str, Any]],
    results_by_condition: dict[str, list[EpisodeResult]],
    task_description: str,
) -> None:
    condition_summaries = []

    for condition in conditions:
        condition_id = str(condition["id"])

        summary = summarize_condition(
            condition=condition,
            results=results_by_condition[condition_id],
            initial_state_index=args.initial_state_index,
            expected_episodes=args.num_trials,
            control_frequency_hz=args.control_frequency_hz,
        )
        condition_summaries.append(summary)

        condition_directory = output_root / condition_id
        condition_directory.mkdir(
            parents=True,
            exist_ok=True,
        )

        with (
            condition_directory / "run_summary.json"
        ).open("w", encoding="utf-8") as file:
            json.dump(summary, file, indent=2)

    completed_episodes = sum(
        len(results)
        for results in results_by_condition.values()
    )
    expected_episodes = (
        len(conditions) * args.num_trials
    )

    sweep_summary = {
        "complete": completed_episodes == expected_episodes,
        "completed_episodes": completed_episodes,
        "expected_episodes": expected_episodes,
        "arguments": dataclasses.asdict(args),
        "config": config,
        "task_description": task_description,
        "conditions": condition_summaries,
    }

    with (output_root / "sweep_summary.json").open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(sweep_summary, file, indent=2)


def write_schedule(
    *,
    output_root: Path,
    conditions: list[dict[str, Any]],
    num_trials: int,
) -> None:
    schedule = []

    for trial_index in range(num_trials):
        ordered_conditions = cyclic_condition_order(
            conditions,
            trial_index,
        )

        schedule.append(
            {
                "trial_index": trial_index,
                "condition_order": [
                    str(condition["id"])
                    for condition in ordered_conditions
                ],
            }
        )

    with (output_root / "schedule.json").open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(schedule, file, indent=2)


def main(args: Args) -> None:
    if args.num_trials <= 0:
        raise ValueError(
            "num_trials must be greater than zero."
        )

    if args.control_frequency_hz <= 0:
        raise ValueError(
            "control_frequency_hz must be greater than zero."
        )

    config = load_config(Path(args.config_path))
    conditions = select_conditions(
        config["offsets"],
        args.condition_ids,
    )

    output_root = Path(args.output_dir)
    output_root.mkdir(parents=True, exist_ok=True)

    write_schedule(
        output_root=output_root,
        conditions=conditions,
        num_trials=args.num_trials,
    )

    results_by_condition: dict[
        str,
        list[EpisodeResult],
    ] = {
        str(condition["id"]): []
        for condition in conditions
    }

    env: Any | None = None
    policy: OpenPiLiberoPolicy | None = None

    try:
        env, task_description, initial_states = create_libero_task(
            task_suite_name=str(config["task_suite_name"]),
            task_id=int(config["task_id"]),
            resolution=args.resolution,
            seed=args.seed,
        )

        if not 0 <= args.initial_state_index < len(initial_states):
            raise ValueError(
                f"Initial-state index "
                f"{args.initial_state_index} is invalid; "
                f"{len(initial_states)} states are available."
            )

        if "cream cheese" not in task_description.lower():
            raise ValueError(
                "Selected task is not the expected "
                f"cream-cheese task: {task_description!r}"
            )

        logging.info("Task: %s", task_description)
        logging.info(
            "Fixed initial state: %d",
            args.initial_state_index,
        )
        logging.info(
            "Conditions: %s",
            ", ".join(
                str(condition["id"])
                for condition in conditions
            ),
        )
        logging.info(
            "Execution schedule: balanced cyclic round-robin"
        )
        logging.info("Resume enabled: %s", args.resume)

        for trial_index in range(args.num_trials):
            ordered_conditions = cyclic_condition_order(
                conditions,
                trial_index,
            )

            logging.info(
                "Starting round %d/%d with order: %s",
                trial_index + 1,
                args.num_trials,
                ", ".join(
                    str(condition["id"])
                    for condition in ordered_conditions
                ),
            )

            for condition in ordered_conditions:
                condition_id = str(condition["id"])
                delta_x = float(condition["dx"])
                delta_y = float(condition["dy"])

                summary_path = episode_summary_path(
                    output_root=output_root,
                    condition_id=condition_id,
                    task_id=int(config["task_id"]),
                    initial_state_index=(
                        args.initial_state_index
                    ),
                    trial_index=trial_index,
                )

                completed_result = None

                if args.resume:
                    completed_result = load_completed_result(
                        summary_path,
                        condition_id=condition_id,
                        trial_index=trial_index,
                        initial_state_index=(
                            args.initial_state_index
                        ),
                        delta_x=delta_x,
                        delta_y=delta_y,
                    )

                if completed_result is not None:
                    results_by_condition[
                        condition_id
                    ].append(completed_result)

                    logging.info(
                        "Skipping completed episode: "
                        "condition=%s, trial=%d, success=%s",
                        condition_id,
                        trial_index,
                        completed_result.success,
                    )

                    write_progress(
                        output_root=output_root,
                        args=args,
                        config=config,
                        conditions=conditions,
                        results_by_condition=(
                            results_by_condition
                        ),
                        task_description=task_description,
                    )
                    continue

                if policy is None:
                    policy = OpenPiLiberoPolicy(
                        host=args.host,
                        port=args.port,
                        resize_size=args.resize_size,
                    )

                logging.info(
                    "Running condition=%s, trial=%d, "
                    "dx=%.3f, dy=%.3f",
                    condition_id,
                    trial_index,
                    delta_x,
                    delta_y,
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
                    initial_state_index=(
                        args.initial_state_index
                    ),
                    trial_index=trial_index,
                    output_root=output_root,
                    object_joint_name=str(
                        config["joint_name"]
                    ),
                    object_body_name=str(
                        config["body_name"]
                    ),
                    delta_x=delta_x,
                    delta_y=delta_y,
                    replan_steps=args.replan_steps,
                    num_steps_wait=args.num_steps_wait,
                    max_steps=args.max_steps,
                    video_fps=args.video_fps,
                )

                results_by_condition[
                    condition_id
                ].append(result)

                logging.info(
                    "Finished condition=%s, trial=%d: "
                    "success=%s, steps=%d",
                    condition_id,
                    trial_index,
                    result.success,
                    result.control_steps,
                )

                write_progress(
                    output_root=output_root,
                    args=args,
                    config=config,
                    conditions=conditions,
                    results_by_condition=(
                        results_by_condition
                    ),
                    task_description=task_description,
                )

    finally:
        if env is not None:
            close = getattr(env, "close", None)

            if callable(close):
                close()

    write_progress(
        output_root=output_root,
        args=args,
        config=config,
        conditions=conditions,
        results_by_condition=results_by_condition,
        task_description=task_description,
    )

    completed = sum(
        len(results)
        for results in results_by_condition.values()
    )

    logging.info(
        "Sweep complete: %d/%d episodes",
        completed,
        len(conditions) * args.num_trials,
    )


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )
    main(tyro.cli(Args))
