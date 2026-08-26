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
from saps.evaluation.experiment_session import json_file_identity
from saps.evaluation.gate2_v2_protocol import (
    build_gate2_v2_autonomous_schedule,
)
from saps.evaluation.gate2_v2_protocol import GATE2_V2_CONFIG_SHA256
from saps.evaluation.gate2_v2_protocol import (
    GATE2_V2_AUTONOMOUS_EXPERIMENT_ID,
)
from saps.evaluation.gate2_v2_protocol import (
    GATE2_V2_AUTONOMOUS_PROTOCOL_PATH,
)
from saps.evaluation.gate2_v2_protocol import GATE2_V2_AUTONOMOUS_OUTPUT_ROOT
from saps.evaluation.gate2_v2_protocol import (
    load_gate2_v2_autonomous_protocol,
)
from saps.evaluation.runner import EpisodeResult
from saps.evaluation.runner import run_episode
from saps.policies.openpi_client import OpenPiLiberoPolicy
from saps.policies.seeding import make_policy_episode_seed
from saps.policies.seeding import SEED_PROTOCOL


@dataclasses.dataclass
class Args:
    # Experiment configuration
    config_path: str = (
        "configs/libero_cream_cheese_offsets.json"
    )
    condition_ids: str = ""
    num_trials: int = 1
    initial_state_index: int = 0
    resume: bool = True
    required_protocol_id: str = ""
    protocol_path: str = GATE2_V2_AUTONOMOUS_PROTOCOL_PATH
    repository_commit: str = ""

    # Matched policy-sampling configuration
    deterministic_policy: bool = True
    policy_base_seed: int = 20260724

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


def write_json_atomic(
    path: Path,
    payload: dict[str, Any] | list[Any],
) -> None:
    """Write JSON without exposing a partially written destination file."""

    path.parent.mkdir(parents=True, exist_ok=True)

    temporary_path = path.with_name(
        f".{path.name}.tmp"
    )

    with temporary_path.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(payload, file, indent=2)
        file.write("\n")

    temporary_path.replace(path)


def freeze_json(path: Path, payload: dict[str, Any]) -> None:
    """Write immutable protocol metadata or verify the stored copy."""

    if path.exists():
        with path.open("r", encoding="utf-8") as file:
            stored = json.load(file)
        if stored != payload:
            raise ValueError(
                f"Frozen autonomous provenance differs at {path}. "
                "Use a new output directory."
            )
        return
    write_json_atomic(path, payload)


def validate_gate2_v2_args(
    *,
    args: Args,
    protocol: dict[str, Any],
) -> None:
    """Require every collection argument fixed by Gate-2 v2."""

    expected = {
        "config_path": protocol["config_path"],
        "condition_ids": ",".join(protocol["conditions"]),
        "num_trials": len(protocol["trials"]),
        "initial_state_index": protocol["initial_state_index"],
        "resume": True,
        "deterministic_policy": True,
        "policy_base_seed": protocol["policy_base_seed"],
        "seed": protocol["environment_seed"],
        "resolution": protocol["resolution"],
        "resize_size": protocol["resize_size"],
        "replan_steps": protocol["replan_steps"],
        "num_steps_wait": protocol["settle_steps"],
        "max_steps": protocol["max_steps"],
        "control_frequency_hz": protocol["control_frequency_hz"],
        "video_fps": protocol["video_fps"],
        "output_dir": protocol["output_root"],
    }
    actual = dataclasses.asdict(args)
    drift = {
        name: (actual[name], value)
        for name, value in expected.items()
        if actual[name] != value
    }
    if drift:
        raise ValueError(f"Gate-2 v2 autonomous arguments drifted: {drift}")
    commit = args.repository_commit.strip().lower()
    if len(commit) != 40 or any(
        character not in "0123456789abcdef" for character in commit
    ):
        raise ValueError(
            "Gate-2 v2 repository_commit must be a full Git hash."
        )


def freeze_gate2_v2_provenance(
    *,
    args: Args,
    protocol: dict[str, Any],
    output_root: Path,
) -> None:
    """Freeze protocol, config, schedule, and repository identity."""

    protocol_identity = json_file_identity(Path(args.protocol_path))
    config_identity = json_file_identity(Path(args.config_path))
    if config_identity["sha256"] != GATE2_V2_CONFIG_SHA256:
        raise ValueError("Gate-2 v2 perturbation configuration drifted.")
    freeze_json(output_root / "protocol.json", protocol)
    freeze_json(output_root / "perturbation_config.json", config_identity)
    freeze_json(
        output_root / "repository_provenance.json",
        {
            "repository_commit": args.repository_commit.strip().lower(),
            "protocol_path": protocol_identity["path"],
            "protocol_sha256": protocol_identity["sha256"],
        },
    )
    schedule = build_gate2_v2_autonomous_schedule(protocol)
    freeze_json(output_root / "schedule.json", schedule)
    expected_summaries = {
        episode_summary_path(
            output_root=output_root,
            condition_id=str(episode["condition_id"]),
            task_id=int(protocol["task_id"]),
            initial_state_index=int(episode["initial_state_index"]),
            trial_index=int(episode["trial_index"]),
        )
        for episode in schedule["episodes"]
    }
    unexpected_summaries = set(output_root.rglob("summary.json")).difference(
        expected_summaries
    )
    if unexpected_summaries:
        raise ValueError(
            "Gate-2 autonomous output contains summaries outside the "
            "frozen 20 cells: "
            + ", ".join(str(path) for path in sorted(unexpected_summaries))
        )


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
            f"Configuration is missing fields: "
            f"{sorted(missing)}"
        )

    offsets = config["offsets"]

    if not isinstance(offsets, list) or not offsets:
        raise ValueError(
            "Configuration must contain at least one offset."
        )

    condition_ids = [
        str(offset["id"])
        for offset in offsets
    ]

    if len(condition_ids) != len(set(condition_ids)):
        raise ValueError(
            "Condition IDs in the configuration "
            "must be unique."
        )

    for offset in offsets:
        for field in ("id", "dx", "dy"):
            if field not in offset:
                raise ValueError(
                    f"Offset is missing field {field!r}: "
                    f"{offset}"
                )

    return config


def select_conditions(
    offsets: list[dict[str, Any]],
    requested_ids: str,
) -> list[dict[str, Any]]:
    if not requested_ids.strip():
        return list(offsets)

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
    """Rotate the starting condition for each trial round."""

    if not conditions:
        return []

    shift = trial_index % len(conditions)

    return (
        conditions[shift:]
        + conditions[:shift]
    )


def episode_policy_seed(
    *,
    args: Args,
    condition_id: str,
    trial_index: int,
    task_id: int,
) -> int | None:
    """Return the matched seed for one condition/trial unit."""

    if not args.deterministic_policy:
        return None

    return make_policy_episode_seed(
        base_seed=args.policy_base_seed,
        condition_id=condition_id,
        trial_index=trial_index,
        task_id=task_id,
        initial_state_index=args.initial_state_index,
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
    task_id: int,
    trial_index: int,
    initial_state_index: int,
    delta_x: float,
    delta_y: float,
    expected_policy_episode_seed: int | None,
) -> EpisodeResult | None:
    """Load a compatible completed episode for resume support."""

    if not path.is_file():
        return None

    try:
        with path.open("r", encoding="utf-8") as file:
            data = json.load(file)

        if str(data["condition_id"]) != condition_id:
            return None

        if int(data["task_id"]) != task_id:
            return None

        if int(data["trial_index"]) != trial_index:
            return None

        if (
            int(data["initial_state_index"])
            != initial_state_index
        ):
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

        stored_policy_seed = data.get(
            "policy_episode_seed"
        )

        if expected_policy_episode_seed is None:
            if stored_policy_seed is not None:
                return None
        else:
            if stored_policy_seed is None:
                return None

            if (
                int(stored_policy_seed)
                != expected_policy_episode_seed
            ):
                return None

            if (
                data.get("policy_seed_protocol")
                != SEED_PROTOCOL
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


def validate_gate2_v2_completed_result(
    *,
    result: EpisodeResult,
    summary_path: Path,
    protocol: dict[str, Any],
) -> None:
    """Require a complete, internally consistent result before resuming."""

    if result.arbitration_mode != "autonomous":
        raise ValueError("Gate-2 autonomous result has another mode.")
    if not 1 <= result.control_steps <= int(protocol["max_steps"]):
        raise ValueError("Gate-2 autonomous result has an invalid step count.")
    if not result.success and result.control_steps != int(
        protocol["max_steps"]
    ):
        raise ValueError(
            "Gate-2 autonomous failure is not a complete 280-step timeout."
        )
    if result.simulation_steps != (
        int(protocol["settle_steps"]) + result.control_steps
    ):
        raise ValueError(
            "Gate-2 autonomous simulation-step count is inconsistent."
        )
    expected_replans = math.ceil(
        result.control_steps / int(protocol["replan_steps"])
    )
    if result.policy_replan_count != expected_replans:
        raise ValueError("Gate-2 autonomous replan count is inconsistent.")
    steps_path = summary_path.with_name("steps.jsonl")
    if not steps_path.is_file():
        raise ValueError("Gate-2 autonomous steps.jsonl is missing.")
    with steps_path.open("r", encoding="utf-8") as file:
        steps = [json.loads(line) for line in file if line.strip()]
    if len(steps) != result.control_steps:
        raise ValueError("Gate-2 autonomous logged step count is inconsistent.")
    if any(
        int(step.get("policy_episode_seed", -1))
        != result.policy_episode_seed
        for step in steps
    ):
        raise ValueError("Gate-2 autonomous step seed is inconsistent.")


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
        [
            result.control_steps
            for result in ordered_results
        ],
        dtype=np.float64,
    )

    successful_steps = np.asarray(
        [
            result.control_steps
            for result in successes
        ],
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

    def mean_or_none(
        values: np.ndarray,
    ) -> float | None:
        if not len(values):
            return None

        return float(np.mean(values))

    def std_or_none(
        values: np.ndarray,
    ) -> float | None:
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
        "mean_control_steps_all_episodes": (
            mean_or_none(all_steps)
        ),
        "std_control_steps_all_episodes": (
            std_or_none(all_steps)
        ),
        "mean_episode_duration_seconds_all": (
            mean_or_none(all_durations)
        ),
        "std_episode_duration_seconds_all": (
            std_or_none(all_durations)
        ),
        "mean_successful_completion_steps": (
            mean_or_none(successful_steps)
        ),
        "std_successful_completion_steps": (
            std_or_none(successful_steps)
        ),
        "mean_successful_completion_seconds": (
            mean_or_none(successful_durations)
        ),
        "std_successful_completion_seconds": (
            std_or_none(successful_durations)
        ),
        "policy_episode_seeds": [
            result.policy_episode_seed
            for result in ordered_results
        ],
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
    results_by_condition: dict[
        str,
        list[EpisodeResult],
    ],
    task_description: str,
) -> None:
    condition_summaries = []

    for condition in conditions:
        condition_id = str(condition["id"])

        summary = summarize_condition(
            condition=condition,
            results=results_by_condition[
                condition_id
            ],
            initial_state_index=(
                args.initial_state_index
            ),
            expected_episodes=args.num_trials,
            control_frequency_hz=(
                args.control_frequency_hz
            ),
        )

        condition_summaries.append(summary)

        write_json_atomic(
            output_root
            / condition_id
            / "run_summary.json",
            summary,
        )

    completed_episodes = sum(
        len(results)
        for results in results_by_condition.values()
    )

    expected_episodes = (
        len(conditions) * args.num_trials
    )

    sweep_summary = {
        "complete": (
            completed_episodes == expected_episodes
        ),
        "completed_episodes": completed_episodes,
        "expected_episodes": expected_episodes,
        "task_description": task_description,
        "arguments": dataclasses.asdict(args),
        "sampling": {
            "deterministic_policy": (
                args.deterministic_policy
            ),
            "policy_base_seed": (
                args.policy_base_seed
                if args.deterministic_policy
                else None
            ),
            "policy_seed_protocol": (
                SEED_PROTOCOL
                if args.deterministic_policy
                else None
            ),
            "seed_excludes_arbitration_mode": True,
        },
        "config": config,
        "conditions": condition_summaries,
    }

    write_json_atomic(
        output_root / "sweep_summary.json",
        sweep_summary,
    )


def write_schedule(
    *,
    output_root: Path,
    args: Args,
    conditions: list[dict[str, Any]],
    task_id: int,
) -> None:
    schedule = []

    for trial_index in range(args.num_trials):
        ordered_conditions = cyclic_condition_order(
            conditions,
            trial_index,
        )

        scheduled_conditions = []

        for condition in ordered_conditions:
            condition_id = str(condition["id"])

            scheduled_conditions.append(
                {
                    "condition_id": condition_id,
                    "delta_x": float(
                        condition["dx"]
                    ),
                    "delta_y": float(
                        condition["dy"]
                    ),
                    "policy_episode_seed": (
                        episode_policy_seed(
                            args=args,
                            condition_id=condition_id,
                            trial_index=trial_index,
                            task_id=task_id,
                        )
                    ),
                }
            )

        schedule.append(
            {
                "trial_index": trial_index,
                "conditions": scheduled_conditions,
            }
        )

    write_json_atomic(
        output_root / "schedule.json",
        {
            "deterministic_policy": (
                args.deterministic_policy
            ),
            "policy_base_seed": (
                args.policy_base_seed
                if args.deterministic_policy
                else None
            ),
            "policy_seed_protocol": (
                SEED_PROTOCOL
                if args.deterministic_policy
                else None
            ),
            "rounds": schedule,
        },
    )


def main(args: Args) -> None:
    required_protocol_id = args.required_protocol_id.strip()
    protocol: dict[str, Any] | None = None
    if required_protocol_id:
        if required_protocol_id != GATE2_V2_AUTONOMOUS_EXPERIMENT_ID:
            raise ValueError(
                f"Unsupported required_protocol_id {required_protocol_id!r}."
            )
        protocol = load_gate2_v2_autonomous_protocol(Path(args.protocol_path))
        validate_gate2_v2_args(args=args, protocol=protocol)
    elif Path(args.output_dir).as_posix() == GATE2_V2_AUTONOMOUS_OUTPUT_ROOT:
        raise ValueError("Gate-2 v2 autonomous output requires its guard.")

    if args.num_trials <= 0:
        raise ValueError(
            "num_trials must be greater than zero."
        )

    if args.initial_state_index < 0:
        raise ValueError(
            "initial_state_index must be non-negative."
        )

    if args.control_frequency_hz <= 0:
        raise ValueError(
            "control_frequency_hz must be greater than zero."
        )

    if args.replan_steps <= 0:
        raise ValueError(
            "replan_steps must be greater than zero."
        )

    if args.max_steps <= 0:
        raise ValueError(
            "max_steps must be greater than zero."
        )

    if (
        args.deterministic_policy
        and args.policy_base_seed < 0
    ):
        raise ValueError(
            "policy_base_seed must be non-negative."
        )

    config = load_config(
        Path(args.config_path)
    )

    conditions = select_conditions(
        config["offsets"],
        args.condition_ids,
    )

    task_id = int(config["task_id"])
    output_root = Path(args.output_dir)
    output_root.mkdir(
        parents=True,
        exist_ok=True,
    )

    if protocol is not None:
        freeze_gate2_v2_provenance(
            args=args,
            protocol=protocol,
            output_root=output_root,
        )
    else:
        write_schedule(
            output_root=output_root,
            args=args,
            conditions=conditions,
            task_id=task_id,
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
        (
            env,
            task_description,
            initial_states,
        ) = create_libero_task(
            task_suite_name=str(
                config["task_suite_name"]
            ),
            task_id=task_id,
            resolution=args.resolution,
            seed=args.seed,
        )

        if not (
            0
            <= args.initial_state_index
            < len(initial_states)
        ):
            raise ValueError(
                f"Initial-state index "
                f"{args.initial_state_index} is invalid; "
                f"{len(initial_states)} states are available."
            )

        if "cream cheese" not in (
            task_description.lower()
        ):
            raise ValueError(
                "Selected task is not the expected "
                f"cream-cheese task: "
                f"{task_description!r}"
            )

        logging.info(
            "Task: %s",
            task_description,
        )
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
            "Execution schedule: balanced "
            "cyclic round-robin"
        )
        logging.info(
            "Resume enabled: %s",
            args.resume,
        )
        logging.info(
            "Deterministic policy sampling: %s",
            args.deterministic_policy,
        )

        if args.deterministic_policy:
            logging.info(
                "Policy seed protocol: %s",
                SEED_PROTOCOL,
            )
            logging.info(
                "Policy base seed: %d",
                args.policy_base_seed,
            )

        for trial_index in range(
            args.num_trials
        ):
            ordered_conditions = (
                cyclic_condition_order(
                    conditions,
                    trial_index,
                )
            )

            logging.info(
                "Starting round %d/%d with order: %s",
                trial_index + 1,
                args.num_trials,
                ", ".join(
                    str(condition["id"])
                    for condition
                    in ordered_conditions
                ),
            )

            for condition in ordered_conditions:
                condition_id = str(
                    condition["id"]
                )
                delta_x = float(
                    condition["dx"]
                )
                delta_y = float(
                    condition["dy"]
                )

                policy_episode_seed = (
                    episode_policy_seed(
                        args=args,
                        condition_id=condition_id,
                        trial_index=trial_index,
                        task_id=task_id,
                    )
                )

                summary_path = (
                    episode_summary_path(
                        output_root=output_root,
                        condition_id=condition_id,
                        task_id=task_id,
                        initial_state_index=(
                            args.initial_state_index
                        ),
                        trial_index=trial_index,
                    )
                )

                completed_result = None

                if args.resume:
                    completed_result = (
                        load_completed_result(
                            summary_path,
                            condition_id=condition_id,
                            task_id=task_id,
                            trial_index=trial_index,
                            initial_state_index=(
                                args.initial_state_index
                            ),
                            delta_x=delta_x,
                            delta_y=delta_y,
                            expected_policy_episode_seed=(
                                policy_episode_seed
                            ),
                        )
                    )

                if (
                    protocol is not None
                    and summary_path.exists()
                    and completed_result is None
                ):
                    raise ValueError(
                        "Existing Gate-2 autonomous summary is incomplete "
                        f"or incompatible and will not be overwritten: "
                        f"{summary_path}"
                    )

                if completed_result is not None:
                    if protocol is not None:
                        validate_gate2_v2_completed_result(
                            result=completed_result,
                            summary_path=summary_path,
                            protocol=protocol,
                        )
                    results_by_condition[
                        condition_id
                    ].append(completed_result)

                    logging.info(
                        "Skipping completed episode: "
                        "condition=%s, trial=%d, "
                        "policy_seed=%s, success=%s",
                        condition_id,
                        trial_index,
                        policy_episode_seed,
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
                        task_description=(
                            task_description
                        ),
                    )
                    continue

                if policy is None:
                    policy = OpenPiLiberoPolicy(
                        host=args.host,
                        port=args.port,
                        resize_size=(
                            args.resize_size
                        ),
                    )
                    if protocol is not None:
                        policy.validate_policy_identity(
                            config_name=protocol["policy_config_name"],
                            checkpoint=protocol["policy_checkpoint"],
                        )

                logging.info(
                    "Running condition=%s, trial=%d, "
                    "dx=%.3f, dy=%.3f, "
                    "policy_seed=%s",
                    condition_id,
                    trial_index,
                    delta_x,
                    delta_y,
                    policy_episode_seed,
                )

                result = run_episode(
                    env=env,
                    policy=policy,
                    condition_id=condition_id,
                    task_id=task_id,
                    task_description=(
                        task_description
                    ),
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
                    replan_steps=(
                        args.replan_steps
                    ),
                    num_steps_wait=(
                        args.num_steps_wait
                    ),
                    max_steps=args.max_steps,
                    policy_episode_seed=(
                        policy_episode_seed
                    ),
                    video_fps=args.video_fps,
                )

                results_by_condition[
                    condition_id
                ].append(result)

                logging.info(
                    "Finished condition=%s, "
                    "trial=%d: success=%s, "
                    "steps=%d, replans=%d",
                    condition_id,
                    trial_index,
                    result.success,
                    result.control_steps,
                    result.policy_replan_count,
                )

                write_progress(
                    output_root=output_root,
                    args=args,
                    config=config,
                    conditions=conditions,
                    results_by_condition=(
                        results_by_condition
                    ),
                    task_description=(
                        task_description
                    ),
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

        completed = sum(
            len(results)
            for results
            in results_by_condition.values()
        )

        logging.info(
            "Sweep complete: %d/%d episodes",
            completed,
            len(conditions) * args.num_trials,
        )

    finally:
        if env is not None:
            close = getattr(
                env,
                "close",
                None,
            )

            if callable(close):
                close()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format=(
            "%(asctime)s | %(levelname)s | "
            "%(message)s"
        ),
    )

    main(tyro.cli(Args))
