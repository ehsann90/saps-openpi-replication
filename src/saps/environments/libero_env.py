"""Utilities for constructing a single LIBERO evaluation task."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from libero.libero import benchmark
from libero.libero import get_libero_path
from libero.libero.envs import OffScreenRenderEnv


def create_libero_task(
    *,
    task_suite_name: str,
    task_id: int,
    resolution: int,
    seed: int,
    horizon: int | None = None,
) -> tuple[Any, str, Any]:
    """Create one LIBERO task and return its environment and initial states."""

    benchmark_dict = benchmark.get_benchmark_dict()

    if task_suite_name not in benchmark_dict:
        available = ", ".join(sorted(benchmark_dict))
        raise ValueError(
            f"Unknown LIBERO suite {task_suite_name!r}. "
            f"Available suites: {available}"
        )

    task_suite = benchmark_dict[task_suite_name]()

    if not 0 <= task_id < task_suite.n_tasks:
        raise ValueError(
            f"task_id={task_id} is outside the valid range "
            f"[0, {task_suite.n_tasks - 1}] for {task_suite_name}."
        )

    task = task_suite.get_task(task_id)
    initial_states = task_suite.get_task_init_states(task_id)

    bddl_path = (
        Path(get_libero_path("bddl_files"))
        / task.problem_folder
        / task.bddl_file
    )

    env_kwargs: dict[str, Any] = {
        "bddl_file_name": bddl_path,
        "camera_heights": resolution,
        "camera_widths": resolution,
    }

    if horizon is not None:
        if horizon <= 0:
            raise ValueError(
                "horizon must be positive when provided."
            )

        env_kwargs["horizon"] = int(horizon)

    env = OffScreenRenderEnv(**env_kwargs)

    # LIBERO reports that the seed can affect object positions, even when a
    # saved initial state is subsequently restored.
    env.seed(seed)

    return env, str(task.language), initial_states
