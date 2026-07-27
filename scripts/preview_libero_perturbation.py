#!/usr/bin/env python3
"""Preview a controlled cream-cheese displacement without running OpenPI."""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path
from typing import Any

import imageio.v2 as imageio
import numpy as np
import tyro

from saps.environments.libero_env import create_libero_task
from saps.environments.perturbations import apply_planar_object_offset
from saps.environments.perturbations import get_object_pose


DUMMY_ACTION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]


@dataclasses.dataclass
class Args:
    task_suite_name: str = "libero_object"
    task_id: int = 1
    initial_state_index: int = 0
    seed: int = 7
    resolution: int = 256

    joint_name: str = "cream_cheese_1_joint0"
    body_name: str = "cream_cheese_1_main"

    delta_x: float = 0.0
    delta_y: float = 0.0
    settle_steps: int = 10

    label: str = "preview"
    output_root: str = "outputs/perturbation_preview"


def save_agent_image(path: Path, obs: dict[str, Any]) -> None:
    """Save the agent-view image in OpenPI's displayed orientation."""

    image = np.ascontiguousarray(
        obs["agentview_image"][::-1, ::-1]
    )
    imageio.imwrite(path, image)


def main(args: Args) -> None:
    output_dir = Path(args.output_root) / args.label
    output_dir.mkdir(parents=True, exist_ok=True)

    env: Any | None = None

    try:
        env, task_description, initial_states = create_libero_task(
            task_suite_name=args.task_suite_name,
            task_id=args.task_id,
            resolution=args.resolution,
            seed=args.seed,
        )

        if not 0 <= args.initial_state_index < len(initial_states):
            raise ValueError(
                f"Initial-state index {args.initial_state_index} is invalid; "
                f"{len(initial_states)} states are available."
            )

        env.reset()
        nominal_obs = env.set_init_state(
            initial_states[args.initial_state_index]
        )

        save_agent_image(
            output_dir / "01_nominal.png",
            nominal_obs,
        )

        perturbed_obs, perturbation = apply_planar_object_offset(
            env,
            joint_name=args.joint_name,
            body_name=args.body_name,
            delta_x=args.delta_x,
            delta_y=args.delta_y,
        )

        save_agent_image(
            output_dir / "02_perturbed_before_settle.png",
            perturbed_obs,
        )

        obs = perturbed_obs
        done = False

        for _ in range(args.settle_steps):
            obs, reward, done, info = env.step(DUMMY_ACTION)

        save_agent_image(
            output_dir / "03_perturbed_after_settle.png",
            obs,
        )

        settled_qpos, settled_body_position = get_object_pose(
            env,
            joint_name=args.joint_name,
            body_name=args.body_name,
        )

        report = {
            "task_description": task_description,
            "task_suite_name": args.task_suite_name,
            "task_id": args.task_id,
            "initial_state_index": args.initial_state_index,
            "seed": args.seed,
            "settle_steps": args.settle_steps,
            "requested_offset": {
                "delta_x": args.delta_x,
                "delta_y": args.delta_y,
                "distance": float(
                    np.hypot(args.delta_x, args.delta_y)
                ),
            },
            "perturbation": dataclasses.asdict(perturbation),
            "settled_joint_qpos": settled_qpos.tolist(),
            "settled_body_position": settled_body_position.tolist(),
            "success_after_settling": bool(done),
        }

        with (output_dir / "report.json").open(
            "w",
            encoding="utf-8",
        ) as file:
            json.dump(report, file, indent=2)

        print(json.dumps(report, indent=2))
        print(f"\nOutputs saved under {output_dir}")

    finally:
        if env is not None:
            close = getattr(env, "close", None)
            if callable(close):
                close()


if __name__ == "__main__":
    main(tyro.cli(Args))
