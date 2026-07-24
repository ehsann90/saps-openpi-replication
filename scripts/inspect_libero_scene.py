#!/usr/bin/env python3
"""Inspect MuJoCo bodies and joints for one LIBERO task."""

from __future__ import annotations

import dataclasses
import json
import logging
from pathlib import Path
from typing import Any

import imageio.v2 as imageio
import numpy as np
import tyro

from saps.environments.libero_env import create_libero_task


@dataclasses.dataclass
class Args:
    task_suite_name: str = "libero_object"
    task_id: int = 1
    initial_state_index: int = 0
    seed: int = 7
    resolution: int = 256
    output_dir: str = "outputs/scene_inspection"


def get_sim(env: Any) -> Any:
    """Return the MuJoCo simulation object through common wrapper layouts."""

    sim = getattr(env, "sim", None)
    if sim is not None:
        return sim

    inner_env = getattr(env, "env", None)
    if inner_env is not None:
        sim = getattr(inner_env, "sim", None)
        if sim is not None:
            return sim

    raise RuntimeError("Could not locate the MuJoCo simulation object.")


def matches_target(name: str) -> bool:
    lowered = name.lower()
    return "cream" in lowered or "cheese" in lowered


def main(args: Args) -> None:
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    env = None

    try:
        env, task_description, initial_states = create_libero_task(
            task_suite_name=args.task_suite_name,
            task_id=args.task_id,
            resolution=args.resolution,
            seed=args.seed,
        )

        if not 0 <= args.initial_state_index < len(initial_states):
            raise ValueError(
                f"Invalid initial-state index {args.initial_state_index}; "
                f"{len(initial_states)} states are available."
            )

        env.reset()
        obs = env.set_init_state(
            initial_states[args.initial_state_index]
        )

        sim = get_sim(env)

        joint_names = [
            name for name in sim.model.joint_names
            if name is not None
        ]
        body_names = [
            name for name in sim.model.body_names
            if name is not None
        ]

        candidate_joints = [
            name for name in joint_names
            if matches_target(name)
        ]
        candidate_bodies = [
            name for name in body_names
            if matches_target(name)
        ]

        report: dict[str, Any] = {
            "task_description": task_description,
            "task_id": args.task_id,
            "initial_state_index": args.initial_state_index,
            "candidate_joints": [],
            "candidate_bodies": [],
        }

        print(f"\nTask: {task_description}")

        print("\nCandidate cream-cheese joints:")
        if not candidate_joints:
            print("  None found")
        else:
            for joint_name in candidate_joints:
                joint_id = sim.model.joint_name2id(joint_name)
                joint_type = int(sim.model.jnt_type[joint_id])
                qpos = np.asarray(
                    sim.data.get_joint_qpos(joint_name)
                ).copy()

                entry = {
                    "name": joint_name,
                    "joint_id": int(joint_id),
                    "joint_type": joint_type,
                    "qpos_shape": list(qpos.shape),
                    "qpos": qpos.tolist(),
                }
                report["candidate_joints"].append(entry)

                print(f"  Name: {joint_name}")
                print(f"  Joint ID: {joint_id}")
                print(f"  MuJoCo joint type: {joint_type}")
                print(f"  qpos shape: {qpos.shape}")
                print(f"  qpos: {qpos}")

        print("\nCandidate cream-cheese bodies:")
        if not candidate_bodies:
            print("  None found")
        else:
            for body_name in candidate_bodies:
                body_id = sim.model.body_name2id(body_name)
                position = np.asarray(
                    sim.data.get_body_xpos(body_name)
                ).copy()

                entry = {
                    "name": body_name,
                    "body_id": int(body_id),
                    "world_position": position.tolist(),
                }
                report["candidate_bodies"].append(entry)

                print(f"  Name: {body_name}")
                print(f"  Body ID: {body_id}")
                print(f"  World position: {position}")

        if not candidate_joints:
            print("\nAll joint names:")
            for name in joint_names:
                print(f"  {name}")

        if not candidate_bodies:
            print("\nAll body names:")
            for name in body_names:
                print(f"  {name}")

        report_path = output_dir / "scene_report.json"
        with report_path.open("w", encoding="utf-8") as file:
            json.dump(report, file, indent=2)

        # LIBERO camera images are rotated 180 degrees during OpenPI
        # preprocessing. Save the same human-readable orientation here.
        image = np.ascontiguousarray(
            obs["agentview_image"][::-1, ::-1]
        )
        imageio.imwrite(
            output_dir / "nominal_initial_scene.png",
            image,
        )

        print(f"\nReport saved to: {report_path}")
        print(
            "Initial scene saved to: "
            f"{output_dir / 'nominal_initial_scene.png'}"
        )

    finally:
        if env is not None:
            close = getattr(env, "close", None)
            if callable(close):
                close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main(tyro.cli(Args))
