#!/usr/bin/env python3
"""Verify deterministic OpenPI action sampling."""

from __future__ import annotations

import dataclasses
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
import tyro

from saps.environments.libero_env import create_libero_task
from saps.policies.openpi_client import OpenPiLiberoPolicy
from saps.policies.seeding import make_policy_episode_seed


DUMMY_ACTION = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    -1.0,
]


@dataclasses.dataclass
class Args:
    base_seed: int = 20260724
    condition_id: str = "nominal"
    trial_index: int = 0

    task_suite_name: str = "libero_object"
    task_id: int = 1
    initial_state_index: int = 0
    environment_seed: int = 7
    settle_steps: int = 10

    host: str = "0.0.0.0"
    port: int = 8000
    output_dir: str = "outputs/seeded_policy_probe"


def action_hash(actions: np.ndarray) -> str:
    array = np.ascontiguousarray(
        actions,
        dtype=np.float32,
    )
    return hashlib.sha256(
        array.tobytes()
    ).hexdigest()


def main(args: Args) -> None:
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    env: Any | None = None

    try:
        env, task_description, initial_states = create_libero_task(
            task_suite_name=args.task_suite_name,
            task_id=args.task_id,
            resolution=256,
            seed=args.environment_seed,
        )

        env.reset()
        obs = env.set_init_state(
            initial_states[args.initial_state_index]
        )

        for _ in range(args.settle_steps):
            obs, _, _, _ = env.step(DUMMY_ACTION)

        policy = OpenPiLiberoPolicy(
            host=args.host,
            port=args.port,
        )

        policy_input, _ = policy.prepare_observation(
            obs,
            task_description,
        )

        seed_a = make_policy_episode_seed(
            base_seed=args.base_seed,
            condition_id=args.condition_id,
            trial_index=args.trial_index,
            task_id=args.task_id,
            initial_state_index=args.initial_state_index,
        )

        seed_b = make_policy_episode_seed(
            base_seed=args.base_seed,
            condition_id=args.condition_id,
            trial_index=args.trial_index + 1,
            task_id=args.task_id,
            initial_state_index=args.initial_state_index,
        )

        actions_a1 = policy.infer(
            policy_input,
            policy_episode_seed=seed_a,
            replan_index=0,
        )

        metadata_a1 = policy.last_sampling_metadata

        actions_a2 = policy.infer(
            policy_input,
            policy_episode_seed=seed_a,
            replan_index=0,
        )

        actions_b = policy.infer(
            policy_input,
            policy_episode_seed=seed_b,
            replan_index=0,
        )

        actions_a_next = policy.infer(
            policy_input,
            policy_episode_seed=seed_a,
            replan_index=1,
        )

        same_seed_identical = np.array_equal(
            actions_a1,
            actions_a2,
        )
        different_seed_differs = not np.array_equal(
            actions_a1,
            actions_b,
        )
        different_replan_differs = not np.array_equal(
            actions_a1,
            actions_a_next,
        )

        np.savez(
            output_dir / "actions.npz",
            actions_a1=actions_a1,
            actions_a2=actions_a2,
            actions_b=actions_b,
            actions_a_next=actions_a_next,
        )

        report = {
            "base_seed": args.base_seed,
            "condition_id": args.condition_id,
            "trial_index": args.trial_index,
            "policy_episode_seed_a": seed_a,
            "policy_episode_seed_b": seed_b,
            "same_seed_identical": (
                same_seed_identical
            ),
            "different_seed_differs": (
                different_seed_differs
            ),
            "different_replan_differs": (
                different_replan_differs
            ),
            "action_hash_a1": action_hash(actions_a1),
            "action_hash_a2": action_hash(actions_a2),
            "action_hash_b": action_hash(actions_b),
            "action_hash_a_next": action_hash(
                actions_a_next
            ),
            "sampling_metadata_a1": metadata_a1,
        }

        (output_dir / "report.json").write_text(
            json.dumps(report, indent=2),
            encoding="utf-8",
        )

        print(json.dumps(report, indent=2))

        if not same_seed_identical:
            raise RuntimeError(
                "Same observation, seed, and replan index "
                "did not reproduce identical actions."
            )

        if not different_seed_differs:
            raise RuntimeError(
                "Different episode seeds produced identical actions."
            )

        if not different_replan_differs:
            raise RuntimeError(
                "Different replan indices produced identical actions."
            )

        print("\nDeterministic sampling checks passed.")

    finally:
        if env is not None:
            close = getattr(env, "close", None)

            if callable(close):
                close()


if __name__ == "__main__":
    main(tyro.cli(Args))
