#!/usr/bin/env python3
"""Run one deterministic SAPS shared-autonomy LIBERO episode."""

from __future__ import annotations

import dataclasses
import logging
from pathlib import Path
import time
from typing import Any

import imageio.v2 as imageio
import numpy as np
import tyro

from saps.arbitration import ArbitrationMode
from saps.environments.libero_env import create_libero_task
from saps.environments.perturbations import (
    apply_planar_object_offset,
)
from saps.environments.perturbations import get_object_pose
from saps.evaluation.operator_episode import (
    load_config,
)
from saps.evaluation.operator_episode import (
    operator_view_rgb,
)
from saps.evaluation.operator_episode import (
    save_agent_image,
)
from saps.evaluation.operator_episode import (
    select_condition,
)
from saps.evaluation.operator_episode import (
    wait_until_armed,
)
from saps.evaluation.operator_episode import (
    write_json_atomic,
)
from saps.evaluation.shared_episode_loop import (
    run_shared_episode_loop,
)
from saps.human_input.web_operator import (
    BrowserOperatorServer,
)
from saps.policies.async_worker import (
    AsyncPolicyWorker,
)
from saps.policies.openpi_client import (
    OpenPiLiberoPolicy,
)
from saps.policies.seeding import make_policy_episode_seed
from saps.policies.seeding import SEED_PROTOCOL


LIBERO_DUMMY_ACTION = np.asarray(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
    dtype=np.float32,
)


@dataclasses.dataclass
class Args:
    # Experiment
    config_path: str = (
        "configs/libero_cream_cheese_offsets.json"
    )
    condition_id: str = "nominal"
    trial_index: int = 0
    initial_state_index: int = 0
    arbitration_mode: str = "takeover"
    fixed_autonomy_weight: float = 0.5
    cosine_gain: float = 6.0

    # Reproducibility
    environment_seed: int = 7
    policy_base_seed: int = 20260724

    # OpenPI
    policy_host: str = "0.0.0.0"
    policy_port: int = 8000
    resize_size: int = 224
    replan_steps: int = 5

    # LIBERO
    resolution: int = 256
    num_steps_wait: int = 10
    max_steps: int = 1200
    control_frequency_hz: float = 20.0
    video_fps: int = 20

    # Browser operator
    host: str = "0.0.0.0"
    websocket_port: int = 8765
    http_port: int = 8766

    translation_gain: float = 0.14
    rotation_gain: float = 0.18

    fine_translation_gain: float = 0.07
    fine_rotation_gain: float = 0.10
    fast_translation_gain: float = 0.25
    fast_rotation_gain: float = 0.30
    default_speed_mode: str = "fine"

    jpeg_quality: int = 85
    client_timeout_seconds: float = 120.0
    arm_timeout_seconds: float = 300.0

    # Output
    output_dir: str = "outputs/shared_autonomy_smoke"


@dataclasses.dataclass(frozen=True)
class SharedAutonomyEpisodeResult:
    condition_id: str
    arbitration_mode: str
    fixed_autonomy_weight: float | None
    cosine_gain: float | None
    task_id: int
    task_description: str
    trial_index: int
    initial_state_index: int

    environment_seed: int
    policy_episode_seed: int
    policy_seed_protocol: str
    policy_replan_count: int
    sampling_protocol_version: int | None

    delta_x: float
    delta_y: float
    offset_distance: float

    success: bool
    termination_reason: str
    simulation_steps: int
    control_steps: int
    control_frequency_hz: float
    simulated_control_seconds: float
    control_elapsed_seconds: float
    total_elapsed_seconds: float

    operator_connected_at_start: bool

    object_position_before: list[float]
    object_position_after_settle: list[float]
    object_position_final: list[float]

    output_directory: str



def _fixed_weight_directory_name(
    fixed_autonomy_weight: float,
) -> str:
    """Return a stable path component for one blend weight."""

    value = f"{fixed_autonomy_weight:.3f}".replace(
        ".",
        "p",
    )
    return f"alpha_{value}"


def _cosine_gain_directory_name(
    cosine_gain: float,
) -> str:
    """Return a stable path component for one cosine gain."""

    value = f"{cosine_gain:.3f}".replace(
        ".",
        "p",
    )
    return f"k_{value}"


def validate_args(args: Args) -> ArbitrationMode:
    if args.trial_index < 0:
        raise ValueError(
            "trial_index must be non-negative."
        )

    if args.initial_state_index < 0:
        raise ValueError(
            "initial_state_index must be non-negative."
        )

    if args.environment_seed < 0:
        raise ValueError(
            "environment_seed must be non-negative."
        )

    if args.policy_base_seed < 0:
        raise ValueError(
            "policy_base_seed must be non-negative."
        )

    if args.replan_steps <= 0:
        raise ValueError(
            "replan_steps must be positive."
        )

    if args.num_steps_wait < 0:
        raise ValueError(
            "num_steps_wait must be non-negative."
        )

    if args.max_steps <= 0:
        raise ValueError(
            "max_steps must be positive."
        )

    if args.control_frequency_hz <= 0.0:
        raise ValueError(
            "control_frequency_hz must be positive."
        )

    if args.client_timeout_seconds <= 0.0:
        raise ValueError(
            "client_timeout_seconds must be positive."
        )

    if args.arm_timeout_seconds <= 0.0:
        raise ValueError(
            "arm_timeout_seconds must be positive."
        )

    if (
        not np.isfinite(args.fixed_autonomy_weight)
        or not 0.0 <= args.fixed_autonomy_weight <= 1.0
    ):
        raise ValueError(
            "fixed_autonomy_weight must be finite and "
            "within [0, 1]."
        )

    if not np.isfinite(args.cosine_gain) or args.cosine_gain <= 0.0:
        raise ValueError(
            "cosine_gain must be finite and positive."
        )

    try:
        return ArbitrationMode(
            args.arbitration_mode
        )
    except ValueError as error:
        supported = ", ".join(
            mode.value for mode in ArbitrationMode
        )
        raise ValueError(
            f"Unsupported arbitration mode "
            f"{args.arbitration_mode!r}. "
            f"Supported modes: {supported}."
        ) from error


def main(args: Args) -> None:
    mode = validate_args(args)
    total_start = time.perf_counter()

    config = load_config(
        Path(args.config_path)
    )
    condition = select_condition(
        config,
        args.condition_id,
    )

    task_id = int(config["task_id"])
    delta_x = float(condition["dx"])
    delta_y = float(condition["dy"])

    policy_episode_seed = make_policy_episode_seed(
        base_seed=args.policy_base_seed,
        condition_id=args.condition_id,
        trial_index=args.trial_index,
        task_id=task_id,
        initial_state_index=args.initial_state_index,
    )

    output_root = Path(args.output_dir)
    mode_output_root = output_root / mode.value

    if mode is ArbitrationMode.FIXED_BLEND:
        mode_output_root = (
            mode_output_root
            / _fixed_weight_directory_name(
                args.fixed_autonomy_weight
            )
        )
    elif mode is ArbitrationMode.COSINE_BLEND:
        mode_output_root = (
            mode_output_root
            / _cosine_gain_directory_name(
                args.cosine_gain
            )
        )

    episode_directory = (
        mode_output_root
        / args.condition_id
        / f"task_{task_id:02d}"
        / f"init_{args.initial_state_index:03d}"
        / f"trial_{args.trial_index:03d}"
    )
    episode_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    env: Any | None = None
    operator: BrowserOperatorServer | None = None
    policy_worker: AsyncPolicyWorker | None = None

    simulation_steps = 0
    operator_connected_at_start = False

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
            seed=args.environment_seed,
            horizon=(
                args.num_steps_wait
                + args.max_steps
                + 1
            ),
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

        if "cream cheese" not in task_description.lower():
            raise ValueError(
                "Selected task is not the expected "
                f"cream-cheese task: {task_description!r}"
            )

        env.reset()
        nominal_obs = env.set_init_state(
            initial_states[
                args.initial_state_index
            ]
        )

        save_agent_image(
            episode_directory
            / "01_nominal_initial.png",
            nominal_obs,
        )

        obs, perturbation = (
            apply_planar_object_offset(
                env,
                joint_name=str(
                    config["joint_name"]
                ),
                body_name=str(
                    config["body_name"]
                ),
                delta_x=delta_x,
                delta_y=delta_y,
            )
        )

        save_agent_image(
            episode_directory
            / "02_perturbed_before_settle.png",
            obs,
        )

        done = False

        for _ in range(args.num_steps_wait):
            obs, reward, done, info = env.step(
                LIBERO_DUMMY_ACTION.tolist()
            )
            simulation_steps += 1

        if done:
            raise RuntimeError(
                "The task terminated during settling."
            )

        save_agent_image(
            episode_directory
            / "03_perturbed_after_settle.png",
            obs,
        )

        (
            settled_qpos,
            settled_body_position,
        ) = get_object_pose(
            env,
            joint_name=str(
                config["joint_name"]
            ),
            body_name=str(
                config["body_name"]
            ),
        )

        write_json_atomic(
            episode_directory
            / "perturbation.json",
            {
                "condition_id": args.condition_id,
                "requested_offset": {
                    "delta_x": delta_x,
                    "delta_y": delta_y,
                    "distance": float(
                        np.hypot(delta_x, delta_y)
                    ),
                },
                "immediate_result": (
                    dataclasses.asdict(
                        perturbation
                    )
                ),
                "settled_joint_qpos": (
                    settled_qpos.tolist()
                ),
                "settled_body_position": (
                    settled_body_position.tolist()
                ),
            },
        )

        policy = OpenPiLiberoPolicy(
            host=args.policy_host,
            port=args.policy_port,
            resize_size=args.resize_size,
        )

        policy_worker = AsyncPolicyWorker(
            policy=policy,
            policy_episode_seed=policy_episode_seed,
        )

        initial_request = policy_worker.submit(
            observation=obs,
            task_description=task_description,
            request_control_step=-1,
            generation=0,
            reason="initial_prefetch",
        )

        if initial_request is None:
            raise RuntimeError(
                "Could not submit initial policy prefetch."
            )

        operator = BrowserOperatorServer(
            host=args.host,
            websocket_port=args.websocket_port,
            http_port=args.http_port,
            fine_translation_gain=(
                args.fine_translation_gain
            ),
            normal_translation_gain=(
                args.translation_gain
            ),
            fast_translation_gain=(
                args.fast_translation_gain
            ),
            fine_rotation_gain=(
                args.fine_rotation_gain
            ),
            normal_rotation_gain=(
                args.rotation_gain
            ),
            fast_rotation_gain=(
                args.fast_rotation_gain
            ),
            default_speed_mode=(
                args.default_speed_mode
            ),
            jpeg_quality=args.jpeg_quality,
        )
        operator.start()

        scene_image = operator_view_rgb(obs)

        operator.publish_frame_rgb(
            scene_image,
            runtime_status={
                "phase": "waiting_for_browser",
                "arbitration_mode": mode.value,
                "configured_autonomy_weight": (
                    args.fixed_autonomy_weight
                    if mode
                    is ArbitrationMode.FIXED_BLEND
                    else None
                ),
                "cosine_gain": (
                    args.cosine_gain
                    if mode
                    is ArbitrationMode.COSINE_BLEND
                    else None
                ),
                "condition_id": args.condition_id,
                "trial_index": args.trial_index,
                "task": task_description,
                "policy_episode_seed": (
                    policy_episode_seed
                ),
            },
        )

        print()
        print("SAPS shared-autonomy episode")
        print()
        print(f"Mode: {mode.value}")
        if mode is ArbitrationMode.FIXED_BLEND:
            print(
                "Configured autonomy weight: "
                f"{args.fixed_autonomy_weight:.3f}"
            )
        elif mode is ArbitrationMode.COSINE_BLEND:
            print(f"Cosine gain: {args.cosine_gain:.3f}")
        print(f"Task: {task_description}")
        print(
            f"Condition: {args.condition_id} "
            f"(dx={delta_x:.3f}, dy={delta_y:.3f})"
        )
        print(
            "Matched policy seed: "
            f"{policy_episode_seed}"
        )
        print(f"Replan steps: {args.replan_steps}")
        print()
        print(f"Open: {operator.operator_url}")
        print()
        print(
            "Click 'Arm controls' to begin. "
            "Escape aborts the episode."
        )
        print()

        if not operator.wait_for_client(
            timeout_seconds=(
                args.client_timeout_seconds
            )
        ):
            raise TimeoutError(
                "Timed out waiting for the operator browser."
            )

        operator_connected_at_start = True

        wait_until_armed(
            operator=operator,
            scene_image=scene_image,
            timeout_seconds=args.arm_timeout_seconds,
            episode_label="shared-autonomy episode",
        )

        logging.info(
            "Starting %s control at %.1f Hz.",
            mode.value,
            args.control_frequency_hz,
        )

        loop_result = run_shared_episode_loop(
            env=env,
            operator=operator,
            policy_worker=policy_worker,
            initial_observation=obs,
            task_description=task_description,
            object_body_name=str(
                config["body_name"]
            ),
            arbitration_mode=mode.value,
            fixed_autonomy_weight=(
                args.fixed_autonomy_weight
            ),
            cosine_gain=args.cosine_gain,
            replan_steps=args.replan_steps,
            policy_episode_seed=(
                policy_episode_seed
            ),
            environment_seed=(
                args.environment_seed
            ),
            max_steps=args.max_steps,
            control_frequency_hz=(
                args.control_frequency_hz
            ),
            steps_path=(
                episode_directory / "steps.jsonl"
            ),
        )

        simulation_steps += (
            loop_result.simulation_steps
        )

        _, final_object_position = get_object_pose(
            env,
            joint_name=str(
                config["joint_name"]
            ),
            body_name=str(
                config["body_name"]
            ),
        )

        suffix = {
            "success": "success",
            "operator_abort": "operator_abort",
            "operator_disconnected": (
                "operator_disconnected"
            ),
            "timeout": "timeout",
            "environment_terminated": (
                "environment_terminated"
            ),
        }.get(
            loop_result.termination_reason,
            "terminated",
        )

        if loop_result.replay_images:
            imageio.mimwrite(
                episode_directory
                / f"rollout_{suffix}.mp4",
                list(loop_result.replay_images),
                fps=args.video_fps,
            )

        result = SharedAutonomyEpisodeResult(
            condition_id=args.condition_id,
            arbitration_mode=mode.value,
            fixed_autonomy_weight=(
                args.fixed_autonomy_weight
                if mode
                is ArbitrationMode.FIXED_BLEND
                else None
            ),
            cosine_gain=(
                args.cosine_gain
                if mode
                is ArbitrationMode.COSINE_BLEND
                else None
            ),
            task_id=task_id,
            task_description=task_description,
            trial_index=args.trial_index,
            initial_state_index=(
                args.initial_state_index
            ),
            environment_seed=(
                args.environment_seed
            ),
            policy_episode_seed=(
                policy_episode_seed
            ),
            policy_seed_protocol=SEED_PROTOCOL,
            policy_replan_count=(
                loop_result.policy_replan_count
            ),
            sampling_protocol_version=(
                loop_result.sampling_protocol_version
            ),
            delta_x=delta_x,
            delta_y=delta_y,
            offset_distance=float(
                np.hypot(delta_x, delta_y)
            ),
            success=loop_result.success,
            termination_reason=(
                loop_result.termination_reason
            ),
            simulation_steps=simulation_steps,
            control_steps=(
                loop_result.control_steps
            ),
            control_frequency_hz=(
                args.control_frequency_hz
            ),
            simulated_control_seconds=(
                loop_result.control_steps
                / args.control_frequency_hz
            ),
            control_elapsed_seconds=(
                loop_result.control_elapsed_seconds
            ),
            total_elapsed_seconds=(
                time.perf_counter() - total_start
            ),
            operator_connected_at_start=(
                operator_connected_at_start
            ),
            object_position_before=(
                perturbation.body_position_before
            ),
            object_position_after_settle=(
                settled_body_position.tolist()
            ),
            object_position_final=(
                final_object_position.tolist()
            ),
            output_directory=str(
                episode_directory
            ),
        )

        write_json_atomic(
            episode_directory / "summary.json",
            dataclasses.asdict(result),
        )

        logging.info(
            "Shared-autonomy episode finished: "
            "mode=%s reason=%s success=%s "
            "steps=%d replans=%d "
            "simulated_time=%.2f s elapsed=%.2f s",
            mode.value,
            result.termination_reason,
            result.success,
            result.control_steps,
            result.policy_replan_count,
            result.simulated_control_seconds,
            result.control_elapsed_seconds,
        )

    finally:
        if policy_worker is not None:
            policy_worker.close()

        if operator is not None:
            operator.close()

        if env is not None:
            close = getattr(env, "close", None)

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
