#!/usr/bin/env python3
"""Run one browser-controlled LIBERO teleoperation episode."""

from __future__ import annotations

import dataclasses
import json
import logging
import math
from pathlib import Path
import time
from typing import Any

import imageio.v2 as imageio
import numpy as np
import tyro

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
from saps.evaluation.operator_episode import validated_human_action
from saps.human_input.web_operator import (
    BrowserOperatorServer,
)
from saps.human_input.spacemouse import parse_axis_mapping
from saps.human_input.spacemouse import parse_axis_maxima
from saps.human_input.spacemouse import parse_axis_signs
from saps.human_input.spacemouse_profile import load_spacemouse_profile
from saps.policies.seeding import make_policy_episode_seed
from saps.policies.seeding import SEED_PROTOCOL


LIBERO_DUMMY_ACTION = np.asarray(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
    dtype=np.float32,
)


@dataclasses.dataclass
class Args:
    # Task and perturbation
    config_path: str = (
        "configs/libero_cream_cheese_offsets.json"
    )
    condition_id: str = "nominal"
    trial_index: int = 0
    initial_state_index: int = 0

    # Reproducibility
    environment_seed: int = 7
    policy_base_seed: int = 20260724

    # Environment
    resolution: int = 256
    num_steps_wait: int = 10
    max_steps: int = 1200
    control_frequency_hz: float = 20.0
    video_fps: int = 20

    # Operator console
    host: str = "0.0.0.0"
    websocket_port: int = 8765
    http_port: int = 8766
    # Normal mode retains these concise CLI names.
    translation_gain: float = 0.14
    rotation_gain: float = 0.18

    fine_translation_gain: float = 0.07
    fine_rotation_gain: float = 0.10
    fast_translation_gain: float = 0.25
    fast_rotation_gain: float = 0.30
    default_speed_mode: str = "fine"

    input_source: str = "keyboard"
    spacemouse_device_path: str = ""
    spacemouse_deadzone: float = 0.08
    spacemouse_axis_mapping: str = (
        "ABS_X,ABS_Y,ABS_Z,ABS_RX,ABS_RY,ABS_RZ"
    )
    spacemouse_axis_signs: str = "1,1,1,1,1,1"
    spacemouse_axis_maxima: str = "350,350,350,350,350,350"
    spacemouse_stale_input_timeout_seconds: float = 0.25
    spacemouse_open_button: int = 256
    spacemouse_close_button: int = 257
    spacemouse_profile_path: str = ""

    jpeg_quality: int = 85
    client_timeout_seconds: float = 120.0
    arm_timeout_seconds: float = 300.0

    # Outputs
    output_dir: str = "outputs/teleoperation_smoke"


@dataclasses.dataclass(frozen=True)
class TeleoperationResult:
    condition_id: str
    arbitration_mode: str
    task_id: int
    task_description: str
    trial_index: int
    initial_state_index: int
    environment_seed: int

    policy_episode_seed: int
    policy_seed_protocol: str
    policy_used: bool

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


def main(args: Args) -> None:
    if args.trial_index < 0:
        raise ValueError(
            "trial_index must be non-negative."
        )

    if args.initial_state_index < 0:
        raise ValueError(
            "initial_state_index must be non-negative."
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

    spacemouse_config = None
    if args.spacemouse_profile_path:
        if args.input_source.strip().lower() != "spacemouse":
            raise ValueError(
                "spacemouse_profile_path requires "
                "input_source='spacemouse'."
            )
        spacemouse_config = load_spacemouse_profile(
            Path(args.spacemouse_profile_path)
        ).to_config(
            device_path=args.spacemouse_device_path
        )

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
    episode_directory = (
        output_root
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

    replay_images: list[np.ndarray] = []
    simulation_steps = 0
    control_steps = 0
    success = False
    termination_reason = "initialization_error"
    operator_connected_at_start = False
    control_elapsed_seconds = 0.0

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

        perturbation_report = {
            "condition_id": args.condition_id,
            "requested_offset": {
                "delta_x": delta_x,
                "delta_y": delta_y,
                "distance": float(
                    np.hypot(delta_x, delta_y)
                ),
            },
            "immediate_result": dataclasses.asdict(
                perturbation
            ),
            "settled_joint_qpos": (
                settled_qpos.tolist()
            ),
            "settled_body_position": (
                settled_body_position.tolist()
            ),
        }

        write_json_atomic(
            episode_directory
            / "perturbation.json",
            perturbation_report,
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
            input_source=args.input_source,
            spacemouse_device_path=(
                args.spacemouse_device_path
            ),
            spacemouse_deadzone=args.spacemouse_deadzone,
            spacemouse_axis_mapping=parse_axis_mapping(
                args.spacemouse_axis_mapping
            ),
            spacemouse_axis_signs=parse_axis_signs(
                args.spacemouse_axis_signs
            ),
            spacemouse_axis_maxima=parse_axis_maxima(
                args.spacemouse_axis_maxima
            ),
            spacemouse_stale_input_timeout_seconds=(
                args.spacemouse_stale_input_timeout_seconds
            ),
            spacemouse_open_button=args.spacemouse_open_button,
            spacemouse_close_button=args.spacemouse_close_button,
            spacemouse_config=spacemouse_config,
            jpeg_quality=args.jpeg_quality,
        )
        operator.start()

        scene_image = operator_view_rgb(obs)

        operator.publish_frame_rgb(
            scene_image,
            runtime_status={
                "phase": "waiting_for_browser",
                "condition_id": args.condition_id,
                "trial_index": args.trial_index,
                "task": task_description,
            },
        )

        print()
        print("SAPS pure-teleoperation episode")
        print()
        print(f"Task: {task_description}")
        print(
            f"Condition: {args.condition_id} "
            f"(dx={delta_x:.3f}, dy={delta_y:.3f})"
        )
        print(
            "Matched policy seed: "
            f"{policy_episode_seed}"
        )
        print()
        print(f"Open: {operator.operator_url}")
        print()
        print(
            "After the scene appears, click "
            "'Arm controls' to begin."
        )
        print(
            "Escape or 'Abort episode' terminates "
            "the rollout."
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
            episode_label="teleoperation episode",
        )

        logging.info(
            "Operator armed. Starting teleoperation "
            "control loop at %.1f Hz.",
            args.control_frequency_hz,
        )

        control_period = (
            1.0 / args.control_frequency_hz
        )
        control_start = time.perf_counter()
        control_start_monotonic = time.monotonic()
        next_deadline = control_start_monotonic

        steps_path = (
            episode_directory / "steps.jsonl"
        )

        termination_reason = "timeout"

        with steps_path.open(
            "w",
            encoding="utf-8",
        ) as steps_file:
            for control_step in range(
                args.max_steps
            ):
                loop_start = time.monotonic()
                sample = operator.sample()

                if sample.abort_requested:
                    termination_reason = "operator_abort"
                    break

                if not sample.connected:
                    termination_reason = (
                        "operator_disconnected"
                    )
                    break

                human_action = validated_human_action(
                    sample.action
                )

                # Pure teleoperation:
                # executed action equals human action.
                executed_action = (
                    human_action.copy()
                )

                try:
                    obs, reward, done, info = env.step(
                        executed_action.tolist()
                    )
                except ValueError as error:
                    if (
                        "executing action in terminated episode"
                        not in str(error)
                    ):
                        raise

                    termination_reason = (
                        "environment_terminated"
                    )

                    logging.error(
                        "LIBERO terminated before the "
                        "configured operator horizon at "
                        "control step %d.",
                        control_step,
                    )
                    break

                simulation_steps += 1
                control_steps += 1

                frame = operator_view_rgb(obs)
                replay_images.append(frame)

                eef_position = np.asarray(
                    obs["robot0_eef_pos"],
                    dtype=np.float32,
                )

                object_position = np.asarray(
                    env.sim.data.get_body_xpos(
                        str(config["body_name"])
                    ),
                    dtype=np.float64,
                )

                gripper_qpos = np.asarray(
                    obs["robot0_gripper_qpos"],
                    dtype=np.float32,
                )

                record = {
                    "simulation_step": simulation_steps,
                    "control_step": control_step,
                    "arbitration_mode": (
                        "teleoperation"
                    ),
                    "environment_seed": (
                        args.environment_seed
                    ),
                    "policy_episode_seed": (
                        policy_episode_seed
                    ),
                    "policy_seed_protocol": (
                        SEED_PROTOCOL
                    ),
                    "policy_used": False,
                    "policy_replan_index": None,
                    "operator_connected": (
                        sample.connected
                    ),
                    "operator_armed": sample.armed,
                    "operator_abort_requested": (
                        sample.abort_requested
                    ),
                    "operator_motion_active": (
                        sample.motion_active
                    ),
                    "operator_speed_mode": (
                        sample.speed_mode
                    ),
                    "operator_translation_gain": (
                        sample.translation_gain
                    ),
                    "operator_rotation_gain": (
                        sample.rotation_gain
                    ),
                    "operator_gripper_command": (
                        sample.gripper_command
                    ),
                    "operator_pressed_keys": list(
                        sample.pressed_keys
                    ),
                    "operator_sample_monotonic_seconds": (
                        sample.sample_monotonic_seconds
                    ),
                    "operator_last_event_monotonic_seconds": (
                        sample.last_event_monotonic_seconds
                    ),
                    "human_input": sample.as_dict(),
                    "human_action": (
                        human_action.tolist()
                    ),
                    "autonomous_action": None,
                    "autonomy_weight": 0.0,
                    "executed_action": (
                        executed_action.tolist()
                    ),
                    "reward": float(reward),
                    "done": bool(done),
                    "eef_position": (
                        eef_position.tolist()
                    ),
                    "object_position": (
                        object_position.tolist()
                    ),
                    "gripper_qpos": (
                        gripper_qpos.tolist()
                    ),
                    "step_wall_time_unix_seconds": (
                        time.time()
                    ),
                    "step_loop_seconds": (
                        time.monotonic() - loop_start
                    ),
                }

                steps_file.write(
                    json.dumps(record) + "\n"
                )
                steps_file.flush()

                operator.publish_frame_rgb(
                    frame,
                    runtime_status={
                        "phase": "teleoperation",
                        "condition_id": (
                            args.condition_id
                        ),
                        "trial_index": (
                            args.trial_index
                        ),
                        "control_step": (
                            control_step
                        ),
                        "max_steps": args.max_steps,
                        "simulated_seconds": (
                            control_steps
                            / args.control_frequency_hz
                        ),
                        "remaining_steps": max(
                            0,
                            args.max_steps - control_steps,
                        ),
                        "remaining_seconds": max(
                            0.0,
                            (
                                args.max_steps
                                - control_steps
                            )
                            / args.control_frequency_hz,
                        ),
                        "episode_horizon_seconds": (
                            args.max_steps
                            / args.control_frequency_hz
                        ),
                        "task": task_description,
                        "success": bool(done),
                        "reward": float(reward),
                        "operator_armed": (
                            sample.armed
                        ),
                        "motion_active": (
                            sample.motion_active
                        ),
                        "speed_mode": (
                            sample.speed_mode
                        ),
                        "translation_gain": (
                            sample.translation_gain
                        ),
                        "rotation_gain": (
                            sample.rotation_gain
                        ),
                        "gripper_command": (
                            sample.gripper_command
                        ),
                        "gripper_qpos": (
                            gripper_qpos.tolist()
                        ),
                        "eef_position": (
                            eef_position.tolist()
                        ),
                        "object_position": (
                            object_position.tolist()
                        ),
                    },
                )

                if done:
                    success = True
                    termination_reason = "success"
                    break

                next_deadline += control_period
                remaining = (
                    next_deadline
                    - time.monotonic()
                )

                if remaining > 0.0:
                    time.sleep(remaining)
                else:
                    # If rendering or simulation falls behind,
                    # do not accumulate an ever-growing delay.
                    next_deadline = time.monotonic()

        control_elapsed_seconds = (
            time.perf_counter() - control_start
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
        }.get(
            termination_reason,
            "terminated",
        )

        if replay_images:
            imageio.mimwrite(
                episode_directory
                / f"rollout_{suffix}.mp4",
                replay_images,
                fps=args.video_fps,
            )

        total_elapsed_seconds = (
            time.perf_counter() - total_start
        )

        result = TeleoperationResult(
            condition_id=args.condition_id,
            arbitration_mode="teleoperation",
            task_id=task_id,
            task_description=task_description,
            trial_index=args.trial_index,
            initial_state_index=(
                args.initial_state_index
            ),
            environment_seed=args.environment_seed,
            policy_episode_seed=policy_episode_seed,
            policy_seed_protocol=SEED_PROTOCOL,
            policy_used=False,
            delta_x=delta_x,
            delta_y=delta_y,
            offset_distance=float(
                np.hypot(delta_x, delta_y)
            ),
            success=success,
            termination_reason=termination_reason,
            simulation_steps=simulation_steps,
            control_steps=control_steps,
            control_frequency_hz=(
                args.control_frequency_hz
            ),
            simulated_control_seconds=(
                control_steps
                / args.control_frequency_hz
            ),
            control_elapsed_seconds=(
                control_elapsed_seconds
            ),
            total_elapsed_seconds=(
                total_elapsed_seconds
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
            "Teleoperation episode finished: "
            "reason=%s success=%s steps=%d "
            "simulated_time=%.2f s",
            termination_reason,
            success,
            control_steps,
            result.simulated_control_seconds,
        )

    finally:
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
