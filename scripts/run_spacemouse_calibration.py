#!/usr/bin/env python3
"""Run a disposable nominal-scene SpaceMouse calibration shakedown."""

from __future__ import annotations

import dataclasses
import logging
import os
from pathlib import Path
import time
from typing import Any
from typing import Optional

import tyro

from saps.environments.libero_env import create_libero_task
from saps.evaluation.calibration import reset_nominal_calibration_scene
from saps.evaluation.operator_episode import load_config
from saps.evaluation.operator_episode import operator_view_rgb
from saps.evaluation.operator_episode import validated_human_action
from saps.human_input.spacemouse import SpaceMouseConfig
from saps.human_input.spacemouse_profile import load_spacemouse_profile
from saps.human_input.web_operator import BrowserOperatorServer


@dataclasses.dataclass
class Args:
    config_path: str = "configs/libero_cream_cheese_offsets.json"
    initial_state_index: int = 0
    environment_seed: int = 7
    resolution: int = 256
    num_steps_wait: int = 10
    control_frequency_hz: float = 20.0

    host: str = "0.0.0.0"
    websocket_port: int = 8765
    http_port: int = 8766
    jpeg_quality: int = 85
    client_timeout_seconds: float = 300.0

    device_path: str = ""
    profile_path: str = "configs/spacemouse_profile.json"
    load_existing_profile: bool = True
    translation_gain: float = 0.30
    rotation_gain: float = 0.08
    deadzone: float = 0.08
    stale_input_timeout_seconds: float = 0.25
    open_button: int = 256
    close_button: int = 257


def initial_calibration_config(args: Args) -> SpaceMouseConfig:
    """Load a profile or start from the staged six-axis candidate."""

    profile_path = Path(args.profile_path)
    if args.load_existing_profile and profile_path.is_file():
        return load_spacemouse_profile(profile_path).to_config(
            device_path=args.device_path
        )

    return SpaceMouseConfig(
        device_path=args.device_path,
        translation_gain=args.translation_gain,
        rotation_gain=args.rotation_gain,
        deadzone=args.deadzone,
        axis_mapping=(
            "ABS_Y",
            "ABS_X",
            "ABS_Z",
            "ABS_RY",
            "ABS_RX",
            "ABS_RZ",
        ),
        axis_signs=(-1.0, 1.0, -1.0, -1.0, 1.0, 1.0),
        axis_scales=(1.0,) * 6,
        axis_enabled=(True, True, True, False, False, False),
        stale_input_timeout_seconds=(
            args.stale_input_timeout_seconds
        ),
        open_button=args.open_button,
        close_button=args.close_button,
    )


def _owner_from_environment(name: str) -> Optional[int]:
    value = os.environ.get(name, "").strip()
    if not value:
        return None
    try:
        parsed = int(value)
    except ValueError:
        return None
    return parsed if parsed >= 0 else None


def main(args: Args) -> None:
    if args.initial_state_index < 0:
        raise ValueError("initial_state_index must be non-negative.")
    if args.num_steps_wait < 0:
        raise ValueError("num_steps_wait must be non-negative.")
    if args.control_frequency_hz <= 0.0:
        raise ValueError("control_frequency_hz must be positive.")
    if args.client_timeout_seconds <= 0.0:
        raise ValueError("client_timeout_seconds must be positive.")

    config = load_config(Path(args.config_path))
    input_config = initial_calibration_config(args)
    operator = BrowserOperatorServer(
        host=args.host,
        websocket_port=args.websocket_port,
        http_port=args.http_port,
        input_source="spacemouse",
        spacemouse_config=input_config,
        calibration_mode=True,
        calibration_profile_path=args.profile_path,
        calibration_profile_owner_uid=_owner_from_environment(
            "LOCAL_UID"
        ),
        calibration_profile_owner_gid=_owner_from_environment(
            "LOCAL_GID"
        ),
        jpeg_quality=args.jpeg_quality,
    )

    env: Any | None = None
    operator.start()
    try:
        env, task_description, initial_states = create_libero_task(
            task_suite_name=str(config["task_suite_name"]),
            task_id=int(config["task_id"]),
            resolution=args.resolution,
            seed=args.environment_seed,
            horizon=100000,
        )
        if not 0 <= args.initial_state_index < len(initial_states):
            raise ValueError(
                f"initial_state_index={args.initial_state_index} is "
                f"invalid for {len(initial_states)} states."
            )

        observation, simulation_steps = reset_nominal_calibration_scene(
            env=env,
            initial_states=initial_states,
            initial_state_index=args.initial_state_index,
            num_steps_wait=args.num_steps_wait,
        )
        resets = 0
        operator.publish_frame_rgb(
            operator_view_rgb(observation),
            runtime_status={
                "phase": "spacemouse_calibration",
                "disposable_calibration": True,
                "task": task_description,
                "message": "Connect, adjust while disarmed, then arm.",
            },
        )

        print()
        print("SAPS SpaceMouse calibration/shakedown")
        print("No policy server, arbitration, schedule, or experiment data.")
        print(f"Open: {operator.operator_url}")
        print(f"Profile save path: {args.profile_path}")
        print("Use browser Disarm or Abort for immediate safety.")
        print()

        if not operator.wait_for_client(args.client_timeout_seconds):
            raise TimeoutError(
                "Timed out waiting for the calibration browser."
            )

        period = 1.0 / args.control_frequency_hz
        next_deadline = time.monotonic()
        needs_reset = False
        while True:
            sample = operator.sample()
            if sample.abort_requested:
                print("Calibration abort requested.")
                break

            reset_requested = operator.consume_calibration_reset_request()
            if reset_requested:
                observation, reset_steps = reset_nominal_calibration_scene(
                    env=env,
                    initial_states=initial_states,
                    initial_state_index=args.initial_state_index,
                    num_steps_wait=args.num_steps_wait,
                )
                simulation_steps += reset_steps
                resets += 1
                needs_reset = False
                operator.publish_calibration_status(
                    "Nominal scene reset complete while disarmed. "
                    "Re-arm to continue."
                )

            reward = 0.0
            done = False
            if sample.armed and not needs_reset:
                action = validated_human_action(sample.action)
                observation, reward, done, info = env.step(
                    action.tolist()
                )
                del info
                simulation_steps += 1
                if done:
                    needs_reset = True
                    operator.disarm()
                    operator.publish_calibration_status(
                        "Scene terminated; controls disarmed. "
                        "Use Reset nominal scene."
                    )

            operator.publish_frame_rgb(
                operator_view_rgb(observation),
                runtime_status={
                    "phase": "spacemouse_calibration",
                    "disposable_calibration": True,
                    "task": task_description,
                    "armed": sample.armed,
                    "physical_device_connected": (
                        sample.physical_device_connected
                    ),
                    "stale": sample.stale_input,
                    "simulation_steps": simulation_steps,
                    "scene_resets": resets,
                    "scene_requires_reset": needs_reset,
                    "reward": float(reward),
                    "done": bool(done),
                },
            )

            next_deadline += period
            remaining = next_deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_deadline = time.monotonic()
    finally:
        operator.close()
        if env is not None:
            env.close()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )
    main(tyro.cli(Args))
