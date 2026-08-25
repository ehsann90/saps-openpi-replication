#!/usr/bin/env python3
"""Inspect SpaceMouse input without starting LIBERO or a policy server."""

from __future__ import annotations

import dataclasses
import logging
from pathlib import Path
import sys
import time

import numpy as np
import tyro

from saps.human_input.spacemouse import AXIS_NAMES
from saps.human_input.spacemouse import parse_axis_mapping
from saps.human_input.spacemouse import parse_axis_maxima
from saps.human_input.spacemouse import parse_axis_signs
from saps.human_input.spacemouse import SpaceMouseBackend
from saps.human_input.spacemouse import SpaceMouseConfig
from saps.human_input.spacemouse import SpaceMouseUnavailableError
from saps.human_input.spacemouse_profile import load_spacemouse_profile


@dataclasses.dataclass
class Args:
    device_path: str = ""
    profile_path: str = ""
    translation_gain: float = 0.14
    rotation_gain: float = 0.18
    deadzone: float = 0.08
    axis_mapping: str = (
        "ABS_X,ABS_Y,ABS_Z,ABS_RX,ABS_RY,ABS_RZ"
    )
    axis_signs: str = "1,1,1,1,1,1"
    axis_maxima: str = "350,350,350,350,350,350"
    stale_input_timeout_seconds: float = 0.25
    open_button: int = 256
    close_button: int = 257
    refresh_frequency_hz: float = 20.0
    duration_seconds: float = 0.0


def make_config(args: Args) -> SpaceMouseConfig:
    """Build the same processing configuration used by live runners."""

    if args.profile_path:
        return load_spacemouse_profile(Path(args.profile_path)).to_config(
            device_path=args.device_path
        )

    return SpaceMouseConfig(
        device_path=args.device_path,
        translation_gain=args.translation_gain,
        rotation_gain=args.rotation_gain,
        deadzone=args.deadzone,
        axis_mapping=parse_axis_mapping(args.axis_mapping),
        axis_signs=parse_axis_signs(args.axis_signs),
        axis_maxima=parse_axis_maxima(args.axis_maxima),
        stale_input_timeout_seconds=(
            args.stale_input_timeout_seconds
        ),
        open_button=args.open_button,
        close_button=args.close_button,
    )


def run(args: Args) -> int:
    """Acquire the device and continuously print its normalized state."""

    if args.refresh_frequency_hz <= 0.0:
        raise ValueError("refresh_frequency_hz must be positive.")
    if args.duration_seconds < 0.0:
        raise ValueError("duration_seconds must be non-negative.")

    backend = SpaceMouseBackend(make_config(args))
    try:
        info = backend.start()
    except SpaceMouseUnavailableError as error:
        print(f"SpaceMouse initialization failed: {error}", file=sys.stderr)
        return 2

    print("SpaceMouse diagnostic (no LIBERO, policy, or robot)")
    print(f"Selected path: {info.path}")
    print(f"Selected name: {info.name}")
    print(
        "Processing profile: "
        f"{args.profile_path or 'none (raw CLI settings)'}"
    )
    print(f"Axis mapping: {backend.config.axis_mapping}")
    print(f"Axis signs: {backend.config.axis_signs}")
    print(
        "Gains: translation="
        f"{backend.config.translation_gain} "
        f"rotation={backend.config.rotation_gain}"
    )
    print(f"Required axes: {', '.join(AXIS_NAMES)}")
    print(f"Reported ranges: {info.axis_ranges}")
    print("Exclusive access: acquired with EVIOCGRAB")
    print("Press Ctrl+C to stop.")

    gripper_command = -1.0
    started = time.monotonic()
    period = 1.0 / args.refresh_frequency_hz

    try:
        while True:
            sample = backend.sample()
            if sample.close_button_pressed:
                gripper_command = 1.0
            elif sample.open_button_pressed:
                gripper_command = -1.0

            action = np.asarray(
                [*sample.final_motion, gripper_command],
                dtype=np.float32,
            )
            print(
                "connected={connected} stale={stale} active={active} "
                "raw={raw} mapped={mapped} action={action} "
                "buttons(open={open_button}, close={close_button}) "
                "native_ts={timestamp} error={error}".format(
                    connected=sample.connected,
                    stale=sample.stale,
                    active=sample.motion_active,
                    raw=sample.raw_axes,
                    mapped=np.array2string(
                        np.asarray(sample.mapped_axes),
                        precision=3,
                        suppress_small=True,
                    ),
                    action=np.array2string(
                        action,
                        precision=3,
                        suppress_small=True,
                    ),
                    open_button=sample.open_button_pressed,
                    close_button=sample.close_button_pressed,
                    timestamp=sample.native_event_timestamp_seconds,
                    error=sample.error,
                ),
                flush=True,
            )

            if (
                args.duration_seconds > 0.0
                and time.monotonic() - started >= args.duration_seconds
            ):
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nSpaceMouse diagnostic stopped.")
    finally:
        backend.close()

    return 0


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )
    raise SystemExit(run(tyro.cli(Args)))
