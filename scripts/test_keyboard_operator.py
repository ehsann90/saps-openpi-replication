#!/usr/bin/env python3
"""Test browser keyboard input without moving a robot."""

from __future__ import annotations

import dataclasses
import logging
import time

import cv2
import numpy as np
import tyro

from saps.human_input.web_operator import (
    BrowserOperatorServer,
)


@dataclasses.dataclass
class Args:
    host: str = "0.0.0.0"
    websocket_port: int = 8765
    http_port: int = 8766

    translation_gain: float = 0.35
    rotation_gain: float = 0.35

    duration_seconds: float = 300.0
    frame_frequency_hz: float = 10.0


def make_test_frame(
    *,
    elapsed_seconds: float,
    sample_action: np.ndarray,
    pressed_keys: tuple[str, ...],
    armed: bool,
    motion_active: bool,
) -> np.ndarray:
    frame = np.full(
        (480, 640, 3),
        28,
        dtype=np.uint8,
    )

    lines = [
        "SAPS Phase 2 operator-input test",
        f"Elapsed: {elapsed_seconds:7.2f} s",
        f"Armed: {armed}",
        f"Motion active: {motion_active}",
        (
            "Pressed: "
            + (
                ", ".join(pressed_keys)
                if pressed_keys
                else "none"
            )
        ),
        "Action:",
        np.array2string(
            sample_action,
            precision=3,
            suppress_small=True,
        ),
    ]

    y = 55

    for index, line in enumerate(lines):
        font_scale = 0.8 if index == 0 else 0.6

        cv2.putText(
            frame,
            line,
            (28, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (220, 235, 245),
            2 if index == 0 else 1,
            cv2.LINE_AA,
        )

        y += 55 if index == 0 else 42

    return cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2RGB,
    )


def main(args: Args) -> None:
    if args.duration_seconds <= 0.0:
        raise ValueError(
            "duration_seconds must be positive."
        )

    if args.frame_frequency_hz <= 0.0:
        raise ValueError(
            "frame_frequency_hz must be positive."
        )

    operator = BrowserOperatorServer(
        host=args.host,
        websocket_port=args.websocket_port,
        http_port=args.http_port,
        translation_gain=args.translation_gain,
        rotation_gain=args.rotation_gain,
    )

    operator.start()

    print()
    print("SAPS browser operator console is running.")
    print()
    print(f"Open: {operator.operator_url}")
    print()
    print(
        "Click 'Arm controls', then hold and release "
        "the mapped keys."
    )
    print(
        "Press Escape or click 'Abort episode' "
        "to end the test."
    )
    print()

    start_time = time.monotonic()
    frame_period = 1.0 / args.frame_frequency_hz
    next_frame_time = start_time
    previous_signature = None

    try:
        while True:
            now = time.monotonic()
            elapsed = now - start_time
            sample = operator.sample()

            signature = (
                sample.connected,
                sample.armed,
                sample.abort_requested,
                sample.motion_active,
                sample.pressed_keys,
                tuple(
                    float(value)
                    for value in sample.action
                ),
            )

            if signature != previous_signature:
                logging.info(
                    "connected=%s armed=%s active=%s "
                    "abort=%s keys=%s action=%s",
                    sample.connected,
                    sample.armed,
                    sample.motion_active,
                    sample.abort_requested,
                    sample.pressed_keys,
                    np.array2string(
                        sample.action,
                        precision=3,
                        suppress_small=True,
                    ),
                )
                previous_signature = signature

            if now >= next_frame_time:
                frame = make_test_frame(
                    elapsed_seconds=elapsed,
                    sample_action=sample.action,
                    pressed_keys=sample.pressed_keys,
                    armed=sample.armed,
                    motion_active=sample.motion_active,
                )

                operator.publish_frame_rgb(
                    frame,
                    runtime_status={
                        "phase": "operator_input_test",
                        "elapsed_seconds": elapsed,
                        "connected": sample.connected,
                        "armed": sample.armed,
                        "motion_active": (
                            sample.motion_active
                        ),
                        "abort_requested": (
                            sample.abort_requested
                        ),
                    },
                )

                next_frame_time += frame_period

                if next_frame_time < now:
                    next_frame_time = (
                        now + frame_period
                    )

            if sample.abort_requested:
                print(
                    "Abort received; ending operator test."
                )
                break

            if elapsed >= args.duration_seconds:
                print(
                    "Test duration reached; ending."
                )
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received.")

    finally:
        operator.close()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format=(
            "%(asctime)s | %(levelname)s | "
            "%(message)s"
        ),
    )

    main(tyro.cli(Args))
