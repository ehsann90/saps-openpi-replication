"""Reusable setup and display helpers for operator-controlled episodes."""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any

import cv2
import imageio.v2 as imageio
import numpy as np

from saps.human_input.web_operator import BrowserOperatorServer


def load_config(path: Path) -> dict[str, Any]:
    """Load and minimally validate an experiment configuration."""

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
            "Configuration is missing fields: "
            f"{sorted(missing)}"
        )

    return config


def select_condition(
    config: dict[str, Any],
    condition_id: str,
) -> dict[str, Any]:
    """Return one uniquely identified perturbation condition."""

    matches = [
        offset
        for offset in config["offsets"]
        if str(offset["id"]) == condition_id
    ]

    if not matches:
        available = [
            str(offset["id"])
            for offset in config["offsets"]
        ]
        raise ValueError(
            f"Unknown condition {condition_id!r}. "
            f"Available conditions: {available}"
        )

    if len(matches) != 1:
        raise ValueError(
            f"Condition {condition_id!r} is not unique."
        )

    return matches[0]


def agent_view_rgb(
    obs: dict[str, Any],
) -> np.ndarray:
    """Return the upright agent-view image used by OpenPI."""

    return np.ascontiguousarray(
        obs["agentview_image"][::-1, ::-1],
        dtype=np.uint8,
    )


def wrist_view_rgb(
    obs: dict[str, Any],
) -> np.ndarray:
    """Return the upright wrist-camera image."""

    return np.ascontiguousarray(
        obs["robot0_eye_in_hand_image"][
            ::-1,
            ::-1,
        ],
        dtype=np.uint8,
    )


def _label_camera(
    image_rgb: np.ndarray,
    label: str,
) -> np.ndarray:
    image = np.asarray(
        image_rgb,
        dtype=np.uint8,
    ).copy()

    cv2.rectangle(
        image,
        (0, 0),
        (image.shape[1], 28),
        (0, 0, 0),
        thickness=-1,
    )

    cv2.putText(
        image,
        label,
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )

    return image


def operator_view_rgb(
    obs: dict[str, Any],
) -> np.ndarray:
    """Combine labeled agent and wrist views for the browser."""

    agent = _label_camera(
        agent_view_rgb(obs),
        "Agent view",
    )
    wrist = _label_camera(
        wrist_view_rgb(obs),
        "Wrist view",
    )

    if wrist.shape[:2] != agent.shape[:2]:
        wrist = cv2.resize(
            wrist,
            (
                agent.shape[1],
                agent.shape[0],
            ),
            interpolation=cv2.INTER_AREA,
        )

    separator = np.full(
        (
            agent.shape[0],
            16,
            3,
        ),
        220,
        dtype=np.uint8,
    )

    return np.concatenate(
        (agent, separator, wrist),
        axis=1,
    )


def save_agent_image(
    path: Path,
    obs: dict[str, Any],
) -> None:
    """Save the upright agent-view image."""

    imageio.imwrite(
        path,
        agent_view_rgb(obs),
    )


def write_json_atomic(
    path: Path,
    payload: dict[str, Any],
) -> None:
    """Write JSON without exposing a partially written file."""

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

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


def validated_human_action(action: np.ndarray) -> np.ndarray:
    """Validate and clip one controller-neutral seven-dimensional action."""

    human_action = np.asarray(action, dtype=np.float32)
    if human_action.shape != (7,):
        raise ValueError(
            "Expected human action shape (7,), "
            f"received {human_action.shape}."
        )
    if not np.all(np.isfinite(human_action)):
        raise ValueError(
            "Human action must contain only finite values."
        )
    return np.clip(human_action, -1.0, 1.0).astype(np.float32)


def wait_until_armed(
    *,
    operator: BrowserOperatorServer,
    scene_image: np.ndarray,
    timeout_seconds: float,
    episode_label: str = "operator-controlled episode",
) -> None:
    """Publish the scene until the connected operator arms controls."""

    start = time.monotonic()

    while True:
        sample = operator.sample()
        elapsed = time.monotonic() - start

        operator.publish_frame_rgb(
            scene_image,
            runtime_status={
                "phase": "waiting_for_arm",
                "message": (
                    "Click 'Arm controls' to begin "
                    f"the {episode_label}."
                ),
                "connected": sample.connected,
                "armed": sample.armed,
                "abort_requested": (
                    sample.abort_requested
                ),
                "elapsed_seconds": elapsed,
            },
        )

        if sample.abort_requested:
            raise RuntimeError(
                "Operator aborted before the episode began."
            )

        if not sample.connected:
            raise RuntimeError(
                "Operator browser disconnected before arming."
            )

        if sample.armed:
            return

        if elapsed >= timeout_seconds:
            raise TimeoutError(
                "Timed out waiting for the operator "
                "to arm the controls."
            )

        time.sleep(0.05)
