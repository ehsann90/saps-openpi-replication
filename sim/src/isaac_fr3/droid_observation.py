"""Convert one settled Isaac FR3 state into a raw DROID observation."""

from __future__ import annotations

import time
from typing import Any

import numpy as np

from saps.policies.openpi_droid import DROID_POLICY_INPUT_KEYS
from saps.policies.openpi_droid import prepare_droid_observation


ARM_JOINT_NAMES = tuple(f"fr3_joint{index}" for index in range(1, 8))
FINGER_JOINT_NAMES = ("fr3_finger_joint1", "fr3_finger_joint2")
RAW_IMAGE_SHAPE = (180, 320, 3)


def measured_arm(joint_names: list[str], positions: np.ndarray) -> np.ndarray:
    """Select actual articulation positions in canonical FR3 order."""
    if len(joint_names) != len(set(joint_names)):
        raise ValueError("Articulation joint names must be unique")
    value = np.asarray(positions)
    if value.shape != (len(joint_names),) or not np.isfinite(value).all():
        raise ValueError("Measured articulation positions are invalid")
    if not set(ARM_JOINT_NAMES).issubset(joint_names):
        raise ValueError("Missing canonical FR3 arm joint")
    return np.asarray(
        [value[joint_names.index(name)] for name in ARM_JOINT_NAMES],
        dtype=np.float32,
    )


def measured_gripper(
    finger_positions: np.ndarray, maximum_width_m: float = 0.08
) -> tuple[float, np.ndarray]:
    """Map measured finger width to DROID closure in [0, 1]."""
    fingers = np.asarray(finger_positions)
    if fingers.shape != (2,) or not np.isfinite(fingers).all():
        raise ValueError("Measured finger positions must be two finite values")
    if not np.isfinite(maximum_width_m) or maximum_width_m <= 0:
        raise ValueError("Maximum gripper width must be positive and finite")
    width = float(np.sum(fingers, dtype=np.float64))
    closure = np.asarray(
        [np.clip(1.0 - width / maximum_width_m, 0.0, 1.0)],
        dtype=np.float32,
    )
    return width, closure


def build_observation(
    *, exterior_image: np.ndarray, wrist_image: np.ndarray,
    joint_names: list[str], joint_positions: np.ndarray,
    finger_positions: np.ndarray, prompt: str,
    maximum_width_m: float = 0.08,
) -> tuple[dict[str, Any], float]:
    """Validate source shapes and use the shared DROID request constructor."""
    for name, image in (("exterior", exterior_image), ("wrist", wrist_image)):
        if not isinstance(image, np.ndarray) or image.shape != RAW_IMAGE_SHAPE:
            raise ValueError(f"{name} image must have shape {RAW_IMAGE_SHAPE}")
        if image.dtype != np.uint8:
            raise TypeError(f"{name} image must have dtype uint8")
    arm = measured_arm(joint_names, joint_positions)
    width, gripper = measured_gripper(finger_positions, maximum_width_m)
    request = prepare_droid_observation(
        exterior_image=exterior_image, wrist_image=wrist_image,
        joint_position=arm, gripper_position=gripper, prompt=prompt,
    )
    if tuple(request) != DROID_POLICY_INPUT_KEYS:
        raise RuntimeError("Unexpected DROID policy input keys")
    return request, width


def capture_observation(handles: Any, rig: Any, config: dict, prompt: str):
    """Capture both camera buffers and measured joints without stepping physics."""
    from isaac_fr3.cameras import capture_rgb

    names = config["robot"]["arm_dof_names"] + config["robot"]["finger_dof_names"]
    if tuple(config["robot"]["finger_dof_names"]) != FINGER_JOINT_NAMES:
        raise ValueError("Unexpected FR3 finger joint order")
    dof_indices = np.asarray(
        handles.fr3.get_dof_indices(names), dtype=np.int64
    ).reshape(-1)
    stamps = {}
    capture_start = time.perf_counter_ns()
    external = capture_rgb(rig.external, config["cameras"]["external"]["resolution_wh"])
    stamps["exterior_image_capture_ns"] = time.time_ns()
    wrist = capture_rgb(rig.wrist, config["cameras"]["wrist"]["resolution_wh"])
    stamps["wrist_image_capture_ns"] = time.time_ns()
    all_positions = np.asarray(handles.fr3.get_dof_positions()).reshape(-1)
    stamps["joint_state_read_ns"] = time.time_ns()
    if max(dof_indices) >= len(all_positions):
        raise ValueError("Measured articulation vector is too short")
    selected = np.array(all_positions[dof_indices], copy=True)
    arm = selected[:7]
    stamps["gripper_state_read_ns"] = time.time_ns()
    fingers = selected[7:]
    capture_seconds = (time.perf_counter_ns() - capture_start) / 1e9
    prepare_start = time.perf_counter_ns()
    request, width = build_observation(
        exterior_image=external, wrist_image=wrist,
        joint_names=names, joint_positions=selected,
        finger_positions=fingers, prompt=prompt,
        maximum_width_m=float(config["droid"]["gripper_max_width_m"]),
    )
    stamps["request_construction_ns"] = time.time_ns()
    preparation_seconds = (time.perf_counter_ns() - prepare_start) / 1e9
    return request, fingers.copy(), width, stamps, capture_seconds, preparation_seconds
