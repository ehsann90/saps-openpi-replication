"""Controller-neutral normalized human-input samples."""

from __future__ import annotations

import dataclasses
from typing import Any
from typing import Optional
from typing import Tuple

import numpy as np


@dataclasses.dataclass(frozen=True)
class HumanInputSample:
    """One normalized operator-command snapshot from any controller."""

    action: np.ndarray
    motion_active: bool
    connected: bool
    armed: bool
    abort_requested: bool
    pressed_keys: Tuple[str, ...]
    gripper_command: float

    speed_mode: str
    translation_gain: float
    rotation_gain: float

    sample_monotonic_seconds: float
    last_event_monotonic_seconds: Optional[float]

    input_source: str = "keyboard"
    physical_device_connected: Optional[bool] = None
    selected_device_name: Optional[str] = None
    selected_device_path: Optional[str] = None
    raw_axes: Optional[Tuple[int, ...]] = None
    mapped_axes: Optional[Tuple[float, ...]] = None
    deadzone: Optional[float] = None
    axis_mapping: Optional[Tuple[str, ...]] = None
    axis_signs: Optional[Tuple[float, ...]] = None
    axis_maxima: Optional[Tuple[float, ...]] = None
    axis_scales: Optional[Tuple[float, ...]] = None
    axis_enabled: Optional[Tuple[bool, ...]] = None
    stale_input: bool = False
    stale_input_timeout_seconds: Optional[float] = None
    open_button: Optional[int] = None
    close_button: Optional[int] = None
    open_button_pressed: bool = False
    close_button_pressed: bool = False
    native_event_timestamp_seconds: Optional[float] = None
    physical_device_error: Optional[str] = None
    calibration_status: Optional[str] = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "action": self.action.tolist(),
            "motion_active": self.motion_active,
            "connected": self.connected,
            "armed": self.armed,
            "abort_requested": self.abort_requested,
            "pressed_keys": list(self.pressed_keys),
            "gripper_command": self.gripper_command,
            "speed_mode": self.speed_mode,
            "translation_gain": self.translation_gain,
            "rotation_gain": self.rotation_gain,
            "sample_monotonic_seconds": (
                self.sample_monotonic_seconds
            ),
            "last_event_monotonic_seconds": (
                self.last_event_monotonic_seconds
            ),
            "input_source": self.input_source,
            "physical_device_connected": (
                self.physical_device_connected
            ),
            "selected_device_name": self.selected_device_name,
            "selected_device_path": self.selected_device_path,
            "raw_axes": (
                list(self.raw_axes)
                if self.raw_axes is not None
                else None
            ),
            "mapped_axes": (
                list(self.mapped_axes)
                if self.mapped_axes is not None
                else None
            ),
            "deadzone": self.deadzone,
            "axis_mapping": (
                list(self.axis_mapping)
                if self.axis_mapping is not None
                else None
            ),
            "axis_signs": (
                list(self.axis_signs)
                if self.axis_signs is not None
                else None
            ),
            "axis_maxima": (
                list(self.axis_maxima)
                if self.axis_maxima is not None
                else None
            ),
            "axis_scales": (
                list(self.axis_scales)
                if self.axis_scales is not None
                else None
            ),
            "axis_enabled": (
                list(self.axis_enabled)
                if self.axis_enabled is not None
                else None
            ),
            "stale_input": self.stale_input,
            "stale_input_timeout_seconds": (
                self.stale_input_timeout_seconds
            ),
            "open_button": self.open_button,
            "close_button": self.close_button,
            "open_button_pressed": self.open_button_pressed,
            "close_button_pressed": self.close_button_pressed,
            "native_event_timestamp_seconds": (
                self.native_event_timestamp_seconds
            ),
            "physical_device_error": self.physical_device_error,
            "calibration_status": self.calibration_status,
        }
