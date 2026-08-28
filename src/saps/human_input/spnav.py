"""Non-actuating spnavd SpaceMouse input for physical SAPS."""

from __future__ import annotations

from collections.abc import Sequence
import dataclasses
from pathlib import Path
import time
from typing import Any
from typing import Optional
from typing import Protocol

import numpy as np

from saps.human_input.sample import HumanInputSample


SPNAV_AXIS_MAPPING = (
    "-translation_z",
    "+translation_x",
    "+translation_y",
    "-rotation_z",
    "+rotation_x",
    "+rotation_y",
)
TCP_FRAME = "fr3_hand_tcp"
SAPS_FRAME = "fr3_link0"


@dataclasses.dataclass(frozen=True)
class SpnavConfig:
    """Physical lab mapping preserved from the pinned controller repo."""

    maximum_raw_value: float = 500.0
    deadzone: float = 0.3
    stale_timeout_seconds: float = 0.25
    idle_threshold: float = 1e-3
    open_button: int = 0
    close_button: int = 1
    device_path: str = (
        "/dev/input/by-id/"
        "usb-3Dconnexion_SpaceMouse_Wireless-event-joystick"
    )

    def __post_init__(self) -> None:
        for field_name, value in (
            ("maximum_raw_value", self.maximum_raw_value),
            ("stale_timeout_seconds", self.stale_timeout_seconds),
        ):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{field_name} must be finite and positive.")
        if not np.isfinite(self.deadzone) or not 0.0 <= self.deadzone < 1.0:
            raise ValueError("deadzone must be finite and within [0, 1).")
        if not np.isfinite(self.idle_threshold) or self.idle_threshold < 0.0:
            raise ValueError("idle_threshold must be finite and non-negative.")
        if self.open_button < 0 or self.close_button < 0:
            raise ValueError("SpaceMouse button identifiers must be non-negative.")
        if self.open_button == self.close_button:
            raise ValueError("open_button and close_button must differ.")


@dataclasses.dataclass(frozen=True)
class MotionEvent:
    """One raw six-axis event from spnavd."""

    translation: tuple[int, int, int]
    rotation: tuple[int, int, int]


@dataclasses.dataclass(frozen=True)
class ButtonEvent:
    """One button transition from spnavd."""

    button: int
    pressed: bool


@dataclasses.dataclass(frozen=True)
class TimedButtonEvent:
    """One button transition with its local receive timestamp."""

    button: int
    pressed: bool
    receive_monotonic_seconds: float


@dataclasses.dataclass(frozen=True)
class ProcessedSpnavMotion:
    """Raw-normalized and lab-mapped SpaceMouse values."""

    normalized_raw: np.ndarray
    deadzoned_raw: np.ndarray
    tcp_frame_normalized: np.ndarray


@dataclasses.dataclass(frozen=True)
class SpnavSapsSample:
    """HumanInputSample plus explicit frame and event diagnostics."""

    human_input: HumanInputSample
    tcp_frame_normalized: np.ndarray
    base_frame_normalized: np.ndarray
    last_raw_motion: tuple[int, ...]
    last_motion_monotonic_seconds: Optional[float]
    event_age_seconds: Optional[float]
    button_events: tuple[TimedButtonEvent, ...]
    source_frame: str = TCP_FRAME
    target_frame: str = SAPS_FRAME


class SpnavBoundary(Protocol):
    """Small mockable boundary around the optional spnav Python package."""

    def open(self) -> None:
        """Open one client connection to spnavd."""

    def poll_event(self) -> MotionEvent | ButtonEvent | None:
        """Return one pending event without blocking."""

    def close(self) -> None:
        """Close the spnavd connection."""

    def monotonic(self) -> float:
        """Return local monotonic time."""

    def device_exists(self, path: str) -> bool:
        """Inspect, but never open, the configured physical device node."""


class PythonSpnavBoundary:
    """Runtime adapter for python-spnav and the spnavd Unix socket."""

    def __init__(self, module: Any | None = None) -> None:
        if module is None:
            import spnav as module
        self._module = module

    def open(self) -> None:
        self._module.spnav_open()

    def poll_event(self) -> MotionEvent | ButtonEvent | None:
        event = self._module.spnav_poll_event()
        if event is None:
            return None
        if isinstance(event, self._module.SpnavMotionEvent):
            return MotionEvent(
                translation=tuple(int(value) for value in event.translation),
                rotation=tuple(int(value) for value in event.rotation),
            )
        if isinstance(event, self._module.SpnavButtonEvent):
            return ButtonEvent(
                button=int(event.bnum),
                pressed=bool(event.press),
            )
        raise TypeError(f"Unexpected spnav event type: {type(event).__name__}.")

    def close(self) -> None:
        self._module.spnav_close()

    def monotonic(self) -> float:
        return time.monotonic()

    def device_exists(self, path: str) -> bool:
        return Path(path).exists()


def process_spnav_motion(
    raw_axes: Sequence[int],
    config: SpnavConfig,
) -> ProcessedSpnavMotion:
    """Apply the pinned lab normalize/deadzone/axis map without clipping."""

    raw = np.asarray(raw_axes)
    if raw.shape != (6,):
        raise ValueError(f"raw_axes must have shape (6,), received {raw.shape}.")
    if not np.issubdtype(raw.dtype, np.number):
        raise TypeError("raw_axes must be numeric.")
    numeric = np.asarray(raw, dtype=np.float64)
    if not np.all(np.isfinite(numeric)):
        raise ValueError("raw_axes must be finite.")
    normalized = numeric / config.maximum_raw_value
    deadzoned = normalized.copy()
    deadzoned[np.abs(deadzoned) < config.deadzone] = 0.0
    transform = np.asarray(
        [[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        dtype=np.float64,
    )
    mapped = np.concatenate(
        (transform @ deadzoned[:3], transform @ deadzoned[3:])
    )
    return ProcessedSpnavMotion(
        normalized_raw=_readonly(normalized),
        deadzoned_raw=_readonly(deadzoned),
        tcp_frame_normalized=_readonly(mapped),
    )


def rotate_normalized_cartesian_motion(
    tcp_frame_normalized: np.ndarray,
    base_rotation_tcp: np.ndarray,
) -> np.ndarray:
    """Resolve same-TCP-point linear/angular components in fr3_link0.

    Both M2 normalization blocks are isotropic, so
    ``S^-1 diag(R, R) S h_tcp = diag(R, R) h_tcp``. There is no
    translational adjoint term because the represented point remains the TCP.
    """

    motion = np.asarray(tcp_frame_normalized)
    rotation = np.asarray(base_rotation_tcp)
    if motion.shape != (6,):
        raise ValueError(
            "tcp_frame_normalized must have shape (6,), received "
            f"{motion.shape}."
        )
    if rotation.shape != (3, 3):
        raise ValueError(
            "base_rotation_tcp must have shape (3, 3), received "
            f"{rotation.shape}."
        )
    if not np.issubdtype(motion.dtype, np.floating):
        raise TypeError("tcp_frame_normalized must have a floating dtype.")
    if not np.issubdtype(rotation.dtype, np.floating):
        raise TypeError("base_rotation_tcp must have a floating dtype.")
    if not np.all(np.isfinite(motion)) or not np.all(np.isfinite(rotation)):
        raise ValueError("motion and rotation must be finite.")
    identity = rotation.T @ rotation
    determinant = float(np.linalg.det(rotation))
    if not np.allclose(identity, np.eye(3), atol=1e-8, rtol=1e-8) or not np.isclose(
        determinant,
        1.0,
        atol=1e-8,
        rtol=1e-8,
    ):
        raise ValueError("base_rotation_tcp must be a proper rotation matrix.")
    result = np.concatenate((rotation @ motion[:3], rotation @ motion[3:]))
    return _readonly(result)


def gripper_command_from_buttons(
    *,
    open_pressed: bool,
    close_pressed: bool,
) -> float:
    """Return explicit open=-1, neutral=0, close=+1 intent."""

    if close_pressed:
        return 1.0
    if open_pressed:
        return -1.0
    return 0.0


class SpnavHumanInputBackend:
    """Poll spnavd and expose base-frame HumanInputSample data only."""

    def __init__(
        self,
        config: SpnavConfig = SpnavConfig(),
        *,
        boundary: SpnavBoundary | None = None,
    ) -> None:
        self.config = config
        self._boundary = boundary or PythonSpnavBoundary()
        self._connected = False
        self._error: Optional[str] = None
        self._raw_axes = (0,) * 6
        self._last_motion_monotonic: Optional[float] = None
        self._last_event_monotonic: Optional[float] = None
        self._button_states: dict[int, bool] = {
            config.open_button: False,
            config.close_button: False,
        }

    @property
    def connected(self) -> bool:
        return self._connected

    def start(self) -> None:
        """Connect to spnavd; this opens no evdev node and publishes nothing."""

        if self._connected:
            raise RuntimeError("spnavd backend is already started.")
        try:
            self._boundary.open()
        except Exception as error:
            self._error = str(error)
            raise RuntimeError(f"Could not connect to spnavd: {error}") from error
        self._connected = True
        self._error = None
        self._raw_axes = (0,) * 6
        self._last_motion_monotonic = None
        self._last_event_monotonic = None
        for button in self._button_states:
            self._button_states[button] = False

    def sample(
        self,
        *,
        base_rotation_tcp: np.ndarray,
        armed: bool = False,
        abort_requested: bool = False,
    ) -> SpnavSapsSample:
        """Drain events and zero motion once the last real event is stale."""

        button_events: list[TimedButtonEvent] = []
        if self._connected:
            try:
                while True:
                    event = self._boundary.poll_event()
                    if event is None:
                        break
                    now = self._boundary.monotonic()
                    self._last_event_monotonic = now
                    if isinstance(event, MotionEvent):
                        self._raw_axes = event.translation + event.rotation
                        self._last_motion_monotonic = now
                    elif isinstance(event, ButtonEvent):
                        self._button_states[event.button] = event.pressed
                        button_events.append(
                            TimedButtonEvent(
                                button=event.button,
                                pressed=event.pressed,
                                receive_monotonic_seconds=now,
                            )
                        )
                    else:
                        raise TypeError(
                            f"Unexpected boundary event {type(event).__name__}."
                        )
            except Exception as error:
                self._error = str(error)
                self._connected = False
                for button in self._button_states:
                    self._button_states[button] = False
                try:
                    self._boundary.close()
                except Exception:
                    pass

        now = self._boundary.monotonic()
        event_age = (
            None
            if self._last_motion_monotonic is None
            else now - self._last_motion_monotonic
        )
        stale = (
            event_age is None
            or event_age > self.config.stale_timeout_seconds
        )
        processed = process_spnav_motion(self._raw_axes, self.config)
        physical_connected = self._boundary.device_exists(
            self.config.device_path
        )
        tcp_motion = (
            np.zeros(6, dtype=np.float64)
            if not self._connected or not physical_connected or stale
            else processed.tcp_frame_normalized
        )
        base_motion = rotate_normalized_cartesian_motion(
            np.asarray(tcp_motion, dtype=np.float64),
            base_rotation_tcp,
        )
        open_pressed = self._button_states.get(
            self.config.open_button,
            False,
        )
        close_pressed = self._button_states.get(
            self.config.close_button,
            False,
        )
        gripper_command = gripper_command_from_buttons(
            open_pressed=open_pressed,
            close_pressed=close_pressed,
        )
        action = np.concatenate(
            (base_motion, np.asarray([gripper_command], dtype=np.float64))
        ).astype(np.float32)
        human_input = HumanInputSample(
            action=action,
            motion_active=bool(
                np.linalg.norm(base_motion) > self.config.idle_threshold
            ),
            connected=self._connected,
            armed=bool(armed),
            abort_requested=bool(abort_requested),
            pressed_keys=(),
            gripper_command=gripper_command,
            speed_mode="analog",
            translation_gain=1.0,
            rotation_gain=1.0,
            sample_monotonic_seconds=now,
            last_event_monotonic_seconds=self._last_event_monotonic,
            input_source="spnavd",
            physical_device_connected=physical_connected,
            selected_device_name="3Dconnexion SpaceMouse Wireless",
            selected_device_path=self.config.device_path,
            raw_axes=self._raw_axes,
            mapped_axes=tuple(
                float(value) for value in processed.tcp_frame_normalized
            ),
            deadzone=self.config.deadzone,
            axis_mapping=SPNAV_AXIS_MAPPING,
            axis_signs=(-1.0, 1.0, 1.0, -1.0, 1.0, 1.0),
            axis_maxima=(self.config.maximum_raw_value,) * 6,
            axis_scales=(1.0,) * 6,
            axis_enabled=(True,) * 6,
            stale_input=stale,
            stale_input_timeout_seconds=self.config.stale_timeout_seconds,
            open_button=self.config.open_button,
            close_button=self.config.close_button,
            open_button_pressed=open_pressed,
            close_button_pressed=close_pressed,
            native_event_timestamp_seconds=None,
            physical_device_error=self._error,
            calibration_status="pinned igd_fr3_control spnav mapping",
        )
        return SpnavSapsSample(
            human_input=human_input,
            tcp_frame_normalized=_readonly(tcp_motion),
            base_frame_normalized=base_motion,
            last_raw_motion=self._raw_axes,
            last_motion_monotonic_seconds=self._last_motion_monotonic,
            event_age_seconds=event_age,
            button_events=tuple(button_events),
        )

    def close(self) -> None:
        """Close only the spnavd client connection."""

        try:
            if self._connected:
                self._boundary.close()
        finally:
            self._connected = False
            self._raw_axes = (0,) * 6
            self._last_motion_monotonic = None
            self._last_event_monotonic = None
            for button in self._button_states:
                self._button_states[button] = False


def _readonly(value: np.ndarray) -> np.ndarray:
    result = np.array(value, dtype=np.float64, copy=True)
    result.setflags(write=False)
    return result
