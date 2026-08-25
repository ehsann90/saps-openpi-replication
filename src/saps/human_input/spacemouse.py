"""Native Linux SpaceMouse input and normalized action processing."""

from __future__ import annotations

import dataclasses
import errno
import fcntl
import glob
import logging
import os
import struct
import threading
import time
from typing import Optional
from typing import Sequence

import numpy as np


LOGGER = logging.getLogger(__name__)

EV_SYN = 0
EV_KEY = 1
EV_ABS = 3

ABS_X = 0
ABS_Y = 1
ABS_Z = 2
ABS_RX = 3
ABS_RY = 4
ABS_RZ = 5

AXIS_NAMES = (
    "ABS_X",
    "ABS_Y",
    "ABS_Z",
    "ABS_RX",
    "ABS_RY",
    "ABS_RZ",
)
AXIS_CODES = (ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ)
AXIS_NAME_TO_INDEX = {
    name: index for index, name in enumerate(AXIS_NAMES)
}
REQUIRED_AXIS_CODES = frozenset(AXIS_CODES)

DEFAULT_DEVICE_GLOB = (
    "/dev/input/by-id/*-event-joystick"
)
FALLBACK_DEVICE_GLOB = "/dev/input/event*"

_INPUT_EVENT = struct.Struct("llHHi")
_ABS_INFO = struct.Struct("iiiiii")

_IOC_NRBITS = 8
_IOC_TYPEBITS = 8
_IOC_SIZEBITS = 14
_IOC_NRSHIFT = 0
_IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS
_IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS
_IOC_DIRSHIFT = _IOC_SIZESHIFT + _IOC_SIZEBITS
_IOC_WRITE = 1
_IOC_READ = 2


def _ioc(direction: int, type_value: int, number: int, size: int) -> int:
    return (
        (direction << _IOC_DIRSHIFT)
        | (type_value << _IOC_TYPESHIFT)
        | (number << _IOC_NRSHIFT)
        | (size << _IOC_SIZESHIFT)
    )


def _eviocgbit(event_type: int, length: int) -> int:
    return _ioc(
        _IOC_READ,
        ord("E"),
        0x20 + event_type,
        length,
    )


def _eviocgabs(axis_code: int) -> int:
    return _ioc(
        _IOC_READ,
        ord("E"),
        0x40 + axis_code,
        _ABS_INFO.size,
    )


def _eviocgname(length: int) -> int:
    return _ioc(_IOC_READ, ord("E"), 0x06, length)


EVIOCGRAB = _ioc(_IOC_WRITE, ord("E"), 0x90, struct.calcsize("i"))


class SpaceMouseUnavailableError(RuntimeError):
    """Raised when no exclusively usable six-axis input node exists."""


@dataclasses.dataclass(frozen=True)
class SpaceMouseConfig:
    """Configuration for SpaceMouse discovery and action processing."""

    device_path: str = ""
    translation_gain: float = 0.14
    rotation_gain: float = 0.18
    deadzone: float = 0.08
    axis_mapping: tuple[str, ...] = AXIS_NAMES
    axis_signs: tuple[float, ...] = (1.0,) * 6
    axis_maxima: tuple[float, ...] = (350.0,) * 6
    axis_scales: tuple[float, ...] = (1.0,) * 6
    axis_enabled: tuple[bool, ...] = (True,) * 6
    stale_input_timeout_seconds: float = 0.25
    open_button: int = 256
    close_button: int = 257
    reconnect_interval_seconds: float = 1.0
    idle_threshold: float = 1e-3

    def __post_init__(self) -> None:
        if not 0.0 < self.translation_gain <= 1.0:
            raise ValueError(
                "translation_gain must be within (0, 1]."
            )
        if not 0.0 < self.rotation_gain <= 1.0:
            raise ValueError(
                "rotation_gain must be within (0, 1]."
            )
        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError("deadzone must be within [0, 1).")
        if len(self.axis_mapping) != 6 or set(
            self.axis_mapping
        ) != set(AXIS_NAMES):
            raise ValueError(
                "axis_mapping must contain each of "
                f"{AXIS_NAMES} exactly once."
            )
        if len(self.axis_signs) != 6 or any(
            sign not in {-1.0, 1.0}
            for sign in self.axis_signs
        ):
            raise ValueError(
                "axis_signs must contain six values, each -1 or 1."
            )
        if len(self.axis_maxima) != 6 or any(
            not np.isfinite(maximum) or maximum <= 0.0
            for maximum in self.axis_maxima
        ):
            raise ValueError(
                "axis_maxima must contain six positive values."
            )
        if len(self.axis_scales) != 6 or any(
            not np.isfinite(scale) or not 0.0 <= scale <= 10.0
            for scale in self.axis_scales
        ):
            raise ValueError(
                "axis_scales must contain six finite values within "
                "[0, 10]."
            )
        if len(self.axis_enabled) != 6 or any(
            not isinstance(enabled, bool)
            for enabled in self.axis_enabled
        ):
            raise ValueError(
                "axis_enabled must contain six boolean values."
            )
        if (
            not np.isfinite(self.stale_input_timeout_seconds)
            or self.stale_input_timeout_seconds <= 0.0
        ):
            raise ValueError(
                "stale_input_timeout_seconds must be positive."
            )
        if (
            not np.isfinite(self.reconnect_interval_seconds)
            or self.reconnect_interval_seconds < 0.0
        ):
            raise ValueError(
                "reconnect_interval_seconds must be non-negative."
            )
        if self.open_button < 0 or self.close_button < 0:
            raise ValueError("button codes must be non-negative.")
        if self.open_button == self.close_button:
            raise ValueError(
                "open_button and close_button must differ."
            )
        if (
            not np.isfinite(self.idle_threshold)
            or self.idle_threshold < 0.0
        ):
            raise ValueError(
                "idle_threshold must be non-negative."
            )


@dataclasses.dataclass(frozen=True)
class DeviceInfo:
    """One inspected Linux input node."""

    path: str
    name: str
    axis_codes: frozenset[int]
    axis_ranges: tuple[tuple[int, int], ...]


@dataclasses.dataclass(frozen=True)
class ProcessedAxes:
    """Intermediate and final values from the analog pipeline."""

    normalized_axes: tuple[float, ...]
    deadzone_axes: tuple[float, ...]
    mapped_axes: tuple[float, ...]
    final_motion: tuple[float, ...]


@dataclasses.dataclass(frozen=True)
class SpaceMouseState:
    """Latest physical-device state and processed motion."""

    connected: bool
    device_name: Optional[str]
    device_path: Optional[str]
    raw_axes: tuple[int, ...]
    mapped_axes: tuple[float, ...]
    final_motion: tuple[float, ...]
    motion_active: bool
    stale: bool
    open_button_pressed: bool
    close_button_pressed: bool
    native_event_timestamp_seconds: Optional[float]
    last_event_monotonic_seconds: Optional[float]
    error: Optional[str]


def parse_axis_mapping(value: str) -> tuple[str, ...]:
    """Parse a comma-separated mapping in application-axis order."""

    return tuple(
        item.strip().upper()
        for item in value.split(",")
        if item.strip()
    )


def parse_axis_signs(value: str) -> tuple[float, ...]:
    """Parse six comma-separated axis signs."""

    try:
        return tuple(
            float(item.strip())
            for item in value.split(",")
            if item.strip()
        )
    except ValueError as error:
        raise ValueError(
            "axis signs must be comma-separated numbers."
        ) from error


def parse_axis_maxima(value: str) -> tuple[float, ...]:
    """Parse six comma-separated positive raw-axis maxima."""

    try:
        return tuple(
            float(item.strip())
            for item in value.split(",")
            if item.strip()
        )
    except ValueError as error:
        raise ValueError(
            "axis maxima must be comma-separated numbers."
        ) from error


def _rescaled_deadzone(values: np.ndarray, deadzone: float) -> np.ndarray:
    magnitudes = np.abs(values)
    outside = magnitudes > deadzone
    result = np.zeros(6, dtype=np.float64)
    result[outside] = (
        np.sign(values[outside])
        * (magnitudes[outside] - deadzone)
        / (1.0 - deadzone)
    )
    return result


def process_spacemouse_axes(
    raw_axes: Sequence[int],
    config: SpaceMouseConfig,
) -> ProcessedAxes:
    """Apply normalize, clip, deadzone, map, gain, and final clip."""

    raw = np.asarray(raw_axes, dtype=np.float64)
    if raw.shape != (6,):
        raise ValueError(
            f"raw_axes must have shape (6,), received {raw.shape}."
        )

    maxima = np.asarray(config.axis_maxima, dtype=np.float64)
    normalized = np.clip(raw / maxima, -1.0, 1.0)
    deadzoned = _rescaled_deadzone(normalized, config.deadzone)

    mapping_indices = np.asarray(
        [AXIS_NAME_TO_INDEX[name] for name in config.axis_mapping],
        dtype=np.int64,
    )
    signs = np.asarray(config.axis_signs, dtype=np.float64)
    mapped = deadzoned[mapping_indices] * signs

    gained = mapped.copy()
    gained[:3] *= config.translation_gain
    gained[3:] *= config.rotation_gain
    gained *= np.asarray(config.axis_scales, dtype=np.float64)
    gained *= np.asarray(config.axis_enabled, dtype=np.float64)
    final = np.clip(gained, -1.0, 1.0)

    return ProcessedAxes(
        normalized_axes=tuple(float(value) for value in normalized),
        deadzone_axes=tuple(float(value) for value in deadzoned),
        mapped_axes=tuple(float(value) for value in mapped),
        final_motion=tuple(float(value) for value in final),
    )


class LinuxInputBoundary:
    """Small mockable boundary around Linux evdev system calls."""

    def glob(self, pattern: str) -> list[str]:
        return sorted(glob.glob(pattern))

    def open(self, path: str) -> int:
        flags = os.O_RDONLY | os.O_NONBLOCK
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        return os.open(path, flags)

    def close(self, file_descriptor: int) -> None:
        os.close(file_descriptor)

    def read(self, file_descriptor: int, size: int) -> bytes:
        return os.read(file_descriptor, size)

    def grab(self, file_descriptor: int, enabled: bool) -> None:
        fcntl.ioctl(file_descriptor, EVIOCGRAB, int(enabled))

    def device_name(self, file_descriptor: int) -> str:
        buffer = bytearray(256)
        fcntl.ioctl(
            file_descriptor,
            _eviocgname(len(buffer)),
            buffer,
            True,
        )
        return bytes(buffer).split(b"\0", 1)[0].decode(
            "utf-8",
            errors="replace",
        )

    def axis_codes(self, file_descriptor: int) -> frozenset[int]:
        buffer = bytearray(8)
        fcntl.ioctl(
            file_descriptor,
            _eviocgbit(EV_ABS, len(buffer)),
            buffer,
            True,
        )
        return frozenset(
            code
            for code in range(len(buffer) * 8)
            if buffer[code // 8] & (1 << (code % 8))
        )

    def axis_range(
        self,
        file_descriptor: int,
        axis_code: int,
    ) -> tuple[int, int]:
        buffer = bytearray(_ABS_INFO.size)
        fcntl.ioctl(
            file_descriptor,
            _eviocgabs(axis_code),
            buffer,
            True,
        )
        _, minimum, maximum, _, _, _ = _ABS_INFO.unpack(buffer)
        return minimum, maximum

    def realpath(self, path: str) -> str:
        return os.path.realpath(path)

    def monotonic(self) -> float:
        return time.monotonic()


def inspect_device(
    path: str,
    *,
    boundary: LinuxInputBoundary,
) -> DeviceInfo:
    """Inspect one node without claiming exclusive ownership."""

    file_descriptor = boundary.open(path)
    try:
        axis_codes = boundary.axis_codes(file_descriptor)
        axis_ranges = tuple(
            boundary.axis_range(file_descriptor, code)
            for code in AXIS_CODES
            if code in axis_codes
        )
        return DeviceInfo(
            path=path,
            name=boundary.device_name(file_descriptor),
            axis_codes=axis_codes,
            axis_ranges=axis_ranges,
        )
    finally:
        boundary.close(file_descriptor)


def select_spacemouse_device(
    *,
    explicit_path: str = "",
    boundary: Optional[LinuxInputBoundary] = None,
) -> DeviceInfo:
    """Select explicit, by-id, then capability-matched event input."""

    system = boundary or LinuxInputBoundary()
    groups: list[tuple[str, list[str]]]

    if explicit_path:
        groups = [("explicit", [explicit_path])]
    else:
        by_id = system.glob(DEFAULT_DEVICE_GLOB)
        fallback = [
            path
            for path in system.glob(FALLBACK_DEVICE_GLOB)
            if path not in set(by_id)
        ]
        groups = [("by-id", by_id), ("capability", fallback)]

    inspected_any = False
    errors: list[str] = []

    for group_name, paths in groups:
        for path in paths:
            try:
                info = inspect_device(path, boundary=system)
                inspected_any = True
            except OSError as error:
                errors.append(f"{path}: {error}")
                continue

            missing = REQUIRED_AXIS_CODES.difference(info.axis_codes)
            if not missing:
                return info

            missing_names = [
                AXIS_NAMES[AXIS_CODES.index(code)]
                for code in sorted(missing)
            ]
            errors.append(
                f"{path}: missing required axes {missing_names}"
            )

        if explicit_path:
            break

    if explicit_path and inspected_any:
        detail = errors[-1] if errors else explicit_path
        raise SpaceMouseUnavailableError(
            "Configured SpaceMouse device is not a six-axis motion "
            f"node: {detail}"
        )

    suffix = f" Inspected: {'; '.join(errors)}" if errors else ""
    raise SpaceMouseUnavailableError(
        "No usable Linux input node exposing ABS_X, ABS_Y, ABS_Z, "
        f"ABS_RX, ABS_RY, and ABS_RZ was found.{suffix}"
    )


def _ownership_diagnostic(path: str, boundary: LinuxInputBoundary) -> str:
    event_path = boundary.realpath(path)
    return (
        "Exclusive SpaceMouse access failed. Another process may own "
        "the event node (spacenavd is a possible owner). Inspect it "
        f"with: sudo fuser -v {event_path}"
    )


class SpaceMouseBackend:
    """Own and sample one six-axis SpaceMouse Linux event node."""

    def __init__(
        self,
        config: SpaceMouseConfig,
        *,
        boundary: Optional[LinuxInputBoundary] = None,
    ) -> None:
        self.config = config
        self._boundary = boundary or LinuxInputBoundary()
        self._lock = threading.RLock()
        self._file_descriptor: Optional[int] = None
        self._device_info: Optional[DeviceInfo] = None
        self._grabbed = False
        self._raw_axes = [0] * 6
        self._button_states = {
            config.open_button: False,
            config.close_button: False,
        }
        self._event_buffer = b""
        self._last_axis_event_monotonic: Optional[float] = None
        self._last_event_monotonic: Optional[float] = None
        self._native_event_timestamp: Optional[float] = None
        self._last_error: Optional[str] = None
        self._next_reconnect_monotonic = 0.0

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._file_descriptor is not None

    @property
    def device_info(self) -> Optional[DeviceInfo]:
        with self._lock:
            return self._device_info

    def start(self) -> DeviceInfo:
        """Discover, validate, and exclusively acquire the device."""

        with self._lock:
            if self._file_descriptor is not None:
                raise RuntimeError("SpaceMouse backend is already started.")
            return self._connect()

    def update_calibration(
        self,
        *,
        axis_mapping: tuple[str, ...],
        axis_signs: tuple[float, ...],
        translation_gain: float,
        rotation_gain: float,
        deadzone: float,
        axis_scales: tuple[float, ...],
        axis_enabled: tuple[bool, ...],
    ) -> SpaceMouseConfig:
        """Atomically replace processing-only calibration values."""

        with self._lock:
            updated = dataclasses.replace(
                self.config,
                axis_mapping=axis_mapping,
                axis_signs=axis_signs,
                translation_gain=translation_gain,
                rotation_gain=rotation_gain,
                deadzone=deadzone,
                axis_scales=axis_scales,
                axis_enabled=axis_enabled,
            )
            self.config = updated
            # A remap must never reinterpret a pre-apply raw command.
            # Require a fresh axis event after the operator re-arms.
            self._raw_axes = [0] * 6
            self._last_axis_event_monotonic = None
            return updated

    def _connect(self) -> DeviceInfo:
        selected_path = self.config.device_path
        try:
            info = select_spacemouse_device(
                explicit_path=self.config.device_path,
                boundary=self._boundary,
            )
            selected_path = info.path
            file_descriptor = self._boundary.open(info.path)
            try:
                axis_codes = self._boundary.axis_codes(file_descriptor)
                missing = REQUIRED_AXIS_CODES.difference(axis_codes)
                if missing:
                    raise SpaceMouseUnavailableError(
                        f"Selected node {info.path} lost required axes "
                        f"before acquisition: {sorted(missing)}"
                    )
                self._boundary.grab(file_descriptor, True)
            except Exception:
                self._boundary.close(file_descriptor)
                raise

            self._file_descriptor = file_descriptor
            self._device_info = info
            self._grabbed = True
            self._raw_axes = [0] * 6
            self._button_states = {
                self.config.open_button: False,
                self.config.close_button: False,
            }
            self._event_buffer = b""
            self._last_axis_event_monotonic = None
            self._last_event_monotonic = None
            self._native_event_timestamp = None
            self._last_error = None
            LOGGER.info(
                "Acquired SpaceMouse input %s (%s) with EVIOCGRAB.",
                info.path,
                info.name,
            )
            return info
        except (OSError, SpaceMouseUnavailableError) as error:
            path = (
                selected_path
                or (
                    self._device_info.path
                    if self._device_info is not None
                    else "/dev/input/<event-node>"
                )
            )
            if isinstance(error, OSError) and error.errno in {
                errno.EACCES,
                errno.EBUSY,
                errno.EPERM,
            }:
                message = (
                    f"{error}. "
                    + _ownership_diagnostic(path, self._boundary)
                )
            else:
                message = str(error)
            self._last_error = message
            self._next_reconnect_monotonic = (
                self._boundary.monotonic()
                + self.config.reconnect_interval_seconds
            )
            raise SpaceMouseUnavailableError(message) from error

    def _disconnect(self, message: str) -> None:
        file_descriptor = self._file_descriptor
        if file_descriptor is not None:
            if self._grabbed:
                try:
                    self._boundary.grab(file_descriptor, False)
                except OSError:
                    LOGGER.debug(
                        "Could not release failed SpaceMouse grab.",
                        exc_info=True,
                    )
            try:
                self._boundary.close(file_descriptor)
            except OSError:
                LOGGER.debug(
                    "Could not close failed SpaceMouse node.",
                    exc_info=True,
                )

        self._file_descriptor = None
        self._grabbed = False
        self._raw_axes = [0] * 6
        for button in self._button_states:
            self._button_states[button] = False
        self._event_buffer = b""
        self._last_axis_event_monotonic = None
        self._last_event_monotonic = None
        self._native_event_timestamp = None
        self._last_error = message
        self._next_reconnect_monotonic = (
            self._boundary.monotonic()
            + self.config.reconnect_interval_seconds
        )
        LOGGER.error("SpaceMouse disconnected: %s", message)

    def _drain_events(self) -> None:
        file_descriptor = self._file_descriptor
        if file_descriptor is None:
            return

        while True:
            try:
                data = self._boundary.read(
                    file_descriptor,
                    _INPUT_EVENT.size * 64,
                )
            except BlockingIOError:
                break
            except OSError as error:
                if error.errno in {
                    errno.EAGAIN,
                    errno.EWOULDBLOCK,
                }:
                    break
                self._disconnect(str(error))
                return

            if not data:
                self._disconnect("event device returned end-of-file")
                return

            self._event_buffer += data
            while len(self._event_buffer) >= _INPUT_EVENT.size:
                packet = self._event_buffer[: _INPUT_EVENT.size]
                self._event_buffer = self._event_buffer[
                    _INPUT_EVENT.size :
                ]
                seconds, microseconds, event_type, code, value = (
                    _INPUT_EVENT.unpack(packet)
                )
                now = self._boundary.monotonic()

                if event_type == EV_ABS and code in REQUIRED_AXIS_CODES:
                    self._raw_axes[AXIS_CODES.index(code)] = int(value)
                    self._last_axis_event_monotonic = now
                elif event_type == EV_KEY and code in self._button_states:
                    self._button_states[code] = value != 0
                else:
                    continue

                self._last_event_monotonic = now
                self._native_event_timestamp = (
                    float(seconds) + float(microseconds) / 1_000_000.0
                )

    def sample(self) -> SpaceMouseState:
        """Return current state, safely zeroing stale or lost motion."""

        with self._lock:
            now = self._boundary.monotonic()
            if (
                self._file_descriptor is None
                and now >= self._next_reconnect_monotonic
            ):
                try:
                    self._connect()
                except SpaceMouseUnavailableError:
                    pass

            self._drain_events()
            now = self._boundary.monotonic()
            connected = self._file_descriptor is not None
            stale = (
                self._last_axis_event_monotonic is None
                or now - self._last_axis_event_monotonic
                > self.config.stale_input_timeout_seconds
            )
            processed = process_spacemouse_axes(
                self._raw_axes,
                self.config,
            )
            final_motion = (
                (0.0,) * 6
                if not connected or stale
                else processed.final_motion
            )
            motion_active = bool(
                np.linalg.norm(final_motion)
                > self.config.idle_threshold
            )
            info = self._device_info

            return SpaceMouseState(
                connected=connected,
                device_name=info.name if info is not None else None,
                device_path=info.path if info is not None else None,
                raw_axes=tuple(self._raw_axes),
                mapped_axes=processed.mapped_axes,
                final_motion=tuple(final_motion),
                motion_active=motion_active,
                stale=stale,
                open_button_pressed=self._button_states.get(
                    self.config.open_button,
                    False,
                ),
                close_button_pressed=self._button_states.get(
                    self.config.close_button,
                    False,
                ),
                native_event_timestamp_seconds=(
                    self._native_event_timestamp
                ),
                last_event_monotonic_seconds=(
                    self._last_event_monotonic
                ),
                error=self._last_error,
            )

    def close(self) -> None:
        """Release EVIOCGRAB and close the event node."""

        with self._lock:
            file_descriptor = self._file_descriptor
            if file_descriptor is None:
                return
            try:
                if self._grabbed:
                    try:
                        self._boundary.grab(file_descriptor, False)
                    except OSError:
                        LOGGER.warning(
                            "Could not release SpaceMouse EVIOCGRAB.",
                            exc_info=True,
                        )
            finally:
                try:
                    self._boundary.close(file_descriptor)
                except OSError:
                    LOGGER.warning(
                        "Could not close SpaceMouse event node.",
                        exc_info=True,
                    )
                self._file_descriptor = None
                self._grabbed = False
                self._raw_axes = [0] * 6
                for button in self._button_states:
                    self._button_states[button] = False
