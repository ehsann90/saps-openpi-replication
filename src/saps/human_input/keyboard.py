"""Keyboard mapping for seven-dimensional LIBERO control."""

from __future__ import annotations

import dataclasses
import time
from typing import Any
from typing import Iterable
from typing import Optional
from typing import Tuple

import numpy as np


SPEED_MODES = ("fine", "normal", "fast")

MOTION_KEYS = frozenset(
    {
        # Camera-relative translation.
        "w",
        "a",
        "s",
        "d",
        "space",
        "shift",
        # Orientation.
        "q",
        "e",
        "arrowup",
        "arrowdown",
        "arrowleft",
        "arrowright",
    }
)

GRIPPER_KEYS = frozenset({"z", "x"})
ALLOWED_KEYS = MOTION_KEYS | GRIPPER_KEYS


@dataclasses.dataclass(frozen=True)
class SpeedProfile:
    translation_gain: float
    rotation_gain: float


@dataclasses.dataclass(frozen=True)
class HumanInputSample:
    """One snapshot of the operator command state."""

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
        }


class KeyboardActionMapper:
    """Map keyboard state to a normalized LIBERO action.

    Action convention:

        [dx, dy, dz, droll, dpitch, dyaw, gripper]

    The displayed agent image is rotated by 180 degrees relative to the
    raw MuJoCo image. In the displayed view:

        screen up    approximately corresponds to world +x
        screen right approximately corresponds to world -y

    Therefore the camera-relative planar mapping is:

        W/S -> +x/-x
        A/D -> -y/+y
    """

    def __init__(
        self,
        *,
        fine_translation_gain: float = 0.07,
        normal_translation_gain: float = 0.14,
        fast_translation_gain: float = 0.25,
        fine_rotation_gain: float = 0.10,
        normal_rotation_gain: float = 0.18,
        fast_rotation_gain: float = 0.30,
        default_speed_mode: str = "fine",
        idle_threshold: float = 1e-3,
    ) -> None:
        self._speed_profiles = {
            "fine": SpeedProfile(
                translation_gain=float(
                    fine_translation_gain
                ),
                rotation_gain=float(
                    fine_rotation_gain
                ),
            ),
            "normal": SpeedProfile(
                translation_gain=float(
                    normal_translation_gain
                ),
                rotation_gain=float(
                    normal_rotation_gain
                ),
            ),
            "fast": SpeedProfile(
                translation_gain=float(
                    fast_translation_gain
                ),
                rotation_gain=float(
                    fast_rotation_gain
                ),
            ),
        }

        for mode, profile in self._speed_profiles.items():
            if not 0.0 < profile.translation_gain <= 1.0:
                raise ValueError(
                    f"{mode} translation gain must be "
                    "within (0, 1]."
                )

            if not 0.0 < profile.rotation_gain <= 1.0:
                raise ValueError(
                    f"{mode} rotation gain must be "
                    "within (0, 1]."
                )

        if default_speed_mode not in SPEED_MODES:
            raise ValueError(
                f"Unknown default speed mode "
                f"{default_speed_mode!r}."
            )

        if idle_threshold < 0.0:
            raise ValueError(
                "idle_threshold must be non-negative."
            )

        self.default_speed_mode = default_speed_mode
        self.idle_threshold = float(idle_threshold)

    @staticmethod
    def _normalize_to_gain(
        vector: np.ndarray,
        gain: float,
    ) -> np.ndarray:
        norm = float(np.linalg.norm(vector))

        if norm > 0.0:
            vector = vector * (gain / norm)

        return vector

    def sample(
        self,
        *,
        pressed_keys: Iterable[str],
        gripper_command: float,
        speed_mode: str,
        connected: bool,
        armed: bool,
        abort_requested: bool,
        last_event_monotonic_seconds: Optional[float],
    ) -> HumanInputSample:
        if speed_mode not in self._speed_profiles:
            raise ValueError(
                f"Unknown speed mode {speed_mode!r}."
            )

        profile = self._speed_profiles[speed_mode]

        keys = {
            str(key).lower()
            for key in pressed_keys
            if str(key).lower() in ALLOWED_KEYS
        }

        translation = np.zeros(
            3,
            dtype=np.float32,
        )
        rotation = np.zeros(
            3,
            dtype=np.float32,
        )

        if connected and armed and not abort_requested:
            # Camera-relative translation in the displayed agent view.
            translation[0] = float("w" in keys) - float(
                "s" in keys
            )
            translation[1] = float("d" in keys) - float(
                "a" in keys
            )
            translation[2] = float(
                "space" in keys
            ) - float("shift" in keys)

            # Orientation commands.
            rotation[0] = float(
                "arrowright" in keys
            ) - float("arrowleft" in keys)

            rotation[1] = float(
                "arrowup" in keys
            ) - float("arrowdown" in keys)

            rotation[2] = float("q" in keys) - float(
                "e" in keys
            )

            translation = self._normalize_to_gain(
                translation,
                profile.translation_gain,
            )
            rotation = self._normalize_to_gain(
                rotation,
                profile.rotation_gain,
            )

        action = np.concatenate(
            (
                translation,
                rotation,
                np.asarray(
                    [
                        np.clip(
                            gripper_command,
                            -1.0,
                            1.0,
                        )
                    ],
                    dtype=np.float32,
                ),
            )
        ).astype(np.float32)

        action = np.clip(
            action,
            -1.0,
            1.0,
        ).astype(np.float32)

        motion_active = bool(
            np.linalg.norm(action[:6])
            > self.idle_threshold
        )

        return HumanInputSample(
            action=action,
            motion_active=motion_active,
            connected=bool(connected),
            armed=bool(armed),
            abort_requested=bool(abort_requested),
            pressed_keys=tuple(sorted(keys)),
            gripper_command=float(action[6]),
            speed_mode=speed_mode,
            translation_gain=(
                profile.translation_gain
            ),
            rotation_gain=profile.rotation_gain,
            sample_monotonic_seconds=time.monotonic(),
            last_event_monotonic_seconds=(
                last_event_monotonic_seconds
            ),
        )
