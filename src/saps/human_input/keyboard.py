"""Keyboard mapping for seven-dimensional LIBERO control."""

from __future__ import annotations

import dataclasses
import time
from typing import Any
from typing import Iterable
from typing import Optional
from typing import Tuple

import numpy as np


MOTION_KEYS = frozenset(
    {
        # Cartesian translation.
        "a",
        "d",
        "w",
        "s",
        "r",
        "f",
        # Cartesian orientation.
        "u",
        "o",
        "i",
        "k",
        "j",
        "l",
    }
)

GRIPPER_KEYS = frozenset({"z", "x"})
ALLOWED_KEYS = MOTION_KEYS | GRIPPER_KEYS


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
            "sample_monotonic_seconds": (
                self.sample_monotonic_seconds
            ),
            "last_event_monotonic_seconds": (
                self.last_event_monotonic_seconds
            ),
        }


class KeyboardActionMapper:
    """Map held keyboard keys to a normalized LIBERO action.

    Action convention:

        [dx, dy, dz, droll, dpitch, dyaw, gripper]

    The first six dimensions describe Cartesian motion. The final
    dimension is -1 for open and +1 for close.
    """

    def __init__(
        self,
        *,
        translation_gain: float = 0.35,
        rotation_gain: float = 0.35,
        idle_threshold: float = 1e-3,
    ) -> None:
        if not 0.0 < translation_gain <= 1.0:
            raise ValueError(
                "translation_gain must be within (0, 1]."
            )

        if not 0.0 < rotation_gain <= 1.0:
            raise ValueError(
                "rotation_gain must be within (0, 1]."
            )

        if idle_threshold < 0.0:
            raise ValueError(
                "idle_threshold must be non-negative."
            )

        self.translation_gain = float(translation_gain)
        self.rotation_gain = float(rotation_gain)
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
        connected: bool,
        armed: bool,
        abort_requested: bool,
        last_event_monotonic_seconds: Optional[float],
    ) -> HumanInputSample:
        keys = {
            str(key).lower()
            for key in pressed_keys
            if str(key).lower() in ALLOWED_KEYS
        }

        translation = np.zeros(3, dtype=np.float32)
        rotation = np.zeros(3, dtype=np.float32)

        # Motion commands are disabled until the operator explicitly
        # arms the interface.
        if connected and armed and not abort_requested:
            # Base-frame translation:
            # D/A: +x/-x
            # W/S: +y/-y
            # R/F: +z/-z
            translation[0] = float("d" in keys) - float(
                "a" in keys
            )
            translation[1] = float("w" in keys) - float(
                "s" in keys
            )
            translation[2] = float("r" in keys) - float(
                "f" in keys
            )

            # Axis-angle rotation:
            # O/U: +roll/-roll
            # I/K: +pitch/-pitch
            # L/J: +yaw/-yaw
            rotation[0] = float("o" in keys) - float(
                "u" in keys
            )
            rotation[1] = float("i" in keys) - float(
                "k" in keys
            )
            rotation[2] = float("l" in keys) - float(
                "j" in keys
            )

            translation = self._normalize_to_gain(
                translation,
                self.translation_gain,
            )
            rotation = self._normalize_to_gain(
                rotation,
                self.rotation_gain,
            )

        action = np.concatenate(
            (
                translation,
                rotation,
                np.asarray(
                    [np.clip(gripper_command, -1.0, 1.0)],
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
            sample_monotonic_seconds=time.monotonic(),
            last_event_monotonic_seconds=(
                last_event_monotonic_seconds
            ),
        )
