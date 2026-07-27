"""Action-level arbitration for SAPS shared autonomy."""

from __future__ import annotations

import dataclasses
import enum
from typing import Any

import numpy as np


ACTION_DIMENSION = 7
MOTION_DIMENSION = 6
SAPS_ACTIVITY_THRESHOLD = 1e-3


class ArbitrationMode(str, enum.Enum):
    """Supported action-arbitration modes."""

    AUTONOMOUS = "autonomous"
    TAKEOVER = "takeover"


class ActivityState(str, enum.Enum):
    """Human motion activity at one control step."""

    IDLE = "idle"
    ACTIVE = "active"


@dataclasses.dataclass(frozen=True)
class ArbitrationResult:
    """Structured result of one arbitration decision."""

    arbitration_mode: ArbitrationMode
    activity_state: ActivityState
    human_motion_norm: float
    autonomy_weight: float

    human_action: np.ndarray
    autonomous_action: np.ndarray
    executed_action: np.ndarray

    @property
    def human_active(self) -> bool:
        """Return whether human end-effector motion is active."""

        return self.activity_state is ActivityState.ACTIVE

    def as_dict(self) -> dict[str, Any]:
        """Return JSON-compatible fields for step-level logging."""

        return {
            "arbitration_mode": self.arbitration_mode.value,
            "activity_state": self.activity_state.value,
            "human_active": self.human_active,
            "human_motion_norm": self.human_motion_norm,
            "autonomy_weight": self.autonomy_weight,
            "human_action": self.human_action.tolist(),
            "autonomous_action": self.autonomous_action.tolist(),
            "executed_action": self.executed_action.tolist(),
        }


@dataclasses.dataclass(frozen=True)
class ActionArbitrator:
    """Select an executed action from human and autonomous commands."""

    mode: ArbitrationMode | str
    activity_threshold: float = SAPS_ACTIVITY_THRESHOLD

    def __post_init__(self) -> None:
        try:
            parsed_mode = (
                self.mode
                if isinstance(self.mode, ArbitrationMode)
                else ArbitrationMode(str(self.mode))
            )
        except ValueError as error:
            supported = ", ".join(
                mode.value for mode in ArbitrationMode
            )
            raise ValueError(
                f"Unsupported arbitration mode {self.mode!r}. "
                f"Supported modes: {supported}."
            ) from error

        threshold = float(self.activity_threshold)

        if not np.isfinite(threshold) or threshold < 0.0:
            raise ValueError(
                "activity_threshold must be finite and non-negative."
            )

        object.__setattr__(self, "mode", parsed_mode)
        object.__setattr__(self, "activity_threshold", threshold)

    def arbitrate(
        self,
        *,
        autonomous_action: np.ndarray,
        human_action: np.ndarray,
    ) -> ArbitrationResult:
        """Produce one autonomous or hard-takeover action."""

        autonomous = _validated_action(
            "autonomous_action",
            autonomous_action,
        )
        human = _validated_action(
            "human_action",
            human_action,
        )

        human_motion_norm = float(
            np.linalg.norm(human[:MOTION_DIMENSION])
        )
        human_active = (
            human_motion_norm > self.activity_threshold
        )
        activity_state = (
            ActivityState.ACTIVE
            if human_active
            else ActivityState.IDLE
        )

        if self.mode is ArbitrationMode.AUTONOMOUS:
            autonomy_weight = 1.0
            executed = autonomous.copy()
        elif self.mode is ArbitrationMode.TAKEOVER:
            autonomy_weight = 0.0 if human_active else 1.0

            executed = np.empty(
                ACTION_DIMENSION,
                dtype=np.float32,
            )
            executed[:MOTION_DIMENSION] = (
                human[:MOTION_DIMENSION]
                if human_active
                else autonomous[:MOTION_DIMENSION]
            )

            # SAPS handles the gripper independently from motion
            # arbitration. With -1=open and +1=close, max() lets
            # either source initiate closing and biases conflicts
            # toward closing.
            executed[6] = max(
                float(autonomous[6]),
                float(human[6]),
            )
        else:
            raise RuntimeError(
                f"Unhandled arbitration mode {self.mode!r}."
            )

        executed.setflags(write=False)

        return ArbitrationResult(
            arbitration_mode=self.mode,
            activity_state=activity_state,
            human_motion_norm=human_motion_norm,
            autonomy_weight=autonomy_weight,
            human_action=human,
            autonomous_action=autonomous,
            executed_action=executed,
        )


def _validated_action(
    name: str,
    action: np.ndarray,
) -> np.ndarray:
    """Validate and copy one seven-dimensional action."""

    array = np.asarray(
        action,
        dtype=np.float32,
    )

    if array.shape != (ACTION_DIMENSION,):
        raise ValueError(
            f"{name} must have shape "
            f"({ACTION_DIMENSION},), received {array.shape}."
        )

    if not np.all(np.isfinite(array)):
        raise ValueError(
            f"{name} must contain only finite values."
        )

    result = array.copy()
    result.setflags(write=False)
    return result
