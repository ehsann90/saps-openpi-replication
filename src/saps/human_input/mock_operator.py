"""Deterministic, control-step-indexed operator traces for sweeps."""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path
from typing import Any

import numpy as np

from saps.arbitration.core import ACTION_DIMENSION
from saps.arbitration.core import SAPS_ACTIVITY_THRESHOLD
from saps.human_input.keyboard import HumanInputSample


TRACE_FORMAT_VERSION = 1


@dataclasses.dataclass(frozen=True)
class OperatorTraceSegment:
    """One inclusive-start, exclusive-end interval of direct actions."""

    start_control_step: int
    end_control_step: int
    action: np.ndarray
    label: str | None = None


class MockOperatorTrace:
    """Replay direct operator actions indexed by executed control steps.

    The trace advances only after an environment step. Scheduler waits therefore
    cannot change the intervention sequence, which makes comparisons across
    policy schedulers reproducible.
    """

    def __init__(
        self,
        *,
        segments: tuple[OperatorTraceSegment, ...],
        control_frequency_hz: float,
    ) -> None:
        if (
            not np.isfinite(control_frequency_hz)
            or control_frequency_hz <= 0
        ):
            raise ValueError(
                "control_frequency_hz must be finite and positive."
            )

        previous_end = 0
        for segment in segments:
            if segment.start_control_step < previous_end:
                raise ValueError(
                    "Trace segments must be ordered and "
                    "non-overlapping."
                )
            previous_end = segment.end_control_step

        self._segments = segments
        self._control_frequency_hz = float(control_frequency_hz)
        self._control_step = 0

    @classmethod
    def from_json(
        cls,
        path: Path,
        *,
        control_frequency_hz: float,
    ) -> "MockOperatorTrace":
        """Load a versioned mock-operator trace from JSON."""

        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as error:
            raise ValueError(
                f"Invalid mock operator trace JSON: {path}"
            ) from error

        if not isinstance(payload, dict):
            raise ValueError("Mock operator trace must be a JSON object.")
        if payload.get("trace_format_version") != TRACE_FORMAT_VERSION:
            raise ValueError(
                "Mock operator trace must specify "
                f"trace_format_version={TRACE_FORMAT_VERSION}."
            )

        raw_segments = payload.get("segments")
        if not isinstance(raw_segments, list):
            raise ValueError("Mock operator trace segments must be a JSON list.")

        segments = tuple(
            _parse_segment(raw_segment, index=index)
            for index, raw_segment in enumerate(raw_segments)
        )
        return cls(
            segments=segments,
            control_frequency_hz=control_frequency_hz,
        )

    def sample(self) -> HumanInputSample:
        """Return the action for the current executed-control-step index."""

        segment = next(
            (
                item
                for item in self._segments
                if item.start_control_step <= self._control_step
                < item.end_control_step
            ),
            None,
        )
        action = (
            segment.action.copy()
            if segment is not None
            else np.asarray([0.0] * 6 + [-1.0], dtype=np.float32)
        )
        logical_time = self._control_step / self._control_frequency_hz
        last_event_time = (
            segment.start_control_step / self._control_frequency_hz
            if segment is not None
            else None
        )

        return HumanInputSample(
            action=action,
            motion_active=bool(
                np.linalg.norm(action[:6]) > SAPS_ACTIVITY_THRESHOLD
            ),
            connected=True,
            armed=True,
            abort_requested=False,
            pressed_keys=(),
            gripper_command=float(action[6]),
            speed_mode="mock_trace",
            translation_gain=1.0,
            rotation_gain=1.0,
            sample_monotonic_seconds=logical_time,
            last_event_monotonic_seconds=last_event_time,
        )

    def control_step_completed(self) -> None:
        """Advance after, and only after, a successful environment step."""

        self._control_step += 1

    def publish_frame_rgb(
        self,
        image_rgb: np.ndarray,
        runtime_status: dict[str, Any],
    ) -> None:
        """Satisfy the live-operator interface without rendering a browser."""


def _parse_segment(
    raw_segment: Any,
    *,
    index: int,
) -> OperatorTraceSegment:
    """Validate and convert one JSON trace segment."""

    if not isinstance(raw_segment, dict):
        raise ValueError(f"Trace segment {index} must be a JSON object.")

    start = raw_segment.get("start_control_step")
    end = raw_segment.get("end_control_step")
    if isinstance(start, bool) or not isinstance(start, int) or start < 0:
        raise ValueError(f"Trace segment {index} start_control_step is invalid.")
    if isinstance(end, bool) or not isinstance(end, int) or end <= start:
        raise ValueError(f"Trace segment {index} end_control_step is invalid.")

    raw_action = raw_segment.get("action")
    if (
        not isinstance(raw_action, list)
        or len(raw_action) != ACTION_DIMENSION
        or any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            for value in raw_action
        )
    ):
        raise ValueError(
            f"Trace segment {index} action must contain "
            f"{ACTION_DIMENSION} numeric values."
        )

    action = np.asarray(raw_action, dtype=np.float32)
    if action.shape != (ACTION_DIMENSION,):
        raise ValueError(
            f"Trace segment {index} action must contain "
            f"{ACTION_DIMENSION} values."
        )
    if not np.all(np.isfinite(action)) or np.any(np.abs(action) > 1.0):
        raise ValueError(
            f"Trace segment {index} action values must be finite "
            "and within [-1, 1]."
        )

    label = raw_segment.get("label")
    if label is not None and not isinstance(label, str):
        raise ValueError(f"Trace segment {index} label must be a string.")

    return OperatorTraceSegment(
        start_control_step=start,
        end_control_step=end,
        action=action,
        label=label,
    )
