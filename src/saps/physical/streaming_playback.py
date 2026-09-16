"""C1-B prerecorded playback; no ROS, inference, or accumulated targets."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import io
from pathlib import Path
import time
from typing import Any, Callable

import numpy as np

from saps.physical.single_action import first_action_from_chunk, safety_gate

NSEC_PER_SEC = 1_000_000_000
POLICY_HZ = 15
ACTION_COUNT = 8

# One nominal 15 Hz period, used only for "a complete interval was missed"
# checks. Do not multiply this value to construct absolute deadlines.
PERIOD_NS = (NSEC_PER_SEC + POLICY_HZ // 2) // POLICY_HZ

def tick_offset_ns(index: int) -> int:
    """Nearest integer-nanosecond offset for index / 15 seconds."""
    if isinstance(index, bool) or not isinstance(index, int) or index < 0:
        raise ValueError("tick index must be a non-negative integer")
    return (
        index * NSEC_PER_SEC + POLICY_HZ // 2
    ) // POLICY_HZ

ARTIFACT = Path(
    "outputs/physical_pi05_droid_p1b/"
    "p1b_execute_20260914T113026Z/actions.npz"
)
ARTIFACT_SHA256 = (
    "0a9079ad0b429d16c18f1ef294cf66bced70d727db127a30cd9f7a5180d8dec0"
)
LAB_COMMIT = "9e535b665626cf8f3894fc4d2ef9136790abad92"


def selected_actions(chunk: Any, count: int = ACTION_COUNT) -> np.ndarray:
    """Validate the full native chunk before any possible publication."""
    first_action_from_chunk(chunk)
    if isinstance(count, bool) or count != ACTION_COUNT:
        raise ValueError("C1-B requires exactly actions 0 through 7.")
    actions = np.array(chunk[:count], dtype=np.float64, copy=True)
    actions.setflags(write=False)
    return actions


def load_canonical(root: Path) -> tuple[np.ndarray, bytes]:
    """Verify bytes before loading; retain the original archive for provenance."""
    raw = (root / ARTIFACT).read_bytes()
    if hashlib.sha256(raw).hexdigest() != ARTIFACT_SHA256:
        raise ValueError("Canonical C1-B chunk checksum mismatch.")
    with np.load(io.BytesIO(raw), allow_pickle=False) as archive:
        chunk = archive["actions"].copy()
    selected_actions(chunk)
    return chunk, raw


def deadlines(t0_ns: int) -> tuple[int, ...]:
    """Eight action ticks plus hold from independently rounded 15 Hz offsets."""
    return tuple(
        t0_ns + tick_offset_ns(k)
        for k in range(ACTION_COUNT + 1)
    )


def wait_until(deadline_ns: int, *, now: Callable[[], int] = time.monotonic_ns,
               sleep: Callable[[float], None] = time.sleep) -> None:
    """Wait against one absolute CLOCK_MONOTONIC deadline without chained drift."""
    while True:
        remaining = deadline_ns - now()
        if remaining <= 0:
            return
        sleep((remaining - 200_000) / 1e9 if remaining > 200_000 else 0)


@dataclass(frozen=True)
class MeasuredState:
    q: np.ndarray
    source_stamp_ns: int
    receive_monotonic_ns: int
    ros_now_ns: int
    readiness_reasons: tuple[str, ...] = ()
    dq: Any = None


def prepare(action: np.ndarray | None, state: MeasuredState,
            limits: dict[str, Any], *, now_ns: int, start_ns: int,
            scheduled_ns: int, kind: str,
            action_index: int | None) -> dict[str, Any]:
    """Reuse the unchanged safety gate, including for zero-displacement holds."""
    gate = safety_gate(
        np.zeros(8) if action is None else action, state.q, limits,
        state_age=(state.ros_now_ns - state.source_stamp_ns) / 1e9,
        receive_age=(now_ns - state.receive_monotonic_ns) / 1e9,
        # Prerecorded C1-B age is playback age, not original inference age.
        # Holds are freshly created commands, not aged policy actions.
        action_age=0.0 if action is None else (now_ns - start_ns) / 1e9,
        readiness_reasons=list(state.readiness_reasons),
    )
    return {
        "type": kind, "action_index": action_index,
        "raw_policy_action": None if action is None else action.copy(),
        "q_ref": state.q.copy(), "dq_ref": state.dq,
        "q_ref_source_stamp_ns": state.source_stamp_ns,
        "q_ref_receive_monotonic_ns": state.receive_monotonic_ns,
        "q_ref_age_at_mapping_seconds": gate["state_age_seconds"],
        "q_ref_receive_age_at_mapping_seconds": gate["receive_age_seconds"],
        "mapped_delta": gate.get("delta_q_rad"),
        "q_target": gate.get("proposed_q_target_rad"),
        "safety_gate": gate, "scheduled_monotonic_ns": scheduled_ns,
        "actual_publish_t0_ns": None, "schedule_lateness_ns": None,
    }


def playback(chunk: Any, boundary: Any, limits: dict[str, Any], *,
             dry_run: bool = False,
             now: Callable[[], int] = time.monotonic_ns,
             wait: Callable[[int], None] = wait_until) -> dict[str, Any]:
    """Publish at most eight policy targets and one hold, with no convergence wait.

    Boundary.snapshot returns fresh state/readiness evidence. Boundary.publish
    checks freshness/readiness again immediately before the one publish call and
    records its T0 even if the call raises. Any failure terminates policy playback.
    """
    actions = selected_actions(chunk)
    start = now()
    schedule = deadlines(start)
    rows: list[dict[str, Any]] = []
    result: dict[str, Any] = {
        "dry_run": dry_run, "start_monotonic_ns": start,
        "policy_frequency_hz": POLICY_HZ,
        "period_ns": PERIOD_NS,
        "deadline_offsets_ns": [tick_offset_ns(k) for k in range(ACTION_COUNT + 1)],
        "nominal_policy_window_seconds": ACTION_COUNT / POLICY_HZ,
        "action_age_basis": "elapsed_since_playback_start_not_inference_age",
        "rows": rows, "status": "running", "gripper_commands_issued": 0,
    }

    def send(action: np.ndarray | None, index: int | None,
             kind: str, deadline: int) -> None:
        state = boundary.snapshot()
        row = prepare(action, state, limits, now_ns=now(), start_ns=start,
                      scheduled_ns=deadline, kind=kind, action_index=index)
        rows.append(row)
        if not row["safety_gate"]["accepted"]:
            raise RuntimeError("safety_gate_rejected")
        if dry_run:
            row["publication_call_outcome"] = "dry_run_no_publication"
        else:
            boundary.publish(row, state, start)

    for index, deadline in enumerate(schedule):
        terminal = index == ACTION_COUNT
        try:
            wait(deadline)
            # Never burst overdue actions to catch up to an absolute schedule.
            if now() - deadline >= PERIOD_NS:
                raise RuntimeError("missed_complete_action_interval")
            send(None if terminal else actions[index],
                 None if terminal else index,
                 "terminal_hold" if terminal else "policy_action", deadline)
        except (RuntimeError, ValueError, KeyboardInterrupt) as error:
            result["status"] = (
                "terminal_hold_failed" if terminal
                else f"aborted_before_action_{index}"
            )
            result["error"] = f"{type(error).__name__}: {error}"
            # A failed terminal hold already consumed the sole hold attempt.
            if not terminal:
                try:
                    send(None, None, "abort_hold", now())
                    result["abort_hold"] = "dry_run" if dry_run else "published"
                except (RuntimeError, ValueError, KeyboardInterrupt) as hold_error:
                    result["abort_hold"] = "unsafe_or_failed_no_retry"
                    result["hold_error"] = str(hold_error)
            break
    else:
        result["status"] = "dry_run_complete" if dry_run else "published"
    first = next((r["actual_publish_t0_ns"] for r in rows
                  if r["type"] == "policy_action"), None)
    last = next((r["actual_publish_t0_ns"] for r in rows
                 if r["type"] == "terminal_hold"), None)
    result["action0_to_terminal_hold_t0_ns"] = (
        last - first if first is not None and last is not None else None
    )
    return result
