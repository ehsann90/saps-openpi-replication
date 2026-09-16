"""C1-C1: two measured-q holds surrounding one evidence-only DROID request."""

from __future__ import annotations

import copy
import time
from typing import Any, Callable

import numpy as np

from saps.physical.live_shadow import infer_live_request
from saps.physical.streaming_playback import prepare
from saps.physical.streaming_ros import (
    correlate, validate_streaming_delivery, validate_streaming_runtime_health,
)

PRE_HOLD = "pre_inference_hold"
POST_HOLD = "post_inference_hold"
ABORT_HOLD = "abort_hold"
POST_HOLD_DRAIN_SECONDS = 2.0  # Unchanged C1-B evidence drain convention.


def captured_records(boundary: Any, start: int) -> list[dict[str, Any]]:
    with boundary.lock:
        return copy.deepcopy(boundary.records[start:])


def controller_identity(records: list[dict[str, Any]]) -> tuple[str, int]:
    """Require one instance/activation in captured controller period evidence."""
    identities = {
        (r["data"]["instance"], sample["activation"])
        for r in records if r["kind"] == "controller"
        and r["data"].get("event") == "samples"
        for sample in r["data"]["samples"]
    }
    if len(identities) != 1:
        raise RuntimeError("missing_or_changing_controller_identity")
    instance, activation = identities.pop()
    if (not isinstance(instance, str) or not instance
            or type(activation) is not int or activation <= 0):
        raise RuntimeError("invalid_controller_identity")
    return instance, activation


def validate_holds(
    records: list[dict[str, Any]], rows: list[dict[str, Any]], analyzer: Any,
    expected_identity: tuple[str, int],
) -> dict[str, Any]:
    """Join the original raw records, then enforce labelled hold invariants."""
    summary = correlate(records, rows, analyzer)
    result = validate_streaming_delivery(
        rows, summary, expected_types=[row["type"] for row in rows],
        expected_indices=[None] * len(rows),
    )
    reasons = result["reasons"]
    if result["controller_identity"] != list(expected_identity):
        reasons.append("unexpected_controller_instance_or_activation")
    try:
        if controller_identity(records) != expected_identity:
            reasons.append("controller_instance_or_activation_changed")
    except RuntimeError as error:
        reasons.append(str(error))
    sent = [r for r in records if r["kind"] == "sent"]
    if len(sent) != len(rows):
        reasons.append("unexpected_target_publication_count")
    for row, publication in zip(rows, sent):
        if (row["type"] not in (PRE_HOLD, POST_HOLD, ABORT_HOLD)
                or row.get("raw_policy_action") is not None
                or not row["safety_gate"]["accepted"]
                or not np.array_equal(row["q_ref"], row["q_target"])):
            reasons.append("not_an_admitted_measured_q_hold")
        if (publication["run"] != row.get("run")
                or publication["source_stamp_ns"] != row.get("source_stamp_ns")
                or publication["t0_ns"] != row.get("actual_publish_t0_ns")
                or publication.get("publication_call_outcome") != "returned"
                or not np.array_equal(publication["q"], row["q_target"])):
            reasons.append("publication_identity_or_target_mismatch")
    if any(r["kind"] == "invalid_evidence" for r in records):
        reasons.append("invalid_raw_evidence")
    if any(r["kind"] == "franka_state" and r.get("health_reasons")
           for r in records):
        reasons.append("franka_health_violation_observed")
    result.update(accepted=not reasons, timing_summary=summary)
    return result


def wait_for_application(
    *, boundary: Any, analysis_start: int, rows: list[dict[str, Any]],
    analyzer: Any, expected_identity: tuple[str, int], timeout: float,
    now: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """A publish return never admits inference; only complete C1-A2 T4 does."""
    deadline = now() + int(timeout * 1e9)
    while now() < deadline:
        validation = validate_holds(
            captured_records(boundary, analysis_start), rows, analyzer,
            expected_identity,
        )
        if validation["accepted"] and now() < deadline:
            if rows[-1]["timing"]["t4_ns"] > now():
                raise RuntimeError("controller_application_is_in_the_future")
            rows[-1]["application_confirmation"] = copy.deepcopy(validation)
            rows[-1]["confirmation_monotonic_ns"] = now()
            return validation
        rows[-1]["last_application_validation"] = validation
        sleep(0.01)  # Poll below the pinned controller's 20 ms evidence drain.
    raise TimeoutError("hold_application_confirmation_timeout")


def run_hold_sequence(
    *, boundary: Any, limits: dict[str, Any], analyzer: Any,
    expected_identity: tuple[str, int], analysis_start: int,
    application_timeout: float, observation_timeout: float,
    inference_arguments: dict[str, Any], result: dict[str, Any],
    now: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    """One request, no action executor, no gripper interface, no retry loop."""
    if any(not np.isfinite(v) or v <= 0
           for v in (application_timeout, observation_timeout)):
        raise ValueError("Hold and observation timeouts must be positive finite")
    rows: list[dict[str, Any]] = []
    result.update(status="running", rows=rows, policy_actions_executed=0,
                  gripper_commands_issued=0, completed_requests=0,
                  inference_attempts=0, policy_inference={})

    def hold(kind: str) -> None:
        requested = now()
        deadline = requested + int(observation_timeout * 1e9)
        previous_stamp = rows[-1]["q_ref_source_stamp_ns"] if rows else 0
        while True:
            state = boundary.snapshot()
            if (state.receive_monotonic_ns >= requested
                    and state.source_stamp_ns > previous_stamp):
                break
            if now() >= deadline:
                raise TimeoutError("no_new_measured_q_for_hold")
            sleep(0.01)
        scheduled = now()
        row = prepare(None, state, limits, now_ns=scheduled,
                      start_ns=scheduled, scheduled_ns=scheduled,
                      kind=kind, action_index=None)
        # The safety gate evaluates zero displacement with the C1-B limits.
        # The published equilibrium is explicitly the original measured double q.
        row["q_target"] = state.q.copy()
        row["hold_requested_monotonic_ns"] = requested
        rows.append(row)
        if not row["safety_gate"]["accepted"]:
            raise RuntimeError("hold_safety_gate_rejected")
        boundary.publish(row, state, scheduled)
        wait_for_application(
            boundary=boundary, analysis_start=analysis_start, rows=rows,
            analyzer=analyzer, expected_identity=expected_identity,
            timeout=application_timeout, now=now, sleep=sleep,
        )

    phase = PRE_HOLD
    try:
        hold(PRE_HOLD)
        phase = "policy_inference"
        timing = result["policy_inference"]
        # No publication path is called inside this blocking operation.
        try:
            infer_live_request(**inference_arguments, call_timing=timing,
                               monotonic_ns=now)
        finally:
            result["inference_attempts"] = int(
                "request_start_monotonic_ns" in timing
            )
        result["completed_requests"] = 1
        phase = POST_HOLD
        hold(POST_HOLD)
        result["status"] = "holds_applied"
    except (Exception, KeyboardInterrupt) as error:
        result.update(status="failed", failed_phase=phase,
                      error=f"{type(error).__name__}: {error}")
        if phase == "policy_inference":
            try:
                hold(ABORT_HOLD)
                result["failure_hold"] = "applied"
            except (Exception, KeyboardInterrupt) as hold_error:
                result["failure_hold"] = "unsafe_or_unconfirmed_no_retry"
                result["failure_hold_error"] = str(hold_error)
    finally:
        # Keep callbacks running for delayed application and health evidence.
        sleep(POST_HOLD_DRAIN_SECONDS)


def finalize_hold_sequence(
    *, records: list[dict[str, Any]], result: dict[str, Any], analyzer: Any,
    expected_identity: tuple[str, int],
) -> bool:
    """Offline C1-A2 analysis remains mandatory after the complete drain."""
    rows = result["rows"]
    if not rows:
        result["status"] = "failed"
        return False
    delivery = validate_holds(records, rows, analyzer, expected_identity)
    result["delivery_validation"] = delivery
    final_hold = rows[-1]
    applied_at = final_hold.get("timing", {}).get("t4_ns")
    health = validate_streaming_runtime_health(
        records, rows, hold_type=final_hold["type"],
        after_monotonic_ns=applied_at,
    )
    result["health_evidence_after_monotonic_ns"] = applied_at
    if final_hold["type"] == ABORT_HOLD:
        result["failure_hold_final_application_status"] = (
            final_hold.get("timing", {}).get("status", "unconfirmed")
        )
    result["runtime_health_validation"] = health
    timing = result["policy_inference"]
    start = timing.get("request_start_monotonic_ns")
    end = timing.get("response_completion_monotonic_ns")
    sent = [r for r in records if r["kind"] == "sent"]
    ordered = (
        len(rows) == 2 and [r["type"] for r in rows] == [PRE_HOLD, POST_HOLD]
        and type(start) is int and type(end) is int
        and rows[0].get("timing", {}).get("t4_ns") is not None
        and rows[0]["timing"]["t4_ns"] <= start <= end
        and end <= rows[1]["hold_requested_monotonic_ns"]
        and rows[1]["q_ref_receive_monotonic_ns"] >= end
        and rows[1]["q_ref_source_stamp_ns"] > rows[0]["q_ref_source_stamp_ns"]
    )
    no_publication = (type(start) is int and type(end) is int
                      and not any(start <= r["t0_ns"] <= end for r in sent))
    result["publication_audit"] = {
        "sent_targets": [{k: r[k] for k in ("run", "source_stamp_ns", "t0_ns", "q")}
                         for r in sent],
        "no_publication_during_inference": no_publication,
        "pre_t4_inference_post_hold_order_valid": bool(ordered),
    }
    success = (
        result["status"] == "holds_applied" and delivery["accepted"]
        and health["accepted"] and ordered and no_publication
        and result["completed_requests"] == result["inference_attempts"] == 1
        and result["policy_actions_executed"] == result["gripper_commands_issued"] == 0
    )
    result["status"] = "success" if success else "failed"
    return bool(success)
