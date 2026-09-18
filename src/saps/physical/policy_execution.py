"""C1-C2 repeated arm execution using the frozen physical safety boundary."""

from __future__ import annotations

import copy
from enum import Enum
import json
from pathlib import Path
import time
from typing import Any, Callable

import numpy as np

from saps.physical.droid_gripper import finalize_gripper_evidence
from saps.physical.live_inference_hold import (
    captured_records, controller_identity, POST_HOLD_DRAIN_SECONDS,
    wait_for_application,
)
from saps.physical.live_shadow import (
    CameraPairGate, infer_live_request, prepare_live_request,
)
from saps.physical.policy_warmup import PolicyWarmup, prepare_warmed_observation
from saps.physical.streaming_playback import (
    PERIOD_NS, deadlines, prepare, selected_actions, wait_until,
)
from saps.physical.streaming_ros import (
    correlate, validate_streaming_delivery, validate_streaming_runtime_health,
)


class TaskOutcome(str, Enum):
    CONTINUE = "continue"
    SUCCESS = "success"
    FAILURE = "failure"
    ABORT = "abort"


def unevaluated_outcome(replan: dict[str, Any]) -> TaskOutcome:
    """Validation-only provider; no physical success detector is installed."""
    return TaskOutcome.CONTINUE


def execution_evidence_snapshot(
    boundary: Any, start: int,
) -> list[dict[str, Any]]:
    """Isolate C1-C2 evidence without blocking callbacks during deep copy.

    The boundary appends completed records and never mutates saved records.
    Capture the prefix under its lock, then detach nested data outside it.
    """
    with boundary.lock:
        records = boundary.records[start:]
    return copy.deepcopy(records)


def validate_execution_delivery(
    records: list[dict[str, Any]], rows: list[dict[str, Any]],
    analyzer: Any, expected_identity: tuple[str, int],
) -> dict[str, Any]:
    """Validate the whole publication prefix, including intervening holds."""
    summary = correlate(records, rows, analyzer)
    result = validate_streaming_delivery(
        rows, summary, expected_types=[r["type"] for r in rows],
        expected_indices=[r["action_index"] for r in rows],
    )
    reasons = result["reasons"]
    if result["controller_identity"] != list(expected_identity):
        reasons.append("unexpected_controller_identity")
    try:
        if controller_identity(records) != expected_identity:
            reasons.append("controller_identity_changed")
    except RuntimeError as error:
        reasons.append(str(error))
    sent = [r for r in records if r["kind"] == "sent"]
    if len(sent) != len(rows):
        reasons.append("unexpected_publication_count")
    for row, publication in zip(rows, sent):
        if (not row["safety_gate"]["accepted"]
                or publication.get("publication_call_outcome") != "returned"
                or publication["run"] != row.get("run")
                or publication["source_stamp_ns"] != row.get("source_stamp_ns")
                or publication["t0_ns"] != row.get("actual_publish_t0_ns")
                or not np.array_equal(publication["q"], row["q_target"])):
            reasons.append("publication_identity_or_target_mismatch")
    if any(r["kind"] == "invalid_evidence" for r in records):
        reasons.append("invalid_raw_evidence")
    if any(r["kind"] == "franka_state" and r.get("health_reasons")
           for r in records):
        reasons.append("franka_health_violation_observed")
    first_sent = min((r["t0_ns"] for r in sent), default=0)
    if any(r["kind"] == "controller_readiness"
           and r.get("response_monotonic_ns", 0) >= first_sent
           and r.get("readiness_reasons") for r in records):
        reasons.append("controller_readiness_violation_observed")
    result["accepted"] = not reasons
    return result


def audit_inference_hold(
    records: list[dict[str, Any]], pre_hold: dict[str, Any],
    inference: dict[str, Any],
) -> dict[str, Any]:
    """Require the installed target and sequence throughout sampled inference."""
    start = inference["request_start_monotonic_ns"]
    end = inference["response_completion_monotonic_ns"]
    reasons = []
    if not pre_hold["timing"]["t4_ns"] < start <= end:
        reasons.append("invalid_pre_hold_inference_order")
    if any(r["kind"] == "sent" and start <= r["t0_ns"] <= end
           for r in records):
        reasons.append("publication_during_inference")
    states = [r["data"] for r in records if r["kind"] == "controller_state"
              and start <= r["receive_monotonic_ns"] <= end]
    if not states:
        reasons.append("no_controller_state_during_inference")
    for state in states:
        if (len(state) != 39 or not np.isfinite(state).all()
                or state[38] != 1
                or state[37] != pre_hold["timing"]["sequence"]
                or not np.allclose(state[14:21], pre_hold["q_target"],
                                   atol=1e-12, rtol=0)):
            reasons.append("installed_pre_hold_changed_during_inference")
            break
    return {"accepted": not reasons, "reasons": reasons,
            "controller_state_samples": len(states),
            "gripper_commands_issued": 0}


def run_policy_episode(
    *, boundary: Any, collector: Any, policy: Any, config: dict[str, Any],
    output_dir: Path, limits: dict[str, Any], analyzer: Any,
    expected_identity: tuple[str, int], analysis_start: int,
    application_timeout: float, observation_timeout: float,
    policy_episode_seed: int, warmup_policy_seed: int,
    max_replans: int, max_executed_policy_chunks: int | None,
    spin_once: Callable[[], None], ros_now: Callable[[], float],
    result: dict[str, Any],
    outcome_provider: Callable[[dict[str, Any]], TaskOutcome] = unevaluated_outcome,
    check_runtime: Callable[[], None] = lambda: None,
    gripper: Any | None = None,
    now: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    """One warm-up, then hold/observe/infer/execute/hold/evaluate per replan.

    Optional gripper requests are nonblocking. The finite replan
    bound and bounded observation, transport and confirmation waits limit the
    episode. Runtime acceptance never implies manipulation-task success.
    """
    if (type(max_replans) is not int or max_replans <= 0
            or (max_executed_policy_chunks is not None and
                (type(max_executed_policy_chunks) is not int
                 or max_executed_policy_chunks <= 0))
            or any(not np.isfinite(v) or v <= 0 for v in
                   (application_timeout, observation_timeout))):
        raise ValueError("Positive finite bounds are required")
    warmup = PolicyWarmup(warmup_policy_seed=warmup_policy_seed,
                          policy_episode_seed=policy_episode_seed)
    rows: list[dict[str, Any]] = []
    result.update(
        status="running", warmup=warmup.record, rows=rows, replans=[],
        episode={"policy_episode_seed": policy_episode_seed,
                 "max_replans": max_replans,
                 "max_executed_policy_chunks": max_executed_policy_chunks,
                 "gripper_actuation": ("droid_binary_franka_hand" if gripper is not None
                                       else "disabled_arm_only"),
                 "outcome_provider": getattr(outcome_provider, "__name__",
                                             type(outcome_provider).__name__)},
        termination={"task_outcome": "not_evaluated"}, runtime_health={},
        warmup_requests=0, main_policy_requests=0, completed_main_replans=0,
        policy_actions_scheduled=0, policy_actions_executed=0,
        gripper_commands_issued=0, terminal_holds_applied=0,
    )
    monotonic = lambda: now() / 1e9
    current: dict[str, Any] | None = None
    phase = "warmup"
    confirmation_start: int | None = None

    def evidence() -> list[dict[str, Any]]:
        return execution_evidence_snapshot(boundary, analysis_start)

    def delivery() -> dict[str, Any]:
        return validate_execution_delivery(evidence(), rows, analyzer,
                                           expected_identity)

    def confirm() -> None:
        if confirmation_start is None:
            raise RuntimeError("missing_hold_evidence_cursor")
        row = rows[-1]
        deadline = now() + int(application_timeout * 1e9)
        while now() < deadline:
            # Copy and join only evidence acquired since this hold's publish
            # boundary. Full-prefix audits remain outside the online deadline.
            validation = validate_execution_delivery(
                captured_records(boundary, confirmation_start), [row],
                analyzer, expected_identity,
            )
            row["last_application_validation"] = validation
            if validation["accepted"] and now() < deadline:
                if row["timing"]["t4_ns"] > now():
                    raise RuntimeError("controller_application_is_in_the_future")
                row["application_confirmation"] = validation
                row["confirmation_monotonic_ns"] = now()
                return
            sleep(.01)
        raise TimeoutError("hold_application_confirmation_timeout")

    def send(action: Any, index: int | None, kind: str,
             eligible: int, scheduled: int, origin: int) -> dict[str, Any]:
        nonlocal confirmation_start
        if kind != "abort_hold":
            check_runtime()
            if gripper is not None and kind != "terminal_hold":
                gripper.check()
        previous = rows[-1]["q_ref_source_stamp_ns"] if rows else 0
        timeout = eligible + int(observation_timeout * 1e9)
        while True:
            state = boundary.snapshot()
            if (state.receive_monotonic_ns >= eligible
                    and state.source_stamp_ns > previous):
                break
            if now() >= timeout or (kind in ("policy_action", "terminal_hold")
                                   and now() - scheduled >= PERIOD_NS):
                raise TimeoutError("no_new_measured_q_for_step")
            sleep(.0005)
        prepared = now()
        if kind in ("pre_inference_hold", "abort_hold"):
            scheduled = prepared
        row = prepare(action, state, limits, now_ns=prepared,
                      start_ns=origin, scheduled_ns=scheduled,
                      kind=kind, action_index=index)
        row.update(preparation_eligible_monotonic_ns=eligible,
                   target_preparation_monotonic_ns=prepared)
        if action is None:
            row["q_target"] = state.q.copy()
        rows.append(row)
        if current is not None:
            if kind == "policy_action":
                current["actions"].append(row)
            else:
                current[kind] = row
        if not row["safety_gate"]["accepted"]:
            raise RuntimeError("safety_gate_rejected")
        if now() - scheduled >= PERIOD_NS:
            raise RuntimeError("missed_complete_action_interval")
        if kind in ("pre_inference_hold", "terminal_hold"):
            with boundary.lock:
                confirmation_start = len(boundary.records)
        if gripper is not None and kind not in ("abort_hold", "terminal_hold"):
            gripper.check()
        boundary.publish(row, state, origin)
        if gripper is not None and kind == "policy_action":
            row["gripper"] = gripper.command(float(action[7]))
        return row

    def hold(kind: str) -> dict[str, Any]:
        requested = now()
        return send(None, None, kind, requested, requested, requested)

    def reacquire_readiness(record: dict[str, Any]) -> None:
        start = now()
        deadline = start + int(application_timeout * 1e9)
        record.update(start_monotonic_ns=start, checks=0, accepted=False)
        try:
            while now() < deadline:
                record["checks"] += 1
                reasons = boundary.snapshot().readiness_reasons
                record["last_readiness_reasons"] = list(reasons)
                if not reasons and now() < deadline:
                    record["accepted"] = True
                    return
                remaining = deadline - now()
                if remaining > 0:
                    sleep(min(.01, remaining / 1e9))
            raise TimeoutError("readiness_reacquisition_timeout")
        finally:
            record["end_monotonic_ns"] = now()

    try:
        readiness = boundary.snapshot().readiness_reasons
        if readiness:
            raise RuntimeError("; ".join(readiness))
        warm_dir = output_dir / "startup"
        warm_dir.mkdir(exist_ok=False)
        # This barrier proof is intentionally discarded, never cached for infer.
        prepare_warmed_observation(
            warmup=warmup, collector=collector, policy=policy,
            output_dir=warm_dir, observation_timeout=observation_timeout,
            spin_once=spin_once, ros_now=ros_now, config=config,
            record=result, monotonic=monotonic,
        )
        gate = CameraPairGate()
        for index in range(max_replans):
            current = {"replan_index": index, "actions": [], "inference": {},
                       "policy_observation": {}, "runtime_health": {},
                       "delivery_validation": {},
                       "outcome_after_chunk": "not_evaluated"}
            result["replans"].append(current)
            phase = "pre_hold"
            current["pre_hold"] = hold("pre_inference_hold")
            confirm()
            # Sample ROS time immediately after unique T4 confirmation.
            barrier = ros_now()
            current["pre_hold_source_barrier_ros_seconds"] = barrier
            phase = "observation"
            observation, sample_dir = prepare_live_request(
                collector=collector, output_dir=output_dir, index=index,
                policy_episode_seed=policy_episode_seed,
                observation_timeout=observation_timeout, spin_once=spin_once,
                config=config, record=current["policy_observation"], gate=gate,
                source_barrier_ros_seconds=barrier, monotonic=monotonic,
            )
            current["policy_observation"]["request"] = json.loads(
                (sample_dir / "request.json").read_text())
            phase = "inference"
            check_runtime()
            if gripper is not None:
                gripper.check_inference()
            infer_live_request(
                observation=observation, sample_dir=sample_dir, policy=policy,
                index=index, policy_episode_seed=policy_episode_seed,
                ros_now=ros_now, config=config, monotonic=monotonic,
                monotonic_ns=now, call_timing=current["inference"],
                audit_model_input=index == 0,
            )
            current["inference"]["response"] = json.loads(
                (sample_dir / "response.json").read_text())
            current["inference"]["hold_audit"] = audit_inference_hold(
                evidence(), current["pre_hold"], current["inference"])
            if (not current["inference"]["hold_audit"]["accepted"]
                    or not delivery()["accepted"]):
                raise RuntimeError("inference_hold_evidence_failed")
            with np.load(sample_dir / "actions.npz", allow_pickle=False) as data:
                actions = selected_actions(data["actions"])
            phase = "actions"
            origin = now()
            schedule = deadlines(origin)
            current["schedule_origin_monotonic_ns"] = origin
            current["deadlines_monotonic_ns"] = schedule
            result["policy_actions_scheduled"] += 8
            for k in range(9):
                wait_until(schedule[k], now=now, sleep=sleep)
                if k == 8:
                    phase = "terminal_hold"
                send(None if k == 8 else actions[k],
                     None if k == 8 else k,
                     "terminal_hold" if k == 8 else "policy_action",
                     schedule[k], schedule[k], origin)
            confirm()
            result["terminal_holds_applied"] += 1
            phase = "delivery_validation"
            sleep(POST_HOLD_DRAIN_SECONDS)
            timing = {"drain_completion_monotonic_ns": now()}
            current["post_chunk_timing"] = timing
            check_runtime()
            timing["evidence_snapshot_start_monotonic_ns"] = now()
            records = evidence()
            timing["evidence_snapshot_end_monotonic_ns"] = now()
            current["delivery_validation"] = validate_execution_delivery(
                records, rows, analyzer, expected_identity)
            current["inference"]["hold_audit"] = audit_inference_hold(
                records, current["pre_hold"], current["inference"])
            current["runtime_health"] = validate_streaming_runtime_health(
                records, current["actions"] + [current["terminal_hold"]],
                hold_type="terminal_hold",
                after_monotonic_ns=current["terminal_hold"]["timing"]["t4_ns"],
            )
            timing["validation_completion_monotonic_ns"] = now()
            if (not current["delivery_validation"]["accepted"]
                    or not current["inference"]["hold_audit"]["accepted"]
                    or not current["runtime_health"]["accepted"]):
                raise RuntimeError("chunk_delivery_or_health_failed")
            result["completed_main_replans"] += 1
            phase = "outcome"
            outcome = outcome_provider(current)
            if not isinstance(outcome, TaskOutcome):
                raise ValueError("Outcome provider must return TaskOutcome")
            current["outcome_after_chunk"] = outcome.value
            result["termination"]["task_outcome"] = outcome.value
            if outcome is not TaskOutcome.CONTINUE:
                result["termination"]["termination_reason"] = "task_" + outcome.value
                break
            if (max_executed_policy_chunks is not None
                    and result["completed_main_replans"] >= max_executed_policy_chunks):
                result["termination"]["termination_reason"] = "test_chunk_limit_reached"
                break
            if index + 1 < max_replans:
                phase = "readiness_reacquisition"
                current["readiness_reacquisition"] = {}
                reacquire_readiness(current["readiness_reacquisition"])
        else:
            result["termination"]["termination_reason"] = "safety_replan_limit_reached"
        if gripper is not None:
            gripper.check()
        result["status"] = "success"  # Software validation only.
    except (Exception, KeyboardInterrupt) as error:
        result.update(status="failed", failed_phase=phase,
                      error=f"{type(error).__name__}: {error}")
        result["termination"].update(termination_reason="runtime_abort",
                                     task_outcome="abort")
        if gripper is not None:
            gripper.stop()
        # Exactly one best-effort fresh hold; no remaining action is attempted.
        # During readiness reacquisition the confirmed terminal hold remains
        # installed; the command-free barrier must not attempt another hold.
        # A terminal-hold failure has already consumed that hold attempt.
        if phase not in ("terminal_hold", "readiness_reacquisition"):
            try:
                with boundary.lock:
                    abort_start = len(boundary.records)
                hold("abort_hold")
                wait_for_application(
                    boundary=boundary, analysis_start=abort_start,
                    rows=[rows[-1]], analyzer=analyzer,
                    expected_identity=expected_identity,
                    timeout=application_timeout, now=now, sleep=sleep,
                )
                result["failure_hold"] = "applied"
            except (Exception, KeyboardInterrupt) as hold_error:
                result["failure_hold"] = "unsafe_or_unconfirmed_no_retry"
                result["failure_hold_error"] = str(hold_error)
        sleep(POST_HOLD_DRAIN_SECONDS)
    finally:
        if gripper is not None:
            gripper.stop()
            finalize_gripper_evidence(result, gripper)
        try:
            if rows:
                result["runtime_health"]["final_delivery_validation"] = delivery()
                if not result["runtime_health"]["final_delivery_validation"]["accepted"]:
                    result["status"] = "failed"
        except (KeyError, TypeError, ValueError, RuntimeError) as error:
            result.update(status="failed", timing_analysis_error=str(error))
        if result["status"] == "failed":
            result["termination"]["termination_reason"] = "runtime_abort"
        result["warmup_requests"] = warmup.record["request_count"]
        result["main_policy_requests"] = sum(
            "request_start_monotonic_ns" in r["inference"]
            for r in result["replans"])
        result["policy_actions_published"] = sum(
            r["type"] == "policy_action"
            and r.get("publication_call_outcome") == "returned" for r in rows)
        result["policy_actions_executed"] = sum(
            r["type"] == "policy_action"
            and r.get("timing", {}).get("status") == "applied" for r in rows)
        for replan in result["replans"]:
            replan["policy_actions_executed"] = sum(
                r.get("timing", {}).get("status") == "applied"
                for r in replan["actions"])
        result["runtime_validation_status"] = (
            "passed" if result["status"] == "success" else "failed")
