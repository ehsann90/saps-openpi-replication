"""Descriptive matched analysis for the excluded Gate-2 v2 pilot."""

from __future__ import annotations

from collections import Counter
import csv
import json
from pathlib import Path
from typing import Any, Iterable

from saps.evaluation.experiment_session import load_manifest
from saps.evaluation.experiment_session import manifest_sha256
from saps.evaluation.experiment_session import json_file_identity
from saps.evaluation.gate2_analysis import _aggregate
from saps.evaluation.gate2_analysis import _aggregate_cosine_diagnostics
from saps.evaluation.gate2_analysis import _base_episode_row
from saps.evaluation.gate2_analysis import _blank_cosine
from saps.evaluation.gate2_analysis import _blank_fixed
from saps.evaluation.gate2_analysis import _blank_wait
from saps.evaluation.gate2_analysis import _episode_analysis
from saps.evaluation.gate2_analysis import _finite_float
from saps.evaluation.gate2_analysis import _latency_metrics
from saps.evaluation.gate2_analysis import _public_cosine_row
from saps.evaluation.gate2_analysis import _read_json
from saps.evaluation.gate2_analysis import _read_jsonl
from saps.evaluation.gate2_analysis import _selected_attempts
from saps.evaluation.gate2_v2_protocol import (
    build_gate2_v2_autonomous_schedule,
)
from saps.evaluation.gate2_v2_protocol import (
    build_gate2_v2_shared_schedule,
)
from saps.evaluation.gate2_v2_protocol import GATE2_V2_ALL_MODES
from saps.evaluation.gate2_v2_protocol import (
    GATE2_V2_AUTONOMOUS_EXPERIMENT_ID,
)
from saps.evaluation.gate2_v2_protocol import (
    GATE2_V2_AUTONOMOUS_PROTOCOL_PATH,
)
from saps.evaluation.gate2_v2_protocol import GATE2_V2_CONDITIONS
from saps.evaluation.gate2_v2_protocol import GATE2_V2_CONFIG_SHA256
from saps.evaluation.gate2_v2_protocol import GATE2_V2_CONTROL_FREQUENCY_HZ
from saps.evaluation.gate2_v2_protocol import GATE2_V2_MANIFEST_PATH
from saps.evaluation.gate2_v2_protocol import GATE2_V2_MAX_STEPS
from saps.evaluation.gate2_v2_protocol import GATE2_V2_PROFILE_SHA256
from saps.evaluation.gate2_v2_protocol import GATE2_V2_SHARED_EXPERIMENT_ID
from saps.evaluation.gate2_v2_protocol import GATE2_V2_SHARED_MODES
from saps.evaluation.gate2_v2_protocol import GATE2_V2_SHARED_OUTPUT_ROOT
from saps.evaluation.gate2_v2_protocol import GATE2_V2_TASK_ID
from saps.evaluation.gate2_v2_protocol import (
    load_gate2_v2_autonomous_protocol,
)
from saps.evaluation.gate2_v2_protocol import (
    validate_gate2_v2_autonomous_schedule,
)
from saps.evaluation.gate2_v2_protocol import validate_gate2_v2_manifest
from saps.evaluation.gate2_v2_protocol import (
    validate_gate2_v2_shared_schedule,
)
from saps.policies.seeding import SEED_PROTOCOL


PAIRING_FIELDS = (
    "condition_id",
    "trial_index",
    "initial_state_index",
    "policy_episode_seed",
    "policy_seed_protocol",
)
TRIPLET_FIELDS = (
    *PAIRING_FIELDS,
    "autonomous_success",
    "fixed_blend_success",
    "cosine_blend_success",
    "autonomous_simulated_duration_seconds",
    "fixed_blend_simulated_duration_seconds",
    "cosine_blend_simulated_duration_seconds",
    "autonomous_wall_control_duration_seconds",
    "fixed_blend_wall_control_duration_seconds",
    "cosine_blend_wall_control_duration_seconds",
    "autonomous_summary_path",
    "fixed_blend_summary_path",
    "cosine_blend_summary_path",
)
POLICY_ACCOUNTING_FIELDS = (
    "episode_id",
    "mode",
    "condition_id",
    "trial_index",
    "success",
    "termination_reason",
    "control_steps",
    "accounting_model",
    "accounting_status",
    "summary_policy_replan_count",
    "policy_requests_submitted",
    "logged_request_submission_count",
    "unique_submitted_replan_indices",
    "policy_results_completed_and_logged",
    "logged_policy_result_count",
    "unique_result_replan_indices",
    "inference_latency_count",
    "non_null_inference_latency_count",
    "last_submitted_replan_index",
    "last_result_replan_index",
    "last_logged_policy_worker_pending",
    "submitted_minus_result_count",
    "submitted_minus_latency_count",
    "terminal_unobserved_request_count",
    "terminal_unobserved_replan_index",
)

COLLECTION_COMMIT = "d4013d7998b9843bf1e1a5fb25c7bbce515d0fdb"
ACCOUNTING_ANALYSIS_COMMIT = "2d2d8fe5efa0a59a05ce8e59a6814f1c1895209f"
OPENPI_COMMIT = "15a9616a00943ada6c20a0f158e3adb39df2ccac"
LIBERO_COMMIT = "f78abd68ee283de9f9be3c8f7e2a9ad60246e95c"
POLICY_CHECKPOINT = "gs://openpi-assets/checkpoints/pi05_libero"
SHARED_MANIFEST_SHA256 = (
    "61c3d346af87ffdef16b378fed9383a395b3d27947eabf768da1bd314491383a"
)
AUTONOMOUS_PROTOCOL_SHA256 = (
    "47d84fed0dcb1909d9d99412af2515989fe46559e0e9fb0a06bbf70d1d10bd18"
)


def _ordered_records(
    steps: list[dict[str, Any]],
    waits: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Return scheduler records in collection order."""

    records = [*steps, *waits]
    return sorted(
        records,
        key=lambda record: int(record.get("scheduler_tick", -1)),
    )


def _indices(
    records: list[dict[str, Any]],
    field: str,
) -> tuple[list[int], list[int]]:
    values = [
        int(record[field])
        for record in records
        if record.get(field) is not None
    ]
    return values, sorted(set(values))


def _accounting_identity(
    *,
    episode_id: str,
    mode: str,
    condition_id: str,
    trial_index: int,
    success: bool,
    termination_reason: str,
    control_steps: int,
) -> dict[str, Any]:
    return {
        "episode_id": episode_id,
        "mode": mode,
        "condition_id": condition_id,
        "trial_index": trial_index,
        "success": int(success),
        "termination_reason": termination_reason,
        "control_steps": control_steps,
    }


def _blank_policy_accounting(
    row: dict[str, Any],
    *,
    accounting_model: str,
) -> dict[str, Any]:
    return {
        "episode_id": row["episode_id"],
        "mode": row["mode"],
        "condition_id": row["condition_id"],
        "trial_index": row["trial_index"],
        "success": None,
        "termination_reason": None,
        "control_steps": None,
        "accounting_model": accounting_model,
        "accounting_status": "not_available",
        "summary_policy_replan_count": None,
        "policy_requests_submitted": None,
        "logged_request_submission_count": None,
        "unique_submitted_replan_indices": None,
        "policy_results_completed_and_logged": None,
        "logged_policy_result_count": None,
        "unique_result_replan_indices": None,
        "inference_latency_count": None,
        "non_null_inference_latency_count": None,
        "last_submitted_replan_index": None,
        "last_result_replan_index": None,
        "last_logged_policy_worker_pending": None,
        "submitted_minus_result_count": None,
        "submitted_minus_latency_count": None,
        "terminal_unobserved_request_count": None,
        "terminal_unobserved_replan_index": None,
    }


def _shared_policy_accounting(
    *,
    episode_id: str,
    summary: dict[str, Any],
    steps: list[dict[str, Any]],
    waits: list[dict[str, Any]],
    errors: list[str],
) -> dict[str, Any]:
    """Validate submitted versus observed asynchronous policy requests."""

    records = _ordered_records(steps, waits)
    submitted_values, submitted_indices = _indices(
        records,
        "policy_request_replan_index",
    )
    result_values, result_indices = _indices(
        records,
        "policy_result_replan_index",
    )
    latency_count = sum(
        record.get("inference_latency_seconds") is not None
        for record in records
    )
    submitted_count = int(summary["policy_replan_count"])
    completed_count = len(result_indices)
    submitted_minus_result = submitted_count - completed_count
    submitted_minus_latency = submitted_count - latency_count
    expected_indices = list(range(submitted_count))
    terminal_index = submitted_count - 1
    final_step = steps[-1]
    terminal_outstanding = (
        submitted_minus_result == 1
        and result_indices == expected_indices[:-1]
        and final_step.get("policy_request_replan_index") == terminal_index
        and final_step.get("policy_worker_pending") is True
    )

    accounting_errors: list[str] = []
    if len(submitted_values) != len(submitted_indices):
        accounting_errors.append("duplicate submitted replan indices")
    if len(result_values) != completed_count:
        accounting_errors.append("duplicate completed replan indices")
    if completed_count != latency_count:
        accounting_errors.append(
            "completed policy-result count does not match inference-latency "
            "count"
        )
    if submitted_minus_result < 0 or submitted_minus_result > 1:
        accounting_errors.append(
            "submitted-minus-completed policy request count is outside [0, 1]"
        )
    if submitted_minus_latency != submitted_minus_result:
        accounting_errors.append(
            "submitted-minus-latency count does not match "
            "submitted-minus-completed count"
        )
    if result_indices != expected_indices[:completed_count]:
        accounting_errors.append(
            "completed replan indices are not a contiguous prefix"
        )
    if sorted(set(submitted_indices).union(result_indices)) != expected_indices:
        accounting_errors.append(
            "submitted requests cannot be reconstructed as contiguous indices"
        )
    if submitted_minus_result == 1 and not terminal_outstanding:
        accounting_errors.append(
            "single outstanding request is not demonstrably terminal/in-flight"
        )
    action_indices = {
        int(step["policy_replan_index"])
        for step in steps
        if step.get("policy_replan_index") is not None
    }
    unexplained_action_indices = sorted(action_indices.difference(result_indices))
    if unexplained_action_indices:
        accounting_errors.append(
            "executed action references unlogged policy results "
            f"{unexplained_action_indices}"
        )
    errors.extend(
        f"{episode_id}: policy accounting: {message}."
        for message in accounting_errors
    )

    last_record = records[-1]
    status = (
        "invalid"
        if accounting_errors
        else (
            "terminal_request_unobserved"
            if terminal_outstanding
            else "complete"
        )
    )
    return {
        **_accounting_identity(
            episode_id=episode_id,
            mode=str(summary["arbitration_mode"]),
            condition_id=str(summary["condition_id"]),
            trial_index=int(summary["trial_index"]),
            success=bool(summary["success"]),
            termination_reason=str(summary["termination_reason"]),
            control_steps=int(summary["control_steps"]),
        ),
        "accounting_model": "shared_asynchronous",
        "accounting_status": status,
        "summary_policy_replan_count": submitted_count,
        "policy_requests_submitted": submitted_count,
        "logged_request_submission_count": len(submitted_indices),
        "unique_submitted_replan_indices": json.dumps(submitted_indices),
        "policy_results_completed_and_logged": completed_count,
        "logged_policy_result_count": completed_count,
        "unique_result_replan_indices": json.dumps(result_indices),
        "inference_latency_count": latency_count,
        "non_null_inference_latency_count": latency_count,
        "last_submitted_replan_index": (
            submitted_indices[-1] if submitted_indices else None
        ),
        "last_result_replan_index": (
            result_indices[-1] if result_indices else None
        ),
        "last_logged_policy_worker_pending": bool(
            last_record.get("policy_worker_pending", False)
        ),
        "submitted_minus_result_count": submitted_minus_result,
        "submitted_minus_latency_count": submitted_minus_latency,
        "terminal_unobserved_request_count": int(terminal_outstanding),
        "terminal_unobserved_replan_index": (
            terminal_index if terminal_outstanding else None
        ),
    }


def _autonomous_policy_accounting(
    *,
    episode_id: str,
    summary: dict[str, Any],
    steps: list[dict[str, Any]],
    errors: list[str],
) -> dict[str, Any]:
    """Validate synchronous autonomous replans and measured latencies."""

    replan_records = [step for step in steps if bool(step.get("replanned"))]
    replan_values = [
        int(step["policy_replan_index"])
        for step in replan_records
        if step.get("policy_replan_index") is not None
    ]
    replan_indices = sorted(set(replan_values))
    latency_count = sum(
        step.get("inference_latency_seconds") is not None for step in steps
    )
    submitted_count = int(summary["policy_replan_count"])
    accounting_errors: list[str] = []
    if len(replan_values) != len(replan_records):
        accounting_errors.append("a synchronous replan lacks a replan index")
    if len(replan_values) != len(replan_indices):
        accounting_errors.append("duplicate synchronous replan indices")
    if replan_indices != list(range(submitted_count)):
        accounting_errors.append(
            "synchronous replan indices do not match submitted requests"
        )
    if len(replan_records) != latency_count:
        accounting_errors.append(
            "synchronous replan count does not match inference-latency count"
        )
    action_mismatches = [
        int(step.get("control_step", index))
        for index, step in enumerate(steps)
        if step.get("executed_action") is not None
        and step.get("policy_action") is not None
        and step["executed_action"] != step["policy_action"]
    ]
    if action_mismatches:
        accounting_errors.append(
            "autonomous executed actions differ from logged policy actions at "
            f"steps {action_mismatches}"
        )
    errors.extend(
        f"{episode_id}: policy accounting: {message}."
        for message in accounting_errors
    )
    completed_count = len(replan_records)
    return {
        **_accounting_identity(
            episode_id=episode_id,
            mode="autonomous",
            condition_id=str(summary["condition_id"]),
            trial_index=int(summary["trial_index"]),
            success=bool(summary["success"]),
            termination_reason=("success" if summary["success"] else "timeout"),
            control_steps=int(summary["control_steps"]),
        ),
        "accounting_model": "autonomous_synchronous",
        "accounting_status": "invalid" if accounting_errors else "complete",
        "summary_policy_replan_count": submitted_count,
        "policy_requests_submitted": submitted_count,
        "logged_request_submission_count": len(replan_indices),
        "unique_submitted_replan_indices": json.dumps(replan_indices),
        "policy_results_completed_and_logged": completed_count,
        "logged_policy_result_count": completed_count,
        "unique_result_replan_indices": json.dumps(replan_indices),
        "inference_latency_count": latency_count,
        "non_null_inference_latency_count": latency_count,
        "last_submitted_replan_index": (
            replan_indices[-1] if replan_indices else None
        ),
        "last_result_replan_index": (
            replan_indices[-1] if replan_indices else None
        ),
        "last_logged_policy_worker_pending": False,
        "submitted_minus_result_count": submitted_count - completed_count,
        "submitted_minus_latency_count": submitted_count - latency_count,
        "terminal_unobserved_request_count": 0,
        "terminal_unobserved_replan_index": None,
    }


def _write_csv(
    path: Path,
    rows: list[dict[str, Any]],
    *,
    fieldnames: Iterable[str] | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(fieldnames) if fieldnames is not None else None
    if not rows and fields is None:
        path.write_text("\n", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=fields or list(rows[0]),
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)


def _shared_inputs(
    session_root: Path,
    *,
    errors: list[str],
    warnings: list[str],
) -> tuple[Any, dict[str, Any]]:
    manifest_path = session_root / "manifest.json"
    schedule_path = session_root / "schedule.json"
    if not manifest_path.exists() and not schedule_path.exists():
        manifest = load_manifest(Path(GATE2_V2_MANIFEST_PATH))
        schedule = build_gate2_v2_shared_schedule(
            manifest=manifest,
            task_id=GATE2_V2_TASK_ID,
            output_root=Path(GATE2_V2_SHARED_OUTPUT_ROOT),
        )
        warnings.append("Shared Gate-2 v2 collection has not started.")
        return manifest, schedule
    if not manifest_path.is_file() or not schedule_path.is_file():
        errors.append("Shared collection has incomplete frozen root metadata.")
        manifest = load_manifest(Path(GATE2_V2_MANIFEST_PATH))
        return manifest, build_gate2_v2_shared_schedule(
            manifest=manifest,
            task_id=GATE2_V2_TASK_ID,
            output_root=Path(GATE2_V2_SHARED_OUTPUT_ROOT),
        )
    manifest = load_manifest(manifest_path)
    validate_gate2_v2_manifest(manifest)
    schedule = _read_json(schedule_path)
    validate_gate2_v2_shared_schedule(schedule, manifest=manifest)
    if schedule.get("manifest_sha256") != manifest_sha256(manifest):
        errors.append("Shared schedule manifest hash does not match.")
    return manifest, schedule


def _validate_shared_provenance(
    session_root: Path,
    *,
    manifest: Any,
    errors: list[str],
) -> None:
    if not (session_root / "manifest.json").is_file():
        return
    required = (
        "human_input.json",
        "perturbation_config.json",
        "repository_provenance.json",
        "session_protocol.json",
    )
    values: dict[str, dict[str, Any]] = {}
    for name in required:
        path = session_root / name
        if not path.is_file():
            errors.append(f"Shared frozen {name} is missing.")
        else:
            values[name] = _read_json(path)
    human = values.get("human_input.json", {})
    profile = human.get("spacemouse_profile")
    if (
        human.get("input_source") != "spacemouse"
        or not isinstance(profile, dict)
        or profile.get("sha256") != GATE2_V2_PROFILE_SHA256
    ):
        errors.append("Shared frozen SpaceMouse identity does not match v2.")
    perturbation = values.get("perturbation_config.json", {})
    if perturbation.get("sha256") != GATE2_V2_CONFIG_SHA256:
        errors.append("Shared perturbation configuration does not match v2.")
    repository = values.get("repository_provenance.json", {})
    commit = str(repository.get("repository_commit", ""))
    if len(commit) != 40 or repository.get(
        "manifest_sha256"
    ) != manifest_sha256(manifest):
        errors.append("Shared repository provenance does not match v2.")
    protocol = values.get("session_protocol.json", {})
    if protocol.get("required_protocol_id") != GATE2_V2_SHARED_EXPERIMENT_ID:
        errors.append("Shared required protocol identity does not match v2.")
    expected_autonomous = json_file_identity(
        Path(GATE2_V2_AUTONOMOUS_PROTOCOL_PATH)
    )
    if protocol.get("matched_autonomous_protocol") != expected_autonomous:
        errors.append("Shared frozen autonomous protocol does not match v2.")


def _shared_rows(
    *,
    session_root: Path,
    manifest: Any,
    schedule: dict[str, Any],
    errors: list[str],
) -> tuple[
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
]:
    episode_rows: list[dict[str, Any]] = []
    fixed_rows: list[dict[str, Any]] = []
    cosine_rows: list[dict[str, Any]] = []
    wait_rows: list[dict[str, Any]] = []
    accounting_rows: list[dict[str, Any]] = []
    root_started = (session_root / "schedule.json").is_file()
    for episode in schedule["episodes"]:
        row = _base_episode_row(episode)
        selected = _selected_attempts(episode) if root_started else []
        if len(selected) > 1:
            errors.append(
                f"{episode['episode_id']}: multiple selected attempts."
            )
        if len(selected) != 1:
            if root_started and episode.get("status") == "completed":
                errors.append(
                    f"{episode['episode_id']}: completed without one "
                    "selected valid attempt."
                )
            episode_rows.append(row)
            wait_rows.append(_blank_wait(row))
            accounting_rows.append(
                _blank_policy_accounting(
                    row,
                    accounting_model="shared_asynchronous",
                )
            )
            if episode["mode"] == "fixed_blend":
                fixed_rows.append(
                    _blank_fixed(row, manifest.fixed_autonomy_weight)
                )
            else:
                cosine_rows.append(_blank_cosine(row))
            continue
        attempt = selected[0]
        summary_path = Path(str(attempt["summary_path"]))
        try:
            analyzed, fixed, cosine, wait = _episode_analysis(
                episode=episode,
                summary_path=summary_path,
                manifest=manifest,
                expected_profile_sha256=GATE2_V2_PROFILE_SHA256,
                errors=errors,
                require_submitted_latency_equality=False,
            )
            summary = _read_json(summary_path)
            steps = _read_jsonl(summary_path.with_name("steps.jsonl"))
            waits = _read_jsonl(
                summary_path.with_name("scheduler_waits.jsonl")
            )
            accounting = _shared_policy_accounting(
                episode_id=str(episode["episode_id"]),
                summary=summary,
                steps=steps,
                waits=waits,
                errors=errors,
            )
            analyzed["selected_attempt_number"] = int(
                attempt["attempt_number"]
            )
            analyzed["selected_attempt_valid"] = 1
        except (KeyError, OSError, TypeError, ValueError) as error:
            errors.append(f"{episode['episode_id']}: {error}")
            analyzed = row
            fixed = None
            cosine = None
            wait = _blank_wait(row)
            accounting = _blank_policy_accounting(
                row,
                accounting_model="shared_asynchronous",
            )
        episode_rows.append(analyzed)
        wait_rows.append(wait)
        accounting_rows.append(accounting)
        if episode["mode"] == "fixed_blend":
            fixed_rows.append(
                fixed or _blank_fixed(row, manifest.fixed_autonomy_weight)
            )
        else:
            cosine_rows.append(cosine or _blank_cosine(row))
    return (
        episode_rows,
        fixed_rows,
        cosine_rows,
        wait_rows,
        accounting_rows,
    )


def _autonomous_summary_path(root: Path, episode: dict[str, Any]) -> Path:
    return (
        root
        / str(episode["condition_id"])
        / f"task_{GATE2_V2_TASK_ID:02d}"
        / f"init_{int(episode['initial_state_index']):03d}"
        / f"trial_{int(episode['trial_index']):03d}"
        / "summary.json"
    )


def _autonomous_row(
    *,
    episode: dict[str, Any],
    summary_path: Path,
    errors: list[str],
) -> tuple[dict[str, Any], dict[str, Any]]:
    row = _base_episode_row(
        {
            **episode,
            "episode_id": (
                f"trial_{int(episode['trial_index']):03d}__"
                f"condition_{episode['condition_id']}__mode_autonomous"
            ),
            "status": "completed",
        }
    )
    summary = _read_json(summary_path)
    expected = {
        "arbitration_mode": "autonomous",
        "condition_id": episode["condition_id"],
        "task_id": GATE2_V2_TASK_ID,
        "trial_index": episode["trial_index"],
        "initial_state_index": episode["initial_state_index"],
        "policy_episode_seed": episode["policy_episode_seed"],
        "policy_seed_protocol": SEED_PROTOCOL,
    }
    mismatch = [
        field for field, value in expected.items() if summary.get(field) != value
    ]
    if mismatch:
        errors.append(
            f"{row['episode_id']}: autonomous summary identity mismatch "
            f"in {mismatch}."
        )
        return row, _blank_policy_accounting(
            row,
            accounting_model="autonomous_synchronous",
        )
    control_steps = int(summary["control_steps"])
    success = bool(summary["success"])
    if not 1 <= control_steps <= GATE2_V2_MAX_STEPS:
        errors.append(f"{row['episode_id']}: invalid autonomous step count.")
        return row, _blank_policy_accounting(
            row,
            accounting_model="autonomous_synchronous",
        )
    if not success and control_steps != GATE2_V2_MAX_STEPS:
        errors.append(
            f"{row['episode_id']}: autonomous failure is not a full timeout."
        )
        return row, _blank_policy_accounting(
            row,
            accounting_model="autonomous_synchronous",
        )
    steps_path = summary_path.with_name("steps.jsonl")
    if not steps_path.is_file():
        errors.append(f"{row['episode_id']}: autonomous steps.jsonl is missing.")
        return row, _blank_policy_accounting(
            row,
            accounting_model="autonomous_synchronous",
        )
    steps = _read_jsonl(steps_path)
    if len(steps) != control_steps:
        errors.append(
            f"{row['episode_id']}: autonomous logged steps do not match."
        )
        return row, _blank_policy_accounting(
            row,
            accounting_model="autonomous_synchronous",
        )
    accounting = _autonomous_policy_accounting(
        episode_id=str(row["episode_id"]),
        summary=summary,
        steps=steps,
        errors=errors,
    )
    latency = _latency_metrics(steps, [])
    simulated = control_steps / GATE2_V2_CONTROL_FREQUENCY_HZ
    wall_control = _finite_float(summary["control_elapsed_seconds"])
    wall_total = _finite_float(summary["total_elapsed_seconds"])
    row.update(
        {
            "selected_attempt_number": 1,
            "selected_attempt_valid": 1,
            "metrics_available": 1,
            "success": int(success),
            "termination_reason": "success" if success else "timeout",
            "control_steps": control_steps,
            "logged_steps": len(steps),
            "simulated_duration_seconds": simulated,
            "raw_simulated_duration_seconds": simulated,
            "wall_control_duration_seconds": wall_control,
            "wall_total_duration_seconds": wall_total,
            "wall_simulation_ratio": wall_control / simulated,
            "human_active_steps": 0,
            "human_active_duration_seconds": 0.0,
            "human_active_fraction": 0.0,
            "correction_segments": 0,
            "translation_magnitude_mean": None,
            "translation_magnitude_active_mean": None,
            "translation_magnitude_max": None,
            "rotation_magnitude_mean": None,
            "rotation_magnitude_active_mean": None,
            "rotation_magnitude_max": None,
            "policy_wait_ticks": 0,
            "policy_wait_events": 0,
            "policy_wait_duration_seconds": 0.0,
            "policy_wait_wall_seconds": 0.0,
            "policy_wait_fraction": 0.0,
            "human_active_policy_wait_ticks": 0,
            "human_active_policy_wait_seconds": 0.0,
            "human_active_policy_wait_fraction": 0.0,
            **latency,
            "summary_path": str(summary_path),
        }
    )
    return row, accounting


def _validate_autonomous_root(
    *,
    root: Path,
    protocol: dict[str, Any],
    schedule: dict[str, Any],
    errors: list[str],
    warnings: list[str],
) -> None:
    if not root.exists():
        warnings.append("Autonomous Gate-2 v2 collection has not started.")
        return
    expected_summaries = {
        _autonomous_summary_path(root, episode)
        for episode in schedule["episodes"]
    }
    unexpected_summaries = set(root.rglob("summary.json")).difference(
        expected_summaries
    )
    if unexpected_summaries:
        errors.append(
            "Autonomous root contains summaries outside the frozen 20 cells: "
            + ", ".join(str(path) for path in sorted(unexpected_summaries))
        )
    required = {
        "protocol.json": protocol,
        "schedule.json": schedule,
    }
    for name, expected in required.items():
        path = root / name
        if not path.is_file():
            errors.append(f"Autonomous frozen {name} is missing.")
        elif _read_json(path) != expected:
            errors.append(f"Autonomous frozen {name} does not match v2.")
    perturbation_path = root / "perturbation_config.json"
    if (
        not perturbation_path.is_file()
        or _read_json(perturbation_path).get("sha256")
        != GATE2_V2_CONFIG_SHA256
    ):
        errors.append("Autonomous perturbation config does not match v2.")
    provenance_path = root / "repository_provenance.json"
    if not provenance_path.is_file():
        errors.append("Autonomous repository provenance is missing.")
    else:
        provenance = _read_json(provenance_path)
        commit = str(provenance.get("repository_commit", ""))
        if len(commit) != 40:
            errors.append("Autonomous repository commit is invalid.")
        expected_protocol = json_file_identity(
            Path(GATE2_V2_AUTONOMOUS_PROTOCOL_PATH)
        )
        if (
            provenance.get("protocol_path") != expected_protocol["path"]
            or provenance.get("protocol_sha256")
            != expected_protocol["sha256"]
        ):
            errors.append("Autonomous protocol provenance does not match v2.")


def _triplets(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    available = [row for row in rows if row["metrics_available"]]
    by_identity: dict[tuple[Any, ...], dict[str, dict[str, Any]]] = {}
    for row in available:
        key = tuple(row[field] for field in PAIRING_FIELDS)
        by_identity.setdefault(key, {})[str(row["mode"])] = row
    output = []
    for key, modes in sorted(by_identity.items()):
        if set(modes) != set(GATE2_V2_ALL_MODES):
            continue
        record = dict(zip(PAIRING_FIELDS, key))
        for mode in GATE2_V2_ALL_MODES:
            record[f"{mode}_success"] = modes[mode]["success"]
            record[f"{mode}_simulated_duration_seconds"] = modes[mode][
                "simulated_duration_seconds"
            ]
            record[f"{mode}_wall_control_duration_seconds"] = modes[mode][
                "wall_control_duration_seconds"
            ]
            record[f"{mode}_summary_path"] = modes[mode]["summary_path"]
        output.append(record)
    return output


def _summary_by_mode(
    rows: list[dict[str, Any]],
) -> dict[str, dict[str, Any]]:
    return {str(row["mode"]): row for row in rows}


def _summary_by_condition_mode(
    rows: list[dict[str, Any]],
) -> dict[tuple[str, str], dict[str, Any]]:
    return {
        (str(row["condition_id"]), str(row["mode"])): row
        for row in rows
    }


def _count(value: Any) -> str:
    return "—" if value is None else str(int(value))


def _percentage(value: Any) -> str:
    return "—" if value is None else f"{100.0 * float(value):.1f}%"


def _seconds(value: Any) -> str:
    return "—" if value is None else f"{float(value):.1f}"


def _decimal(value: Any, digits: int = 2) -> str:
    return "—" if value is None else f"{float(value):.{digits}f}"


def _write_report(
    path: Path,
    *,
    report: dict[str, Any],
    mode_summary: list[dict[str, Any]],
    condition_mode_summary: list[dict[str, Any]],
    fixed_rows: list[dict[str, Any]],
    cosine_aggregate: dict[str, Any],
) -> None:
    by_mode = _summary_by_mode(mode_summary)
    by_cell = _summary_by_condition_mode(condition_mode_summary)

    def success_cell(condition: str, mode: str) -> str:
        row = by_cell[(condition, mode)]
        valid = int(row["n_selected_valid"])
        if valid == 0:
            return "—"
        return f"{int(row['n_success'])}/{valid}"

    fixed = by_mode["fixed_blend"]
    cosine = by_mode["cosine_blend"]
    fixed_wait_ticks = int(fixed["policy_wait_ticks_total"])
    cosine_wait_ticks = int(cosine["policy_wait_ticks_total"])
    fixed_wait_active = int(fixed["human_active_policy_wait_ticks_total"])
    cosine_wait_active = int(cosine["human_active_policy_wait_ticks_total"])
    fixed_wait_active_fraction = (
        fixed_wait_active / fixed_wait_ticks if fixed_wait_ticks else None
    )
    cosine_wait_active_fraction = (
        cosine_wait_active / cosine_wait_ticks if cosine_wait_ticks else None
    )
    fixed_available = [
        row for row in fixed_rows if row["metrics_available"]
    ]
    fixed_within_tolerance = sum(
        bool(row["within_tolerance"]) for row in fixed_available
    )
    complete_report = bool(report["ready_for_complete_gate2_report"])
    purpose = (
        "This completed pilot closes the LIBERO simulation-baseline stage of "
        "the SAPS–OpenPI replication. It establishes reference behavior and "
        "validates the deployment, operator-control, arbitration, logging, "
        "provenance, and analysis pipeline before transfer to a fixed physical "
        "robot. It is descriptive infrastructure evidence, not a powered "
        "comparison of arbitration methods."
        if complete_report
        else (
            "This report validates an incomplete or local copy of the frozen "
            "LIBERO matched-pilot design. It must not be presented as the final "
            "simulation baseline until all coverage checks pass."
        )
    )
    matched_interpretation = (
        "Fixed recovered some autonomous failures while preserving the "
        "autonomous successes in this small sample. Cosine recovered all "
        "eight matched autonomous failures. Two `p02` identities that "
        "succeeded autonomously failed under Cosine. Those two failures do "
        "not establish an intrinsic Cosine regression: operator input, "
        "intervention timing, post-intervention policy state, arbitration, "
        "trajectory-specific effects, and their interactions cannot be "
        "distinguished with this experiment."
        if complete_report
        else (
            "Matched outcome interpretation is withheld until all 60 outcomes "
            "and 20 exact triplets pass validation."
        )
    )
    conclusion = (
        "The simulation study established a reproducible π0.5/SAPS baseline "
        "and validated the complete shared-autonomy execution and logging "
        "pipeline. Autonomous performance degraded under larger object-position "
        "perturbations, while Fixed and Cosine action blending enabled recovery "
        "in several cases where autonomous execution failed. Because the pilot "
        "used a single task, one operator, selected perturbations, and five "
        "repetitions per condition-method cell, it is not intended as a powered "
        "comparison between arbitration methods. It provides reference behavior "
        "and deployment infrastructure for the next stage: ordinary chunked-VLA "
        "SAPS on a fixed physical robot, followed by research on autonomous-"
        "continuation risk, intervention, recovery, autonomy resumption, and "
        "selective learning from intervention."
        if complete_report
        else (
            "The collection is not yet complete and cannot be presented as the "
            "final simulation baseline."
        )
    )
    terminal_count = int(
        report["policy_accounting"][
            "shared_async_terminal_unobserved_episode_count"
        ]
    )
    accounting_clarification = (
        f"The {terminal_count} terminal requests have no logged latency and "
        "are correctly excluded from latency statistics. This does not "
        "invalidate any raw trajectory. All 20 autonomous episodes have "
        "complete synchronous accounting."
        if complete_report
        else (
            "Any accepted terminal request has no logged latency and is "
            "excluded from latency statistics. Complete-collection accounting "
            "claims are withheld until all outcomes validate."
        )
    )

    lines = [
        "# LIBERO matched simulation pilot report",
        "",
        purpose,
        "",
        "## Experimental design",
        "",
        (
            "The frozen matched design contains 20 autonomous, 20 Fixed, and "
            "20 Cosine outcomes. Conditions `nominal`, `p02`, `p06`, and `p09` "
            "each have five trials per mode. The 20 condition/trial identities "
            "are exact matched triplets with the same initial state and policy "
            "seed protocol. Fixed uses autonomy weight `0.5`; Cosine uses gain "
            "`k = 6`."
        ),
        "",
        "## Frozen provenance",
        "",
        "| Identity | Frozen value |",
        "|---|---|",
        f"| Collection commit | `{COLLECTION_COMMIT}` |",
        f"| Accounting-analysis commit | `{ACCOUNTING_ANALYSIS_COMMIT}` |",
        f"| OpenPI submodule | `{OPENPI_COMMIT}` |",
        f"| LIBERO submodule | `{LIBERO_COMMIT}` |",
        f"| Policy checkpoint | `{POLICY_CHECKPOINT}` |",
        f"| SpaceMouse profile SHA-256 | `{GATE2_V2_PROFILE_SHA256}` |",
        f"| Shared manifest SHA-256 | `{SHARED_MANIFEST_SHA256}` |",
        (
            "| Autonomous protocol SHA-256 | "
            f"`{AUTONOMOUS_PROTOCOL_SHA256}` |"
        ),
        f"| Perturbation config SHA-256 | `{GATE2_V2_CONFIG_SHA256}` |",
        "",
        (
            "The collection commit is the repository revision recorded by both "
            "raw collections. The later accounting-analysis commit is kept "
            "separate because it changed validation of asynchronous request/"
            "result accounting, not the collected trajectories."
        ),
        "",
        f"Raw data remain immutable at `{report['session_root']}` and "
        f"`{report['autonomous_root']}`. This report and its CSV/JSON "
        "companions are derived artifacts.",
        "",
        "## Coverage and validation",
        "",
        "- Expected outcomes: `60`",
        (
            "- Analyzable outcomes: "
            f"`{report['selected_analyzable_episode_count']}/60`"
        ),
        f"- Exact complete triplets: `{report['matched_triplet_count']}/20`",
        f"- Complete collection: `{report['collection_complete']}`",
        f"- Analysis valid: `{report['analysis_valid']}`",
        f"- Blocking validation errors: `{len(report['blocking_errors'])}`",
        "",
        "## Descriptive success outcomes",
        "",
        "| Condition | Autonomous | Fixed | Cosine |",
        "|---|---:|---:|---:|",
        *(
            (
                f"| {condition} | {success_cell(condition, 'autonomous')} | "
                f"{success_cell(condition, 'fixed_blend')} | "
                f"{success_cell(condition, 'cosine_blend')} |"
            )
            for condition in GATE2_V2_CONDITIONS
        ),
        (
            "| Overall | "
            f"{_count(by_mode['autonomous']['n_success'])}/"
            f"{_count(by_mode['autonomous']['n_selected_valid'])} "
            f"({_percentage(by_mode['autonomous']['success_rate_observed'])}) | "
            f"{_count(fixed['n_success'])}/{_count(fixed['n_selected_valid'])} "
            f"({_percentage(fixed['success_rate_observed'])}) | "
            f"{_count(cosine['n_success'])}/"
            f"{_count(cosine['n_selected_valid'])} "
            f"({_percentage(cosine['success_rate_observed'])}) |"
        ),
        "",
        (
            "These are descriptive observations only; no statistical-"
            "significance claim is attached."
        ),
        "",
        "## Matched outcome interpretation",
        "",
        matched_interpretation,
        "",
        "## Human effort",
        "",
        (
            "`human_active_duration_seconds` and `human_active_fraction` use "
            "only actual environment/control steps. They exclude human input "
            "during policy-wait ticks, when the simulation is paused. That "
            "wait-period activity is logged separately."
        ),
        "",
        "| Metric | Fixed | Cosine |",
        "|---|---:|---:|",
        (
            "| Mean step-based human-active fraction | "
            f"{_percentage(fixed['human_active_fraction_mean'])} | "
            f"{_percentage(cosine['human_active_fraction_mean'])} |"
        ),
        (
            "| Mean step-based human-active duration (s) | "
            f"{_seconds(fixed['human_active_duration_mean_seconds'])} | "
            f"{_seconds(cosine['human_active_duration_mean_seconds'])} |"
        ),
        (
            "| Mean correction segments per episode | "
            f"{_decimal(fixed['correction_segments_mean'])} | "
            f"{_decimal(cosine['correction_segments_mean'])} |"
        ),
        (
            "| Policy-wait duration, all episodes (s) | "
            f"{_seconds(fixed['policy_wait_duration_total_seconds'])} | "
            f"{_seconds(cosine['policy_wait_duration_total_seconds'])} |"
        ),
        (
            "| Human-active policy-wait duration (s) | "
            f"{fixed_wait_active / GATE2_V2_CONTROL_FREQUENCY_HZ:.1f} | "
            f"{cosine_wait_active / GATE2_V2_CONTROL_FREQUENCY_HZ:.1f} |"
        ),
        (
            "| Human-active share of policy-wait ticks | "
            f"{_percentage(fixed_wait_active_fraction)} | "
            f"{_percentage(cosine_wait_active_fraction)} |"
        ),
        "",
        (
            "Including wait-period activity modestly changes the overall "
            "intervention picture but does not explain the roughly 50% "
            "step-based rates. These fractions must not be compared directly "
            "with the roughly 10.8% and 30% LIBERO-PRO rates reported by SAPS, "
            "which use a different benchmark and protocol. SAPS real-world "
            "fractions are more comparable in magnitude, but this pilot does "
            "not establish exact cross-paper equivalence."
        ),
        "",
        "## Arbitration diagnostics",
        "",
        (
            f"All `{fixed_within_tolerance}/{len(fixed_available)}` analyzable "
            "Fixed episodes used active-human autonomy weight `0.5` within "
            "absolute tolerance `1e-9`; no deviation was detected."
        ),
        "",
        (
            "Cosine produced "
            f"`{_count(cosine_aggregate.get('weight_count'))}` defined active "
            "weights and "
            f"`{_count(cosine_aggregate.get('undefined_weight_count'))}` "
            "undefined weights. Across active steps, the aggregate mean weight "
            f"was `{_decimal(cosine_aggregate.get('weight_mean'), 3)}`, "
            f"with `{_percentage(cosine_aggregate.get('near_zero_fraction'))}` "
            "near zero, "
            f"`{_percentage(cosine_aggregate.get('near_one_fraction'))}` near "
            "one, and "
            f"`{_percentage(cosine_aggregate.get('intermediate_fraction'))}` "
            "intermediate. These are implementation diagnostics, not evidence "
            "of method superiority."
        ),
        "",
        "## Timing semantics",
        "",
        (
            "In this simulation baseline, policy waits do not advance robot "
            "simulation state, environment state, or simulation/environment "
            "time; wall clock does advance. Simulated execution time is "
            "`control_steps / 20 Hz`, while wall-control and total wall time "
            "are retained separately. Autonomous synchronous inference also "
            "contributes to wall time."
        ),
        "",
        (
            "For the next conventional physical chunked-VLA baseline, the "
            "robot may hold its commanded state or stop motion while waiting "
            "for the next action chunk, but the physical external environment "
            "and wall clock continue evolving. Stop/replan/continue is a valid "
            "baseline. Continuous execution methods such as real-time chunking "
            "may later reduce pauses, but RTC is outside this baseline."
        ),
        "",
        "## Policy accounting",
        "",
        (
            "Shared `policy_replan_count` counts submitted asynchronous "
            "requests. Completed/logged results must equal measured latency "
            "samples. A difference of one is accepted only for a contiguous "
            "final request logged as pending on the terminal step."
        ),
        "",
        (
            "- Shared complete accounting: "
            f"`{report['policy_accounting']['shared_async_complete_episode_count']}`"
        ),
        (
            "- Shared terminal-unobserved requests: "
            "`"
            f"{report['policy_accounting']['shared_async_terminal_unobserved_episode_count']}"
            "`"
        ),
        (
            "- Autonomous synchronous complete accounting: "
            f"`{report['policy_accounting']['autonomous_sync_complete_episode_count']}`"
        ),
        "",
        accounting_clarification,
        "",
        "## Limitations",
        "",
        "- One LIBERO task.",
        "- Four selected perturbation conditions.",
        "- Five repetitions per condition-mode cell.",
        "- One nonexpert operator and one SpaceMouse interface.",
        "- No multi-operator study.",
        "- Descriptive excluded pilot, not a powered method comparison.",
        "",
        (
            "A single nonexpert operator is not by itself inconsistent with "
            "the SAPS within-benchmark setup, but this sample and its task and "
            "participant coverage are insufficient for a paper-level "
            "comparison of arbitration methods."
        ),
        "",
        "## Conclusion and transition",
        "",
        conclusion,
    ]
    if report["blocking_errors"]:
        lines.extend(["", "## Blocking errors", ""])
        lines.extend(f"- {value}" for value in report["blocking_errors"])
    if report["warnings"]:
        lines.extend(["", "## Warnings", ""])
        lines.extend(f"- {value}" for value in report["warnings"])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze_gate2_v2_collection(
    *,
    session_root: Path,
    autonomous_root: Path,
    output_dir: Path,
) -> dict[str, Any]:
    """Analyze any partial state of the two frozen Gate-2 v2 datasets."""

    errors: list[str] = []
    warnings = [
        "This is a descriptive excluded pilot, not a powered experiment."
    ]
    manifest, shared_schedule = _shared_inputs(
        session_root,
        errors=errors,
        warnings=warnings,
    )
    _validate_shared_provenance(
        session_root,
        manifest=manifest,
        errors=errors,
    )
    (
        shared_rows,
        fixed_rows,
        cosine_rows,
        wait_rows,
        shared_accounting_rows,
    ) = _shared_rows(
        session_root=session_root,
        manifest=manifest,
        schedule=shared_schedule,
        errors=errors,
    )

    autonomous_protocol = load_gate2_v2_autonomous_protocol(
        Path(GATE2_V2_AUTONOMOUS_PROTOCOL_PATH)
    )
    autonomous_schedule = build_gate2_v2_autonomous_schedule(
        autonomous_protocol
    )
    validate_gate2_v2_autonomous_schedule(
        autonomous_schedule,
        protocol=autonomous_protocol,
    )
    _validate_autonomous_root(
        root=autonomous_root,
        protocol=autonomous_protocol,
        schedule=autonomous_schedule,
        errors=errors,
        warnings=warnings,
    )
    autonomous_rows = []
    autonomous_accounting_rows = []
    for episode in autonomous_schedule["episodes"]:
        summary_path = _autonomous_summary_path(autonomous_root, episode)
        if summary_path.is_file():
            try:
                row, accounting = _autonomous_row(
                    episode=episode,
                    summary_path=summary_path,
                    errors=errors,
                )
            except (KeyError, OSError, TypeError, ValueError) as error:
                row = _base_episode_row(
                    {
                        **episode,
                        "episode_id": (
                            f"trial_{int(episode['trial_index']):03d}__"
                            f"condition_{episode['condition_id']}__"
                            "mode_autonomous"
                        ),
                        "status": "invalid",
                    }
                )
                errors.append(f"{row['episode_id']}: {error}")
                accounting = _blank_policy_accounting(
                    row,
                    accounting_model="autonomous_synchronous",
                )
        else:
            row = _base_episode_row(
                {
                    **episode,
                    "episode_id": (
                        f"trial_{int(episode['trial_index']):03d}__"
                        f"condition_{episode['condition_id']}__mode_autonomous"
                    ),
                    "status": "pending",
                }
            )
            accounting = _blank_policy_accounting(
                row,
                accounting_model="autonomous_synchronous",
            )
        autonomous_rows.append(row)
        autonomous_accounting_rows.append(accounting)

    all_rows = [*autonomous_rows, *shared_rows]
    mode_summary = _aggregate(
        all_rows,
        group_fields=("mode",),
        group_values=[(mode,) for mode in GATE2_V2_ALL_MODES],
    )
    condition_mode_summary = _aggregate(
        all_rows,
        group_fields=("condition_id", "mode"),
        group_values=[
            (condition, mode)
            for condition in GATE2_V2_CONDITIONS
            for mode in GATE2_V2_ALL_MODES
        ],
    )
    triplets = _triplets(all_rows)
    available_count = sum(int(row["metrics_available"]) for row in all_rows)
    analyzable_by_mode = Counter(
        str(row["mode"]) for row in all_rows if row["metrics_available"]
    )
    if available_count < 60:
        warnings.append(
            f"Collection is incomplete: {available_count}/60 outcomes are "
            "analyzable."
        )
    fixed_deviations = [
        str(row["episode_id"])
        for row in fixed_rows
        if row["metrics_available"] and not row["within_tolerance"]
    ]
    if fixed_deviations:
        errors.append(
            "Fixed-blend alpha differs from 0.5 in: "
            + ", ".join(fixed_deviations)
        )
    accounting_rows = [
        *autonomous_accounting_rows,
        *shared_accounting_rows,
    ]
    shared_terminal_outstanding = sum(
        int(row["terminal_unobserved_request_count"] or 0)
        for row in shared_accounting_rows
    )
    if shared_terminal_outstanding:
        warnings.append(
            f"{shared_terminal_outstanding} shared episodes ended with one "
            "terminal policy request still in flight; those requests have no "
            "logged latency and are excluded from latency statistics."
        )
    report = {
        "schema_version": 1,
        "shared_experiment_id": GATE2_V2_SHARED_EXPERIMENT_ID,
        "autonomous_experiment_id": GATE2_V2_AUTONOMOUS_EXPERIMENT_ID,
        "session_root": str(session_root),
        "autonomous_root": str(autonomous_root),
        "scheduled_episode_count": 60,
        "scheduled_by_mode": {mode: 20 for mode in GATE2_V2_ALL_MODES},
        "selected_analyzable_episode_count": available_count,
        "analyzable_by_mode": dict(analyzable_by_mode),
        "matched_triplet_count": len(triplets),
        "collection_complete": available_count == 60,
        "analysis_valid": not errors,
        "ready_for_descriptive_analysis": available_count > 0 and not errors,
        "ready_for_complete_gate2_report": (
            available_count == 60 and len(triplets) == 20 and not errors
        ),
        "timing": {
            "environment_time_definition": "control_steps / 20 Hz",
            "environment_step_horizon": GATE2_V2_MAX_STEPS,
            "nominal_timeout_simulated_seconds": 14.0,
            "wall_clock_is_separate": True,
            "shared_wait_ticks_advance_environment": False,
            "autonomous_inference_contributes_to_wall_time": True,
        },
        "fixed_blend_deviation_episode_ids": fixed_deviations,
        "policy_accounting": {
            "shared_async_complete_episode_count": sum(
                row["accounting_status"] == "complete"
                for row in shared_accounting_rows
            ),
            "shared_async_terminal_unobserved_episode_count": (
                shared_terminal_outstanding
            ),
            "shared_async_invalid_episode_count": sum(
                row["accounting_status"] == "invalid"
                for row in shared_accounting_rows
            ),
            "autonomous_sync_complete_episode_count": sum(
                row["accounting_status"] == "complete"
                for row in autonomous_accounting_rows
            ),
            "autonomous_sync_invalid_episode_count": sum(
                row["accounting_status"] == "invalid"
                for row in autonomous_accounting_rows
            ),
        },
        "blocking_errors": errors,
        "warnings": warnings,
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(output_dir / "episode_metrics.csv", all_rows)
    _write_csv(output_dir / "mode_summary.csv", mode_summary)
    _write_csv(
        output_dir / "condition_mode_summary.csv",
        condition_mode_summary,
    )
    _write_csv(
        output_dir / "matched_triplets.csv",
        triplets,
        fieldnames=TRIPLET_FIELDS,
    )
    _write_csv(output_dir / "fixed_blend_diagnostics.csv", fixed_rows)
    cosine_aggregate = _aggregate_cosine_diagnostics(cosine_rows)
    _write_csv(
        output_dir / "cosine_blend_diagnostics.csv",
        [
            *(_public_cosine_row(row) for row in cosine_rows),
            cosine_aggregate,
        ],
    )
    _write_csv(output_dir / "policy_wait_summary.csv", wait_rows)
    _write_csv(
        output_dir / "policy_accounting_diagnostics.csv",
        accounting_rows,
        fieldnames=POLICY_ACCOUNTING_FIELDS,
    )
    (output_dir / "validation_report.json").write_text(
        json.dumps(report, indent=2) + "\n",
        encoding="utf-8",
    )
    _write_report(
        output_dir / "REPORT.md",
        report=report,
        mode_summary=mode_summary,
        condition_mode_summary=condition_mode_summary,
        fixed_rows=fixed_rows,
        cosine_aggregate=cosine_aggregate,
    )
    return report
