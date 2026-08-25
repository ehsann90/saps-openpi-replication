"""Descriptive analysis and readiness validation for the Gate-2 pilot."""

from __future__ import annotations

from collections import Counter
import csv
import json
import math
from pathlib import Path
import statistics
from typing import Any, Iterable

from saps.arbitration import SAPS_ACTIVITY_THRESHOLD
from saps.evaluation.experiment_session import load_manifest
from saps.evaluation.experiment_session import manifest_sha256
from saps.evaluation.gate2_protocol import GATE2_CONDITIONS
from saps.evaluation.gate2_protocol import GATE2_CONFIG_SHA256
from saps.evaluation.gate2_protocol import GATE2_EXPERIMENT_ID
from saps.evaluation.gate2_protocol import GATE2_MODES
from saps.evaluation.gate2_protocol import GATE2_PROFILE_SHA256
from saps.evaluation.gate2_protocol import validate_gate2_attempt_completion
from saps.evaluation.gate2_protocol import validate_gate2_manifest
from saps.evaluation.gate2_protocol import validate_gate2_schedule


COSINE_NEAR_ZERO_MAX = 0.10
COSINE_NEAR_ONE_MIN = 0.90
COSINE_MATERIAL_CHANGE_MIN = 0.05
FIXED_WEIGHT_TOLERANCE = 1e-9
GATE2_TIMEOUT_SECONDS = 14.0
PAIRING_FIELDS = (
    "condition_id",
    "trial_index",
    "initial_state_index",
    "policy_episode_seed",
)
MATCHED_COMPARISON_FIELDS = (
    "episode_id",
    "mode",
    "condition_id",
    "trial_index",
    "initial_state_index",
    "policy_episode_seed",
    "operator_success",
    "autonomous_success",
    "success_delta",
    "descriptive_recovery",
    "operator_simulated_duration_seconds",
    "autonomous_simulated_duration_seconds",
    "simulated_duration_delta_seconds",
    "operator_summary_path",
    "autonomous_summary_path",
)


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        value = json.load(file)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object: {path}")
    return value


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(
                    f"Expected an object at {path}:{line_number}."
                )
            records.append(value)
    return records


def _write_csv(
    path: Path,
    rows: list[dict[str, Any]],
    *,
    fieldnames: Iterable[str] | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    selected_fields = list(fieldnames) if fieldnames is not None else None
    if not rows and selected_fields is None:
        path.write_text("\n", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=selected_fields or list(rows[0]),
        )
        writer.writeheader()
        writer.writerows(rows)


def _finite_float(value: Any) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"Expected a finite number, received {value!r}.")
    return parsed


def _mean(values: Iterable[float]) -> float | None:
    selected = [float(value) for value in values]
    return statistics.fmean(selected) if selected else None


def _median(values: Iterable[float]) -> float | None:
    selected = [float(value) for value in values]
    return statistics.median(selected) if selected else None


def _sample_sd(values: Iterable[float]) -> float | None:
    selected = [float(value) for value in values]
    return statistics.stdev(selected) if len(selected) >= 2 else None


def _weighted_mean(
    values: Iterable[tuple[float | None, int]],
) -> float | None:
    selected = [
        (float(value), int(weight))
        for value, weight in values
        if value is not None and int(weight) > 0
    ]
    total_weight = sum(weight for _, weight in selected)
    if not total_weight:
        return None
    return sum(value * weight for value, weight in selected) / total_weight


def _quantile(values: Iterable[float], probability: float) -> float | None:
    selected = sorted(float(value) for value in values)
    if not selected:
        return None
    position = (len(selected) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return selected[lower]
    fraction = position - lower
    return selected[lower] + fraction * (
        selected[upper] - selected[lower]
    )


def _distribution(values: list[float]) -> dict[str, Any]:
    return {
        "count": len(values),
        "mean": _mean(values),
        "median": _median(values),
        "sd": _sample_sd(values),
        "min": min(values) if values else None,
        "max": max(values) if values else None,
        "p10": _quantile(values, 0.10),
        "p25": _quantile(values, 0.25),
        "p75": _quantile(values, 0.75),
        "p90": _quantile(values, 0.90),
    }


def _segments(active: list[bool]) -> int:
    return sum(
        value and (index == 0 or not active[index - 1])
        for index, value in enumerate(active)
    )


def _action_metrics(
    steps: list[dict[str, Any]],
    *,
    episode_id: str,
    errors: list[str],
) -> tuple[list[bool], list[float], list[float]]:
    active: list[bool] = []
    translations: list[float] = []
    rotations: list[float] = []
    for index, step in enumerate(steps):
        action = step.get("human_action")
        if not isinstance(action, list) or len(action) != 7:
            errors.append(
                f"{episode_id}: step {index} has invalid human_action."
            )
            continue
        try:
            values = [_finite_float(value) for value in action]
        except (TypeError, ValueError) as error:
            errors.append(f"{episode_id}: step {index}: {error}")
            continue
        translation = math.sqrt(sum(value * value for value in values[:3]))
        rotation = math.sqrt(sum(value * value for value in values[3:6]))
        motion = math.sqrt(translation * translation + rotation * rotation)
        calculated_active = motion > SAPS_ACTIVITY_THRESHOLD
        logged_active = step.get(
            "human_active",
            step.get("operator_motion_active"),
        )
        if logged_active is not None and bool(logged_active) != calculated_active:
            errors.append(
                f"{episode_id}: step {index} human-active flag disagrees "
                "with the logged action."
            )
        active.append(calculated_active)
        translations.append(translation)
        rotations.append(rotation)
    return active, translations, rotations


def _wait_metrics(
    records: list[dict[str, Any]],
    *,
    control_steps: int,
    frequency: float,
) -> dict[str, Any]:
    wait_ticks = len(records)
    event_count = 0
    event_wall_seconds = 0.0
    active_ticks = sum(bool(record.get("human_active")) for record in records)
    event: list[dict[str, Any]] = []
    previous: dict[str, Any] | None = None

    def finish_event() -> None:
        nonlocal event_wall_seconds
        if not event:
            return
        first = _finite_float(event[0]["wall_time_unix_seconds"])
        last = _finite_float(event[-1]["wall_time_unix_seconds"])
        event_wall_seconds += max(0.0, last - first) + 1.0 / frequency

    for record in records:
        new_event = (
            previous is None
            or int(record.get("autonomy_wait_ticks", 0)) == 1
            or int(record["scheduler_tick"])
            != int(previous["scheduler_tick"]) + 1
            or int(record["control_steps"])
            != int(previous["control_steps"])
        )
        if new_event:
            finish_event()
            event = []
            event_count += 1
        event.append(record)
        previous = record
    finish_event()

    total_scheduler_ticks = control_steps + wait_ticks
    return {
        "policy_wait_ticks": wait_ticks,
        "policy_wait_events": event_count,
        "policy_wait_duration_seconds": wait_ticks / frequency,
        "policy_wait_wall_seconds": event_wall_seconds,
        "policy_wait_fraction": (
            wait_ticks / total_scheduler_ticks
            if total_scheduler_ticks
            else 0.0
        ),
        "human_active_policy_wait_ticks": active_ticks,
        "human_active_policy_wait_seconds": active_ticks / frequency,
        "human_active_policy_wait_fraction": (
            active_ticks / wait_ticks if wait_ticks else 0.0
        ),
    }


def _latency_metrics(
    steps: list[dict[str, Any]],
    waits: list[dict[str, Any]],
) -> dict[str, Any]:
    values = [
        _finite_float(record["inference_latency_seconds"])
        for record in [*steps, *waits]
        if record.get("inference_latency_seconds") is not None
    ]
    distribution = _distribution(values)
    return {
        "inference_count": distribution["count"],
        "inference_latency_mean_seconds": distribution["mean"],
        "inference_latency_median_seconds": distribution["median"],
        "inference_latency_sd_seconds": distribution["sd"],
        "inference_latency_min_seconds": distribution["min"],
        "inference_latency_max_seconds": distribution["max"],
        "inference_latency_p90_seconds": distribution["p90"],
    }


def _base_episode_row(episode: dict[str, Any]) -> dict[str, Any]:
    return {
        "order_index": int(episode["order_index"]),
        "episode_id": str(episode["episode_id"]),
        "mode": str(episode["mode"]),
        "condition_id": str(episode["condition_id"]),
        "trial_index": int(episode["trial_index"]),
        "initial_state_index": int(episode["initial_state_index"]),
        "policy_episode_seed": int(episode["policy_episode_seed"]),
        "schedule_status": str(episode["status"]),
        "selected_attempt_number": None,
        "selected_attempt_valid": 0,
        "metrics_available": 0,
        "success": None,
        "termination_reason": None,
        "control_steps": None,
        "logged_steps": None,
        "simulated_duration_seconds": None,
        "raw_simulated_duration_seconds": None,
        "wall_control_duration_seconds": None,
        "wall_total_duration_seconds": None,
        "wall_simulation_ratio": None,
        "human_active_steps": None,
        "human_active_duration_seconds": None,
        "human_active_fraction": None,
        "correction_segments": None,
        "translation_magnitude_mean": None,
        "translation_magnitude_active_mean": None,
        "translation_magnitude_max": None,
        "rotation_magnitude_mean": None,
        "rotation_magnitude_active_mean": None,
        "rotation_magnitude_max": None,
        "policy_wait_ticks": None,
        "policy_wait_events": None,
        "policy_wait_duration_seconds": None,
        "policy_wait_wall_seconds": None,
        "policy_wait_fraction": None,
        "human_active_policy_wait_ticks": None,
        "human_active_policy_wait_seconds": None,
        "human_active_policy_wait_fraction": None,
        "inference_count": None,
        "inference_latency_mean_seconds": None,
        "inference_latency_median_seconds": None,
        "inference_latency_sd_seconds": None,
        "inference_latency_min_seconds": None,
        "inference_latency_max_seconds": None,
        "inference_latency_p90_seconds": None,
        "summary_path": None,
    }


def _selected_attempts(episode: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        attempt
        for attempt in episode.get("attempts", [])
        if attempt.get("valid")
        and attempt.get("summary_path")
        and attempt.get("selected_for_analysis", True)
    ]


def _validate_summary_identity(
    summary: dict[str, Any],
    episode: dict[str, Any],
) -> list[str]:
    comparisons = {
        "arbitration_mode": episode["mode"],
        "condition_id": episode["condition_id"],
        "trial_index": episode["trial_index"],
        "initial_state_index": episode["initial_state_index"],
        "policy_episode_seed": episode["policy_episode_seed"],
        "policy_seed_protocol": episode["policy_seed_protocol"],
    }
    return [
        f"{episode['episode_id']}: summary field {field!r} does not "
        "match the frozen schedule."
        for field, expected in comparisons.items()
        if summary.get(field) != expected
    ]


def _fixed_diagnostic(
    row: dict[str, Any],
    steps: list[dict[str, Any]],
    active: list[bool],
    expected_weight: float,
) -> dict[str, Any]:
    relevant = [
        step.get("effective_autonomy_weight")
        for step, is_active in zip(steps, active)
        if is_active
    ]
    defined = [
        _finite_float(value) for value in relevant if value is not None
    ]
    deviations = [abs(value - expected_weight) for value in defined]
    deviation_count = sum(
        value > FIXED_WEIGHT_TOLERANCE for value in deviations
    )
    return {
        "episode_id": row["episode_id"],
        "condition_id": row["condition_id"],
        "trial_index": row["trial_index"],
        "metrics_available": 1,
        "expected_autonomy_weight": expected_weight,
        "active_blending_step_count": len(relevant),
        "defined_weight_count": len(defined),
        "undefined_weight_count": len(relevant) - len(defined),
        "mean_effective_autonomy_weight": _mean(defined),
        "max_absolute_deviation": max(deviations) if deviations else None,
        "deviation_count": deviation_count,
        "within_tolerance": int(
            len(defined) == len(relevant) and deviation_count == 0
        ),
        "tolerance": FIXED_WEIGHT_TOLERANCE,
    }


def _cosine_diagnostic(
    row: dict[str, Any],
    steps: list[dict[str, Any]],
    active: list[bool],
) -> dict[str, Any]:
    relevant = [
        step.get("effective_autonomy_weight")
        for step, is_active in zip(steps, active)
        if is_active
    ]
    values = [_finite_float(value) for value in relevant if value is not None]
    distribution = _distribution(values)
    near_zero = sum(value <= COSINE_NEAR_ZERO_MAX for value in values)
    near_one = sum(value >= COSINE_NEAR_ONE_MIN for value in values)
    intermediate = len(values) - near_zero - near_one
    consecutive_changes = []
    for index, (previous, current) in enumerate(zip(steps, steps[1:])):
        if (
            active[index]
            and active[index + 1]
            and int(current["control_step"])
            == int(previous["control_step"]) + 1
            and previous.get("effective_autonomy_weight") is not None
            and current.get("effective_autonomy_weight") is not None
        ):
            consecutive_changes.append(
                abs(
                    _finite_float(current["effective_autonomy_weight"])
                    - _finite_float(previous["effective_autonomy_weight"])
                )
            )
    material = sum(
        value >= COSINE_MATERIAL_CHANGE_MIN
        for value in consecutive_changes
    )
    cosine_values = [
        _finite_float(step["cosine_similarity"])
        for step, is_active in zip(steps, active)
        if is_active and step.get("cosine_similarity") is not None
    ]
    return {
        "episode_id": row["episode_id"],
        "condition_id": row["condition_id"],
        "trial_index": row["trial_index"],
        "metrics_available": 1,
        "active_blending_step_count": len(relevant),
        "weight_count": distribution["count"],
        "undefined_weight_count": len(relevant) - len(values),
        "weight_mean": distribution["mean"],
        "weight_median": distribution["median"],
        "weight_sd": distribution["sd"],
        "weight_min": distribution["min"],
        "weight_max": distribution["max"],
        "weight_p10": distribution["p10"],
        "weight_p25": distribution["p25"],
        "weight_p75": distribution["p75"],
        "weight_p90": distribution["p90"],
        "near_zero_fraction": near_zero / len(values) if values else None,
        "near_one_fraction": near_one / len(values) if values else None,
        "intermediate_fraction": (
            intermediate / len(values) if values else None
        ),
        "consecutive_weight_pair_count": len(consecutive_changes),
        "material_change_fraction": (
            material / len(consecutive_changes)
            if consecutive_changes
            else None
        ),
        "cosine_similarity_count": len(cosine_values),
        "cosine_similarity_mean": _mean(cosine_values),
        "near_zero_max": COSINE_NEAR_ZERO_MAX,
        "near_one_min": COSINE_NEAR_ONE_MIN,
        "material_change_min": COSINE_MATERIAL_CHANGE_MIN,
        "_weight_values": values,
        "_consecutive_changes": consecutive_changes,
        "_cosine_values": cosine_values,
    }


def _blank_fixed(row: dict[str, Any], expected: float) -> dict[str, Any]:
    return {
        "episode_id": row["episode_id"],
        "condition_id": row["condition_id"],
        "trial_index": row["trial_index"],
        "metrics_available": 0,
        "expected_autonomy_weight": expected,
        "active_blending_step_count": None,
        "defined_weight_count": None,
        "undefined_weight_count": None,
        "mean_effective_autonomy_weight": None,
        "max_absolute_deviation": None,
        "deviation_count": None,
        "within_tolerance": None,
        "tolerance": FIXED_WEIGHT_TOLERANCE,
    }


def _blank_cosine(row: dict[str, Any]) -> dict[str, Any]:
    values = {
        key: None
        for key in (
            "active_blending_step_count",
            "weight_count",
            "undefined_weight_count",
            "weight_mean",
            "weight_median",
            "weight_sd",
            "weight_min",
            "weight_max",
            "weight_p10",
            "weight_p25",
            "weight_p75",
            "weight_p90",
            "near_zero_fraction",
            "near_one_fraction",
            "intermediate_fraction",
            "consecutive_weight_pair_count",
            "material_change_fraction",
            "cosine_similarity_count",
            "cosine_similarity_mean",
        )
    }
    return {
        "episode_id": row["episode_id"],
        "condition_id": row["condition_id"],
        "trial_index": row["trial_index"],
        "metrics_available": 0,
        **values,
        "near_zero_max": COSINE_NEAR_ZERO_MAX,
        "near_one_min": COSINE_NEAR_ONE_MIN,
        "material_change_min": COSINE_MATERIAL_CHANGE_MIN,
    }


def _aggregate_cosine_diagnostics(
    rows: list[dict[str, Any]],
) -> dict[str, Any]:
    available = [row for row in rows if row["metrics_available"]]
    weights = [
        value for row in available for value in row["_weight_values"]
    ]
    changes = [
        value
        for row in available
        for value in row["_consecutive_changes"]
    ]
    cosine_values = [
        value for row in available for value in row["_cosine_values"]
    ]
    distribution = _distribution(weights)
    near_zero = sum(value <= COSINE_NEAR_ZERO_MAX for value in weights)
    near_one = sum(value >= COSINE_NEAR_ONE_MIN for value in weights)
    intermediate = len(weights) - near_zero - near_one
    material = sum(
        value >= COSINE_MATERIAL_CHANGE_MIN for value in changes
    )
    return {
        "episode_id": "__all_selected_cosine__",
        "condition_id": "all",
        "trial_index": "",
        "metrics_available": int(bool(available)),
        "active_blending_step_count": sum(
            int(row["active_blending_step_count"]) for row in available
        ),
        "weight_count": distribution["count"],
        "undefined_weight_count": sum(
            int(row["undefined_weight_count"]) for row in available
        ),
        "weight_mean": distribution["mean"],
        "weight_median": distribution["median"],
        "weight_sd": distribution["sd"],
        "weight_min": distribution["min"],
        "weight_max": distribution["max"],
        "weight_p10": distribution["p10"],
        "weight_p25": distribution["p25"],
        "weight_p75": distribution["p75"],
        "weight_p90": distribution["p90"],
        "near_zero_fraction": (
            near_zero / len(weights) if weights else None
        ),
        "near_one_fraction": near_one / len(weights) if weights else None,
        "intermediate_fraction": (
            intermediate / len(weights) if weights else None
        ),
        "consecutive_weight_pair_count": len(changes),
        "material_change_fraction": (
            material / len(changes) if changes else None
        ),
        "cosine_similarity_count": len(cosine_values),
        "cosine_similarity_mean": _mean(cosine_values),
        "near_zero_max": COSINE_NEAR_ZERO_MAX,
        "near_one_min": COSINE_NEAR_ONE_MIN,
        "material_change_min": COSINE_MATERIAL_CHANGE_MIN,
    }


def _public_cosine_row(row: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value for key, value in row.items() if not key.startswith("_")
    }


def _blank_wait(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "episode_id": row["episode_id"],
        "mode": row["mode"],
        "condition_id": row["condition_id"],
        "trial_index": row["trial_index"],
        "metrics_available": 0,
        "control_steps": None,
        "policy_wait_ticks": None,
        "policy_wait_events": None,
        "policy_wait_duration_seconds": None,
        "policy_wait_wall_seconds": None,
        "policy_wait_fraction": None,
        "human_active_policy_wait_ticks": None,
        "human_active_policy_wait_seconds": None,
        "human_active_policy_wait_fraction": None,
        "inference_count": None,
        "inference_latency_mean_seconds": None,
        "inference_latency_median_seconds": None,
        "inference_latency_sd_seconds": None,
        "inference_latency_min_seconds": None,
        "inference_latency_max_seconds": None,
        "inference_latency_p90_seconds": None,
        "wall_simulation_ratio": None,
    }


def _episode_analysis(
    *,
    episode: dict[str, Any],
    summary_path: Path,
    manifest: Any,
    expected_profile_sha256: str,
    errors: list[str],
) -> tuple[dict[str, Any], dict[str, Any] | None, dict[str, Any] | None, dict[str, Any]]:
    row = _base_episode_row(episode)
    summary = _read_json(summary_path)
    identity_errors = _validate_summary_identity(summary, episode)
    errors.extend(identity_errors)
    profile = summary.get("spacemouse_profile")
    if not isinstance(profile, dict) or profile.get("sha256") != expected_profile_sha256:
        errors.append(
            f"{episode['episode_id']}: summary SpaceMouse profile hash "
            "does not match the frozen Gate-2 profile."
        )
    if identity_errors:
        return row, None, None, _blank_wait(row)

    try:
        validate_gate2_attempt_completion(
            summary_path=summary_path,
            summary=summary,
        )
    except (OSError, ValueError) as error:
        errors.append(f"{episode['episode_id']}: {error}")
        return row, None, None, _blank_wait(row)

    steps_path = summary_path.with_name("steps.jsonl")
    steps = _read_jsonl(steps_path)
    if not steps:
        errors.append(f"{episode['episode_id']}: steps.jsonl is empty.")
        return row, None, None, _blank_wait(row)
    active, translations, rotations = _action_metrics(
        steps,
        episode_id=str(episode["episode_id"]),
        errors=errors,
    )
    if len(active) != len(steps):
        return row, None, None, _blank_wait(row)

    mode = str(episode["mode"])
    frequency = _finite_float(summary["control_frequency_hz"])
    control_steps = int(summary["control_steps"])
    if len(steps) != control_steps:
        errors.append(
            f"{episode['episode_id']}: logged step count does not match "
            "summary control_steps."
        )
    raw_simulated = _finite_float(
        summary.get("simulated_control_seconds", control_steps / frequency)
    )
    termination = str(summary["termination_reason"])
    success = bool(summary["success"])
    if success != (termination == "success"):
        errors.append(
            f"{episode['episode_id']}: success and termination_reason "
            "are inconsistent."
        )
    if termination == "timeout" and control_steps != manifest.operator_max_steps:
        errors.append(
            f"{episode['episode_id']}: timeout does not contain the fixed "
            f"{manifest.operator_max_steps}-step horizon."
        )
    simulated = (
        GATE2_TIMEOUT_SECONDS if termination == "timeout" else raw_simulated
    )
    wall_control = _finite_float(summary["control_elapsed_seconds"])
    wall_total = _finite_float(summary["total_elapsed_seconds"])

    waits_path = summary_path.with_name("scheduler_waits.jsonl")
    if mode == "teleoperation":
        waits: list[dict[str, Any]] = []
    elif waits_path.is_file():
        waits = _read_jsonl(waits_path)
    else:
        errors.append(
            f"{episode['episode_id']}: scheduler_waits.jsonl is missing."
        )
        waits = []
    waits_metrics = _wait_metrics(
        waits,
        control_steps=control_steps,
        frequency=frequency,
    )
    latency = _latency_metrics(steps, waits)
    replan_count = summary.get("policy_replan_count")
    if (
        mode != "teleoperation"
        and replan_count is not None
        and int(replan_count) != latency["inference_count"]
    ):
        errors.append(
            f"{episode['episode_id']}: inference-latency count does not "
            "match policy_replan_count."
        )
    active_translations = [
        value for value, is_active in zip(translations, active) if is_active
    ]
    active_rotations = [
        value for value, is_active in zip(rotations, active) if is_active
    ]
    row.update(
        {
            "metrics_available": 1,
            "success": int(success),
            "termination_reason": termination,
            "control_steps": control_steps,
            "logged_steps": len(steps),
            "simulated_duration_seconds": simulated,
            "raw_simulated_duration_seconds": raw_simulated,
            "wall_control_duration_seconds": wall_control,
            "wall_total_duration_seconds": wall_total,
            "wall_simulation_ratio": (
                wall_control / simulated if simulated > 0.0 else None
            ),
            "human_active_steps": sum(active),
            "human_active_duration_seconds": sum(active) / frequency,
            "human_active_fraction": sum(active) / len(active),
            "correction_segments": _segments(active),
            "translation_magnitude_mean": _mean(translations),
            "translation_magnitude_active_mean": _mean(active_translations),
            "translation_magnitude_max": max(translations),
            "rotation_magnitude_mean": _mean(rotations),
            "rotation_magnitude_active_mean": _mean(active_rotations),
            "rotation_magnitude_max": max(rotations),
            **waits_metrics,
            **latency,
            "summary_path": str(summary_path),
        }
    )
    wait_row = {
        "episode_id": row["episode_id"],
        "mode": mode,
        "condition_id": row["condition_id"],
        "trial_index": row["trial_index"],
        "metrics_available": 1,
        "control_steps": control_steps,
        **waits_metrics,
        **latency,
        "wall_simulation_ratio": row["wall_simulation_ratio"],
    }
    fixed = (
        _fixed_diagnostic(
            row,
            steps,
            active,
            float(manifest.fixed_autonomy_weight),
        )
        if mode == "fixed_blend"
        else None
    )
    cosine = (
        _cosine_diagnostic(row, steps, active)
        if mode == "cosine_blend"
        else None
    )
    return row, fixed, cosine, wait_row


def _aggregate(
    rows: list[dict[str, Any]],
    *,
    group_fields: tuple[str, ...],
    group_values: list[tuple[str, ...]],
) -> list[dict[str, Any]]:
    output = []
    for group in group_values:
        selected = [
            row
            for row in rows
            if tuple(str(row[field]) for field in group_fields) == group
        ]
        available = [row for row in selected if row["metrics_available"]]
        successes = sum(int(row["success"]) for row in available)
        failures = len(available) - successes
        record = dict(zip(group_fields, group))
        record.update(
            {
                "n_scheduled": len(selected),
                "n_selected_valid": len(available),
                "n_success": successes,
                "n_failure": failures,
                "n_not_yet_analyzable": len(selected) - len(available),
                "coverage_fraction": (
                    len(available) / len(selected) if selected else None
                ),
                "success_rate_observed": (
                    successes / len(available) if available else None
                ),
                "success_fraction_of_scheduled": (
                    successes / len(selected) if selected else None
                ),
                "simulated_duration_mean_seconds": _mean(
                    row["simulated_duration_seconds"] for row in available
                ),
                "wall_control_duration_mean_seconds": _mean(
                    row["wall_control_duration_seconds"] for row in available
                ),
                "human_active_duration_mean_seconds": _mean(
                    row["human_active_duration_seconds"] for row in available
                ),
                "human_active_fraction_mean": _mean(
                    row["human_active_fraction"] for row in available
                ),
                "correction_segments_mean": _mean(
                    row["correction_segments"] for row in available
                ),
                "translation_magnitude_active_mean": _mean(
                    row["translation_magnitude_active_mean"]
                    for row in available
                    if row["translation_magnitude_active_mean"] is not None
                ),
                "rotation_magnitude_active_mean": _mean(
                    row["rotation_magnitude_active_mean"]
                    for row in available
                    if row["rotation_magnitude_active_mean"] is not None
                ),
                "policy_wait_episode_count": sum(
                    int(row["policy_wait_ticks"]) > 0 for row in available
                ),
                "policy_wait_ticks_total": sum(
                    int(row["policy_wait_ticks"]) for row in available
                ),
                "policy_wait_events_total": sum(
                    int(row["policy_wait_events"]) for row in available
                ),
                "policy_wait_duration_total_seconds": sum(
                    float(row["policy_wait_duration_seconds"])
                    for row in available
                ),
                "policy_wait_fraction_mean": _mean(
                    row["policy_wait_fraction"] for row in available
                ),
                "human_active_policy_wait_ticks_total": sum(
                    int(row["human_active_policy_wait_ticks"])
                    for row in available
                ),
                "inference_count_total": sum(
                    int(row["inference_count"]) for row in available
                ),
                "inference_latency_mean_seconds": _weighted_mean(
                    (
                        row["inference_latency_mean_seconds"],
                        int(row["inference_count"]),
                    )
                    for row in available
                ),
                "wall_simulation_ratio_mean": _mean(
                    row["wall_simulation_ratio"] for row in available
                ),
            }
        )
        output.append(record)
    return output


def _autonomous_index(
    root: Path,
) -> tuple[dict[tuple[Any, ...], dict[str, Any]], list[str]]:
    index: dict[tuple[Any, ...], dict[str, Any]] = {}
    errors: list[str] = []
    for path in sorted(root.rglob("summary.json")):
        summary = _read_json(path)
        if not all(field in summary for field in PAIRING_FIELDS):
            continue
        key = tuple(summary[field] for field in PAIRING_FIELDS)
        if key in index:
            errors.append(f"Duplicate autonomous pairing identity: {key!r}.")
            continue
        index[key] = {**summary, "summary_path": str(path)}
    return index, errors


def _matched_comparisons(
    rows: list[dict[str, Any]],
    autonomous: dict[tuple[Any, ...], dict[str, Any]],
) -> tuple[list[dict[str, Any]], list[str]]:
    comparisons: list[dict[str, Any]] = []
    unmatched: list[str] = []
    for row in rows:
        if not row["metrics_available"]:
            continue
        key = tuple(row[field] for field in PAIRING_FIELDS)
        baseline = autonomous.get(key)
        if baseline is None:
            unmatched.append(str(row["episode_id"]))
            continue
        autonomous_success = int(bool(baseline["success"]))
        autonomous_steps = int(baseline["control_steps"])
        autonomous_duration = (
            GATE2_TIMEOUT_SECONDS
            if not autonomous_success and autonomous_steps >= 280
            else autonomous_steps / 20.0
        )
        comparisons.append(
            {
                "episode_id": row["episode_id"],
                "mode": row["mode"],
                "condition_id": row["condition_id"],
                "trial_index": row["trial_index"],
                "initial_state_index": row["initial_state_index"],
                "policy_episode_seed": row["policy_episode_seed"],
                "operator_success": row["success"],
                "autonomous_success": autonomous_success,
                "success_delta": row["success"] - autonomous_success,
                "descriptive_recovery": int(
                    row["success"] == 1 and autonomous_success == 0
                ),
                "operator_simulated_duration_seconds": (
                    row["simulated_duration_seconds"]
                ),
                "autonomous_simulated_duration_seconds": autonomous_duration,
                "simulated_duration_delta_seconds": (
                    row["simulated_duration_seconds"] - autonomous_duration
                ),
                "operator_summary_path": row["summary_path"],
                "autonomous_summary_path": baseline["summary_path"],
            }
        )
    return comparisons, unmatched


def _profile_validation(
    root: Path,
    *,
    expected_sha256: str,
) -> tuple[dict[str, Any], list[str]]:
    path = root / "human_input.json"
    if not path.is_file():
        return {}, ["Frozen human_input.json is missing."]
    value = _read_json(path)
    profile = value.get("spacemouse_profile")
    errors = []
    if value.get("input_source") != "spacemouse":
        errors.append("Gate-2 human_input.json is not SpaceMouse-backed.")
    if not isinstance(profile, dict) or profile.get("sha256") != expected_sha256:
        errors.append("Frozen SpaceMouse profile hash does not match Gate-2.")
    return value, errors


def _session_provenance_validation(
    root: Path,
    *,
    expected_manifest_sha256: str,
) -> tuple[dict[str, Any], list[str]]:
    errors: list[str] = []
    provenance: dict[str, Any] = {}
    required = {
        "repository_provenance": root / "repository_provenance.json",
        "session_protocol": root / "session_protocol.json",
        "perturbation_config": root / "perturbation_config.json",
    }
    for name, path in required.items():
        if not path.is_file():
            errors.append(f"Frozen {path.name} is missing.")
            continue
        provenance[name] = _read_json(path)

    repository = provenance.get("repository_provenance", {})
    commit = str(repository.get("repository_commit", ""))
    if (
        len(commit) != 40
        or any(
            character not in "0123456789abcdef"
            for character in commit.lower()
        )
    ):
        errors.append("Frozen repository commit is not a full Git hash.")
    if repository.get("manifest_sha256") != expected_manifest_sha256:
        errors.append("Repository provenance manifest hash does not match.")

    protocol = provenance.get("session_protocol", {})
    if protocol.get("required_protocol_id") != GATE2_EXPERIMENT_ID:
        errors.append("Frozen session protocol is not the Gate-2 protocol.")

    perturbation = provenance.get("perturbation_config", {})
    if perturbation.get("sha256") != GATE2_CONFIG_SHA256:
        errors.append("Frozen perturbation configuration hash does not match.")
    return provenance, errors


def _plot_placeholder(axis: Any, message: str) -> None:
    axis.text(0.5, 0.5, message, ha="center", va="center")
    axis.set_axis_off()


def _write_plots(
    output: Path,
    episode_rows: list[dict[str, Any]],
    cosine_rows: list[dict[str, Any]],
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plot_root = output / "plots"
    plot_root.mkdir(parents=True, exist_ok=True)
    available = [row for row in episode_rows if row["metrics_available"]]

    fig, axis = plt.subplots(figsize=(8, 4.5))
    labels = []
    values = []
    for mode in GATE2_MODES:
        for condition in GATE2_CONDITIONS:
            selected = [
                row
                for row in available
                if row["mode"] == mode and row["condition_id"] == condition
            ]
            if selected:
                labels.append(f"{mode}\n{condition}")
                values.append(sum(row["success"] for row in selected) / len(selected))
    if values:
        axis.bar(labels, values)
        axis.set_ylim(0.0, 1.0)
        axis.set_ylabel("Observed success fraction")
        axis.tick_params(axis="x", rotation=45)
    else:
        _plot_placeholder(axis, "No selected Gate-2 attempts yet")
    fig.tight_layout()
    fig.savefig(plot_root / "observed_success_by_cell.png", dpi=180)
    plt.close(fig)

    for field, name, label in (
        ("human_active_fraction", "human_active_fraction_by_mode", "Human-active fraction"),
        ("policy_wait_fraction", "policy_wait_fraction_by_mode", "Policy-wait fraction"),
    ):
        fig, axis = plt.subplots(figsize=(6.5, 4.0))
        data = [
            [row[field] for row in available if row["mode"] == mode]
            for mode in GATE2_MODES
        ]
        if any(data):
            axis.boxplot(data, labels=GATE2_MODES)
            axis.set_ylabel(label)
            axis.tick_params(axis="x", rotation=20)
        else:
            _plot_placeholder(axis, "No selected Gate-2 attempts yet")
        fig.tight_layout()
        fig.savefig(plot_root / f"{name}.png", dpi=180)
        plt.close(fig)

    weights = [
        row["weight_mean"]
        for row in cosine_rows
        if row["metrics_available"] and row["weight_mean"] is not None
    ]
    fig, axis = plt.subplots(figsize=(6.5, 4.0))
    if weights:
        axis.hist(weights, bins=10, range=(0.0, 1.0))
        axis.set(xlabel="Episode mean active cosine autonomy weight", ylabel="Episodes")
    else:
        _plot_placeholder(axis, "No cosine-blend diagnostics yet")
    fig.tight_layout()
    fig.savefig(plot_root / "cosine_weight_diagnostic.png", dpi=180)
    plt.close(fig)


def _format_rate(value: Any) -> str:
    return "—" if value is None else f"{100.0 * float(value):.1f}%"


def _write_report(
    path: Path,
    *,
    mode_summary: list[dict[str, Any]],
    comparisons: list[dict[str, Any]],
    cosine_aggregate: dict[str, Any],
    report: dict[str, Any],
) -> None:
    lines = [
        "# Gate-2 Operator Pilot Report",
        "",
        (
            "Gate 2 is descriptive pilot evidence (`n=5` per "
            "mode-condition), not a powered inferential experiment. "
            "No significance test is used as the central result."
        ),
        "",
        "## Collection coverage",
        "",
        (
            "| Mode | Selected / scheduled | Successes / observed | "
            "Observed success | Scheduled success fraction |"
        ),
        "|---|---:|---:|---:|---:|",
    ]
    for row in mode_summary:
        lines.append(
            f"| {row['mode']} | {row['n_selected_valid']} / {row['n_scheduled']} | "
            f"{row['n_success']} / {row['n_selected_valid']} | "
            f"{_format_rate(row['success_rate_observed'])} | "
            f"{_format_rate(row['success_fraction_of_scheduled'])} |"
        )
    lines.extend(
        [
            "",
            (
                "Failed selected episodes remain in observed success "
                "denominators. Timeouts contribute the fixed 14.0 "
                "seconds of simulated time. The scheduled success "
                "fraction retains all 60 planned episodes as its "
                "denominator and should be read alongside coverage "
                "while collection is incomplete."
            ),
            "",
            "## Severe-condition recovery evidence",
            "",
        ]
    )
    for condition in ("p06", "p09"):
        selected = [row for row in comparisons if row["condition_id"] == condition]
        recoveries = sum(row["descriptive_recovery"] for row in selected)
        baseline_failures = sum(not row["autonomous_success"] for row in selected)
        lines.append(
            f"- `{condition}`: {len(selected)} exact autonomous pairs; "
            f"{baseline_failures} matched autonomous failures and "
            f"{recoveries} operator-mode successes on those failures."
        )
    lines.extend(
        [
            "",
            (
                "These are descriptive recovery counts; no arbitrary "
                "recovery or success threshold is applied."
            ),
            "",
            "## Policy-wait diagnostics",
            "",
            (
                "| Mode | Episodes with waits | Wait events | Wait "
                "duration (s) | Active-overlap ticks | Mean inference "
                "latency (s) | Mean wall/simulation ratio |"
            ),
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in mode_summary:
        lines.append(
            f"| {row['mode']} | {row['policy_wait_episode_count']} | "
            f"{row['policy_wait_events_total']} | "
            f"{row['policy_wait_duration_total_seconds']:.3f} | "
            f"{row['human_active_policy_wait_ticks_total']} | "
            f"{row['inference_latency_mean_seconds']} | "
            f"{row['wall_simulation_ratio_mean']} |"
        )
    lines.extend(
        [
            "",
            "## Diagnostic definitions",
            "",
            f"- Cosine near-zero weight: `α ≤ {COSINE_NEAR_ZERO_MAX:.2f}`.",
            f"- Cosine near-one weight: `α ≥ {COSINE_NEAR_ONE_MIN:.2f}`.",
            f"- Material consecutive change: `|Δα| ≥ {COSINE_MATERIAL_CHANGE_MIN:.2f}`.",
            (
                "- Fixed-weight verification tolerance: "
                f"`{FIXED_WEIGHT_TOLERANCE:g}` around `α=0.5`."
            ),
            "- Policy-wait fraction: wait ticks divided by wait plus executed scheduler ticks.",
            "",
            "## Blending diagnostics",
            "",
            (
                "- Fixed-blend episodes with deviations: "
                f"`{len(report['fixed_blend_deviation_episode_ids'])}`."
            ),
            (
                f"- Cosine active weights: `{cosine_aggregate['weight_count']}` "
                "defined and "
                f"`{cosine_aggregate['undefined_weight_count']}` undefined."
            ),
            (
                "- Cosine active-weight mean / median: "
                f"`{cosine_aggregate['weight_mean']}` / "
                f"`{cosine_aggregate['weight_median']}`."
            ),
            (
                "- Near-zero / intermediate / near-one fractions: "
                f"`{cosine_aggregate['near_zero_fraction']}` / "
                f"`{cosine_aggregate['intermediate_fraction']}` / "
                f"`{cosine_aggregate['near_one_fraction']}`."
            ),
            (
                "- Material consecutive-change fraction: "
                f"`{cosine_aggregate['material_change_fraction']}`."
            ),
            "",
            "## Readiness",
            "",
            f"- Analysis-valid: `{report['analysis_valid']}`",
            f"- Collection complete: `{report['collection_complete']}`",
            f"- Exact autonomous pairs: `{len(comparisons)}`",
        ]
    )
    if report["blocking_errors"]:
        lines.extend(["", "Blocking validation errors:"])
        lines.extend(f"- {error}" for error in report["blocking_errors"])
    if report["warnings"]:
        lines.extend(["", "Warnings:"])
        lines.extend(f"- {warning}" for warning in report["warnings"])
    lines.extend(
        [
            "",
            "## Artifacts",
            "",
            (
                "The CSV files retain episode, mode, condition-mode, "
                "blending, wait, and exact-pair diagnostics. "
                "`validation_report.json` contains machine-readable "
                "readiness details; `plots/` contains descriptive "
                "diagnostics."
            ),
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def analyze_gate2_collection(
    *,
    session_root: Path,
    autonomous_root: Path,
    output_dir: Path,
    write_plots: bool = True,
) -> dict[str, Any]:
    """Analyze one frozen Gate-2 session without modifying raw outputs."""

    manifest = load_manifest(session_root / "manifest.json")
    validate_gate2_manifest(manifest)
    schedule = _read_json(session_root / "schedule.json")
    validate_gate2_schedule(schedule, manifest=manifest)
    blocking_errors: list[str] = []
    identity = manifest_sha256(manifest)
    if schedule.get("manifest_sha256") != identity:
        blocking_errors.append("Schedule manifest hash does not match manifest.json.")
    input_configuration, profile_errors = _profile_validation(
        session_root,
        expected_sha256=GATE2_PROFILE_SHA256,
    )
    blocking_errors.extend(profile_errors)
    provenance, provenance_errors = _session_provenance_validation(
        session_root,
        expected_manifest_sha256=identity,
    )
    blocking_errors.extend(provenance_errors)

    episode_rows: list[dict[str, Any]] = []
    fixed_rows: list[dict[str, Any]] = []
    cosine_rows: list[dict[str, Any]] = []
    wait_rows: list[dict[str, Any]] = []
    selected_attempts: dict[str, int] = {}

    for episode in schedule["episodes"]:
        row = _base_episode_row(episode)
        selected = _selected_attempts(episode)
        if len(selected) > 1:
            blocking_errors.append(
                f"{episode['episode_id']}: multiple attempts are selected for analysis."
            )
        if len(selected) != 1:
            if episode["status"] == "completed":
                blocking_errors.append(
                    f"{episode['episode_id']}: completed episode lacks "
                    "exactly one selected valid attempt."
                )
            episode_rows.append(row)
            wait_rows.append(_blank_wait(row))
            if episode["mode"] == "fixed_blend":
                fixed_rows.append(_blank_fixed(row, manifest.fixed_autonomy_weight))
            elif episode["mode"] == "cosine_blend":
                cosine_rows.append(_blank_cosine(row))
            continue

        attempt = selected[0]
        row["selected_attempt_number"] = int(attempt["attempt_number"])
        row["selected_attempt_valid"] = 1
        selected_attempts[str(episode["episode_id"])] = int(
            attempt["attempt_number"]
        )
        try:
            row, fixed, cosine, wait = _episode_analysis(
                episode=episode,
                summary_path=Path(str(attempt["summary_path"])),
                manifest=manifest,
                expected_profile_sha256=GATE2_PROFILE_SHA256,
                errors=blocking_errors,
            )
            row["selected_attempt_number"] = int(attempt["attempt_number"])
            row["selected_attempt_valid"] = 1
        except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError) as error:
            blocking_errors.append(f"{episode['episode_id']}: {error}")
            fixed = None
            cosine = None
            wait = _blank_wait(row)
        episode_rows.append(row)
        wait_rows.append(wait)
        if episode["mode"] == "fixed_blend":
            fixed_rows.append(
                fixed or _blank_fixed(row, manifest.fixed_autonomy_weight)
            )
        elif episode["mode"] == "cosine_blend":
            cosine_rows.append(cosine or _blank_cosine(row))

    mode_summary = _aggregate(
        episode_rows,
        group_fields=("mode",),
        group_values=[(mode,) for mode in GATE2_MODES],
    )
    condition_mode_summary = _aggregate(
        episode_rows,
        group_fields=("condition_id", "mode"),
        group_values=[
            (condition, mode)
            for condition in GATE2_CONDITIONS
            for mode in GATE2_MODES
        ],
    )
    autonomous, autonomous_errors = _autonomous_index(autonomous_root)
    blocking_errors.extend(autonomous_errors)
    comparisons, unmatched = _matched_comparisons(episode_rows, autonomous)
    cosine_aggregate = _aggregate_cosine_diagnostics(cosine_rows)
    available_count = sum(row["metrics_available"] for row in episode_rows)
    collection_complete = available_count == len(episode_rows)
    warnings = [
        (
            "Gate 2 is a descriptive pilot with n=5 per mode-condition; "
            "do not treat it as a powered inferential experiment."
        )
    ]
    if not collection_complete:
        warnings.append(
            f"Collection is incomplete: {available_count}/60 scheduled "
            "episodes have selected analyzable attempts."
        )
    if unmatched:
        warnings.append(
            f"{len(unmatched)} analyzable operator episodes lack an "
            "exact autonomous pairing identity."
        )
    fixed_deviations = [
        row["episode_id"]
        for row in fixed_rows
        if row["metrics_available"] and not row["within_tolerance"]
    ]
    if fixed_deviations:
        blocking_errors.append(
            "Fixed-blend effective autonomy weight deviates from alpha=0.5 in: "
            + ", ".join(fixed_deviations)
        )
    report = {
        "schema_version": 1,
        "experiment_id": GATE2_EXPERIMENT_ID,
        "session_root": str(session_root),
        "autonomous_root": str(autonomous_root),
        "manifest_sha256": identity,
        "ordering_method": schedule.get("ordering_method"),
        "spacemouse_profile_sha256": (
            input_configuration["spacemouse_profile"].get("sha256")
            if isinstance(
                input_configuration.get("spacemouse_profile"),
                dict,
            )
            else None
        ),
        "frozen_provenance": provenance,
        "scheduled_episode_count": len(episode_rows),
        "selected_analyzable_episode_count": available_count,
        "collection_complete": collection_complete,
        "analysis_valid": not blocking_errors,
        "ready_for_descriptive_analysis": (
            available_count > 0 and not blocking_errors
        ),
        "ready_for_complete_gate2_report": (
            collection_complete and not blocking_errors
        ),
        "selected_attempts": selected_attempts,
        "scheduled_by_mode": dict(Counter(row["mode"] for row in episode_rows)),
        "analyzable_by_mode": dict(
            Counter(
                row["mode"]
                for row in episode_rows
                if row["metrics_available"]
            )
        ),
        "exact_autonomous_pairs_by_mode": dict(
            Counter(row["mode"] for row in comparisons)
        ),
        "unmatched_autonomous_episode_ids": unmatched,
        "fixed_blend_deviation_episode_ids": fixed_deviations,
        "cosine_weight_distribution_all_selected": cosine_aggregate,
        "policy_wait_episodes_by_mode": dict(
            Counter(
                row["mode"]
                for row in wait_rows
                if row["metrics_available"]
                and int(row["policy_wait_ticks"]) > 0
            )
        ),
        "thresholds": {
            "human_activity_motion_norm": SAPS_ACTIVITY_THRESHOLD,
            "cosine_near_zero_max": COSINE_NEAR_ZERO_MAX,
            "cosine_near_one_min": COSINE_NEAR_ONE_MIN,
            "cosine_material_change_min": COSINE_MATERIAL_CHANGE_MIN,
            "fixed_weight_tolerance": FIXED_WEIGHT_TOLERANCE,
            "timeout_seconds": GATE2_TIMEOUT_SECONDS,
        },
        "blocking_errors": blocking_errors,
        "warnings": warnings,
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(output_dir / "episode_metrics.csv", episode_rows)
    _write_csv(output_dir / "mode_summary.csv", mode_summary)
    _write_csv(
        output_dir / "condition_mode_summary.csv",
        condition_mode_summary,
    )
    _write_csv(
        output_dir / "matched_autonomous_comparisons.csv",
        comparisons,
        fieldnames=MATCHED_COMPARISON_FIELDS,
    )
    _write_csv(output_dir / "fixed_blend_diagnostics.csv", fixed_rows)
    _write_csv(
        output_dir / "cosine_blend_diagnostics.csv",
        [
            *(_public_cosine_row(row) for row in cosine_rows),
            cosine_aggregate,
        ],
    )
    _write_csv(output_dir / "policy_wait_summary.csv", wait_rows)
    (output_dir / "validation_report.json").write_text(
        json.dumps(report, indent=2) + "\n",
        encoding="utf-8",
    )
    _write_report(
        output_dir / "REPORT.md",
        mode_summary=mode_summary,
        comparisons=comparisons,
        cosine_aggregate=cosine_aggregate,
        report=report,
    )
    if write_plots:
        _write_plots(output_dir, episode_rows, cosine_rows)
    return report
