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
        writer = csv.DictWriter(file, fieldnames=fields or list(rows[0]))
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
]:
    episode_rows: list[dict[str, Any]] = []
    fixed_rows: list[dict[str, Any]] = []
    cosine_rows: list[dict[str, Any]] = []
    wait_rows: list[dict[str, Any]] = []
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
            if episode["mode"] == "fixed_blend":
                fixed_rows.append(
                    _blank_fixed(row, manifest.fixed_autonomy_weight)
                )
            else:
                cosine_rows.append(_blank_cosine(row))
            continue
        attempt = selected[0]
        try:
            analyzed, fixed, cosine, wait = _episode_analysis(
                episode=episode,
                summary_path=Path(str(attempt["summary_path"])),
                manifest=manifest,
                expected_profile_sha256=GATE2_V2_PROFILE_SHA256,
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
        episode_rows.append(analyzed)
        wait_rows.append(wait)
        if episode["mode"] == "fixed_blend":
            fixed_rows.append(
                fixed or _blank_fixed(row, manifest.fixed_autonomy_weight)
            )
        else:
            cosine_rows.append(cosine or _blank_cosine(row))
    return episode_rows, fixed_rows, cosine_rows, wait_rows


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
) -> dict[str, Any]:
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
        return row
    control_steps = int(summary["control_steps"])
    success = bool(summary["success"])
    if not 1 <= control_steps <= GATE2_V2_MAX_STEPS:
        errors.append(f"{row['episode_id']}: invalid autonomous step count.")
        return row
    if not success and control_steps != GATE2_V2_MAX_STEPS:
        errors.append(
            f"{row['episode_id']}: autonomous failure is not a full timeout."
        )
        return row
    steps_path = summary_path.with_name("steps.jsonl")
    if not steps_path.is_file():
        errors.append(f"{row['episode_id']}: autonomous steps.jsonl is missing.")
        return row
    steps = _read_jsonl(steps_path)
    if len(steps) != control_steps:
        errors.append(
            f"{row['episode_id']}: autonomous logged steps do not match."
        )
        return row
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
    return row


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


def _write_report(path: Path, report: dict[str, Any]) -> None:
    lines = [
        "# Gate-2 v2 matched pilot report",
        "",
        (
            "Gate 2 is excluded descriptive pilot/readiness evidence with "
            "five repetitions per condition-mode cell, not a powered study."
        ),
        "",
        "## Coverage",
        "",
        f"- Expected outcomes: `60`",
        (
            "- Analyzable outcomes: "
            f"`{report['selected_analyzable_episode_count']}`"
        ),
        f"- Exact complete triplets: `{report['matched_triplet_count']}/20`",
        f"- Complete collection: `{report['collection_complete']}`",
        "",
        "## Timing semantics",
        "",
        (
            "Simulated/environment execution time is `control_steps / 20 Hz`. "
            "Wall-control and total wall time are retained separately. Shared "
            "scheduler waits and inference latency are diagnostics and never "
            "increase simulated-step count. Autonomous inference latency is "
            "reported from replan step logs and contributes to wall time."
        ),
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
        "Gate 2 is a descriptive excluded pilot, not a powered experiment."
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
    shared_rows, fixed_rows, cosine_rows, wait_rows = _shared_rows(
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
    for episode in autonomous_schedule["episodes"]:
        summary_path = _autonomous_summary_path(autonomous_root, episode)
        if summary_path.is_file():
            try:
                row = _autonomous_row(
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
        autonomous_rows.append(row)

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
    (output_dir / "validation_report.json").write_text(
        json.dumps(report, indent=2) + "\n",
        encoding="utf-8",
    )
    _write_report(output_dir / "REPORT.md", report)
    return report
