#!/usr/bin/env python3
"""Compare autonomous and operator experiments with paper-style metrics."""

from __future__ import annotations

import csv
import dataclasses
import json
import math
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import tyro


MODE_ORDER = (
    "autonomous",
    "teleoperation",
    "takeover",
    "fixed_blend",
    "cosine_blend",
)
MODE_LABELS = {
    "autonomous": "π0.5",
    "teleoperation": "Teleoperation",
    "takeover": "Takeover",
    "fixed_blend": "Blending",
    "cosine_blend": "Cosine",
}
MODE_COLORS = {
    "autonomous": "#4C78A8",
    "teleoperation": "#F58518",
    "takeover": "#54A24B",
    "fixed_blend": "#E45756",
    "cosine_blend": "#B279A2",
}


@dataclasses.dataclass
class Args:
    autonomous_root: str = (
        "outputs/autonomous_deterministic_n20_state0_v1"
    )
    teleoperation_root: str = "outputs/saps_libero_teleoperation_v1"
    shared_autonomy_root: str = (
        "outputs/saps_libero_shared_autonomy_v1"
    )
    output_dir: str = "results/saps_libero_current"
    control_frequency_hz: float = 20.0
    comparison_max_steps: int = 280


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        value = json.load(file)

    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object: {path}")

    return value


def _read_steps(path: Path) -> list[dict[str, Any]]:
    steps: list[dict[str, Any]] = []

    with path.open("r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            if not line.strip():
                continue

            value = json.loads(line)

            if not isinstance(value, dict):
                raise ValueError(
                    f"Invalid step at {path}:{line_number}."
                )

            steps.append(value)

    if not steps:
        raise ValueError(f"No steps found in {path}.")

    return steps


def _mean(values: Iterable[float]) -> float:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return float(np.mean(finite)) if finite else math.nan


def _sample_sem(values: Iterable[float]) -> float:
    finite = [float(value) for value in values if math.isfinite(float(value))]

    if len(finite) < 2:
        return math.nan

    return float(np.std(finite, ddof=1) / math.sqrt(len(finite)))


def _intervention_segments(active: list[bool]) -> int:
    return sum(
        current and (index == 0 or not active[index - 1])
        for index, current in enumerate(active)
    )


def _path_length(steps: list[dict[str, Any]]) -> float:
    positions = np.asarray(
        [step["eef_position"] for step in steps],
        dtype=np.float64,
    )

    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("eef_position must contain three dimensions.")

    if len(positions) < 2:
        return 0.0

    return float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())


def _episode_metrics(
    summary_path: Path,
    *,
    scheduled_mode: int,
    scheduled_cell: int,
    fallback_frequency_hz: float,
) -> dict[str, Any]:
    summary = _read_json(summary_path)
    steps = _read_steps(summary_path.with_name("steps.jsonl"))
    mode = str(summary.get("arbitration_mode", "autonomous"))
    control_steps = int(summary["control_steps"])
    frequency = float(
        summary.get("control_frequency_hz", fallback_frequency_hz)
    )
    simulated_seconds = float(
        summary.get("simulated_control_seconds", control_steps / frequency)
    )
    active = [
        bool(
            step.get(
                "human_active",
                step.get("operator_motion_active", False),
            )
        )
        for step in steps
    ]
    active_steps = sum(active)
    effective_weights = [
        float(step["effective_autonomy_weight"])
        for step in steps
        if step.get("effective_autonomy_weight") is not None
    ]
    cosine_values = [
        float(step["cosine_similarity"])
        for step in steps
        if step.get("cosine_similarity") is not None
    ]

    return {
        "mode": mode,
        "mode_label": MODE_LABELS.get(mode, mode),
        "condition_id": str(summary["condition_id"]),
        "trial_index": int(summary["trial_index"]),
        "initial_state_index": int(summary["initial_state_index"]),
        "policy_episode_seed": int(summary["policy_episode_seed"]),
        "offset_distance_m": float(summary["offset_distance"]),
        "success": int(bool(summary["success"])),
        "termination_reason": str(
            summary.get(
                "termination_reason",
                "success" if summary["success"] else "timeout",
            )
        ),
        "control_steps": control_steps,
        "completion_time_s": simulated_seconds,
        "wall_control_time_s": float(summary["control_elapsed_seconds"]),
        "eef_path_length_m": _path_length(steps),
        "human_active_steps": active_steps,
        "human_active_fraction": active_steps / len(steps),
        "paper_human_intervention_fraction": (
            0.0
            if mode == "autonomous"
            else 1.0
            if mode == "teleoperation"
            else active_steps / len(steps)
        ),
        "intervention_segments": _intervention_segments(active),
        "mean_effective_autonomy_weight": _mean(effective_weights),
        "mean_cosine_similarity_when_defined": _mean(cosine_values),
        "logged_steps": len(steps),
        "scheduled_episodes_for_mode": scheduled_mode,
        "scheduled_episodes_for_cell": scheduled_cell,
        "summary_path": str(summary_path),
    }


def _load_autonomous(
    root: Path,
    fallback_frequency_hz: float,
) -> tuple[list[dict[str, Any]], int]:
    summaries = sorted(root.rglob("summary.json"))
    condition_counts: dict[str, int] = {}

    for path in summaries:
        condition_id = str(_read_json(path)["condition_id"])
        condition_counts[condition_id] = condition_counts.get(condition_id, 0) + 1

    return (
        [
            _episode_metrics(
                path,
                scheduled_mode=len(summaries),
                scheduled_cell=condition_counts[
                    str(_read_json(path)["condition_id"])
                ],
                fallback_frequency_hz=fallback_frequency_hz,
            )
            for path in summaries
        ],
        len(summaries),
    )


def _load_operator_session(
    root: Path,
    fallback_frequency_hz: float,
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    schedule = _read_json(root / "schedule.json")
    scheduled: dict[str, int] = {}
    scheduled_cells: dict[tuple[str, str], int] = {}

    for episode in schedule["episodes"]:
        mode = str(episode["mode"])
        scheduled[mode] = scheduled.get(mode, 0) + 1
        cell = (mode, str(episode["condition_id"]))
        scheduled_cells[cell] = scheduled_cells.get(cell, 0) + 1

    rows: list[dict[str, Any]] = []

    for episode in schedule["episodes"]:
        if episode["status"] != "completed":
            continue

        valid_attempts = [
            attempt
            for attempt in episode["attempts"]
            if attempt.get("valid")
            and attempt.get("summary_path")
            and attempt.get("selected_for_analysis", True)
        ]

        if len(valid_attempts) != 1:
            raise ValueError(
                f"Expected one valid attempt for {episode['episode_id']}."
            )

        summary_path = Path(valid_attempts[0]["summary_path"])
        row = _episode_metrics(
            summary_path,
            scheduled_mode=scheduled[str(episode["mode"])],
            scheduled_cell=scheduled_cells[
                (str(episode["mode"]), str(episode["condition_id"]))
            ],
            fallback_frequency_hz=fallback_frequency_hz,
        )

        if row["policy_episode_seed"] != int(
            episode["policy_episode_seed"]
        ):
            raise ValueError(
                f"Seed mismatch for {episode['episode_id']}."
            )

        rows.append(row)

    return rows, scheduled


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    if not rows:
        path.write_text("\n", encoding="utf-8")
        return

    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _aggregate(
    rows: list[dict[str, Any]],
    group_fields: tuple[str, ...],
) -> list[dict[str, Any]]:
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}

    for row in rows:
        key = tuple(row[field] for field in group_fields)
        groups.setdefault(key, []).append(row)

    output: list[dict[str, Any]] = []

    for key, episodes in sorted(groups.items(), key=lambda item: item[0]):
        record = dict(zip(group_fields, key))
        scheduled_field = (
            "scheduled_episodes_for_cell"
            if "condition_id" in group_fields
            else "scheduled_episodes_for_mode"
        )
        scheduled = int(episodes[0][scheduled_field])
        record.update(
            {
                "n_completed": len(episodes),
                "n_scheduled": scheduled,
                "coverage_fraction": len(episodes) / scheduled,
                "n_success": sum(row["success"] for row in episodes),
                "success_rate": _mean(row["success"] for row in episodes),
                "completion_time_mean_s": _mean(
                    row["completion_time_s"] for row in episodes
                ),
                "completion_time_sem_s": _sample_sem(
                    row["completion_time_s"] for row in episodes
                ),
                "eef_path_mean_m": _mean(
                    row["eef_path_length_m"] for row in episodes
                ),
                "eef_path_sem_m": _sample_sem(
                    row["eef_path_length_m"] for row in episodes
                ),
                "paper_intervention_mean": _mean(
                    row["paper_human_intervention_fraction"]
                    for row in episodes
                ),
                "active_input_mean": _mean(
                    row["human_active_fraction"] for row in episodes
                ),
                "intervention_segments_mean": _mean(
                    row["intervention_segments"] for row in episodes
                ),
            }
        )
        output.append(record)

    return output


def _pair_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    autonomous = {
        (
            row["condition_id"],
            row["trial_index"],
            row["initial_state_index"],
            row["policy_episode_seed"],
        ): row
        for row in rows
        if row["mode"] == "autonomous"
    }
    paired: list[dict[str, Any]] = []

    for row in rows:
        if row["mode"] == "autonomous":
            continue

        key = (
            row["condition_id"],
            row["trial_index"],
            row["initial_state_index"],
            row["policy_episode_seed"],
        )
        baseline = autonomous.get(key)

        if baseline is None:
            continue

        paired.append(
            {
                "mode": row["mode"],
                "condition_id": row["condition_id"],
                "trial_index": row["trial_index"],
                "policy_episode_seed": row["policy_episode_seed"],
                "operator_success": row["success"],
                "autonomous_success": baseline["success"],
                "success_delta": row["success"] - baseline["success"],
                "completion_time_delta_s": (
                    row["completion_time_s"]
                    - baseline["completion_time_s"]
                ),
                "eef_path_delta_m": (
                    row["eef_path_length_m"]
                    - baseline["eef_path_length_m"]
                ),
            }
        )

    return paired


def _paired_summary(paired: list[dict[str, Any]]) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []

    for mode in MODE_ORDER:
        selected = [row for row in paired if row["mode"] == mode]

        if not selected:
            continue

        output.append(
            {
                "mode": mode,
                "n_pairs": len(selected),
                "operator_success_rate": _mean(
                    row["operator_success"] for row in selected
                ),
                "matched_autonomous_success_rate": _mean(
                    row["autonomous_success"] for row in selected
                ),
                "success_rate_delta": _mean(
                    row["success_delta"] for row in selected
                ),
                "completion_time_delta_mean_s": _mean(
                    row["completion_time_delta_s"] for row in selected
                ),
                "eef_path_delta_mean_m": _mean(
                    row["eef_path_delta_m"] for row in selected
                ),
            }
        )

    return output


def _save_figure(fig: Any, output: Path, name: str) -> None:
    output.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output / f"{name}.png", dpi=200)
    fig.savefig(output / f"{name}.pdf")
    plt.close(fig)


def _plot_success_distance(rows: list[dict[str, Any]], output: Path) -> None:
    summary = _aggregate(rows, ("mode", "offset_distance_m"))
    fig, axis = plt.subplots(figsize=(7.2, 4.2))

    for mode in MODE_ORDER:
        selected = [row for row in summary if row["mode"] == mode]

        if not selected:
            continue

        selected.sort(key=lambda row: row["offset_distance_m"])
        axis.plot(
            [row["offset_distance_m"] for row in selected],
            [100.0 * row["success_rate"] for row in selected],
            marker="o",
            label=MODE_LABELS[mode],
            color=MODE_COLORS[mode],
        )

    axis.set(xlabel="Perturbation distance (m)", ylabel="Success rate (%)")
    axis.set_ylim(-3, 103)
    axis.grid(alpha=0.25)
    axis.legend(ncol=2)
    _save_figure(fig, output, "success_vs_perturbation_distance")


def _plot_mode_metric(
    summaries: list[dict[str, Any]],
    output: Path,
    *,
    field: str,
    ylabel: str,
    name: str,
    scale: float = 1.0,
) -> None:
    selected = [
        next((row for row in summaries if row["mode"] == mode), None)
        for mode in MODE_ORDER
    ]
    selected = [row for row in selected if row is not None]
    fig, axis = plt.subplots(figsize=(7.2, 4.2))
    values = [scale * float(row[field]) for row in selected]
    bars = axis.bar(
        [MODE_LABELS[row["mode"]] for row in selected],
        values,
        color=[MODE_COLORS[row["mode"]] for row in selected],
    )

    for bar, row in zip(bars, selected):
        axis.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height(),
            f"n={row['n_completed']}",
            ha="center",
            va="bottom",
            fontsize=8,
        )

    axis.set_ylabel(ylabel)
    axis.grid(axis="y", alpha=0.25)
    _save_figure(fig, output, name)


def _format_value(value: float, digits: int = 1) -> str:
    return "—" if not math.isfinite(value) else f"{value:.{digits}f}"


def _write_report(
    path: Path,
    mode_summary: list[dict[str, Any]],
    paired: list[dict[str, Any]],
    paired_summary: list[dict[str, Any]],
    over_horizon: list[dict[str, Any]],
) -> None:
    by_mode = {row["mode"]: row for row in mode_summary}
    lines = [
        "# Current SAPS Replication Results",
        "",
        "## Quantitative summary",
        "",
        "| Method | Completed / scheduled | Success | Completion time (s) | EE path (m) | Human intervention |",
        "|---|---:|---:|---:|---:|---:|",
    ]

    for mode in MODE_ORDER:
        row = by_mode.get(mode)

        if row is None:
            continue

        lines.append(
            "| {label} | {done} / {scheduled} | {success:.1f}% | "
            "{time} | {path} | {human:.1f}% |".format(
                label=MODE_LABELS[mode],
                done=row["n_completed"],
                scheduled=row["n_scheduled"],
                success=100.0 * row["success_rate"],
                time=_format_value(row["completion_time_mean_s"]),
                path=_format_value(row["eef_path_mean_m"], 3),
                human=100.0 * row["paper_intervention_mean"],
            )
        )

    lines.extend(
        [
            "",
            "Completion time is simulated control time (`control_steps / 20 Hz`), matching the environment clock rather than inference-dependent wall time. Human intervention follows the paper: autonomous is 0%, teleoperation is 100%, and shared modes use the fraction of steps above the activity threshold.",
            "",
            "## Seed-matched pilot comparison",
            "",
            "| Method | Matched n | Operator success | Matched π0.5 success | Success difference | Time difference (s) | Path difference (m) |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )

    for row in paired_summary:
        lines.append(
            "| {label} | {n} | {operator:.1f}% | {autonomous:.1f}% | "
            "{success:+.1f} pp | {time:+.1f} | {path:+.3f} |".format(
                label=MODE_LABELS[row["mode"]],
                n=row["n_pairs"],
                operator=100.0 * row["operator_success_rate"],
                autonomous=(
                    100.0 * row["matched_autonomous_success_rate"]
                ),
                success=100.0 * row["success_rate_delta"],
                time=row["completion_time_delta_mean_s"],
                path=row["eef_path_delta_mean_m"],
            )
        )

    lines.extend(
        [
            "",
            "Differences are operator-assisted minus the exactly seed-matched autonomous episode. Negative time and path differences indicate improvement.",
            "",
            "## Interpretation limits",
            "",
            f"There are {len(paired)} operator episodes with a seed-matched autonomous baseline. Operator collection is incomplete and unbalanced across methods and perturbations. All currently collected operator episodes succeeded, so these data are suitable for pipeline validation and descriptive pilot reporting, but not for significance testing or final comparative claims.",
            "",
            f"{len(over_horizon)} collected operator episodes exceed the formal 280-step horizon and must be redone or treated as 14-second timeouts for the final comparison.",
            "",
            "Error bars and inferential tests should be added only after the planned matched repetitions are complete. Failed episodes must remain in the denominator and contribute their full timeout to completion-time summaries, as in the paper.",
            "",
            "## Generated artifacts",
            "",
            "- `episode_metrics.csv`: auditable episode-level metrics.",
            "- `mode_summary.csv`: paper-style method summary with denominators.",
            "- `condition_mode_summary.csv`: perturbation-level results.",
            "- `paired_autonomous_comparisons.csv`: matched per-seed deltas.",
            "- `paired_mode_summary.csv`: aggregate matched-mode differences.",
            "- `validation_report.json`: coverage and pairing status.",
            "- `plots/`: PNG and PDF figures.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main(args: Args) -> None:
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    autonomous, autonomous_scheduled = _load_autonomous(
        Path(args.autonomous_root),
        args.control_frequency_hz,
    )
    teleoperation, teleop_scheduled = _load_operator_session(
        Path(args.teleoperation_root),
        args.control_frequency_hz,
    )
    shared, shared_scheduled = _load_operator_session(
        Path(args.shared_autonomy_root),
        args.control_frequency_hz,
    )
    rows = autonomous + teleoperation + shared
    mode_summary = _aggregate(rows, ("mode",))
    condition_summary = _aggregate(
        rows,
        ("mode", "condition_id", "offset_distance_m"),
    )
    paired = _pair_rows(rows)
    paired_summary = _paired_summary(paired)
    over_horizon = [
        {
            "mode": row["mode"],
            "condition_id": row["condition_id"],
            "trial_index": row["trial_index"],
            "control_steps": row["control_steps"],
            "completion_time_s": row["completion_time_s"],
            "summary_path": row["summary_path"],
        }
        for row in rows
        if row["mode"] != "autonomous"
        and row["control_steps"] > args.comparison_max_steps
    ]
    scheduled = {
        "autonomous": autonomous_scheduled,
        **teleop_scheduled,
        **shared_scheduled,
    }

    _write_csv(output / "episode_metrics.csv", rows)
    _write_csv(output / "mode_summary.csv", mode_summary)
    _write_csv(output / "condition_mode_summary.csv", condition_summary)
    _write_csv(output / "paired_autonomous_comparisons.csv", paired)
    _write_csv(output / "paired_mode_summary.csv", paired_summary)
    (output / "validation_report.json").write_text(
        json.dumps(
            {
                "scheduled_by_mode": scheduled,
                "completed_by_mode": {
                    mode: sum(row["mode"] == mode for row in rows)
                    for mode in MODE_ORDER
                },
                "paired_with_autonomous_by_mode": {
                    mode: sum(row["mode"] == mode for row in paired)
                    for mode in MODE_ORDER
                    if mode != "autonomous"
                },
                "operator_collection_complete": all(
                    sum(row["mode"] == mode for row in rows) == count
                    for mode, count in scheduled.items()
                    if mode != "autonomous"
                ),
                "comparison_max_steps": args.comparison_max_steps,
                "operator_episodes_exceeding_horizon": over_horizon,
                "warnings": [
                    "Operator collection is incomplete; do not run final inferential tests.",
                    "Operator modes have unequal sample sizes and perturbation coverage.",
                ],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    plots = output / "plots"
    _plot_success_distance(rows, plots)
    _plot_mode_metric(
        mode_summary,
        plots,
        field="completion_time_mean_s",
        ylabel="Mean completion time (s)",
        name="completion_time_by_method",
    )
    _plot_mode_metric(
        mode_summary,
        plots,
        field="eef_path_mean_m",
        ylabel="Mean end-effector path length (m)",
        name="eef_path_by_method",
    )
    _plot_mode_metric(
        mode_summary,
        plots,
        field="paper_intervention_mean",
        ylabel="Human intervention (%)",
        name="human_intervention_by_method",
        scale=100.0,
    )
    _write_report(
        output / "REPORT.md",
        mode_summary,
        paired,
        paired_summary,
        over_horizon,
    )
    print(f"Wrote comparison analysis to {output}")


if __name__ == "__main__":
    main(tyro.cli(Args))
