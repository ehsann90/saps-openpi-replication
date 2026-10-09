"""Derive separate tracking, timing, and torque metrics from Panda replays."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
from typing import Optional


CONDITIONS = (
    "absolute_franka_hand", "absolute_robotiq", "normalized_actions"
)
RATIO_SUMMARY_MIN_DELTA_RAD = 0.01


def first_fraction_time(
    ticks: list, joint: int, q0: float, delta: float, fraction: float
) -> Optional[float]:
    """Return first discrete simulator-time threshold crossing, if reached."""
    if not ticks or abs(delta) < 1e-12:
        return None
    direction = math.copysign(1.0, delta)
    threshold = fraction * abs(delta)
    start_time = ticks[0]["sim_time_s"]
    for tick in ticks:
        if direction * (tick["q"][joint] - q0) >= threshold:
            return tick["sim_time_s"] - start_time
    return None


def inspect_condition(root: Path, name: str) -> tuple:
    replay = json.loads((root / name / "replay.json").read_text())
    ticks = json.loads((root / name / "server_policy_log.json").read_text())
    if not replay["success"] or len(replay["actions"]) != 8:
        raise ValueError(f"{name}: incomplete replay")
    actions = replay["actions"]
    indices = [action["update_index"] for action in actions]
    if indices != sorted(set(indices)):
        raise ValueError(f"{name}: non-increasing update indices")
    rows = []
    for i, action in enumerate(actions):
        start = indices[i]
        end = indices[i + 1] if i < 7 else min(start + 16, len(ticks))
        window = ticks[start:end]
        if not window:
            raise ValueError(f"{name}: empty response window for action {i}")
        fixed_end = ticks[start + 16] if start + 16 < len(ticks) else None
        for joint in range(7):
            q0 = action["issue_state"]["q"][joint]
            target = action["target_q_rad"][joint]
            q_end = action["end_boundary"]["q"][joint]
            delta = target - q0
            realized = q_end - q0
            direction = math.copysign(1.0, delta) if delta else 0.0
            overshoot = max(
                [0.0] + [direction * (tick["q"][joint] - target)
                         for tick in window]
            )
            rows.append({
                "condition": name,
                "action": i,
                "joint": joint + 1,
                "issue_q_rad": q0,
                "issue_dq_rad_s": action["issue_state"]["dq"][joint],
                "target_q_rad": target,
                "commanded_delta_rad": delta,
                "end_q_rad": q_end,
                "end_dq_rad_s": action["end_boundary"]["dq"][joint],
                "realized_delta_rad": realized,
                "target_minus_end_rad": target - q_end,
                "realized_over_commanded": realized / delta if abs(delta) > 1e-12 else None,
                "ratio_in_summary": abs(delta) >= RATIO_SUMMARY_MIN_DELTA_RAD,
                "fixed_16_tick_end_q_rad": fixed_end["q"][joint] if fixed_end else None,
                "fixed_16_tick_error_rad": (
                    target - fixed_end["q"][joint] if fixed_end else None
                ),
                "time_to_10pct_sim_s": first_fraction_time(
                    window, joint, q0, delta, 0.1
                ),
                "time_to_50pct_sim_s": first_fraction_time(
                    window, joint, q0, delta, 0.5
                ),
                "max_target_exceedance_rad": overshoot,
                "max_abs_tracking_error_in_interval_rad": max(
                    abs(target - tick["q"][joint]) for tick in window
                ),
                "actual_interval_wall_s": (
                    action["end_boundary"]["wall_s"]
                    - action["issue_state"]["wall_s"]
                ),
                "update_tick": start,
                "next_update_tick": indices[i + 1] if i < 7 else None,
            })
    offsets = [
        (action["command_start_wall_s"] - action["planned_issue_wall_s"]) * 1000
        for action in actions
    ]
    tick_intervals = [b - a for a, b in zip(indices, indices[1:])]
    stable_ratios = [
        row["realized_over_commanded"] for row in rows
        if row["ratio_in_summary"]
    ]
    all_ratios = [
        row["realized_over_commanded"] for row in rows
        if row["realized_over_commanded"] is not None
    ]
    lower = replay["panda_limits_rad"]["lower"]
    upper = replay["panda_limits_rad"]["upper"]
    summary = {
        "condition": name,
        "actions": 8,
        "joint_action_pairs": len(rows),
        "median_abs_boundary_target_error_rad": statistics.median(
            abs(row["target_minus_end_rad"]) for row in rows
        ),
        "max_abs_boundary_target_error_rad": max(
            abs(row["target_minus_end_rad"]) for row in rows
        ),
        "median_ratio_all_nonzero_deltas": statistics.median(all_ratios),
        "median_ratio_delta_at_least_0_01_rad": statistics.median(stable_ratios),
        "ratio_summary_count": len(stable_ratios),
        "ratio_all_range": [min(all_ratios), max(all_ratios)],
        "max_target_exceedance_rad": max(
            row["max_target_exceedance_rad"] for row in rows
        ),
        "time_to_10pct_reached_count": sum(
            row["time_to_10pct_sim_s"] is not None for row in rows
        ),
        "median_time_to_10pct_sim_s_when_reached": statistics.median(
            row["time_to_10pct_sim_s"] for row in rows
            if row["time_to_10pct_sim_s"] is not None
        ),
        "time_to_50pct_reached_count": sum(
            row["time_to_50pct_sim_s"] is not None for row in rows
        ),
        "update_ticks": indices,
        "update_tick_intervals": tick_intervals,
        "max_abs_update_interval_deviation_ticks": max(
            abs(interval - 16) for interval in tick_intervals
        ),
        "issue_wall_offset_ms": offsets,
        "max_abs_issue_wall_offset_ms": max(abs(x) for x in offsets),
        "median_action_wall_interval_s": statistics.median(
            row["actual_interval_wall_s"] for row in rows
        ),
        "measured_q_min_limit_margin_rad": min(
            min(q - lower[joint], upper[joint] - q)
            for tick in ticks for joint, q in enumerate(tick["q"])
        ),
        "target_min_limit_margin_rad": min(
            action["target_limit_margin_rad"] for action in actions
        ),
        "policy_action_clipped_coordinates": sum(
            sum(abs(a - b) > 1e-12 for a, b in zip(
                action["policy_action"], action["clipped_action"]
            ))
            for action in actions if action["clipped_action"] is not None
        ),
        "max_abs_dq_rad_s_by_joint": [
            max(abs(tick["dq"][joint]) for tick in ticks)
            for joint in range(7)
        ],
        "max_abs_computed_torque_by_joint_Nm": [
            max(abs(tick["joint_torques_computed"][joint]) for tick in ticks)
            for joint in range(7)
        ],
        "max_abs_prev_safened_torque_by_joint_Nm": [
            max(abs(tick["prev_joint_torques_computed_safened"][joint])
                for tick in ticks)
            for joint in range(7)
        ],
        "fault_codes": replay["fault_codes"],
        "all_commands_successful": replay["all_commands_successful"],
        "port_closed": replay["port_closed"],
        "server_policy_log_count": len(ticks),
        "physics_dt_s": replay["physics_dt_s"],
        "robot_timestamp_clock": "protobuf wall clock, not simulator time",
        "sim_time_clock": "server tick index multiplied by PyBullet fixed timestep",
    }
    return rows, summary


def write_tracking_tables(root: Path, rows: list) -> None:
    """Write per-action, per-joint boundary errors and motion ratios."""
    path = root / "tracking_tables.md"
    if path.exists():
        raise FileExistsError(path)
    lines = [
        "# Panda recorded-action tracking tables", "",
        "Signed error is target minus measured q at the next 15 Hz action boundary.",
        "Ratio is measured displacement divided by target minus issue-time q.",
        "A `*` marks a ratio whose commanded displacement is below 0.01 rad;",
        "these are excluded from the thresholded summary median. All exact",
        "values and denominators are in `per_action_joint.csv`.", "",
    ]
    for condition in CONDITIONS:
        condition_rows = [row for row in rows if row["condition"] == condition]
        lines.extend([f"## {condition}", ""])
        for title, key in (
            ("Boundary target error (rad)", "target_minus_end_rad"),
            ("Realized/commanded displacement", "realized_over_commanded"),
        ):
            lines.extend([
                f"### {title}", "",
                "| Action | J1 | J2 | J3 | J4 | J5 | J6 | J7 |",
                "|---:|---:|---:|---:|---:|---:|---:|---:|",
            ])
            for action in range(8):
                selected = [
                    row for row in condition_rows if row["action"] == action
                ]
                cells = []
                for row in selected:
                    value = row[key]
                    cell = "—" if value is None else f"{value:+.3f}"
                    if key == "realized_over_commanded" and not row[
                        "ratio_in_summary"
                    ]:
                        cell += "*"
                    cells.append(cell)
                lines.append(f"| {action} | " + " | ".join(cells) + " |")
            lines.append("")
    path.write_text("\n".join(lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    all_rows = []
    summaries = {}
    for condition in CONDITIONS:
        rows, summary = inspect_condition(args.root, condition)
        all_rows.extend(rows)
        summaries[condition] = summary
    csv_path = args.root / "per_action_joint.csv"
    if csv_path.exists():
        raise FileExistsError(csv_path)
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    summary_path = args.root / "summary.json"
    if summary_path.exists():
        raise FileExistsError(summary_path)
    summary_path.write_text(json.dumps(summaries, indent=2) + "\n")
    write_tracking_tables(args.root, all_rows)
    print(json.dumps({key: {
        "median_ratio_all": value["median_ratio_all_nonzero_deltas"],
        "median_abs_error": value["median_abs_boundary_target_error_rad"],
        "max_abs_error": value["max_abs_boundary_target_error_rad"],
        "tick_intervals": value["update_tick_intervals"],
    } for key, value in summaries.items()}, indent=2))


if __name__ == "__main__":
    main()
