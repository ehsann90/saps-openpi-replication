#!/usr/bin/env python3
"""Analyze saved native DROID gripper predictions without running Isaac."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
import numpy as np


THRESHOLD = 0.5
WINDOWS = ((11, 16), (114, 120), (145, 149))
REQUIRED_REPLAN_FILES = (
    "actions.npy", "selected_actions.npy", "response.json",
    "execution.json", "observation.json",
)


def classify_gripper(values: np.ndarray, executed_count: int) -> dict:
    """Classify native predictions using the strict DROID threshold."""
    close = np.flatnonzero(np.asarray(values) > THRESHOLD)
    first = int(close[0]) if len(close) else None
    return {
        "first_close_index": first,
        "first_close_region": (
            "none" if first is None else
            "executed" if first < executed_count else "discarded"
        ),
        "close_count_native": int(len(close)),
        "close_count_executed": int(np.count_nonzero(close < executed_count)),
        "close_count_discarded": int(np.count_nonzero(close >= executed_count)),
    }


def overlap_metrics(
    previous: np.ndarray, current: np.ndarray, executed_count: int,
) -> tuple[float | None, int | None, int]:
    """Compare prior future slots to new slots at the same nominal time."""
    old = np.asarray(previous)[executed_count:]
    new = np.asarray(current)[:len(old)]
    count = min(len(old), len(new))
    if not count:
        return None, None, 0
    old, new = old[:count], new[:count]
    return (float(np.mean(np.abs(old - new))),
            int(np.count_nonzero((old > THRESHOLD) != (new > THRESHOLD))),
            count)


def read_json(path: Path) -> dict:
    with path.open() as stream:
        return json.load(stream)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def load_replans(run: Path) -> tuple[dict, dict, list[dict], list[str], list[dict]]:
    provenance = read_json(run / "provenance.json")
    episode = read_json(run / "episode.json")
    completed = int(episode["completed_replans"])
    replans = []
    manifest = []
    for source in (run / "provenance.json", run / "episode.json"):
        manifest.append({"path": str(source.relative_to(run)),
                         "sha256": sha256(source)})
    for index in range(completed):
        directory = run / "replans" / f"{index:04d}"
        missing = [name for name in REQUIRED_REPLAN_FILES
                   if not (directory / name).is_file()]
        if missing:
            raise ValueError(f"Replan {index} lacks {missing}")
        for name in REQUIRED_REPLAN_FILES:
            source = directory / name
            manifest.append({"path": str(source.relative_to(run)),
                             "sha256": sha256(source)})
        actions = np.load(directory / "actions.npy", allow_pickle=False)
        selected = np.load(directory / "selected_actions.npy", allow_pickle=False)
        response = read_json(directory / "response.json")
        execution = read_json(directory / "execution.json")
        observation = read_json(directory / "observation.json")
        if (actions.ndim != 2 or actions.shape[1] != 8
                or not np.issubdtype(actions.dtype, np.floating)
                or not np.isfinite(actions).all()):
            raise ValueError(f"Invalid native action array at replan {index}")
        count = int(response["selected_action_count"])
        if (response["native_horizon"] != len(actions)
                or response["discarded_suffix_count"] != len(actions) - count
                or response["native_shape"] != list(actions.shape)
                or selected.shape != (count, 8)
                or not np.array_equal(selected, actions[:count])):
            raise ValueError(f"Response/selection mismatch at replan {index}")
        rows = execution["rows"]
        if (execution["status"] != "complete" or len(rows) != count
                or execution["actions_executed"] != count
                or execution["arm_control_mode"] != provenance["arm_control_mode"]
                or execution["velocity_scale_rad_s_per_action_unit"]
                != provenance["velocity_scale_rad_s_per_action_unit"]
                or execution["control_hz"] != 15
                or execution["physics_steps_per_action"] != 4
                or not np.isclose(execution["physics_dt_seconds"], 1 / 60)
                or response["replan_index"] != index
                or execution["replan_index"] != index
                or observation["replan_index"] != index):
            raise ValueError(f"Incomplete execution at replan {index}")
        for action_index, row in enumerate(rows):
            if (row["action_index"] != action_index
                    or not np.allclose(row["raw_policy_action"],
                                       selected[action_index], rtol=0, atol=1e-6)
                    or row["safety_result"] != "accepted"):
                raise ValueError(f"Executed action mismatch at {index}:{action_index}")
        replans.append({"index": index, "actions": actions,
                        "selected": selected, "response": response,
                        "execution": execution, "observation": observation})
    if sum(len(x["selected"]) for x in replans) != episode["policy_actions_executed"]:
        raise ValueError("Episode action count differs from saved executions")
    partial = []
    for directory in sorted((run / "replans").iterdir()):
        if directory.is_dir() and int(directory.name) >= completed:
            partial.append(f"{directory.name}: " + ", ".join(
                sorted(path.name for path in directory.iterdir() if path.is_file())
            ))
    return provenance, episode, replans, partial, manifest


def contact_counts(summary: dict) -> tuple[int, int, int]:
    return (int(summary["left"]["loaded_events"]),
            int(summary["right"]["loaded_events"]),
            int(summary["bilateral_loaded_reports"]))


def extract_rows(replans: list[dict]) -> tuple[list[dict], list[dict], list[dict]]:
    summary_rows, native_rows, threshold_rows = [], [], []
    prior_actions = None
    prior_observation_step = None
    prior_contact = (0, 0, 0)
    for item in replans:
        index = item["index"]
        values = item["actions"][:, 7]
        execution = item["execution"]
        observation = item["observation"]
        count = len(item["selected"])
        classified = classify_gripper(values, count)
        step = int(observation["simulation_step_index"])
        offset = (step - prior_observation_step
                  if prior_observation_step is not None else None)
        if prior_actions is not None and offset == count * execution["physics_steps_per_action"]:
            overlap_mae, overlap_binary, overlap_count = overlap_metrics(
                prior_actions[:, 7], values, count
            )
        else:
            overlap_mae, overlap_binary, overlap_count = None, None, 0
        prior_actions, prior_observation_step = item["actions"], step
        transitions = execution["gripper_transitions"]
        initial_intent = (
            transitions[0]["from"] if transitions else
            execution["rows"][0]["binary_gripper_intent"]
        )
        summary_rows.append({
            "replan_index": index,
            "observation_step_index": step,
            "observation_step_delta_from_prior": offset,
            "native_horizon": len(values),
            "executed_count": count,
            "discarded_count": len(values) - count,
            "min_gripper_value": float(np.min(values)),
            "max_gripper_value": float(np.max(values)),
            **classified,
            "initial_intent": initial_intent,
            "final_executed_intent": execution["gripper_intent_after"],
            "executed_transition_count": len(transitions),
            "observation_width_m": observation["finger_width_m"],
            "overlap_mae_from_prior": overlap_mae,
            "overlap_binary_disagreements_from_prior": overlap_binary,
            "overlap_compared_slots": overlap_count,
        })
        previous_intent = initial_intent
        for action_index, value in enumerate(values):
            executed = action_index < count
            requested = "CLOSED" if value > THRESHOLD else "OPEN"
            row = execution["rows"][action_index] if executed else None
            if row is not None:
                if row["binary_gripper_intent"] != requested:
                    raise ValueError(f"Gripper intent mismatch at {index}:{action_index}")
                actual_transition = bool(row["gripper_transition_initiated"])
                if actual_transition != (previous_intent != requested):
                    raise ValueError(f"Transition mismatch at {index}:{action_index}")
                current_contact = contact_counts(row["contact_summary_after"])
                contact_delta = tuple(a - b for a, b in zip(current_contact, prior_contact))
                if any(value < 0 for value in contact_delta):
                    raise ValueError("Cumulative contact counter decreased")
                prior_contact = current_contact
                tcp = np.asarray(row["tcp_pose_after"]["position_xyz_m"])
                object_start = np.asarray(
                    execution["cube_pose_before"]["position_xyz_m"]
                )
                tcp_distance = float(np.linalg.norm(tcp - object_start))
                width_before = row["measured_total_width_m"]
                width_after = row["gripper_state_after"]["measured_width_m"]
            else:
                actual_transition = False
                contact_delta = (None, None, None)
                tcp_distance = width_before = width_after = None
            event = {
                "replan_index": index,
                "action_index": action_index,
                "native_gripper_value": float(value),
                "predicted_close": requested == "CLOSED",
                "status": "executed" if executed else "discarded",
                "previous_intent": previous_intent,
                "requested_intent": requested,
                "intent_basis": "observed" if executed else "hypothetical_suffix",
                "actual_transition": actual_transition,
                "hypothetical_suffix_transition": (
                    not executed and previous_intent != requested
                ),
                "measured_width_before_m": width_before,
                "measured_width_after_m": width_after,
                "tcp_to_object_start_snapshot_m": tcp_distance,
                "new_left_loaded_events": contact_delta[0],
                "new_right_loaded_events": contact_delta[1],
                "new_bilateral_loaded_reports": contact_delta[2],
                "observation_step_index": step,
                "nominal_action_step_index": (
                    step + action_index * execution["physics_steps_per_action"]
                ),
                "nominal_simulation_seconds": (
                    (step + action_index * execution["physics_steps_per_action"])
                    * execution["physics_dt_seconds"]
                ),
            }
            native_rows.append(event)
            if (event["predicted_close"] or event["actual_transition"]
                    or event["hypothetical_suffix_transition"]):
                threshold_rows.append(event)
            previous_intent = requested
    return summary_rows, native_rows, threshold_rows


def plot_heatmap(replans: list[dict], output: Path) -> None:
    grid = np.vstack([item["actions"][:, 7] for item in replans])
    norm = TwoSlopeNorm(vmin=min(0, float(grid.min())),
                        vcenter=THRESHOLD, vmax=max(1, float(grid.max())))
    fig, ax = plt.subplots(figsize=(10, 13), layout="constrained")
    image = ax.imshow(grid, cmap="coolwarm", norm=norm, aspect="auto",
                      interpolation="nearest", origin="lower")
    ax.axvline(7.5, color="black", linewidth=2)
    for item in replans:
        for event in item["execution"]["gripper_transitions"]:
            close = event["to"] == "CLOSED"
            ax.scatter(event["action_index"], item["index"],
                       marker="^" if close else "v", s=25,
                       facecolor="black" if close else "white",
                       edgecolor="black", linewidth=0.6)
    ax.set(xlabel="Native action index (0–7 executed; 8–14 discarded)",
           ylabel="Replan index", title="Native gripper action; threshold > 0.5")
    ax.set_xticks(range(grid.shape[1]))
    bar = fig.colorbar(image, ax=ax, shrink=0.75)
    bar.set_label("Continuous predicted gripper action (unclipped)")
    bar.ax.axhline(THRESHOLD, color="black", linewidth=1)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_first_close(rows: list[dict], output: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 4.5), layout="constrained")
    for region, color, label in (
        ("none", "#bbbbbb", "No CLOSE"),
        ("executed", "#215e9e", "First CLOSE in executed prefix"),
        ("discarded", "#e28522", "First CLOSE in discarded suffix"),
    ):
        subset = [r for r in rows if r["first_close_region"] == region]
        ax.scatter([r["replan_index"] for r in subset],
                   [r["first_close_index"] if region != "none" else -1
                    for r in subset], s=27, c=color, label=label)
    ax.axhline(7.5, color="black", linewidth=1)
    ax.set(xlabel="Replan index", ylabel="First predicted CLOSE index",
           title="First native CLOSE prediction per replan",
           xlim=(-1, len(rows)), ylim=(-1.8, 14.8))
    ax.set_yticks([-1, 0, 3, 7, 8, 14],
                  ["none", "0", "3", "7", "8", "14"])
    ax.legend(loc="upper right")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_maximum(rows: list[dict], output: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 4), layout="constrained")
    ax.plot([r["replan_index"] for r in rows],
            [r["max_gripper_value"] for r in rows], color="#215e9e",
            linewidth=1.2, marker=".", markersize=3)
    ax.axhline(THRESHOLD, color="#ad2424", linestyle="--",
               label="CLOSE iff value > 0.5")
    ax.set(xlabel="Replan index", ylabel="Maximum native gripper action",
           title="Maximum predicted gripper value per replan",
           xlim=(-1, len(rows)))
    ax.legend()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_window(
    replans: list[dict], native_rows: list[dict], start: int, end: int,
    output: Path,
) -> None:
    subset = replans[start:end + 1]
    grid = np.vstack([item["actions"][:, 7] for item in subset])
    all_values = np.concatenate([item["actions"][:, 7] for item in replans])
    norm = TwoSlopeNorm(vmin=min(0, float(all_values.min())),
                        vcenter=THRESHOLD, vmax=max(1, float(all_values.max())))
    fig = plt.figure(figsize=(12, 8), layout="constrained")
    axes = fig.subplots(3, 1, gridspec_kw={"height_ratios": [1.2, 1, 0.6]})
    image = axes[0].imshow(grid, cmap="coolwarm", norm=norm, aspect="auto",
                           interpolation="nearest", origin="lower")
    axes[0].axvline(7.5, color="black", linewidth=2)
    for item in subset:
        for event in item["execution"]["gripper_transitions"]:
            close = event["to"] == "CLOSED"
            axes[0].scatter(event["action_index"], item["index"] - start,
                            marker="^" if close else "v", s=65,
                            facecolor="black" if close else "white",
                            edgecolor="black")
    axes[0].set(yticks=range(len(subset)),
                yticklabels=[str(item["index"]) for item in subset],
                xticks=range(grid.shape[1]), ylabel="Replan",
                xlabel="Native index (0–7 executed; 8–14 discarded)",
                title=f"Replans {start}–{end}: predicted gripper action")
    fig.colorbar(image, ax=axes[0], label="Unclipped action; CLOSE > 0.5")
    window_rows = [row for row in native_rows
                   if start <= row["replan_index"] <= end
                   and row["status"] == "executed"]
    ticks = np.array([row["replan_index"] * 8 + row["action_index"]
                      for row in window_rows])
    widths = np.array([row["measured_width_before_m"] for row in window_rows])
    axes[1].plot(ticks, widths * 1000, marker=".", linewidth=1,
                 label="Measured opening before action")
    for row in window_rows:
        if row["actual_transition"]:
            tick = row["replan_index"] * 8 + row["action_index"]
            color = "#a42727" if row["requested_intent"] == "CLOSED" else "#215e9e"
            axes[1].axvline(tick, color=color, alpha=0.8)
            axes[1].text(tick + 0.2, 75, row["requested_intent"],
                         rotation=90, fontsize=7, color=color, va="top")
    axes[1].set(ylabel="Opening before action (mm)",
                title="Executed actions; vertical lines mark observed intent changes")
    axes[1].set_ylim(-4, 92)
    left = np.array([row["new_left_loaded_events"] for row in window_rows])
    right = np.array([row["new_right_loaded_events"] for row in window_rows])
    axes[2].bar(ticks - 0.15, left, width=0.3, label="New left pad events")
    axes[2].bar(ticks + 0.15, right, width=0.3, label="New right pad events")
    if not np.any(left) and not np.any(right):
        axes[2].set_ylim(0, 1)
        axes[2].text(np.mean(ticks), 0.5, "No new loaded pad events",
                     ha="center", va="center")
    axes[2].set(xlabel="Global executed action index (replan × 8 + action)",
                ylabel="New loaded contacts")
    axes[2].legend(loc="upper right", fontsize=8)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def markdown_report(
    run: Path, provenance: dict, episode: dict, replans: list[dict],
    summary: list[dict], native: list[dict], partial: list[str],
) -> str:
    horizons = Counter(row["native_horizon"] for row in summary)
    first = Counter(row["first_close_index"] for row in summary
                    if row["first_close_index"] is not None)
    any_close = sum(row["close_count_native"] > 0 for row in summary)
    prefix = sum(row["close_count_executed"] > 0 for row in summary)
    suffix_only = [row for row in summary
                   if row["first_close_region"] == "discarded"]
    suffix_description = ", ".join(
        "{} (index {})".format(row["replan_index"], row["first_close_index"])
        for row in suffix_only
    )
    suffix_replans = ", ".join(str(row["replan_index"])
                               for row in suffix_only)
    transitions = [row for row in native if row["actual_transition"]]
    overlaps = [row for row in summary
                if row["overlap_mae_from_prior"] is not None]
    all_values = np.array([row["native_gripper_value"] for row in native])
    events = [row for row in native if row["actual_transition"]
              or row["hypothetical_suffix_transition"]]
    event_lines = []
    for row in events:
        width = ("—" if row["measured_width_before_m"] is None else
                 f'{row["measured_width_before_m"]:.5f}')
        distance = ("—" if row["tcp_to_object_start_snapshot_m"] is None else
                    f'{row["tcp_to_object_start_snapshot_m"]:.3f}')
        contact = ("—" if row["new_left_loaded_events"] is None else
                   f'{row["new_left_loaded_events"]}/{row["new_right_loaded_events"]}')
        event_lines.append(
            f'| {row["replan_index"]}:{row["action_index"]} | '
            f'{row["nominal_simulation_seconds"]:.3f} | '
            f'{row["native_gripper_value"]:.3f} | {row["status"]} | '
            f'{row["previous_intent"]} → {row["requested_intent"]} | '
            f'{width} | {distance} | {contact} |'
        )
    window_lines = []
    for start, end in WINDOWS:
        subset = [row for row in summary if start <= row["replan_index"] <= end]
        close_replans = [row["replan_index"] for row in subset
                         if row["close_count_native"]]
        window_events = [row for row in transitions
                         if start <= row["replan_index"] <= end]
        start_object = np.asarray(
            replans[start]["observation"]["objects"][0]["position_xyz_m"]
        )
        end_object = np.asarray(
            replans[end]["execution"]["cube_pose_after"]["position_xyz_m"]
        )
        contact_before = (
            contact_counts(replans[start - 1]["execution"]["contact_summary"])
            if start else (0, 0, 0)
        )
        contact_after = contact_counts(
            replans[end]["execution"]["contact_summary"]
        )
        added = tuple(a - b for a, b in zip(contact_after, contact_before))
        window_lines.append(
            f'| {start}–{end} | {close_replans} | '
            f'{len(window_events)} | {np.linalg.norm(end_object-start_object):.4f} | '
            f'{added[0]}/{added[1]}/{added[2]} |'
        )
    return f"""# Offline native gripper-action analysis

## Source and scope

- Source run: `{run.as_posix()}` (run identifier `{run.name}`).
- Inputs: `provenance.json`, `episode.json`, and each complete
  `replans/0000`–`replans/{len(replans)-1:04d}` directory's `actions.npy`,
  `selected_actions.npy`, `response.json`, `execution.json`, and
  `observation.json`. SHA-256 hashes are in `input_manifest.csv`.
- Archive provenance: repository `{provenance['repository_commit']}`;
  OpenPI `{provenance['openpi_submodule_commit']}`; checkpoint
  `{provenance['checkpoint']}`; scene `{provenance['scene_config']}`;
  prompt `{provenance['prompt']}`; seed `{provenance['policy_episode_seed']}`;
  mode `{provenance['arm_control_mode']}`; scale
  `{provenance['velocity_scale_rad_s_per_action_unit']} rad/s/action unit`.
- Provenance records `repository_dirty={provenance['repository_dirty']}` and
  a scene-config path, but no immutable scene-file hash or copy. The exact
  local lighting values used at launch cannot be independently recovered
  from this archive; the recorded camera specifications and images remain
  available.
- Episode: `{episode['completed_replans']}` complete replans,
  `{episode['policy_actions_executed']}` executed actions, termination
  `{episode['termination_reason']}`. Every execution records 15 Hz and four
  60 Hz physics steps per action.
- Excluded partial replan: `{'; '.join(partial) if partial else 'none'}`.
  It has no complete native action response and contributes no predictions.

The values below are continuous native action coordinate 7, **not** grasp
probabilities. DROID requests CLOSED only when the value is strictly greater
than 0.5. No analysis value was clipped; the saved range is
`[{all_values.min():.6f}, {all_values.max():.6f}]`. There are
`{sum((np.asarray(item['actions'][:, 7]) < 0).any() or (np.asarray(item['actions'][:, 7]) > 1).any() for item in replans)}`
chunks with a value outside `[0, 1]`.

## Verified counts

| Measure | Result |
|---|---:|
| Complete native chunks | {len(replans)} |
| Native horizon distribution | {dict(sorted(horizons.items()))} |
| Executed prefix / discarded suffix per chunk | 8 / 7 |
| Any predicted CLOSE | {any_close} |
| CLOSE in executed prefix | {prefix} |
| CLOSE only in discarded suffix | {len(suffix_only)} |
| No CLOSE anywhere | {len(replans)-any_close} |
| First predicted CLOSE index distribution | {dict(sorted(first.items()))} |
| Actual OPEN → CLOSED / CLOSED → OPEN transitions | {sum(r['requested_intent']=='CLOSED' for r in transitions)} / {sum(r['requested_intent']=='OPEN' for r in transitions)} |

Only replans `{suffix_description}`
first predict CLOSE in the discarded suffix. Their next replans do not
repeat CLOSE in the executed prefix. A threshold prediction is distinct from
an intent transition: several chunks predict CLOSE while the gripper is
already commanded CLOSED.

## Event table

Time is the action's nominal simulation time; discarded actions were never
executed at that scheduled slot. Width is the measured opening **before**
an executed action. TCP distance
uses the TCP position after that action and the object position at the
**start of its chunk**; it is a snapshot comparison, not a clearance or
grasp-feasibility measurement. Loaded contact counts are new left/right pad
events during that action, not cumulative totals. Discarded rows show a
hypothetical intent transition implied by the saved future prediction;
there is no measured state at that unexecuted index.

| Replan:action | Nominal sim time (s) | Value | Status | Previous → requested | Opening (m) | TCP to start object (m) | New pad events L/R |
|---|---:|---:|---|---|---:|---:|---:|
{chr(10).join(event_lines)}

## Grasp-attempt windows

| Replans | Chunks predicting CLOSE | Executed intent transitions | Object start-to-end displacement (m) | New loaded contacts L/R/bilateral |
|---|---|---:|---:|---:|
{chr(10).join(window_lines)}

The object displacement compares only window endpoints. It does not
attribute movement to the gripper. Contact counters are taken from the
original execution records; none of these three windows added a loaded
fingertip contact event. Earlier cumulative contact counts therefore cannot
be treated as contact during these attempted closures. The saved camera,
TCP, gripper, contact, and object-motion plots under the run's `analysis/`
directory provide context, while all numerical claims here were recomputed
from the original per-replan files.

## Replanning comparison and interpretation

Every adjacent observation is 32 physics steps apart, matching eight
executed actions. For nominal future-time comparison, this analysis pairs
the previous chunk's discarded indices 8–14 with the next chunk's indices
0–6. These are predictions made from **different observations** after robot
and object motion; they are not repeated measurements of an identical
physical state. Across {len(overlaps)} such pairs, mean absolute action
difference is `{np.mean([r['overlap_mae_from_prior'] for r in overlaps]):.4f}`
(median `{np.median([r['overlap_mae_from_prior'] for r in overlaps]):.4f}`).
`{sum(r['overlap_binary_disagreements_from_prior'] > 0 for r in overlaps)}`
pairs have a threshold disagreement, totaling
`{sum(r['overlap_binary_disagreements_from_prior'] for r in overlaps)}` of
`{sum(r['overlap_compared_slots'] for r in overlaps)}` aligned slots. Onset
comparisons at replans 12, 115, and 146 have nominal-overlap mean absolute
differences of `{summary[12]['overlap_mae_from_prior']:.3f}`,
`{summary[115]['overlap_mae_from_prior']:.3f}`, and
`{summary[146]['overlap_mae_from_prior']:.3f}` respectively.

1. **Truncation:** yes, it omits native CLOSE predictions in discarded
   suffixes. Only replans {suffix_replans}
   have CLOSE solely there; no executed CLOSE follows from those chunks.
2. **Frequency:** suffix-only CLOSE is occasional here: {len(suffix_only)} of
   {len(replans)} complete chunks. Other suffix CLOSE values mostly continue
   an already executed CLOSE request.
3. **Postponement:** no repeated suffix-only CLOSE is observed across
   successive replans. The two suffix-only predictions disappear below the
   threshold in the next replans, rather than moving into the executed prefix.
   The next-chunk maxima are `{summary[120]['max_gripper_value']:.3f}` after
   replan 119 and `{summary[149]['max_gripper_value']:.3f}` after replan 148.
4. **Stability:** CLOSE predictions change sharply at the three onset
   periods and at release; the nominally aligned suffix/prefix comparison
   above quantifies this. It does not isolate policy instability from
   changing observations.
5. **Three periods:** the first requests two brief closures (replans 12 and
   15); the second sustains CLOSED across replans 115–118; the third begins
   CLOSE at action 3 of replan 146 and continues in 147. Releases at 14,
   119, and 148 differ, with suffix-only CLOSE at 119 and 148. Thus the
   patterns are related but not identical.
6. **Future horizon test:** the archive supports a later *controlled*
   horizon experiment as a question, because two discarded predictions
   would have requested a new CLOSE. It does not show that executing them
   would improve a grasp: each was conditioned on an older observation,
   and neither remained CLOSE after replanning.

## Limitations and policy-server audit

The run contains no counterfactual execution of discarded actions and no
per-action object pose. Continuous gripper values do not measure confidence.
The original records do not establish a physically feasible grasp from one
image, TCP distance, or absence/presence of a single contact event. For the
later policy-server audit, check how the pinned server selects/denormalizes
the full horizon and whether the client ever changes action ordering or
truncation; keep any horizon comparison at the same scene, seed, scale,
timing, and observation protocol.

## Outputs and reproduction

`gripper_heatmap.png`, `first_close_index.png`, `maximum_gripper_value.png`,
`event_window_11_16.png`, `event_window_114_120.png`, and
`event_window_145_149.png` are derived figures. `per_replan_summary.csv`,
`native_gripper_actions.csv`, `threshold_events.csv`, and
`input_manifest.csv` are machine-readable audit data.

```bash
MPLCONFIGDIR=/tmp/mpl-velocity-gripper python3 \\
  tools/analysis/analyze_velocity_gripper_rollout.py \\
  --run {run.as_posix()} \\
  --output results/isaac_droid_velocity_gripper_{run.name}
```
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run = args.run.resolve()
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Refusing to overwrite nonempty output: {output}")
    provenance, episode, replans, partial, manifest = load_replans(run)
    summary, native, thresholds = extract_rows(replans)
    output.mkdir(parents=True, exist_ok=True)
    write_csv(output / "per_replan_summary.csv", summary, list(summary[0]))
    write_csv(output / "native_gripper_actions.csv", native, list(native[0]))
    write_csv(output / "threshold_events.csv", thresholds, list(native[0]))
    write_csv(output / "input_manifest.csv", manifest, ["path", "sha256"])
    plot_heatmap(replans, output / "gripper_heatmap.png")
    plot_first_close(summary, output / "first_close_index.png")
    plot_maximum(summary, output / "maximum_gripper_value.png")
    for start, end in WINDOWS:
        plot_window(replans, native, start, end,
                    output / f"event_window_{start}_{end}.png")
    (output / "report.md").write_text(
        markdown_report(run, provenance, episode, replans, summary,
                        native, partial) + "\n"
    )
    print(f"Analyzed {len(replans)} complete replans into {output}")


if __name__ == "__main__":
    main()
