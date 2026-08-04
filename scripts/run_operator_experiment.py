#!/usr/bin/env python3
"""Run or resume a manifest-driven human operator experiment."""

from __future__ import annotations

import dataclasses
import datetime
import json
import logging
from pathlib import Path
import subprocess
import sys
from typing import Any

import tyro

from saps.evaluation.experiment_session import build_schedule
from saps.evaluation.experiment_session import load_manifest
from saps.evaluation.experiment_session import manifest_sha256
from saps.evaluation.experiment_session import validate_summary
from saps.evaluation.experiment_session import write_json_atomic


@dataclasses.dataclass
class Args:
    manifest_path: str
    repository_commit: str
    output_dir: str = "outputs/operator_experiment"
    dry_run: bool = False
    continue_on_invalid_attempt: bool = False
    redo_episode_ids: str = ""


def _utc_now() -> str:
    """Return one stable UTC timestamp."""

    return datetime.datetime.now(
        datetime.timezone.utc
    ).isoformat()


def _load_task_config(path: Path) -> dict[str, Any]:
    """Load and minimally validate the perturbation configuration."""

    with path.open("r", encoding="utf-8") as file:
        config = json.load(file)

    condition_ids = {
        str(offset["id"])
        for offset in config["offsets"]
    }
    config["condition_ids"] = condition_ids
    return config


def _initialize_experiment(
    *,
    manifest_path: Path,
    output_root: Path,
) -> tuple[Any, dict[str, Any]]:
    """Freeze the manifest and create or load its schedule."""

    manifest = load_manifest(manifest_path)
    identity = manifest_sha256(manifest)
    frozen_manifest_path = output_root / "manifest.json"
    schedule_path = output_root / "schedule.json"

    if frozen_manifest_path.exists():
        frozen = load_manifest(frozen_manifest_path)

        if manifest_sha256(frozen) != identity:
            raise ValueError(
                "The supplied manifest differs from the immutable "
                "manifest already stored in the output directory."
            )
    else:
        write_json_atomic(
            frozen_manifest_path,
            manifest.as_dict(),
        )

    task_config = _load_task_config(Path(manifest.config_path))
    unknown = set(manifest.conditions).difference(
        task_config["condition_ids"]
    )

    if unknown:
        raise ValueError(
            f"Manifest contains unknown conditions: {sorted(unknown)}"
        )

    if schedule_path.exists():
        with schedule_path.open("r", encoding="utf-8") as file:
            schedule = json.load(file)

        if schedule.get("manifest_sha256") != identity:
            raise ValueError(
                "Stored schedule does not belong to this manifest."
            )
    else:
        schedule = build_schedule(
            manifest=manifest,
            task_id=int(task_config["task_id"]),
            output_root=output_root,
        )
        write_json_atomic(schedule_path, schedule)

    return manifest, schedule


def _episode_command(
    *,
    manifest: Any,
    episode: dict[str, Any],
    attempt_root: Path,
) -> list[str]:
    """Build the existing single-episode command for one schedule row."""

    common = [
        "--config-path",
        manifest.config_path,
        "--condition-id",
        str(episode["condition_id"]),
        "--trial-index",
        str(episode["trial_index"]),
        "--initial-state-index",
        str(episode["initial_state_index"]),
        "--environment-seed",
        str(manifest.environment_seed),
        "--policy-base-seed",
        str(manifest.policy_base_seed),
        "--max-steps",
        str(manifest.operator_max_steps),
        "--control-frequency-hz",
        str(manifest.control_frequency_hz),
        "--fine-translation-gain",
        str(manifest.fine_translation_gain),
        "--fine-rotation-gain",
        str(manifest.fine_rotation_gain),
        "--translation-gain",
        str(manifest.normal_translation_gain),
        "--rotation-gain",
        str(manifest.normal_rotation_gain),
        "--fast-translation-gain",
        str(manifest.fast_translation_gain),
        "--fast-rotation-gain",
        str(manifest.fast_rotation_gain),
        "--default-speed-mode",
        manifest.default_speed_mode,
        "--output-dir",
        str(attempt_root),
    ]
    mode = str(episode["mode"])

    if mode == "teleoperation":
        script = "scripts/run_teleoperation_episode.py"
        return [sys.executable, script, *common]

    script = "scripts/run_shared_autonomy_episode.py"
    return [
        sys.executable,
        script,
        "--arbitration-mode",
        mode,
        "--fixed-autonomy-weight",
        str(manifest.fixed_autonomy_weight),
        "--cosine-gain",
        str(manifest.cosine_gain),
        "--replan-steps",
        "5",
        *common,
    ]


def _append_event(path: Path, event: dict[str, Any]) -> None:
    """Append one durable session event."""

    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("a", encoding="utf-8") as file:
        file.write(json.dumps(event) + "\n")
        file.flush()


def _find_summary(attempt_root: Path) -> Path:
    """Return the only summary produced by an attempt."""

    summaries = list(attempt_root.rglob("summary.json"))

    if len(summaries) != 1:
        raise ValueError(
            "Expected exactly one summary.json below "
            f"{attempt_root}, found {len(summaries)}."
        )

    return summaries[0]


def _write_session_summary(
    *,
    output_root: Path,
    schedule: dict[str, Any],
) -> None:
    """Write compact progress counts without changing the schedule."""

    episodes = schedule["episodes"]
    counts: dict[str, int] = {}

    for episode in episodes:
        status = str(episode["status"])
        counts[status] = counts.get(status, 0) + 1

    write_json_atomic(
        output_root / "session_summary.json",
        {
            "experiment_id": schedule["experiment_id"],
            "manifest_sha256": schedule["manifest_sha256"],
            "total_episodes": len(episodes),
            "status_counts": counts,
            "complete": counts.get("completed", 0) == len(episodes),
            "updated_at": _utc_now(),
        },
    )


def main(args: Args) -> None:
    output_root = Path(args.output_dir)
    output_root.mkdir(parents=True, exist_ok=True)
    manifest, schedule = _initialize_experiment(
        manifest_path=Path(args.manifest_path),
        output_root=output_root,
    )

    repository_commit = args.repository_commit.strip()

    if len(repository_commit) != 40 or any(
        character not in "0123456789abcdef"
        for character in repository_commit.lower()
    ):
        raise ValueError(
            "repository_commit must be a full 40-character Git hash."
        )

    provenance_path = output_root / "repository_provenance.json"
    provenance = {
        "repository_commit": repository_commit,
        "manifest_sha256": manifest_sha256(manifest),
    }

    if provenance_path.exists():
        with provenance_path.open("r", encoding="utf-8") as file:
            stored_provenance = json.load(file)

        if stored_provenance != provenance:
            raise ValueError(
                "The session output belongs to a different repository "
                "commit or manifest. Use its original checkout or start "
                "a new output directory."
            )
    else:
        write_json_atomic(provenance_path, provenance)

    schedule_path = output_root / "schedule.json"
    events_path = output_root / "session_events.jsonl"
    redo_ids = {
        value.strip()
        for value in args.redo_episode_ids.split(",")
        if value.strip()
    }
    known_ids = {
        str(episode["episode_id"])
        for episode in schedule["episodes"]
    }
    unknown_redo_ids = redo_ids.difference(known_ids)

    if unknown_redo_ids:
        raise ValueError(
            "Unknown redo episode IDs: "
            f"{sorted(unknown_redo_ids)}."
        )

    for episode in schedule["episodes"]:
        redo_requested = str(episode["episode_id"]) in redo_ids

        if episode["status"] == "completed" and not redo_requested:
            continue

        print()
        print(
            f"Episode {episode['order_index'] + 1}/"
            f"{len(schedule['episodes'])}: "
            f"mode={episode['mode']} "
            f"condition={episode['condition_id']} "
            f"trial={episode['trial_index']}"
        )
        print(
            "Matched autonomous policy seed: "
            f"{episode['policy_episode_seed']}"
        )

        if not args.dry_run:
            response = input(
                "Press Enter to launch, or q to stop the session: "
            ).strip().lower()

            if response == "q":
                break

        attempt_number = int(episode["attempt_count"]) + 1
        previous_selected_valid = any(
            previous_attempt.get("valid")
            and previous_attempt.get("selected_for_analysis", True)
            for previous_attempt in episode["attempts"]
        )
        attempt_root = (
            Path(episode["output_directory"])
            / f"attempt_{attempt_number:03d}"
        )
        command = _episode_command(
            manifest=manifest,
            episode=episode,
            attempt_root=attempt_root,
        )

        if args.dry_run:
            print(" ".join(command))
            continue

        if attempt_root.exists():
            raise FileExistsError(
                f"Attempt directory already exists: {attempt_root}"
            )

        started_at = _utc_now()
        attempt = {
            "attempt_number": attempt_number,
            "started_at": started_at,
            "finished_at": None,
            "return_code": None,
            "output_root": str(attempt_root),
            "summary_path": None,
            "valid": False,
            "selected_for_analysis": False,
            "redo_requested": redo_requested,
            "error": None,
        }
        episode["status"] = "running"
        episode["attempt_count"] = attempt_number
        episode["attempts"].append(attempt)
        write_json_atomic(schedule_path, schedule)
        _append_event(
            events_path,
            {
                "event": "attempt_started",
                "episode_id": episode["episode_id"],
                "attempt_number": attempt_number,
                "timestamp": started_at,
            },
        )

        result = subprocess.run(command, check=False)
        attempt["return_code"] = result.returncode
        attempt["finished_at"] = _utc_now()

        try:
            if result.returncode != 0:
                raise RuntimeError(
                    f"Episode process exited with {result.returncode}."
                )

            summary_path = _find_summary(attempt_root)
            summary = validate_summary(
                summary_path=summary_path,
                episode=episode,
            )
            attempt["summary_path"] = str(summary_path)
            attempt["valid"] = True
            attempt["selected_for_analysis"] = True

            for previous_attempt in episode["attempts"][:-1]:
                if previous_attempt.get("valid"):
                    previous_attempt["selected_for_analysis"] = False

            episode["status"] = "completed"
            episode["termination_reason"] = summary[
                "termination_reason"
            ]
            episode["success"] = bool(summary["success"])
        except (OSError, ValueError, RuntimeError) as error:
            attempt["error"] = str(error)
            episode["status"] = (
                "completed"
                if redo_requested and previous_selected_valid
                else "invalid"
            )
            logging.error(
                "Invalid attempt for %s: %s",
                episode["episode_id"],
                error,
            )

        write_json_atomic(schedule_path, schedule)
        _write_session_summary(
            output_root=output_root,
            schedule=schedule,
        )
        _append_event(
            events_path,
            {
                "event": "attempt_finished",
                "episode_id": episode["episode_id"],
                "attempt_number": attempt_number,
                "timestamp": attempt["finished_at"],
                "valid": attempt["valid"],
                "termination_reason": episode["termination_reason"],
                "success": episode["success"],
                "error": attempt["error"],
            },
        )

        if not attempt["valid"] and not args.continue_on_invalid_attempt:
            raise RuntimeError(
                "Session stopped after an invalid attempt. "
                "Inspect schedule.json and session_events.jsonl."
            )

    _write_session_summary(
        output_root=output_root,
        schedule=schedule,
    )


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )
    main(tyro.cli(Args))
