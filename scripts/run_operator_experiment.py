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
from saps.evaluation.experiment_session import json_file_identity
from saps.evaluation.experiment_session import load_manifest
from saps.evaluation.experiment_session import manifest_sha256
from saps.evaluation.experiment_session import validate_schedule_identity
from saps.evaluation.experiment_session import validate_summary
from saps.evaluation.experiment_session import write_json_atomic
from saps.evaluation.gate2_protocol import GATE2_EXPERIMENT_ID
from saps.evaluation.gate2_protocol import build_gate2_schedule
from saps.evaluation.gate2_protocol import validate_gate2_protocol
from saps.human_input.spacemouse import parse_axis_mapping
from saps.human_input.spacemouse import parse_axis_maxima
from saps.human_input.spacemouse import parse_axis_signs
from saps.human_input.spacemouse import SpaceMouseConfig
from saps.human_input.spacemouse_profile import load_spacemouse_profile
from saps.human_input.spacemouse_profile import spacemouse_profile_identity


@dataclasses.dataclass
class Args:
    manifest_path: str
    repository_commit: str
    output_dir: str = "outputs/operator_experiment"
    required_protocol_id: str = ""
    dry_run: bool = False
    continue_on_invalid_attempt: bool = False
    redo_episode_ids: str = ""
    input_source: str = "keyboard"
    spacemouse_device_path: str = ""
    spacemouse_profile_path: str = ""
    spacemouse_deadzone: float = 0.08
    spacemouse_axis_mapping: str = (
        "ABS_X,ABS_Y,ABS_Z,ABS_RX,ABS_RY,ABS_RZ"
    )
    spacemouse_axis_signs: str = "1,1,1,1,1,1"
    spacemouse_axis_maxima: str = "350,350,350,350,350,350"
    spacemouse_stale_input_timeout_seconds: float = 0.25
    spacemouse_open_button: int = 256
    spacemouse_close_button: int = 257


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
    required_protocol_id: str = "",
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

    schedule_builder = (
        build_gate2_schedule
        if required_protocol_id == GATE2_EXPERIMENT_ID
        else build_schedule
    )
    expected_schedule = schedule_builder(
        manifest=manifest,
        task_id=int(task_config["task_id"]),
        output_root=output_root,
    )
    if schedule_path.exists():
        with schedule_path.open("r", encoding="utf-8") as file:
            schedule = json.load(file)

        validate_schedule_identity(
            stored=schedule,
            expected=expected_schedule,
        )
    else:
        schedule = expected_schedule
        write_json_atomic(schedule_path, schedule)

    return manifest, schedule


def _episode_command(
    *,
    manifest: Any,
    episode: dict[str, Any],
    attempt_root: Path,
    args: Args,
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
        "--input-source",
        args.input_source,
        "--output-dir",
        str(attempt_root),
    ]
    if args.input_source.strip().lower() == "keyboard":
        common.extend(
            [
                "--spacemouse-deadzone",
                str(args.spacemouse_deadzone),
                "--spacemouse-axis-mapping",
                args.spacemouse_axis_mapping,
                "--spacemouse-axis-signs",
                args.spacemouse_axis_signs,
                "--spacemouse-axis-maxima",
                args.spacemouse_axis_maxima,
                "--spacemouse-stale-input-timeout-seconds",
                str(args.spacemouse_stale_input_timeout_seconds),
                "--spacemouse-open-button",
                str(args.spacemouse_open_button),
                "--spacemouse-close-button",
                str(args.spacemouse_close_button),
            ]
        )
    if args.spacemouse_device_path:
        common.extend(
            [
                "--spacemouse-device-path",
                args.spacemouse_device_path,
            ]
        )
    if args.spacemouse_profile_path:
        common.extend(
            [
                "--spacemouse-profile-path",
                args.spacemouse_profile_path,
            ]
        )
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


def _human_input_configuration(
    *,
    args: Args,
    manifest: Any,
) -> dict[str, Any]:
    """Validate and serialize session-level input provenance."""

    input_source = args.input_source.strip().lower()
    if input_source not in {"keyboard", "spacemouse"}:
        raise ValueError(
            "input_source must be 'keyboard' or 'spacemouse'."
        )

    if input_source == "spacemouse":
        if not args.spacemouse_profile_path:
            raise ValueError(
                "SpaceMouse operator sessions require an explicit "
                "spacemouse_profile_path."
            )
        profile = load_spacemouse_profile(
            Path(args.spacemouse_profile_path)
        )
        profile_identity = spacemouse_profile_identity(
            profile,
            path=args.spacemouse_profile_path,
        )
        return {
            "input_source": input_source,
            "spacemouse_device_path": args.spacemouse_device_path,
            "spacemouse_profile": {
                **profile_identity,
                "contents": profile.as_dict(),
            },
        }

    if args.spacemouse_profile_path:
        raise ValueError(
            "spacemouse_profile_path requires "
            "input_source='spacemouse'."
        )

    spacemouse = SpaceMouseConfig(
        device_path=args.spacemouse_device_path,
        translation_gain=manifest.normal_translation_gain,
        rotation_gain=manifest.normal_rotation_gain,
        deadzone=args.spacemouse_deadzone,
        axis_mapping=parse_axis_mapping(
            args.spacemouse_axis_mapping
        ),
        axis_signs=parse_axis_signs(args.spacemouse_axis_signs),
        axis_maxima=parse_axis_maxima(args.spacemouse_axis_maxima),
        stale_input_timeout_seconds=(
            args.spacemouse_stale_input_timeout_seconds
        ),
        open_button=args.spacemouse_open_button,
        close_button=args.spacemouse_close_button,
    )
    return {
        "input_source": input_source,
        "spacemouse": json.loads(
            json.dumps(dataclasses.asdict(spacemouse))
        ),
    }


def _validate_child_human_input(
    *,
    summary: dict[str, Any],
    input_configuration: dict[str, Any],
) -> None:
    """Verify child profile identity against frozen session provenance."""

    if input_configuration["input_source"] != "spacemouse":
        return

    profile = input_configuration["spacemouse_profile"]
    expected = {
        "path": profile["path"],
        "schema_version": profile["schema_version"],
        "sha256": profile["sha256"],
    }
    actual = summary.get("spacemouse_profile")
    if actual != expected:
        raise ValueError(
            "Episode SpaceMouse profile does not match the frozen "
            "session profile."
        )


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
    preview_manifest = load_manifest(Path(args.manifest_path))
    required_protocol_id = args.required_protocol_id.strip()
    if (
        preview_manifest.experiment_id == GATE2_EXPERIMENT_ID
        and required_protocol_id != GATE2_EXPERIMENT_ID
    ):
        raise ValueError(
            "The Gate-2 manifest requires its explicit protocol guard. "
            "Use make gate2-session."
        )
    if required_protocol_id:
        if required_protocol_id != GATE2_EXPERIMENT_ID:
            raise ValueError(
                f"Unsupported required_protocol_id {required_protocol_id!r}."
            )
        validate_gate2_protocol(
            manifest=preview_manifest,
            input_source=args.input_source,
            spacemouse_profile_path=args.spacemouse_profile_path,
            spacemouse_device_path=args.spacemouse_device_path,
            output_root=Path(args.output_dir),
        )
    input_configuration = _human_input_configuration(
        args=args,
        manifest=preview_manifest,
    )
    output_root = Path(args.output_dir)
    output_root.mkdir(parents=True, exist_ok=True)
    manifest, schedule = _initialize_experiment(
        manifest_path=Path(args.manifest_path),
        output_root=output_root,
        required_protocol_id=required_protocol_id,
    )
    perturbation_configuration = json_file_identity(
        Path(manifest.config_path)
    )
    perturbation_configuration_path = (
        output_root / "perturbation_config.json"
    )
    if perturbation_configuration_path.exists():
        with perturbation_configuration_path.open(
            "r",
            encoding="utf-8",
        ) as file:
            stored_perturbation_configuration = json.load(file)
        if stored_perturbation_configuration != perturbation_configuration:
            raise ValueError(
                "The session output belongs to a different perturbation "
                "configuration. Use a new output directory."
            )
    else:
        write_json_atomic(
            perturbation_configuration_path,
            perturbation_configuration,
        )

    session_protocol = {
        "required_protocol_id": required_protocol_id or None,
    }
    session_protocol_path = output_root / "session_protocol.json"
    if session_protocol_path.exists():
        with session_protocol_path.open("r", encoding="utf-8") as file:
            stored_session_protocol = json.load(file)
        if stored_session_protocol != session_protocol:
            raise ValueError(
                "The session output belongs to a different required "
                "protocol. Use a new output directory."
            )
    else:
        write_json_atomic(session_protocol_path, session_protocol)

    input_configuration_path = (
        output_root / "human_input.json"
    )
    if input_configuration_path.exists():
        with input_configuration_path.open(
            "r",
            encoding="utf-8",
        ) as file:
            stored_input_configuration = json.load(file)
        if stored_input_configuration != input_configuration:
            raise ValueError(
                "The session output belongs to a different human-input "
                "configuration. Use a new output directory."
            )
    else:
        write_json_atomic(
            input_configuration_path,
            input_configuration,
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
            args=args,
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
            _validate_child_human_input(
                summary=summary,
                input_configuration=input_configuration,
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
