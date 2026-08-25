"""Fixed validation rules for the excluded Gate-2 operator pilot."""

from __future__ import annotations

from collections import Counter
from pathlib import Path
from typing import Any

from saps.evaluation.experiment_session import build_schedule
from saps.evaluation.experiment_session import ExperimentManifest
from saps.evaluation.experiment_session import json_file_identity
from saps.human_input.spacemouse_profile import load_spacemouse_profile
from saps.human_input.spacemouse_profile import spacemouse_profile_identity
from saps.policies.seeding import make_policy_episode_seed


GATE2_EXPERIMENT_ID = "saps_libero_gate2_operator_pilot_v1"
GATE2_MANIFEST_PATH = "configs/gate2_operator_pilot_manifest.json"
GATE2_OUTPUT_ROOT = "outputs/gate2_operator_pilot_v1"
GATE2_PROFILE_PATH = "configs/spacemouse_profile.json"
GATE2_CONFIG_PATH = "configs/libero_cream_cheese_offsets.json"
GATE2_CONFIG_SHA256 = (
    "43c88fe649362303ec599c6397155380d0de1ece84dbdcf614a2a952829447c5"
)
GATE2_PROFILE_SHA256 = (
    "cb69a38dfd23f2528a356e91b4aa2a14620803ce7385e0035a8c6e1da5fb84d0"
)
GATE2_CONDITIONS = ("nominal", "p02", "p06", "p09")
GATE2_MODES = ("teleoperation", "fixed_blend", "cosine_blend")
GATE2_TRIALS = tuple(range(5))
GATE2_TASK_ID = 1
GATE2_EXPECTED_MANIFEST: dict[str, Any] = {
    "schema_version": 3,
    "experiment_id": GATE2_EXPERIMENT_ID,
    "config_path": GATE2_CONFIG_PATH,
    "conditions": list(GATE2_CONDITIONS),
    "modes": list(GATE2_MODES),
    "trials_per_condition": 5,
    "initial_state_index": 0,
    "environment_seed": 7,
    "policy_base_seed": 20260724,
    "fixed_autonomy_weight": 0.5,
    "cosine_gain": 6.0,
    "control_frequency_hz": 20.0,
    "operator_max_steps": 280,
    "fine_translation_gain": 0.25,
    "fine_rotation_gain": 0.1,
    "normal_translation_gain": 0.5,
    "normal_rotation_gain": 0.2,
    "fast_translation_gain": 1.0,
    "fast_rotation_gain": 0.4,
    "default_speed_mode": "normal",
    "ordering_seed": 20260825,
}


def _same_path(actual: str, expected: str) -> bool:
    """Compare repository-relative protocol paths without following links."""

    return Path(actual).as_posix() == Path(expected).as_posix()


def validate_gate2_manifest(manifest: ExperimentManifest) -> None:
    """Require the exact versioned Gate-2 protocol manifest."""

    if manifest.as_dict() != GATE2_EXPECTED_MANIFEST:
        raise ValueError(
            "Gate-2 manifest does not match the fixed "
            f"{GATE2_EXPERIMENT_ID} protocol."
        )


def validate_gate2_schedule(
    schedule: dict[str, Any],
    *,
    manifest: ExperimentManifest,
) -> None:
    """Validate Gate-2 coverage, identities, and matched policy seeds."""

    episodes = schedule.get("episodes")
    if not isinstance(episodes, list) or len(episodes) != 60:
        raise ValueError("Gate-2 schedule must contain exactly 60 episodes.")

    episode_ids = [str(episode["episode_id"]) for episode in episodes]
    if len(set(episode_ids)) != len(episode_ids):
        raise ValueError("Gate-2 schedule contains duplicate episode IDs.")

    cells = [
        (
            str(episode["mode"]),
            str(episode["condition_id"]),
            int(episode["trial_index"]),
        )
        for episode in episodes
    ]
    expected_cells = {
        (mode, condition_id, trial_index)
        for mode in GATE2_MODES
        for condition_id in GATE2_CONDITIONS
        for trial_index in GATE2_TRIALS
    }
    if set(cells) != expected_cells or len(cells) != len(set(cells)):
        raise ValueError(
            "Gate-2 schedule must contain each mode-condition-trial "
            "cell exactly once."
        )

    mode_counts = Counter(mode for mode, _, _ in cells)
    condition_counts = Counter(condition for _, condition, _ in cells)
    pair_counts = Counter((mode, condition) for mode, condition, _ in cells)
    if set(mode_counts.values()) != {20}:
        raise ValueError("Gate-2 schedule must contain 20 episodes per mode.")
    if set(condition_counts.values()) != {15}:
        raise ValueError(
            "Gate-2 schedule must contain 15 episodes per condition."
        )
    if set(pair_counts.values()) != {5}:
        raise ValueError(
            "Gate-2 schedule must contain 5 episodes per mode-condition."
        )

    for condition_id in GATE2_CONDITIONS:
        for trial_index in GATE2_TRIALS:
            matched = [
                episode
                for episode in episodes
                if episode["condition_id"] == condition_id
                and episode["trial_index"] == trial_index
            ]
            expected_seed = make_policy_episode_seed(
                base_seed=manifest.policy_base_seed,
                condition_id=condition_id,
                trial_index=trial_index,
                task_id=GATE2_TASK_ID,
                initial_state_index=manifest.initial_state_index,
            )
            seeds = {
                int(episode["policy_episode_seed"])
                for episode in matched
            }
            if seeds != {expected_seed}:
                raise ValueError(
                    "Gate-2 policy seeds are not matched across modes."
                )


def validate_gate2_protocol(
    *,
    manifest: ExperimentManifest,
    input_source: str,
    spacemouse_profile_path: str,
    spacemouse_device_path: str,
    output_root: Path,
) -> dict[str, Any]:
    """Validate all fixed Gate-2 inputs without launching an episode."""

    validate_gate2_manifest(manifest)
    if input_source.strip().lower() != "spacemouse":
        raise ValueError("Gate-2 requires input_source='spacemouse'.")
    if not spacemouse_device_path.strip():
        raise ValueError("Gate-2 requires a runtime SpaceMouse device path.")
    if not Path(spacemouse_device_path).is_absolute():
        raise ValueError("Gate-2 SpaceMouse device path must be absolute.")
    if not _same_path(spacemouse_profile_path, GATE2_PROFILE_PATH):
        raise ValueError(
            "Gate-2 requires configs/spacemouse_profile.json."
        )
    if not _same_path(str(output_root), GATE2_OUTPUT_ROOT):
        raise ValueError(
            f"Gate-2 output root must be {GATE2_OUTPUT_ROOT}."
        )

    task_config = json_file_identity(Path(manifest.config_path))
    if (
        not _same_path(str(task_config["path"]), GATE2_CONFIG_PATH)
        or task_config["sha256"] != GATE2_CONFIG_SHA256
    ):
        raise ValueError(
            "Gate-2 perturbation configuration identity does not match "
            "the fixed protocol."
        )
    if int(task_config["contents"].get("task_id", -1)) != GATE2_TASK_ID:
        raise ValueError("Gate-2 requires LIBERO task_id 1.")

    profile = load_spacemouse_profile(Path(spacemouse_profile_path))
    profile_identity = spacemouse_profile_identity(
        profile,
        path=spacemouse_profile_path,
    )
    if profile_identity["sha256"] != GATE2_PROFILE_SHA256:
        raise ValueError(
            "Gate-2 SpaceMouse profile hash does not match the "
            "physically validated calibration."
        )

    schedule = build_schedule(
        manifest=manifest,
        task_id=GATE2_TASK_ID,
        output_root=output_root,
    )
    regenerated = build_schedule(
        manifest=manifest,
        task_id=GATE2_TASK_ID,
        output_root=output_root,
    )
    if schedule != regenerated:
        raise ValueError(
            "Gate-2 schedule did not regenerate deterministically."
        )
    validate_gate2_schedule(schedule, manifest=manifest)

    return {
        "manifest": manifest,
        "task_config": task_config,
        "spacemouse_profile": {
            **profile_identity,
            "contents": profile.as_dict(),
        },
        "spacemouse_device_path": spacemouse_device_path,
        "schedule": schedule,
    }
