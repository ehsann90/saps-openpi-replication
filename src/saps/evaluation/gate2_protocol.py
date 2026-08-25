"""Fixed validation rules for the excluded Gate-2 operator pilot."""

from __future__ import annotations

from collections import Counter
import itertools
import json
from pathlib import Path
import random
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
GATE2_ORDERING_METHOD = "gate2_constrained_counterbalance_v1"
GATE2_UNITS_PER_TRIAL = len(GATE2_CONDITIONS) * len(GATE2_MODES)
GATE2_MODE_PAIRS = tuple(itertools.combinations(GATE2_MODES, 2))
GATE2_MAX_ORDERING_ATTEMPTS = 10000
GATE2_OPERATOR_MAX_STEPS = 280
GATE2_VALID_TERMINATION_REASONS = ("success", "timeout")
GATE2_REDO_REQUIRED_TERMINATION_REASONS = (
    "operator_abort",
    "operator_disconnected",
    "operator_disarmed",
    "input_device_disconnected",
    "environment_terminated",
    "initialization_error",
    "runtime_error",
)
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
    "operator_max_steps": GATE2_OPERATOR_MAX_STEPS,
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


def _read_gate2_jsonl(path: Path) -> list[dict[str, Any]]:
    """Read one Gate-2 audit stream without accepting malformed rows."""

    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(
                    f"Gate-2 audit row {path}:{line_number} "
                    "must be a JSON object."
                )
            records.append(value)
    return records


def validate_gate2_attempt_completion(
    *,
    summary_path: Path,
    summary: dict[str, Any],
) -> None:
    """Require a complete outcome and uninterrupted operator authority."""

    termination_reason = str(summary.get("termination_reason", ""))
    if termination_reason not in GATE2_VALID_TERMINATION_REASONS:
        raise ValueError(
            "Gate-2 attempt requires redo after termination reason "
            f"{termination_reason!r}; only success and timeout are valid."
        )

    success = summary.get("success")
    control_steps = int(summary.get("control_steps", -1))
    if termination_reason == "success":
        if success is not True:
            raise ValueError(
                "Gate-2 success termination requires success=true."
            )
        if not 1 <= control_steps <= GATE2_OPERATOR_MAX_STEPS:
            raise ValueError(
                "Gate-2 success must occur within the "
                f"{GATE2_OPERATOR_MAX_STEPS}-step horizon."
            )
    elif (
        success is not False
        or control_steps != GATE2_OPERATOR_MAX_STEPS
    ):
        raise ValueError(
            "Gate-2 timeout requires success=false and all "
            f"{GATE2_OPERATOR_MAX_STEPS} steps."
        )

    steps_path = summary_path.with_name("steps.jsonl")
    steps = _read_gate2_jsonl(steps_path)
    if len(steps) != control_steps:
        raise ValueError(
            "Gate-2 logged step count does not match summary control_steps."
        )

    records = [("step", index, record) for index, record in enumerate(steps)]
    if summary.get("arbitration_mode") != "teleoperation":
        waits_path = summary_path.with_name("scheduler_waits.jsonl")
        if not waits_path.is_file():
            raise ValueError(
                "Gate-2 shared-autonomy attempt is missing "
                "scheduler_waits.jsonl."
            )
        records.extend(
            ("wait", index, record)
            for index, record in enumerate(_read_gate2_jsonl(waits_path))
        )

    for stream, index, record in records:
        human_input = record.get("human_input")
        if not isinstance(human_input, dict):
            raise ValueError(
                f"Gate-2 {stream} row {index} is missing human_input."
            )
        if human_input.get("input_source") != "spacemouse":
            raise ValueError(
                f"Gate-2 {stream} row {index} is not SpaceMouse input."
            )
        if human_input.get("connected") is not True:
            raise ValueError(
                f"Gate-2 {stream} row {index} records operator_disconnected."
            )
        if human_input.get("physical_device_connected") is not True:
            raise ValueError(
                f"Gate-2 {stream} row {index} records "
                "input_device_disconnected."
            )
        if human_input.get("armed") is not True:
            raise ValueError(
                f"Gate-2 {stream} row {index} records operator_disarmed."
            )


def _interleave_trial_round(
    *,
    mode_orders: dict[str, tuple[str, ...]],
    previous_condition: str | None,
    previous_mode: str | None,
    previous_mode_run_length: int,
    randomizer: random.Random,
) -> list[tuple[str, str]] | None:
    """Interleave four condition queues under Gate-2 run constraints."""

    positions = {condition_id: 0 for condition_id in GATE2_CONDITIONS}
    selected: list[tuple[str, str]] = []

    def search(
        last_condition: str | None,
        last_mode: str | None,
        mode_run_length: int,
    ) -> bool:
        if len(selected) == GATE2_UNITS_PER_TRIAL:
            return True

        candidates: list[tuple[str, str]] = []
        for condition_id in GATE2_CONDITIONS:
            position = positions[condition_id]
            if position >= len(GATE2_MODES):
                continue
            if condition_id == last_condition:
                continue
            mode = mode_orders[condition_id][position]
            if mode == last_mode and mode_run_length >= 2:
                continue
            candidates.append((condition_id, mode))

        randomizer.shuffle(candidates)
        for condition_id, mode in candidates:
            positions[condition_id] += 1
            selected.append((mode, condition_id))
            next_run_length = (
                mode_run_length + 1 if mode == last_mode else 1
            )
            if search(condition_id, mode, next_run_length):
                return True
            selected.pop()
            positions[condition_id] -= 1

        return False

    if search(
        previous_condition,
        previous_mode,
        previous_mode_run_length,
    ):
        return selected
    return None


def _gate2_order_cells(
    manifest: ExperimentManifest,
) -> list[tuple[int, str, str]]:
    """Search deterministically for one constrained five-round ordering."""

    randomizer = random.Random(manifest.ordering_seed)
    mode_permutations = list(itertools.permutations(GATE2_MODES))

    for _ in range(GATE2_MAX_ORDERING_ATTEMPTS):
        orders_by_condition: dict[str, list[tuple[str, ...]]] = {}
        for condition_id in GATE2_CONDITIONS:
            candidates = list(mode_permutations)
            randomizer.shuffle(candidates)
            orders_by_condition[condition_id] = candidates[:5]

        cells: list[tuple[int, str, str]] = []
        previous_condition: str | None = None
        previous_mode: str | None = None
        previous_mode_run_length = 0
        feasible = True

        for trial_index in GATE2_TRIALS:
            mode_orders = {
                condition_id: orders_by_condition[condition_id][trial_index]
                for condition_id in GATE2_CONDITIONS
            }
            trial_units = _interleave_trial_round(
                mode_orders=mode_orders,
                previous_condition=previous_condition,
                previous_mode=previous_mode,
                previous_mode_run_length=previous_mode_run_length,
                randomizer=randomizer,
            )
            if trial_units is None:
                feasible = False
                break

            for mode, condition_id in trial_units:
                cells.append((trial_index, mode, condition_id))
                if mode == previous_mode:
                    previous_mode_run_length += 1
                else:
                    previous_mode = mode
                    previous_mode_run_length = 1
                previous_condition = condition_id

        if feasible:
            return cells

    raise ValueError(
        "Could not construct a Gate-2 schedule satisfying all ordering "
        f"constraints after {GATE2_MAX_ORDERING_ATTEMPTS} attempts."
    )


def build_gate2_schedule(
    *,
    manifest: ExperimentManifest,
    task_id: int,
    output_root: Path,
) -> dict[str, Any]:
    """Build the Gate-2-specific constrained deterministic schedule."""

    if (
        manifest.conditions != GATE2_CONDITIONS
        or manifest.modes != GATE2_MODES
        or manifest.trials_per_condition != len(GATE2_TRIALS)
    ):
        raise ValueError(
            "Gate-2 constrained ordering requires the fixed cells and trials."
        )

    schedule = build_schedule(
        manifest=manifest,
        task_id=task_id,
        output_root=output_root,
    )
    episode_by_cell = {
        (
            int(episode["trial_index"]),
            str(episode["mode"]),
            str(episode["condition_id"]),
        ): episode
        for episode in schedule["episodes"]
    }
    ordered_episodes = []
    for order_index, cell in enumerate(_gate2_order_cells(manifest)):
        episode = episode_by_cell[cell]
        episode["order_index"] = order_index
        ordered_episodes.append(episode)

    schedule["episodes"] = ordered_episodes
    schedule["ordering_method"] = GATE2_ORDERING_METHOD
    validate_gate2_schedule(schedule, manifest=manifest)
    return schedule


def _maximum_run(values: list[str]) -> int:
    maximum = 0
    current = 0
    previous: str | None = None
    for value in values:
        current = current + 1 if value == previous else 1
        maximum = max(maximum, current)
        previous = value
    return maximum


def gate2_ordering_metrics(schedule: dict[str, Any]) -> dict[str, Any]:
    """Summarize all Gate-2 ordering constraints for preflight."""

    episodes = schedule["episodes"]
    positions = {
        (
            str(episode["condition_id"]),
            int(episode["trial_index"]),
            str(episode["mode"]),
        ): index
        for index, episode in enumerate(episodes)
    }
    precedence: dict[str, dict[str, int]] = {}
    for condition_id in GATE2_CONDITIONS:
        condition_counts: dict[str, int] = {}
        for first_mode, second_mode in GATE2_MODE_PAIRS:
            count = sum(
                positions[(condition_id, trial_index, first_mode)]
                < positions[(condition_id, trial_index, second_mode)]
                for trial_index in GATE2_TRIALS
            )
            condition_counts[
                f"{first_mode}_before_{second_mode}"
            ] = count
        precedence[condition_id] = condition_counts

    separations = []
    for condition_id in GATE2_CONDITIONS:
        for trial_index in GATE2_TRIALS:
            identity_positions = sorted(
                positions[(condition_id, trial_index, mode)]
                for mode in GATE2_MODES
            )
            separations.extend(
                second - first - 1
                for first, second in zip(
                    identity_positions,
                    identity_positions[1:],
                )
            )

    return {
        "ordering_method": schedule.get("ordering_method"),
        "maximum_same_mode_run_length": _maximum_run(
            [str(episode["mode"]) for episode in episodes]
        ),
        "maximum_same_condition_run_length": _maximum_run(
            [str(episode["condition_id"]) for episode in episodes]
        ),
        "minimum_same_identity_intervening_episodes": min(separations),
        "pairwise_mode_precedence": precedence,
    }


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
    if schedule.get("ordering_method") != GATE2_ORDERING_METHOD:
        raise ValueError(
            "Gate-2 schedule ordering_method does not match the fixed "
            "counterbalancing protocol."
        )

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

    for trial_index in GATE2_TRIALS:
        start = trial_index * GATE2_UNITS_PER_TRIAL
        trial_episodes = episodes[
            start:start + GATE2_UNITS_PER_TRIAL
        ]
        trial_units = {
            (episode["mode"], episode["condition_id"])
            for episode in trial_episodes
        }
        expected_units = {
            (mode, condition_id)
            for mode in GATE2_MODES
            for condition_id in GATE2_CONDITIONS
        }
        if (
            len(trial_episodes) != GATE2_UNITS_PER_TRIAL
            or trial_units != expected_units
            or {
                int(episode["trial_index"])
                for episode in trial_episodes
            } != {trial_index}
        ):
            raise ValueError(
                f"Gate-2 trial round {trial_index} must contain all 12 "
                "mode-condition units exactly once."
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

    metrics = gate2_ordering_metrics(schedule)
    if metrics["maximum_same_condition_run_length"] > 1:
        raise ValueError(
            "Gate-2 schedule contains consecutive identical conditions."
        )
    if metrics["maximum_same_mode_run_length"] > 2:
        raise ValueError(
            "Gate-2 schedule contains a same-mode run longer than two."
        )
    if metrics["minimum_same_identity_intervening_episodes"] < 1:
        raise ValueError(
            "Gate-2 matched condition-trial modes must have at least one "
            "intervening episode."
        )
    for condition_id, precedence in metrics[
        "pairwise_mode_precedence"
    ].items():
        if any(count not in {2, 3} for count in precedence.values()):
            raise ValueError(
                "Gate-2 pairwise mode precedence must be 2/3 balanced "
                f"for condition {condition_id}."
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

    schedule = build_gate2_schedule(
        manifest=manifest,
        task_id=GATE2_TASK_ID,
        output_root=output_root,
    )
    regenerated = build_gate2_schedule(
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
