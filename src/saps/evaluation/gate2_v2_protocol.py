"""Frozen matched-design rules for the excluded Gate-2 v2 pilot."""

from __future__ import annotations

from collections import Counter
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
from saps.policies.seeding import SEED_PROTOCOL


GATE2_V2_SHARED_EXPERIMENT_ID = (
    "saps_libero_gate2_shared_autonomy_pilot_v2"
)
GATE2_V2_AUTONOMOUS_EXPERIMENT_ID = (
    "saps_libero_gate2_autonomous_pilot_v2"
)
GATE2_V2_MANIFEST_PATH = (
    "configs/gate2_shared_autonomy_pilot_manifest.json"
)
GATE2_V2_AUTONOMOUS_PROTOCOL_PATH = (
    "configs/gate2_autonomous_pilot_protocol.json"
)
GATE2_V2_SHARED_OUTPUT_ROOT = "outputs/gate2_shared_autonomy_pilot_v2"
GATE2_V2_AUTONOMOUS_OUTPUT_ROOT = "outputs/gate2_autonomous_pilot_v2"
GATE2_V2_PROFILE_PATH = "configs/spacemouse_profile.json"
GATE2_V2_CONFIG_PATH = "configs/libero_cream_cheese_offsets.json"
GATE2_V2_CONFIG_SHA256 = (
    "43c88fe649362303ec599c6397155380d0de1ece84dbdcf614a2a952829447c5"
)
GATE2_V2_PROFILE_SHA256 = (
    "3bae6c547e2eec8d33c68a860d65eea4c0b1c39c7fb993dd2f033323b0994afc"
)
GATE2_V2_CONDITIONS = ("nominal", "p02", "p06", "p09")
GATE2_V2_SHARED_MODES = ("fixed_blend", "cosine_blend")
GATE2_V2_ALL_MODES = ("autonomous", *GATE2_V2_SHARED_MODES)
GATE2_V2_TRIALS = tuple(range(5))
GATE2_V2_TASK_ID = 1
GATE2_V2_ORDERING_METHOD = "gate2_v2_two_mode_counterbalance_v1"
GATE2_V2_UNITS_PER_TRIAL = 8
GATE2_V2_MAX_ORDERING_ATTEMPTS = 10000
GATE2_V2_MAX_STEPS = 280
GATE2_V2_CONTROL_FREQUENCY_HZ = 20.0
GATE2_V2_REPLAN_STEPS = 5
GATE2_V2_SETTLE_STEPS = 10

GATE2_V2_EXPECTED_MANIFEST: dict[str, Any] = {
    "schema_version": 4,
    "experiment_id": GATE2_V2_SHARED_EXPERIMENT_ID,
    "config_path": GATE2_V2_CONFIG_PATH,
    "conditions": list(GATE2_V2_CONDITIONS),
    "modes": list(GATE2_V2_SHARED_MODES),
    "trials_per_condition": 5,
    "initial_state_index": 0,
    "environment_seed": 7,
    "policy_base_seed": 20260724,
    "fixed_autonomy_weight": 0.5,
    "cosine_gain": 6.0,
    "control_frequency_hz": GATE2_V2_CONTROL_FREQUENCY_HZ,
    "operator_max_steps": GATE2_V2_MAX_STEPS,
    "keyboard_translation_gain": 0.5,
    "keyboard_rotation_gain": 0.2,
    "ordering_seed": 20260825,
}

GATE2_V2_EXPECTED_AUTONOMOUS_PROTOCOL: dict[str, Any] = {
    "schema_version": 1,
    "experiment_id": GATE2_V2_AUTONOMOUS_EXPERIMENT_ID,
    "config_path": GATE2_V2_CONFIG_PATH,
    "conditions": list(GATE2_V2_CONDITIONS),
    "trials": list(GATE2_V2_TRIALS),
    "task_suite_name": "libero_object",
    "task_id": GATE2_V2_TASK_ID,
    "initial_state_index": 0,
    "environment_seed": 7,
    "policy_base_seed": 20260724,
    "policy_config_name": "pi05_libero",
    "policy_checkpoint": "gs://openpi-assets/checkpoints/pi05_libero",
    "resolution": 256,
    "resize_size": 224,
    "replan_steps": GATE2_V2_REPLAN_STEPS,
    "settle_steps": GATE2_V2_SETTLE_STEPS,
    "max_steps": GATE2_V2_MAX_STEPS,
    "control_frequency_hz": GATE2_V2_CONTROL_FREQUENCY_HZ,
    "video_fps": 10,
    "output_root": GATE2_V2_AUTONOMOUS_OUTPUT_ROOT,
}


def _same_path(actual: str, expected: str) -> bool:
    return Path(actual).as_posix() == Path(expected).as_posix()


def load_gate2_v2_autonomous_protocol(path: Path) -> dict[str, Any]:
    """Load and require the exact autonomous protocol document."""

    with path.open("r", encoding="utf-8") as file:
        value = json.load(file)
    if value != GATE2_V2_EXPECTED_AUTONOMOUS_PROTOCOL:
        raise ValueError(
            "Gate-2 autonomous protocol does not match the frozen v2 design."
        )
    return value


def validate_gate2_v2_manifest(manifest: ExperimentManifest) -> None:
    """Require the exact shared-autonomy v2 manifest."""

    if manifest.as_dict() != GATE2_V2_EXPECTED_MANIFEST:
        raise ValueError(
            "Gate-2 shared manifest does not match the frozen v2 design."
        )


def _interleave_round(
    *,
    mode_orders: dict[str, tuple[str, str]],
    previous_condition: str | None,
    previous_mode: str | None,
    previous_mode_run: int,
    randomizer: random.Random,
) -> list[tuple[str, str]] | None:
    positions = {condition: 0 for condition in GATE2_V2_CONDITIONS}
    selected: list[tuple[str, str]] = []

    def search(
        last_condition: str | None,
        last_mode: str | None,
        same_mode_run: int,
    ) -> bool:
        if len(selected) == GATE2_V2_UNITS_PER_TRIAL:
            return True
        candidates: list[tuple[str, str]] = []
        for condition in GATE2_V2_CONDITIONS:
            position = positions[condition]
            if position >= 2 or condition == last_condition:
                continue
            mode = mode_orders[condition][position]
            if mode == last_mode and same_mode_run >= 2:
                continue
            candidates.append((mode, condition))
        randomizer.shuffle(candidates)
        for mode, condition in candidates:
            positions[condition] += 1
            selected.append((mode, condition))
            next_run = same_mode_run + 1 if mode == last_mode else 1
            if search(condition, mode, next_run):
                return True
            selected.pop()
            positions[condition] -= 1
        return False

    if search(previous_condition, previous_mode, previous_mode_run):
        return selected
    return None


def _ordered_cells(manifest: ExperimentManifest) -> list[tuple[int, str, str]]:
    """Generate five deterministic constrained two-mode rounds."""

    randomizer = random.Random(manifest.ordering_seed)
    for _ in range(GATE2_V2_MAX_ORDERING_ATTEMPTS):
        fixed_first_counts = {
            condition: 2 + randomizer.randrange(2)
            for condition in GATE2_V2_CONDITIONS
        }
        fixed_first_trials: dict[str, set[int]] = {}
        for condition in GATE2_V2_CONDITIONS:
            trials = list(GATE2_V2_TRIALS)
            randomizer.shuffle(trials)
            fixed_first_trials[condition] = set(
                trials[:fixed_first_counts[condition]]
            )

        cells: list[tuple[int, str, str]] = []
        previous_condition: str | None = None
        previous_mode: str | None = None
        previous_mode_run = 0
        feasible = True
        for trial in GATE2_V2_TRIALS:
            orders = {
                condition: (
                    GATE2_V2_SHARED_MODES
                    if trial in fixed_first_trials[condition]
                    else tuple(reversed(GATE2_V2_SHARED_MODES))
                )
                for condition in GATE2_V2_CONDITIONS
            }
            round_cells = _interleave_round(
                mode_orders=orders,
                previous_condition=previous_condition,
                previous_mode=previous_mode,
                previous_mode_run=previous_mode_run,
                randomizer=randomizer,
            )
            if round_cells is None:
                feasible = False
                break
            for mode, condition in round_cells:
                cells.append((trial, mode, condition))
                if mode == previous_mode:
                    previous_mode_run += 1
                else:
                    previous_mode = mode
                    previous_mode_run = 1
                previous_condition = condition
        if feasible:
            return cells
    raise ValueError("Could not construct the constrained Gate-2 v2 schedule.")


def build_gate2_v2_shared_schedule(
    *,
    manifest: ExperimentManifest,
    task_id: int,
    output_root: Path,
) -> dict[str, Any]:
    """Build the exact deterministic 40-row shared schedule."""

    validate_gate2_v2_manifest(manifest)
    if task_id != GATE2_V2_TASK_ID:
        raise ValueError("Gate-2 v2 requires LIBERO task_id 1.")
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
    ordered = []
    for order_index, cell in enumerate(_ordered_cells(manifest)):
        episode = episode_by_cell[cell]
        episode["order_index"] = order_index
        ordered.append(episode)
    schedule["episodes"] = ordered
    schedule["ordering_method"] = GATE2_V2_ORDERING_METHOD
    validate_gate2_v2_shared_schedule(schedule, manifest=manifest)
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


def gate2_v2_ordering_metrics(schedule: dict[str, Any]) -> dict[str, Any]:
    """Return the fixed sequence and precedence metrics."""

    episodes = schedule["episodes"]
    positions = {
        (
            str(episode["condition_id"]),
            int(episode["trial_index"]),
            str(episode["mode"]),
        ): index
        for index, episode in enumerate(episodes)
    }
    precedence = {
        condition: sum(
            positions[(condition, trial, "fixed_blend")]
            < positions[(condition, trial, "cosine_blend")]
            for trial in GATE2_V2_TRIALS
        )
        for condition in GATE2_V2_CONDITIONS
    }
    separations = [
        abs(
            positions[(condition, trial, "fixed_blend")]
            - positions[(condition, trial, "cosine_blend")]
        )
        - 1
        for condition in GATE2_V2_CONDITIONS
        for trial in GATE2_V2_TRIALS
    ]
    return {
        "ordering_method": schedule.get("ordering_method"),
        "maximum_same_mode_run_length": _maximum_run(
            [str(episode["mode"]) for episode in episodes]
        ),
        "maximum_same_condition_run_length": _maximum_run(
            [str(episode["condition_id"]) for episode in episodes]
        ),
        "minimum_pair_intervening_episodes": min(separations),
        "fixed_before_cosine_by_condition": precedence,
    }


def validate_gate2_v2_shared_schedule(
    schedule: dict[str, Any],
    *,
    manifest: ExperimentManifest,
) -> None:
    """Reject any deviation from the 40-row shared design."""

    episodes = schedule.get("episodes")
    if not isinstance(episodes, list) or len(episodes) != 40:
        raise ValueError("Gate-2 v2 shared schedule must contain 40 episodes.")
    if schedule.get("ordering_method") != GATE2_V2_ORDERING_METHOD:
        raise ValueError("Gate-2 v2 ordering-method identity is invalid.")
    cells = [
        (
            str(episode["mode"]),
            str(episode["condition_id"]),
            int(episode["trial_index"]),
        )
        for episode in episodes
    ]
    expected = {
        (mode, condition, trial)
        for mode in GATE2_V2_SHARED_MODES
        for condition in GATE2_V2_CONDITIONS
        for trial in GATE2_V2_TRIALS
    }
    if set(cells) != expected or len(cells) != len(set(cells)):
        raise ValueError("Gate-2 v2 shared cells are not exact and unique.")
    if Counter(mode for mode, _, _ in cells) != {
        mode: 20 for mode in GATE2_V2_SHARED_MODES
    }:
        raise ValueError("Gate-2 v2 requires 20 episodes per shared mode.")
    if Counter(condition for _, condition, _ in cells) != {
        condition: 10 for condition in GATE2_V2_CONDITIONS
    }:
        raise ValueError("Gate-2 v2 requires 10 episodes per condition.")
    if set(Counter((mode, condition) for mode, condition, _ in cells).values()) != {5}:
        raise ValueError("Gate-2 v2 requires five episodes per mode-condition.")
    expected_round = {
        (mode, condition)
        for mode in GATE2_V2_SHARED_MODES
        for condition in GATE2_V2_CONDITIONS
    }
    for trial in GATE2_V2_TRIALS:
        round_episodes = episodes[
            trial * GATE2_V2_UNITS_PER_TRIAL:
            (trial + 1) * GATE2_V2_UNITS_PER_TRIAL
        ]
        if (
            len(round_episodes) != GATE2_V2_UNITS_PER_TRIAL
            or {(row["mode"], row["condition_id"]) for row in round_episodes}
            != expected_round
            or {int(row["trial_index"]) for row in round_episodes} != {trial}
        ):
            raise ValueError(f"Gate-2 v2 trial round {trial} is incomplete.")
    metrics = gate2_v2_ordering_metrics(schedule)
    if metrics["maximum_same_condition_run_length"] > 1:
        raise ValueError("Gate-2 v2 has consecutive identical conditions.")
    if metrics["maximum_same_mode_run_length"] > 2:
        raise ValueError("Gate-2 v2 has a same-mode run longer than two.")
    if metrics["minimum_pair_intervening_episodes"] < 1:
        raise ValueError("Gate-2 v2 matched shared pairs require separation.")
    if any(
        value not in {2, 3}
        for value in metrics["fixed_before_cosine_by_condition"].values()
    ):
        raise ValueError("Gate-2 v2 precedence must be balanced 2/3.")
    if schedule.get("policy_seed_protocol") != SEED_PROTOCOL:
        raise ValueError("Gate-2 v2 policy seed protocol is invalid.")
    for condition in GATE2_V2_CONDITIONS:
        for trial in GATE2_V2_TRIALS:
            rows = [
                row
                for row in episodes
                if row["condition_id"] == condition
                and int(row["trial_index"]) == trial
            ]
            expected_seed = make_policy_episode_seed(
                base_seed=manifest.policy_base_seed,
                condition_id=condition,
                trial_index=trial,
                task_id=GATE2_V2_TASK_ID,
                initial_state_index=manifest.initial_state_index,
            )
            if {int(row["policy_episode_seed"]) for row in rows} != {
                expected_seed
            }:
                raise ValueError("Gate-2 v2 shared policy seeds are not matched.")


def build_gate2_v2_autonomous_schedule(
    protocol: dict[str, Any],
) -> dict[str, Any]:
    """Build the frozen 20-row autonomous design without reading outputs."""

    if protocol != GATE2_V2_EXPECTED_AUTONOMOUS_PROTOCOL:
        raise ValueError("Cannot build a drifted Gate-2 autonomous protocol.")
    rows = []
    order_index = 0
    for trial in GATE2_V2_TRIALS:
        shift = trial % len(GATE2_V2_CONDITIONS)
        conditions = (
            GATE2_V2_CONDITIONS[shift:] + GATE2_V2_CONDITIONS[:shift]
        )
        for condition in conditions:
            rows.append(
                {
                    "order_index": order_index,
                    "mode": "autonomous",
                    "condition_id": condition,
                    "trial_index": trial,
                    "initial_state_index": protocol["initial_state_index"],
                    "policy_episode_seed": make_policy_episode_seed(
                        base_seed=protocol["policy_base_seed"],
                        condition_id=condition,
                        trial_index=trial,
                        task_id=protocol["task_id"],
                        initial_state_index=protocol["initial_state_index"],
                    ),
                    "policy_seed_protocol": SEED_PROTOCOL,
                }
            )
            order_index += 1
    schedule = {
        "schema_version": 1,
        "experiment_id": protocol["experiment_id"],
        "policy_seed_protocol": SEED_PROTOCOL,
        "seed_excludes_arbitration_mode": True,
        "ordering_method": "gate2_v2_autonomous_cyclic_v1",
        "episodes": rows,
    }
    validate_gate2_v2_autonomous_schedule(schedule, protocol=protocol)
    return schedule


def validate_gate2_v2_autonomous_schedule(
    schedule: dict[str, Any],
    *,
    protocol: dict[str, Any],
) -> None:
    """Reject autonomous coverage, identity, seed, or protocol drift."""

    episodes = schedule.get("episodes")
    if not isinstance(episodes, list) or len(episodes) != 20:
        raise ValueError("Gate-2 v2 autonomous schedule must contain 20 episodes.")
    expected_cells = {
        (condition, trial)
        for condition in GATE2_V2_CONDITIONS
        for trial in GATE2_V2_TRIALS
    }
    cells = {
        (str(row["condition_id"]), int(row["trial_index"]))
        for row in episodes
    }
    if cells != expected_cells or len(episodes) != len(cells):
        raise ValueError("Gate-2 v2 autonomous cells are not exact and unique.")
    if {str(row["mode"]) for row in episodes} != {"autonomous"}:
        raise ValueError("Gate-2 autonomous schedule contains another mode.")
    for row in episodes:
        expected_seed = make_policy_episode_seed(
            base_seed=protocol["policy_base_seed"],
            condition_id=str(row["condition_id"]),
            trial_index=int(row["trial_index"]),
            task_id=protocol["task_id"],
            initial_state_index=protocol["initial_state_index"],
        )
        if int(row["policy_episode_seed"]) != expected_seed:
            raise ValueError("Gate-2 autonomous seed derivation is invalid.")
        if row.get("policy_seed_protocol") != SEED_PROTOCOL:
            raise ValueError("Gate-2 autonomous seed protocol is invalid.")
    if protocol["max_steps"] != GATE2_V2_MAX_STEPS:
        raise ValueError("Gate-2 autonomous horizon must be 280 steps.")


def validate_gate2_v2_matched_design(
    *,
    shared_schedule: dict[str, Any],
    autonomous_schedule: dict[str, Any],
) -> dict[str, Any]:
    """Prove the intended design contains exactly 20 matched triplets."""

    fields = (
        "condition_id",
        "trial_index",
        "initial_state_index",
        "policy_episode_seed",
        "policy_seed_protocol",
    )
    by_identity: dict[tuple[Any, ...], set[str]] = {}
    for row in [
        *autonomous_schedule["episodes"],
        *shared_schedule["episodes"],
    ]:
        key = tuple(row[field] for field in fields)
        modes = by_identity.setdefault(key, set())
        mode = str(row["mode"])
        if mode in modes:
            raise ValueError(f"Duplicate Gate-2 v2 mode for identity {key!r}.")
        modes.add(mode)
    expected_modes = set(GATE2_V2_ALL_MODES)
    if len(by_identity) != 20 or any(
        modes != expected_modes for modes in by_identity.values()
    ):
        raise ValueError("Gate-2 v2 design does not form 20 exact triplets.")
    return {
        "total_episodes": 60,
        "matched_triplets": 20,
        "episodes_by_mode": {mode: 20 for mode in GATE2_V2_ALL_MODES},
        "pairing_fields": list(fields),
    }


def validate_gate2_v2_shared_protocol(
    *,
    manifest: ExperimentManifest,
    input_source: str,
    spacemouse_profile_path: str,
    spacemouse_device_path: str,
    output_root: Path,
    autonomous_protocol_path: Path,
) -> dict[str, Any]:
    """Validate shared inputs and the frozen matched autonomous design."""

    validate_gate2_v2_manifest(manifest)
    if input_source.strip().lower() != "spacemouse":
        raise ValueError("Gate-2 v2 requires SpaceMouse input.")
    if not spacemouse_device_path.strip() or not Path(
        spacemouse_device_path
    ).is_absolute():
        raise ValueError("Gate-2 v2 requires an absolute SpaceMouse path.")
    if not _same_path(spacemouse_profile_path, GATE2_V2_PROFILE_PATH):
        raise ValueError("Gate-2 v2 requires the frozen SpaceMouse profile.")
    if not _same_path(str(output_root), GATE2_V2_SHARED_OUTPUT_ROOT):
        raise ValueError("Gate-2 v2 shared output root is fixed.")
    task_config = json_file_identity(Path(manifest.config_path))
    if (
        not _same_path(str(task_config["path"]), GATE2_V2_CONFIG_PATH)
        or task_config["sha256"] != GATE2_V2_CONFIG_SHA256
        or int(task_config["contents"].get("task_id", -1))
        != GATE2_V2_TASK_ID
    ):
        raise ValueError("Gate-2 v2 perturbation configuration drifted.")
    profile = load_spacemouse_profile(Path(spacemouse_profile_path))
    profile_identity = spacemouse_profile_identity(
        profile,
        path=spacemouse_profile_path,
    )
    if profile_identity["sha256"] != GATE2_V2_PROFILE_SHA256:
        raise ValueError("Gate-2 v2 SpaceMouse calibration drifted.")
    autonomous_protocol = load_gate2_v2_autonomous_protocol(
        autonomous_protocol_path
    )
    shared = build_gate2_v2_shared_schedule(
        manifest=manifest,
        task_id=GATE2_V2_TASK_ID,
        output_root=output_root,
    )
    regenerated = build_gate2_v2_shared_schedule(
        manifest=manifest,
        task_id=GATE2_V2_TASK_ID,
        output_root=output_root,
    )
    if shared != regenerated:
        raise ValueError("Gate-2 v2 shared schedule is not deterministic.")
    autonomous = build_gate2_v2_autonomous_schedule(autonomous_protocol)
    matched = validate_gate2_v2_matched_design(
        shared_schedule=shared,
        autonomous_schedule=autonomous,
    )
    return {
        "manifest": manifest,
        "task_config": task_config,
        "spacemouse_profile": {
            **profile_identity,
            "contents": profile.as_dict(),
        },
        "spacemouse_device_path": spacemouse_device_path,
        "schedule": shared,
        "autonomous_protocol": autonomous_protocol,
        "autonomous_schedule": autonomous,
        "matched_design": matched,
    }
