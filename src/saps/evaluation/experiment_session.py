"""Deterministic manifests and schedules for operator experiments."""

from __future__ import annotations

import dataclasses
import hashlib
import json
from pathlib import Path
import random
from typing import Any

from saps.policies.seeding import make_policy_episode_seed
from saps.policies.seeding import SEED_PROTOCOL


MANIFEST_SCHEMA_VERSION = 4
LEGACY_MANIFEST_SCHEMA_VERSION = 3
SCHEDULE_SCHEMA_VERSION = 1
OPERATOR_MODES = (
    "teleoperation",
    "takeover",
    "fixed_blend",
    "cosine_blend",
)


@dataclasses.dataclass(frozen=True)
class ExperimentManifest:
    """Validated immutable configuration for one operator study."""

    schema_version: int
    experiment_id: str
    config_path: str
    conditions: tuple[str, ...]
    modes: tuple[str, ...]
    trials_per_condition: int
    initial_state_index: int
    environment_seed: int
    policy_base_seed: int
    fixed_autonomy_weight: float
    cosine_gain: float
    control_frequency_hz: float
    operator_max_steps: int
    fine_translation_gain: float
    fine_rotation_gain: float
    normal_translation_gain: float
    normal_rotation_gain: float
    fast_translation_gain: float
    fast_rotation_gain: float
    default_speed_mode: str
    ordering_seed: int
    keyboard_translation_gain: float | None = None
    keyboard_rotation_gain: float | None = None

    @classmethod
    def from_dict(
        cls,
        data: dict[str, Any],
    ) -> "ExperimentManifest":
        """Validate and construct a manifest from JSON data."""

        schema_version = int(data["schema_version"])
        common_required = {
            field.name
            for field in dataclasses.fields(cls)
        }.difference(
            {"keyboard_translation_gain", "keyboard_rotation_gain"}
        )
        if schema_version == MANIFEST_SCHEMA_VERSION:
            legacy_gain_fields = {
                "fine_translation_gain",
                "fine_rotation_gain",
                "normal_translation_gain",
                "normal_rotation_gain",
                "fast_translation_gain",
                "fast_rotation_gain",
                "default_speed_mode",
            }
            required = common_required.difference(legacy_gain_fields).union(
                {"keyboard_translation_gain", "keyboard_rotation_gain"}
            )
        else:
            required = common_required
        missing = required.difference(data)

        if missing:
            raise ValueError(
                f"Manifest is missing fields: {sorted(missing)}"
            )
        if schema_version == MANIFEST_SCHEMA_VERSION:
            unknown = set(data).difference(required)
            if unknown:
                raise ValueError(
                    "Schema-v4 manifest contains unsupported fields: "
                    f"{sorted(unknown)}"
                )

        if schema_version == MANIFEST_SCHEMA_VERSION:
            keyboard_translation_gain = float(
                data["keyboard_translation_gain"]
            )
            keyboard_rotation_gain = float(data["keyboard_rotation_gain"])
            fine_translation_gain = keyboard_translation_gain
            fine_rotation_gain = keyboard_rotation_gain
            normal_translation_gain = keyboard_translation_gain
            normal_rotation_gain = keyboard_rotation_gain
            fast_translation_gain = keyboard_translation_gain
            fast_rotation_gain = keyboard_rotation_gain
            default_speed_mode = "normal"
        else:
            keyboard_translation_gain = None
            keyboard_rotation_gain = None
            fine_translation_gain = float(data["fine_translation_gain"])
            fine_rotation_gain = float(data["fine_rotation_gain"])
            normal_translation_gain = float(data["normal_translation_gain"])
            normal_rotation_gain = float(data["normal_rotation_gain"])
            fast_translation_gain = float(data["fast_translation_gain"])
            fast_rotation_gain = float(data["fast_rotation_gain"])
            default_speed_mode = str(data["default_speed_mode"])

        manifest = cls(
            schema_version=schema_version,
            experiment_id=str(data["experiment_id"]),
            config_path=str(data["config_path"]),
            conditions=tuple(
                str(value) for value in data["conditions"]
            ),
            modes=tuple(
                str(value) for value in data["modes"]
            ),
            trials_per_condition=int(
                data["trials_per_condition"]
            ),
            initial_state_index=int(
                data["initial_state_index"]
            ),
            environment_seed=int(data["environment_seed"]),
            policy_base_seed=int(data["policy_base_seed"]),
            fixed_autonomy_weight=float(
                data["fixed_autonomy_weight"]
            ),
            cosine_gain=float(data["cosine_gain"]),
            control_frequency_hz=float(
                data["control_frequency_hz"]
            ),
            operator_max_steps=int(data["operator_max_steps"]),
            fine_translation_gain=fine_translation_gain,
            fine_rotation_gain=fine_rotation_gain,
            normal_translation_gain=normal_translation_gain,
            normal_rotation_gain=normal_rotation_gain,
            fast_translation_gain=fast_translation_gain,
            fast_rotation_gain=fast_rotation_gain,
            default_speed_mode=default_speed_mode,
            ordering_seed=int(data["ordering_seed"]),
            keyboard_translation_gain=keyboard_translation_gain,
            keyboard_rotation_gain=keyboard_rotation_gain,
        )
        manifest.validate()
        return manifest

    def validate(self) -> None:
        """Reject ambiguous or protocol-incompatible values."""

        if self.schema_version not in {
            LEGACY_MANIFEST_SCHEMA_VERSION,
            MANIFEST_SCHEMA_VERSION,
        }:
            raise ValueError(
                "Unsupported manifest schema_version "
                f"{self.schema_version}."
            )

        if not self.experiment_id.strip():
            raise ValueError("experiment_id must not be empty.")

        if not self.conditions or len(set(self.conditions)) != len(
            self.conditions
        ):
            raise ValueError(
                "conditions must be non-empty and unique."
            )

        if not self.modes or len(set(self.modes)) != len(self.modes):
            raise ValueError("modes must be non-empty and unique.")

        unsupported_modes = set(self.modes).difference(OPERATOR_MODES)

        if unsupported_modes:
            raise ValueError(
                "Operator session modes must be selected from "
                f"{OPERATOR_MODES}; received {sorted(unsupported_modes)}."
            )

        if self.trials_per_condition <= 0:
            raise ValueError(
                "trials_per_condition must be positive."
            )

        for name, value in {
            "initial_state_index": self.initial_state_index,
            "environment_seed": self.environment_seed,
            "policy_base_seed": self.policy_base_seed,
            "ordering_seed": self.ordering_seed,
        }.items():
            if value < 0:
                raise ValueError(f"{name} must be non-negative.")

        if not 0.0 <= self.fixed_autonomy_weight <= 1.0:
            raise ValueError(
                "fixed_autonomy_weight must be within [0, 1]."
            )

        if self.cosine_gain <= 0.0:
            raise ValueError("cosine_gain must be positive.")

        if self.control_frequency_hz <= 0.0:
            raise ValueError(
                "control_frequency_hz must be positive."
            )

        if self.operator_max_steps <= 0:
            raise ValueError("operator_max_steps must be positive.")

        gains = (
            self.fine_translation_gain,
            self.fine_rotation_gain,
            self.normal_translation_gain,
            self.normal_rotation_gain,
            self.fast_translation_gain,
            self.fast_rotation_gain,
        )

        if any(not 0.0 < gain <= 1.0 for gain in gains):
            raise ValueError("Operator gains must be within (0, 1].")

        if not (
            self.fine_translation_gain
            <= self.normal_translation_gain
            <= self.fast_translation_gain
        ):
            raise ValueError("Translation gains must be non-decreasing.")

        if not (
            self.fine_rotation_gain
            <= self.normal_rotation_gain
            <= self.fast_rotation_gain
        ):
            raise ValueError("Rotation gains must be non-decreasing.")

        if self.default_speed_mode not in {"fine", "normal", "fast"}:
            raise ValueError("default_speed_mode is invalid.")

        if self.schema_version == MANIFEST_SCHEMA_VERSION:
            if (
                self.keyboard_translation_gain is None
                or self.keyboard_rotation_gain is None
            ):
                raise ValueError(
                    "Schema-v4 manifests require one keyboard translation "
                    "gain and one keyboard rotation gain."
                )
            if any(
                gain != self.keyboard_translation_gain
                for gain in (
                    self.fine_translation_gain,
                    self.normal_translation_gain,
                    self.fast_translation_gain,
                )
            ) or any(
                gain != self.keyboard_rotation_gain
                for gain in (
                    self.fine_rotation_gain,
                    self.normal_rotation_gain,
                    self.fast_rotation_gain,
                )
            ):
                raise ValueError(
                    "Schema-v4 keyboard gains must not vary by speed mode."
                )

    def as_dict(self) -> dict[str, Any]:
        """Return a canonical JSON-compatible representation."""

        data = dataclasses.asdict(self)
        if self.schema_version == MANIFEST_SCHEMA_VERSION:
            for field in (
                "fine_translation_gain",
                "fine_rotation_gain",
                "normal_translation_gain",
                "normal_rotation_gain",
                "fast_translation_gain",
                "fast_rotation_gain",
                "default_speed_mode",
            ):
                data.pop(field)
        else:
            data.pop("keyboard_translation_gain")
            data.pop("keyboard_rotation_gain")
        data["conditions"] = list(self.conditions)
        data["modes"] = list(self.modes)
        return data


def load_manifest(path: Path) -> ExperimentManifest:
    """Load one experiment manifest."""

    with path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    if not isinstance(data, dict):
        raise ValueError("Manifest root must be a JSON object.")

    return ExperimentManifest.from_dict(data)


def manifest_sha256(manifest: ExperimentManifest) -> str:
    """Return the canonical identity hash for a manifest."""

    return canonical_json_sha256(manifest.as_dict())


def canonical_json_sha256(payload: Any) -> str:
    """Return a stable SHA-256 for one JSON-compatible value."""

    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def json_file_identity(path: Path) -> dict[str, Any]:
    """Load and identify a JSON object used by an experiment."""

    with path.open("r", encoding="utf-8") as file:
        contents = json.load(file)

    if not isinstance(contents, dict):
        raise ValueError(f"JSON root must be an object: {path}")

    return {
        "path": str(path),
        "sha256": canonical_json_sha256(contents),
        "contents": contents,
    }


def build_schedule(
    *,
    manifest: ExperimentManifest,
    task_id: int,
    output_root: Path,
) -> dict[str, Any]:
    """Build a deterministic, cyclically counterbalanced schedule."""

    base_units = [
        (mode, condition_id)
        for mode in manifest.modes
        for condition_id in manifest.conditions
    ]
    random.Random(manifest.ordering_seed).shuffle(base_units)
    episodes: list[dict[str, Any]] = []
    order_index = 0

    for trial_index in range(manifest.trials_per_condition):
        shift = trial_index % len(base_units)
        trial_units = base_units[shift:] + base_units[:shift]

        for mode, condition_id in trial_units:
            episode_id = (
                f"trial_{trial_index:03d}__"
                f"condition_{condition_id}__mode_{mode}"
            )
            policy_seed = make_policy_episode_seed(
                base_seed=manifest.policy_base_seed,
                condition_id=condition_id,
                trial_index=trial_index,
                task_id=task_id,
                initial_state_index=manifest.initial_state_index,
            )
            episodes.append(
                {
                    "episode_id": episode_id,
                    "order_index": order_index,
                    "mode": mode,
                    "condition_id": condition_id,
                    "trial_index": trial_index,
                    "initial_state_index": (
                        manifest.initial_state_index
                    ),
                    "policy_episode_seed": policy_seed,
                    "policy_seed_protocol": SEED_PROTOCOL,
                    "status": "pending",
                    "attempt_count": 0,
                    "attempts": [],
                    "output_directory": str(
                        output_root / "attempts" / episode_id
                    ),
                    "termination_reason": None,
                    "success": None,
                }
            )
            order_index += 1

    return {
        "schema_version": SCHEDULE_SCHEMA_VERSION,
        "experiment_id": manifest.experiment_id,
        "manifest_sha256": manifest_sha256(manifest),
        "seed_excludes_arbitration_mode": True,
        "policy_seed_protocol": SEED_PROTOCOL,
        "episodes": episodes,
    }


def validate_schedule_identity(
    *,
    stored: dict[str, Any],
    expected: dict[str, Any],
) -> None:
    """Reject changes to the immutable portion of a stored schedule."""

    root_fields = (
        "schema_version",
        "experiment_id",
        "manifest_sha256",
        "seed_excludes_arbitration_mode",
        "policy_seed_protocol",
    )
    for field in root_fields:
        if stored.get(field) != expected.get(field):
            raise ValueError(
                f"Stored schedule field {field!r} does not match the "
                "deterministically regenerated schedule."
            )

    if (
        "ordering_method" in expected
        and stored.get("ordering_method") != expected["ordering_method"]
    ):
        raise ValueError(
            "Stored schedule field 'ordering_method' does not match the "
            "deterministically regenerated schedule."
        )

    stored_episodes = stored.get("episodes")
    expected_episodes = expected.get("episodes")
    if not isinstance(stored_episodes, list):
        raise ValueError("Stored schedule episodes must be a list.")
    if len(stored_episodes) != len(expected_episodes):
        raise ValueError(
            "Stored schedule episode count does not match the "
            "deterministically regenerated schedule."
        )

    episode_fields = (
        "episode_id",
        "order_index",
        "mode",
        "condition_id",
        "trial_index",
        "initial_state_index",
        "policy_episode_seed",
        "policy_seed_protocol",
        "output_directory",
    )
    for index, (stored_episode, expected_episode) in enumerate(
        zip(stored_episodes, expected_episodes)
    ):
        if not isinstance(stored_episode, dict):
            raise ValueError(
                f"Stored schedule episode {index} must be an object."
            )
        for field in episode_fields:
            if stored_episode.get(field) != expected_episode.get(field):
                raise ValueError(
                    "Stored schedule immutable field mismatch at "
                    f"episode {index}, field {field!r}."
                )


def validate_summary(
    *,
    summary_path: Path,
    episode: dict[str, Any],
) -> dict[str, Any]:
    """Validate one completed episode against its schedule row."""

    with summary_path.open("r", encoding="utf-8") as file:
        summary = json.load(file)

    expected_mode = str(episode["mode"])
    stored_mode = str(summary.get("arbitration_mode", ""))

    if stored_mode != expected_mode:
        raise ValueError(
            f"Summary mode {stored_mode!r} does not match "
            f"scheduled mode {expected_mode!r}."
        )

    for field in (
        "condition_id",
        "trial_index",
        "initial_state_index",
    ):
        if str(summary.get(field)) != str(episode[field]):
            raise ValueError(
                f"Summary field {field!r} does not match schedule."
            )

    if int(summary.get("policy_episode_seed", -1)) != int(
        episode["policy_episode_seed"]
    ):
        raise ValueError(
            "Summary policy seed does not match the autonomous-matched "
            "schedule seed."
        )

    if summary.get("policy_seed_protocol") != SEED_PROTOCOL:
        raise ValueError("Summary policy seed protocol is invalid.")

    for field in (
        "success",
        "termination_reason",
        "control_steps",
    ):
        if field not in summary:
            raise ValueError(f"Summary is missing field {field!r}.")

    steps_path = summary_path.with_name("steps.jsonl")

    if not steps_path.is_file() or steps_path.stat().st_size == 0:
        raise ValueError("Episode steps.jsonl is missing or empty.")

    return summary


def write_json_atomic(path: Path, payload: Any) -> None:
    """Write JSON without exposing a partial destination."""

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")

    with temporary.open("w", encoding="utf-8") as file:
        json.dump(payload, file, indent=2)
        file.write("\n")

    temporary.replace(path)
