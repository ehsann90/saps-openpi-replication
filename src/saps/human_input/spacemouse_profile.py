"""Validated, device-path-neutral SpaceMouse calibration profiles."""

from __future__ import annotations

import dataclasses
import hashlib
import json
import os
from pathlib import Path
from typing import Any
from typing import Optional

from saps.human_input.spacemouse import SpaceMouseConfig


PROFILE_SCHEMA_VERSION = 1


@dataclasses.dataclass(frozen=True)
class SpaceMouseProfile:
    """Portable application-level SpaceMouse calibration."""

    schema_version: int
    device_type: str
    axis_mapping: tuple[str, ...]
    axis_signs: tuple[float, ...]
    axis_maxima: tuple[float, ...]
    translation_gain: float
    rotation_gain: float
    axis_scales: tuple[float, ...]
    axis_enabled: tuple[bool, ...]
    deadzone: float
    stale_input_timeout_seconds: float
    open_button: int
    close_button: int

    def __post_init__(self) -> None:
        if self.schema_version != PROFILE_SCHEMA_VERSION:
            raise ValueError(
                "Unsupported SpaceMouse profile schema_version "
                f"{self.schema_version}."
            )
        if not self.device_type.strip():
            raise ValueError("device_type must not be empty.")
        self.to_config()

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "SpaceMouseProfile":
        """Validate a JSON-compatible profile object."""

        allowed = {
            field.name for field in dataclasses.fields(cls)
        }
        missing = allowed.difference(data)
        extra = set(data).difference(allowed)
        if missing:
            raise ValueError(
                f"SpaceMouse profile is missing fields: {sorted(missing)}"
            )
        if extra:
            raise ValueError(
                f"SpaceMouse profile has unknown fields: {sorted(extra)}"
            )

        if (
            not isinstance(data["schema_version"], int)
            or isinstance(data["schema_version"], bool)
        ):
            raise ValueError("schema_version must be an integer.")
        if not isinstance(data["device_type"], str):
            raise ValueError("device_type must be a string.")
        for field_name in {
            "axis_mapping",
            "axis_signs",
            "axis_maxima",
            "axis_scales",
            "axis_enabled",
        }:
            if not isinstance(data[field_name], list):
                raise ValueError(
                    f"{field_name} must be a JSON array."
                )
        for field_name in {"open_button", "close_button"}:
            value = data[field_name]
            if not isinstance(value, int) or isinstance(value, bool):
                raise ValueError(f"{field_name} must be an integer.")

        raw_enabled = data["axis_enabled"]
        if not isinstance(raw_enabled, list) or any(
            not isinstance(value, bool) for value in raw_enabled
        ):
            raise ValueError(
                "axis_enabled must be a JSON array of booleans."
            )

        try:
            return cls(
                schema_version=data["schema_version"],
                device_type=data["device_type"],
                axis_mapping=tuple(
                    str(value) for value in data["axis_mapping"]
                ),
                axis_signs=tuple(
                    float(value) for value in data["axis_signs"]
                ),
                axis_maxima=tuple(
                    float(value) for value in data["axis_maxima"]
                ),
                translation_gain=float(data["translation_gain"]),
                rotation_gain=float(data["rotation_gain"]),
                axis_scales=tuple(
                    float(value) for value in data["axis_scales"]
                ),
                axis_enabled=tuple(raw_enabled),
                deadzone=float(data["deadzone"]),
                stale_input_timeout_seconds=float(
                    data["stale_input_timeout_seconds"]
                ),
                open_button=data["open_button"],
                close_button=data["close_button"],
            )
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"Invalid SpaceMouse profile value: {error}"
            ) from error

    @classmethod
    def from_config(
        cls,
        config: SpaceMouseConfig,
        *,
        device_type: str,
    ) -> "SpaceMouseProfile":
        """Capture portable fields from a live processing config."""

        return cls(
            schema_version=PROFILE_SCHEMA_VERSION,
            device_type=device_type,
            axis_mapping=config.axis_mapping,
            axis_signs=config.axis_signs,
            axis_maxima=config.axis_maxima,
            translation_gain=config.translation_gain,
            rotation_gain=config.rotation_gain,
            axis_scales=config.axis_scales,
            axis_enabled=config.axis_enabled,
            deadzone=config.deadzone,
            stale_input_timeout_seconds=(
                config.stale_input_timeout_seconds
            ),
            open_button=config.open_button,
            close_button=config.close_button,
        )

    def to_config(self, *, device_path: str = "") -> SpaceMouseConfig:
        """Construct runtime processing config with a separate path."""

        return SpaceMouseConfig(
            device_path=device_path,
            translation_gain=self.translation_gain,
            rotation_gain=self.rotation_gain,
            deadzone=self.deadzone,
            axis_mapping=self.axis_mapping,
            axis_signs=self.axis_signs,
            axis_maxima=self.axis_maxima,
            axis_scales=self.axis_scales,
            axis_enabled=self.axis_enabled,
            stale_input_timeout_seconds=(
                self.stale_input_timeout_seconds
            ),
            open_button=self.open_button,
            close_button=self.close_button,
        )

    def as_dict(self) -> dict[str, Any]:
        """Return canonical JSON-compatible profile data."""

        data = dataclasses.asdict(self)
        for field_name in {
            "axis_mapping",
            "axis_signs",
            "axis_maxima",
            "axis_scales",
            "axis_enabled",
        }:
            data[field_name] = list(data[field_name])
        return data


def load_spacemouse_profile(path: Path) -> SpaceMouseProfile:
    """Load and validate one calibration profile."""

    with path.open("r", encoding="utf-8") as file:
        data = json.load(file)
    if not isinstance(data, dict):
        raise ValueError("SpaceMouse profile root must be a JSON object.")
    return SpaceMouseProfile.from_dict(data)


def spacemouse_profile_sha256(profile: SpaceMouseProfile) -> str:
    """Return a stable identity for validated profile contents."""

    encoded = json.dumps(
        profile.as_dict(),
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def spacemouse_profile_identity(
    profile: SpaceMouseProfile,
    *,
    path: str,
) -> dict[str, Any]:
    """Return compact provenance shared by sessions and episodes."""

    return {
        "path": path,
        "schema_version": profile.schema_version,
        "sha256": spacemouse_profile_sha256(profile),
    }


def save_spacemouse_profile(
    path: Path,
    profile: SpaceMouseProfile,
    *,
    owner_uid: Optional[int] = None,
    owner_gid: Optional[int] = None,
) -> None:
    """Atomically save a small version-controlled calibration profile."""

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_name(f".{path.name}.tmp")
    try:
        with temporary_path.open("w", encoding="utf-8") as file:
            json.dump(profile.as_dict(), file, indent=2)
            file.write("\n")
        temporary_path.replace(path)
        if owner_uid is not None and owner_gid is not None:
            os.chown(path, owner_uid, owner_gid)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()
