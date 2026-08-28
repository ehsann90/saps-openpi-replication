"""Offline DROID observation and response contracts for OpenPI."""

from __future__ import annotations

from collections.abc import Mapping
from collections.abc import Sequence
import dataclasses
import time
from typing import Any

import numpy as np


SAPS_PROTOCOL_VERSION = 1
DROID_ACTION_DIMENSION = 8
DROID_JOINT_DIMENSION = 7
DROID_POLICY_INPUT_KEYS = (
    "observation/exterior_image_1_left",
    "observation/wrist_image_left",
    "observation/joint_position",
    "observation/gripper_position",
    "prompt",
)


@dataclasses.dataclass(frozen=True)
class DroidPolicyResponse:
    """One validated DROID action chunk and its diagnostic metadata."""

    actions: np.ndarray
    client_round_trip_seconds: float
    policy_timing: dict[str, Any] | None
    server_timing: dict[str, Any] | None
    sampling_metadata: dict[str, Any] | None
    response_keys: tuple[str, ...]


@dataclasses.dataclass(frozen=True)
class DroidRunProvenance:
    """JSON-serializable identities required for an offline DROID run."""

    repository_commit: str
    repository_dirty: bool
    openpi_commit: str
    checkpoint: str
    policy_config: str
    dataset_source: str
    sample_identities: Sequence[str]
    runtime: Mapping[str, Any]
    server_metadata: Mapping[str, Any]

    def as_dict(self) -> dict[str, Any]:
        """Return provenance without NumPy or tuple-only values."""

        return {
            "repository_commit": self.repository_commit,
            "repository_dirty": self.repository_dirty,
            "openpi_commit": self.openpi_commit,
            "checkpoint": self.checkpoint,
            "policy_config": self.policy_config,
            "dataset_source": self.dataset_source,
            "sample_identities": list(self.sample_identities),
            "runtime": json_compatible(self.runtime),
            "server_metadata": json_compatible(
                self.server_metadata
            ),
        }


class OpenPiDroidPolicy:
    """Query an OpenPI server without imposing a fixed action horizon."""

    def __init__(
        self,
        *,
        host: str = "0.0.0.0",
        port: int = 8000,
        client: Any | None = None,
    ) -> None:
        if client is None:
            from openpi_client.websocket_client_policy import (
                WebsocketClientPolicy,
            )

            client = WebsocketClientPolicy(host, port)
        self._client = client
        self.last_sampling_metadata: dict[str, Any] | None = None

    @property
    def server_metadata(self) -> dict[str, Any]:
        """Return a JSON-compatible policy-server handshake."""

        return dict(self._client.get_server_metadata())

    def validate_policy_identity(
        self,
        *,
        config_name: str,
        checkpoint: str,
    ) -> None:
        """Require the requested config and checkpoint on a seeded server."""

        seeded = self.server_metadata.get("saps_seeded_sampling")
        if not isinstance(seeded, Mapping):
            raise RuntimeError(
                "Policy server does not advertise seeded-policy identity."
            )

        actual = (
            seeded.get("policy_config_name"),
            seeded.get("policy_checkpoint"),
        )
        expected = (config_name, checkpoint)
        if actual != expected:
            raise RuntimeError(
                "Policy server identity does not match the requested "
                f"DROID policy: expected {expected!r}, received "
                f"{actual!r}."
            )

    def infer(
        self,
        policy_input: dict[str, Any],
        *,
        policy_episode_seed: int | None = None,
        replan_index: int | None = None,
    ) -> DroidPolicyResponse:
        """Request and validate one native eight-dimensional action chunk."""

        if (policy_episode_seed is None) != (replan_index is None):
            raise ValueError(
                "policy_episode_seed and replan_index must either both "
                "be supplied or both be omitted."
            )

        if policy_episode_seed is None:
            request = policy_input
        else:
            request = {
                "__saps_protocol_version__": SAPS_PROTOCOL_VERSION,
                "observation": policy_input,
                "policy_episode_seed": int(policy_episode_seed),
                "replan_index": int(replan_index),
            }

        start = time.perf_counter()
        result = self._client.infer(request)
        round_trip_seconds = time.perf_counter() - start

        if not isinstance(result, Mapping):
            raise TypeError(
                "OpenPI DROID response must be a mapping, received "
                f"{type(result).__name__}."
            )

        actions = validate_droid_action_response(result)
        sampling_metadata = _optional_mapping(
            result.get("saps_sampling"),
            field_name="saps_sampling",
        )

        if policy_episode_seed is not None:
            _validate_sampling_identity(
                sampling_metadata,
                policy_episode_seed=policy_episode_seed,
                replan_index=int(replan_index),
            )

        self.last_sampling_metadata = sampling_metadata

        return DroidPolicyResponse(
            actions=actions,
            client_round_trip_seconds=round_trip_seconds,
            policy_timing=_optional_mapping(
                result.get("policy_timing"),
                field_name="policy_timing",
            ),
            server_timing=_optional_mapping(
                result.get("server_timing"),
                field_name="server_timing",
            ),
            sampling_metadata=sampling_metadata,
            response_keys=tuple(sorted(str(key) for key in result)),
        )


def prepare_droid_observation(
    *,
    exterior_image: np.ndarray,
    wrist_image: np.ndarray,
    joint_position: np.ndarray,
    gripper_position: np.ndarray,
    prompt: str,
) -> dict[str, Any]:
    """Construct the canonical raw-RLDS input to pinned DroidInputs.

    Images remain in source-resolution RGB uint8 HWC form. The pinned OpenPI
    transforms perform the policy's 224-by-224 padded resize on the server.
    """

    exterior = _validate_image(
        exterior_image,
        field_name="exterior_image",
    )
    wrist = _validate_image(
        wrist_image,
        field_name="wrist_image",
    )
    joints = _validate_vector(
        joint_position,
        expected_shape=(DROID_JOINT_DIMENSION,),
        field_name="joint_position",
    )
    gripper = _validate_vector(
        gripper_position,
        expected_shape=(1,),
        field_name="gripper_position",
    )

    if not isinstance(prompt, str) or not prompt.strip():
        raise ValueError("prompt must be a non-empty string.")

    return {
        DROID_POLICY_INPUT_KEYS[0]: exterior,
        DROID_POLICY_INPUT_KEYS[1]: wrist,
        DROID_POLICY_INPUT_KEYS[2]: joints,
        DROID_POLICY_INPUT_KEYS[3]: gripper,
        DROID_POLICY_INPUT_KEYS[4]: prompt,
    }


def validate_droid_action_response(
    response: Mapping[str, Any],
) -> np.ndarray:
    """Validate eight-dimensional actions while observing any horizon."""

    if "actions" not in response:
        raise KeyError(
            "OpenPI DROID response does not contain an 'actions' key."
        )

    actions = np.asarray(response["actions"])
    if not np.issubdtype(actions.dtype, np.floating):
        raise TypeError(
            "OpenPI DROID actions must have a floating dtype, received "
            f"{actions.dtype}."
        )

    if (
        actions.ndim != 2
        or actions.shape[0] <= 0
        or actions.shape[1] != DROID_ACTION_DIMENSION
    ):
        raise ValueError(
            "OpenPI DROID actions must have shape [positive_horizon, 8], "
            f"received {actions.shape}."
        )

    if not np.all(np.isfinite(actions)):
        raise ValueError(
            "OpenPI DROID actions must contain only finite values."
        )

    result = np.array(actions, copy=True)
    result.setflags(write=False)
    return result


def summarize_action_chunk(actions: np.ndarray) -> list[dict[str, Any]]:
    """Return JSON-compatible min, max, mean, and standard deviation."""

    validated = validate_droid_action_response({"actions": actions})
    summaries = []
    for dimension in range(DROID_ACTION_DIMENSION):
        values = validated[:, dimension]
        summaries.append(
            {
                "dimension": dimension,
                "minimum": float(np.min(values)),
                "maximum": float(np.max(values)),
                "mean": float(np.mean(values)),
                "standard_deviation": float(np.std(values)),
            }
        )
    return summaries


def json_compatible(value: Any) -> Any:
    """Recursively convert common scientific values for JSON output."""

    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {
            str(key): json_compatible(item)
            for key, item in value.items()
        }
    if isinstance(value, Sequence) and not isinstance(
        value,
        (str, bytes, bytearray),
    ):
        return [json_compatible(item) for item in value]
    return value


def _validate_image(
    image: np.ndarray,
    *,
    field_name: str,
) -> np.ndarray:
    result = np.asarray(image)
    if result.dtype != np.uint8:
        raise TypeError(
            f"{field_name} must have dtype uint8, received "
            f"{result.dtype}."
        )
    if (
        result.ndim != 3
        or result.shape[0] <= 0
        or result.shape[1] <= 0
        or result.shape[2] != 3
    ):
        raise ValueError(
            f"{field_name} must have shape [height, width, 3], "
            f"received {result.shape}."
        )
    return np.ascontiguousarray(result)


def _validate_vector(
    value: np.ndarray,
    *,
    expected_shape: tuple[int, ...],
    field_name: str,
) -> np.ndarray:
    result = np.asarray(value)
    if result.shape != expected_shape:
        raise ValueError(
            f"{field_name} must have shape {expected_shape}, received "
            f"{result.shape}."
        )
    if not np.issubdtype(result.dtype, np.number) or np.issubdtype(
        result.dtype,
        np.complexfloating,
    ):
        raise TypeError(
            f"{field_name} must have a real numeric dtype, received "
            f"{result.dtype}."
        )
    result = np.asarray(result, dtype=np.float32)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{field_name} must contain only finite values.")
    return np.ascontiguousarray(result)


def _optional_mapping(
    value: Any,
    *,
    field_name: str,
) -> dict[str, Any] | None:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise TypeError(f"{field_name} must be a mapping when present.")
    return dict(value)


def _validate_sampling_identity(
    metadata: dict[str, Any] | None,
    *,
    policy_episode_seed: int,
    replan_index: int,
) -> None:
    if metadata is None:
        raise RuntimeError(
            "The seeded policy server did not return sampling metadata."
        )

    required = {
        "policy_episode_seed",
        "replan_index",
        "protocol_version",
    }
    missing = required.difference(metadata)
    if missing:
        raise RuntimeError(
            "Seeded policy metadata is missing fields: "
            f"{sorted(missing)}."
        )

    actual = (
        int(metadata["policy_episode_seed"]),
        int(metadata["replan_index"]),
        int(metadata["protocol_version"]),
    )
    expected = (
        int(policy_episode_seed),
        int(replan_index),
        SAPS_PROTOCOL_VERSION,
    )
    if actual != expected:
        raise RuntimeError(
            "Seeded policy metadata does not match the request: "
            f"expected {expected!r}, received {actual!r}."
        )
