"""Finite native joint-space inference over subscriber-only observations."""

from __future__ import annotations

import dataclasses
import datetime
import hashlib
import json
from pathlib import Path
import time
from typing import Any, Callable

import numpy as np

from saps.physical.embodiment import FR3_JOINT_NAMES
from saps.physical.live_observation import ACCEPTED_FR3_FINGER_JOINT_NAME_PAIRS
from saps.physical.shadow_audit import save_model_audit
from saps.physical.shadow_config import ACTION_SHAPE
from saps.physical.shadow_config import OPENPI_COMMIT
from saps.physical.shadow_config import POLICY_CHECKPOINT
from saps.physical.shadow_config import POLICY_CONFIG
from saps.policies.model_input_audit import array_evidence
from saps.policies.openpi_droid import DROID_POLICY_INPUT_KEYS
from saps.policies.openpi_droid import json_compatible
from saps.policies.openpi_droid import OpenPiDroidPolicy
from saps.policies.openpi_droid import summarize_action_chunk


ZERO_ACTUATION = {
    "published_robot_command_topics": [], "called_robot_services": [],
    "called_robot_actions": [], "policy_actions_executed": 0,
    "gripper_commands_issued": 0, "robot_command_action_clients": 0,
}


def utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def write_json(path: Path, record: Any) -> None:
    """Exclusive artifacts, including failure evidence, never overwrite a run."""

    with path.open("x", encoding="utf-8") as file:
        json.dump(json_compatible(record), file, indent=2, allow_nan=False)
        file.write("\n")


def file_evidence(path: Path) -> dict[str, Any]:
    return {"path": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


class CameraPairGate:
    """Both source stamps must strictly advance; arm updates do not qualify."""

    def __init__(self) -> None:
        self.previous: tuple[float, float] | None = None
        self.rejected_checks = 0

    def accepts(self, signature: tuple[float, ...] | None) -> bool:
        if signature is None:
            return False
        if self.previous is not None and any(
            current <= previous for current, previous in zip(signature[:2], self.previous)
        ):
            self.rejected_checks += 1
            return False
        return True

    def commit(self, signature: tuple[float, ...]) -> None:
        self.previous = (signature[0], signature[1])


def validate_request(policy_input: dict[str, Any]) -> None:
    if set(policy_input) != set(DROID_POLICY_INPUT_KEYS):
        raise ValueError("P0 requires exactly the five canonical DROID keys.")
    for key, shape, dtype in zip(
        DROID_POLICY_INPUT_KEYS[:4],
        ((180, 320, 3), (180, 320, 3), (7,), (1,)),
        (np.uint8, np.uint8, np.float32, np.float32),
    ):
        value = policy_input[key]
        if (not isinstance(value, np.ndarray) or value.shape != shape
                or value.dtype != dtype or not np.isfinite(value).all()):
            raise ValueError(f"Invalid canonical P0 array {key}.")
    if not isinstance(policy_input["prompt"], str) or not policy_input["prompt"].strip():
        raise ValueError("P0 requires an explicit nonempty prompt.")


def observation_record(observation: Any) -> dict[str, Any]:
    cameras = {}
    for role, frame in (("wrist", observation.wrist_frame),
                        ("exterior", observation.exterior_frame)):
        cameras[role] = {
            name: getattr(frame, name) for name in (
                "serial", "model", "topic", "source_encoding",
                "native_shape", "preprocessing",
            )
        }
        cameras[role]["stamp"] = dataclasses.asdict(frame.stamp)
        cameras[role]["client_image"] = array_evidence(frame.image_rgb)
        if frame.native_image_rgb is not None:
            cameras[role]["native_decoded_rgb"] = array_evidence(frame.native_image_rgb)
    gripper = dataclasses.asdict(observation.gripper_snapshot)
    # Existing adapter stores positions in canonical order, independently of
    # the message's original name order; make that association explicit.
    gripper["finger_position_order"] = next(
        pair for pair in ACCEPTED_FR3_FINGER_JOINT_NAME_PAIRS
        if set(pair) == set(gripper["joint_names"])
    )
    return {
        "assembly_utc": utc_now(),
        "timing": dataclasses.asdict(observation.timing),
        "cameras": cameras,
        "joint_names": FR3_JOINT_NAMES,
        "joint_positions": observation.joint_snapshot.position_rad,
        "joint_stamp": dataclasses.asdict(observation.joint_snapshot.stamp),
        "gripper": gripper, "prompt": observation.policy_input["prompt"],
        "canonical_schema": {
            key: array_evidence(value) if isinstance(value, np.ndarray)
            else {"type": "str", "value": value}
            for key, value in observation.policy_input.items()
        },
    }


def run_live_loop(
    *, collector: Any, policy: OpenPiDroidPolicy, output_dir: Path,
    request_count: int, policy_episode_seed: int, observation_timeout: float,
    spin_once: Callable[[], None], ros_now: Callable[[], float],
    config: dict[str, Any], record: dict[str, Any],
    monotonic: Callable[[], float] = time.monotonic,
) -> None:
    """Persist each observation before inference, then raw native actions.

    No controller, execution callback, or actuator interface is accepted.
    The caller owns run allocation, failure finalization, and ROS cleanup.
    """

    policy.validate_policy_identity(config_name=POLICY_CONFIG, checkpoint=POLICY_CHECKPOINT)
    metadata = policy.server_metadata
    audit_identity = metadata.get("saps_model_input_audit", {})
    if (audit_identity.get("schema_version") != 1
            or audit_identity.get("openpi_commit") != OPENPI_COMMIT):
        raise ValueError("Server must advertise pinned OpenPI and model audit v1.")
    if metadata["saps_seeded_sampling"].get("action_horizon") != 15:
        raise ValueError("P0 server must advertise a 15-action horizon.")
    record["server_metadata"] = metadata
    gate = CameraPairGate()
    record["completed_requests"] = 0
    for index in range(request_count):
        deadline = monotonic() + observation_timeout
        last_error = None
        while True:
            if monotonic() >= deadline:
                raise TimeoutError(
                    f"No fresh advancing camera pair: missing={collector.missing_sources()}, "
                    f"callbacks={collector.errors}, assembly={last_error}"
                )
            spin_once()
            signature = collector.latest_signature()
            record["duplicate_or_nonadvancing_pair_checks"] = gate.rejected_checks
            if not gate.accepts(signature) or collector.errors:
                continue
            try:
                observation = collector.assemble()
            except ValueError as error:
                last_error = str(error)
                record["last_rejected_observation"] = last_error
                continue
            break
        validate_request(observation.policy_input)
        for frame in (observation.wrist_frame, observation.exterior_frame):
            if (list(frame.native_shape) != config["native_images"]["shape"]
                    or frame.source_encoding.lower() != config["native_images"]["encoding"]):
                raise ValueError("Live camera profile differs from configured RGB8 1280x720.")
            if frame.native_image_rgb is None:
                raise ValueError("P0 requires retained native RGB evidence.")
        gate.commit(signature)
        sample_dir = output_dir / f"request_{index:04d}"
        sample_dir.mkdir(exist_ok=False)
        bundle = sample_dir / "observation.npz"
        sample = observation_record(observation)
        np.savez_compressed(bundle, **observation.policy_input)
        native_path = sample_dir / "native_rgb.npz"
        np.savez_compressed(
            native_path, wrist=observation.wrist_frame.native_image_rgb,
            exterior=observation.exterior_frame.native_image_rgb,
        )
        sample.update({
            "request_index": index, "chunk_index": index, "replan_index": index,
            "policy_episode_seed": policy_episode_seed,
            "observation_bundle": file_evidence(bundle),
            "native_rgb_bundle": file_evidence(native_path),
            "camera_pair_source_stamps": signature[:2],
            "source_rates": collector.source_rates(),
            "duplicate_or_nonadvancing_pair_checks": gate.rejected_checks,
            "reference_future_open_loop_horizon": 8,
            "policy_actions_executed": 0,
        })
        write_json(sample_dir / "request.json", sample)
        start_ros, start_mono, start_utc = ros_now(), monotonic(), utc_now()
        age = start_ros - observation.timing.oldest_source_ros_seconds
        if age < 0 or age > config["freshness"]["maximum_source_age_seconds"]:
            raise ValueError("Observation expired before policy submission.")
        try:
            response = policy.infer(
                observation.policy_input, policy_episode_seed=policy_episode_seed,
                replan_index=index, audit_model_input=index == 0,
            )
        finally:
            completed_mono, completed_ros = monotonic(), ros_now()
            completed_utc = utc_now()
            write_json(sample_dir / "request_timing.json", {
                "request_started_utc": start_utc,
                "request_started_ros_seconds": start_ros,
                "request_started_monotonic_seconds": start_mono,
                "observation_age_at_request_seconds": age,
                "call_ended_utc": completed_utc,
                "call_ended_monotonic_seconds": completed_mono,
                "call_ended_ros_seconds": completed_ros,
            })
        # Save even an unexpected finite horizon before rejecting it.
        action_path = sample_dir / "actions.npz"
        np.savez_compressed(action_path, actions=response.actions)
        result = {
            "request_index": index, "replan_index": index, "chunk_index": index,
            "request_started_utc": start_utc,
            "request_started_ros_seconds": start_ros,
            "request_started_monotonic_seconds": start_mono,
            "observation_age_at_request_seconds": age,
            "response_completed_utc": completed_utc,
            "response_completed_ros_seconds": completed_ros,
            "response_completed_monotonic_seconds": completed_mono,
            "observation_age_at_response_seconds": (
                completed_ros - observation.timing.oldest_source_ros_seconds
            ),
            "client_round_trip_seconds": response.client_round_trip_seconds,
            "policy_timing": response.policy_timing, "server_timing": response.server_timing,
            "sampling_metadata": response.sampling_metadata,
            "response_keys": response.response_keys,
            "action": array_evidence(response.actions), "finite": True,
            "action_bundle": file_evidence(action_path),
            "per_dimension_summary": summarize_action_chunk(response.actions),
            "returned_horizon": response.actions.shape[0],
            "reference_future_open_loop_horizon": 8, "policy_actions_executed": 0,
        }
        write_json(sample_dir / "response.json", result)
        if response.actions.shape != ACTION_SHAPE:
            raise ValueError(f"P0 requires native [15, 8], got {response.actions.shape}.")
        sampling = response.sampling_metadata or {}
        digest = sampling.get("noise_sha256", "")
        if (not isinstance(digest, str) or len(digest) != 64
                or any(char not in "0123456789abcdef" for char in digest)):
            raise ValueError("P0 requires the seeded noise SHA-256.")
        if response.policy_timing is None or response.server_timing is None:
            raise ValueError("P0 requires model and server timing evidence.")
        if index == 0:
            audit_record = save_model_audit(
                response.model_input_audit, observation.policy_input,
                sample_dir / "model_audit",
            )
            write_json(sample_dir / "model_audit.json", audit_record)
        record["completed_requests"] = index + 1
        record["duplicate_or_nonadvancing_pair_checks"] = gate.rejected_checks
        print(f"shadow {index + 1}/{request_count}: [15, 8], "
              f"{response.client_round_trip_seconds:.3f}s, executed=0", flush=True)
