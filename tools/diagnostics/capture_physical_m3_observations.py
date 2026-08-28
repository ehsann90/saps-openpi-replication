#!/usr/bin/env python3
"""Capture subscriber-only FR3 observations for M3 shadow inference."""

from __future__ import annotations

import argparse
import dataclasses
import datetime
import hashlib
import json
from pathlib import Path
import platform
import re
import subprocess
import sys
import time
from typing import Any

import numpy as np

from saps.physical.embodiment import FR3_JOINT_NAMES
from saps.physical.live_observation import FR3_FINGER_JOINT_NAMES
from saps.physical.live_observation import ObservationFreshness
from saps.physical.ros_observation import RosCameraContract
from saps.physical.ros_observation import RosLiveTransformReader
from saps.physical.ros_observation import RosObservationContract
from saps.physical.ros_observation import RosPhysicalObservationCollector


WRIST_SERIAL = "342222073510"
M1_COMMIT = "ea762d782c68024ce1f2ce3a9f764b2e6122f198"
M2_COMMIT = "b5a18d1c7c54d8c78afbec71b5f3addbfee60c5b"
POLICY_CHECKPOINT = "gs://openpi-assets/checkpoints/pi05_droid"


@dataclasses.dataclass(frozen=True)
class Args:
    output_dir: Path
    exterior_camera_serial: str
    prompt: str
    wrist_camera_serial: str
    wrist_image_topic: str
    exterior_image_topic: str
    joint_state_topic: str
    gripper_state_topic: str
    observation_count: int
    timeout_seconds: float
    maximum_source_age_seconds: float
    maximum_cross_source_skew_seconds: float
    base_frame: str
    tcp_frame: str
    tf_wait_timeout_seconds: float
    maximum_tf_age_seconds: float
    repository_root: Path
    franka_description_dir: Path
    igd_control_dir: Path


def main(args: Args) -> None:
    """Read physical sources, validate timing, and save an immutable bundle."""

    _validate_args(args)
    camera_inventory = enumerate_realsense_devices()
    wrist_device = _require_camera(
        camera_inventory,
        args.wrist_camera_serial,
        role="wrist",
    )
    exterior_device = _require_camera(
        camera_inventory,
        args.exterior_camera_serial,
        role="exterior",
    )
    if args.wrist_camera_serial == args.exterior_camera_serial:
        raise ValueError("wrist and exterior camera serials must differ.")

    contract = RosObservationContract(
        joint_state_topic=args.joint_state_topic,
        gripper_state_topic=args.gripper_state_topic,
        wrist_camera=RosCameraContract(
            role="wrist",
            serial=args.wrist_camera_serial,
            model=wrist_device["model"],
            topic=args.wrist_image_topic,
        ),
        exterior_camera=RosCameraContract(
            role="exterior",
            serial=args.exterior_camera_serial,
            model=exterior_device["model"],
            topic=args.exterior_image_topic,
        ),
    )
    freshness = ObservationFreshness(
        maximum_source_age_seconds=args.maximum_source_age_seconds,
        maximum_cross_source_skew_seconds=(
            args.maximum_cross_source_skew_seconds
        ),
    )

    import rclpy

    rclpy.init()
    node = rclpy.create_node("saps_physical_m3_observation_subscriber")
    collector = RosPhysicalObservationCollector(
        node,
        contract,
        prompt=args.prompt,
        freshness=freshness,
    )
    transform_reader = RosLiveTransformReader(
        node,
        target_frame=args.base_frame,
        source_frame=args.tcp_frame,
    )
    capture_started_utc = _utc_now()
    observations = []
    last_signature = None
    last_assembly_error: str | None = None
    try:
        initial_transform = transform_reader.wait_until_ready(
            timeout_seconds=args.tf_wait_timeout_seconds,
            maximum_age_seconds=args.maximum_tf_age_seconds,
        )
        start_monotonic = time.monotonic()
        while len(observations) < args.observation_count:
            elapsed = time.monotonic() - start_monotonic
            if elapsed > args.timeout_seconds:
                raise TimeoutError(
                    "Timed out waiting for complete physical observations; "
                    f"captured={len(observations)}, "
                    f"missing={list(collector.missing_sources())}, "
                    f"callback_errors={collector.errors}, "
                    f"last_assembly_error={last_assembly_error!r}."
                )
            rclpy.spin_once(node, timeout_sec=0.05)
            signature = collector.latest_signature()
            if signature is None:
                continue
            if last_signature is not None and (
                signature[0] == last_signature[0]
                or signature[1] == last_signature[1]
            ):
                continue
            try:
                observation = collector.assemble()
            except ValueError as error:
                last_assembly_error = str(error)
                continue
            observations.append(observation)
            last_signature = signature
            last_assembly_error = None
            print(
                f"observation {len(observations)}/{args.observation_count}: "
                f"skew={observation.timing.cross_source_skew_seconds:.4f}s "
                f"q={observation.joint_snapshot.position_rad.tolist()} "
                f"g={observation.gripper_snapshot.closure:.4f}",
                flush=True,
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    arrays = _observation_arrays(observations)
    run_record = {
        "schema_version": 1,
        "milestone": "physical_pi05_droid_m3",
        "diagnostic_scope": (
            "live subscriber-only physical observation capture; no ROS "
            "publisher, service client, action client, policy call, Servo "
            "command, robot command, or gripper command"
        ),
        "capture_started_utc": capture_started_utc,
        "capture_completed_utc": _utc_now(),
        "arguments": _json_compatible(dataclasses.asdict(args)),
        "provenance": _provenance(
            args,
            camera_inventory=camera_inventory,
        ),
        "ros_contract": {
            "joint_state_topic": contract.joint_state_topic,
            "gripper_state_topic": contract.gripper_state_topic,
            "joint_order": list(FR3_JOINT_NAMES),
            "finger_joint_order": list(FR3_FINGER_JOINT_NAMES),
            "wrist_image_topic": contract.wrist_camera.topic,
            "exterior_image_topic": contract.exterior_camera.topic,
            "source_rates": collector.source_rates(),
            "tf": {
                "readiness": transform_reader.readiness_record(),
                "initial_live_transform": initial_transform.as_dict(),
                "maximum_age_seconds": args.maximum_tf_age_seconds,
                "subscriptions_only": ["/tf", "/tf_static"],
            },
        },
        "camera_contract": {
            "wrist": wrist_device,
            "exterior": exterior_device,
            "selection": "distinct explicit serials; never enumeration order",
            "policy_preprocessing": (
                "RGB uint8; deterministic centred 16:9 crop followed by "
                "resize to 320x180"
            ),
        },
        "policy_contract": {
            "config": "pi05_droid",
            "checkpoint": POLICY_CHECKPOINT,
            "observation_keys": list(observations[0].policy_input),
            "expected_action_shape": [15, 8],
        },
        "freshness": dataclasses.asdict(freshness),
        "observations": [
            _observation_record(index, observation)
            for index, observation in enumerate(observations)
        ],
        "bundle": {
            "path": "observation_bundle.npz",
            "arrays": {
                name: _array_contract(value)
                for name, value in arrays.items()
            },
        },
        "actuation": {
            "published_topics": [],
            "called_services": [],
            "called_actions": [],
            "robot_commands_issued": 0,
        },
    }
    _write_capture(args.output_dir, arrays, run_record)
    print(f"Wrote M3 capture to {args.output_dir}")


def enumerate_realsense_devices() -> list[dict[str, str]]:
    """Return the connected SDK inventory without selecting by order."""

    result = subprocess.run(
        ("rs-enumerate-devices", "-s"),
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=20.0,
    )
    devices = []
    for line in result.stdout.splitlines():
        fields = re.split(r"\s{2,}", line.strip())
        if len(fields) != 3 or fields[0] == "Device Name":
            continue
        model, serial, firmware = fields
        if serial.isdigit():
            devices.append(
                {
                    "model": model,
                    "serial": serial,
                    "firmware": firmware,
                }
            )
    return devices


def _require_camera(
    inventory: list[dict[str, str]],
    serial: str,
    *,
    role: str,
) -> dict[str, str]:
    matches = [device for device in inventory if device["serial"] == serial]
    if len(matches) != 1:
        available = [device["serial"] for device in inventory]
        raise RuntimeError(
            f"Required {role} RealSense serial {serial!r} is not uniquely "
            f"connected; available serials={available}. The wrist image "
            "will not be duplicated as an exterior image."
        )
    return {"role": role, **matches[0]}


def _observation_arrays(observations: list[Any]) -> dict[str, np.ndarray]:
    return {
        "exterior_images": np.stack(
            [
                observation.policy_input[
                    "observation/exterior_image_1_left"
                ]
                for observation in observations
            ]
        ),
        "wrist_images": np.stack(
            [
                observation.policy_input["observation/wrist_image_left"]
                for observation in observations
            ]
        ),
        "joint_positions": np.stack(
            [
                observation.policy_input["observation/joint_position"]
                for observation in observations
            ]
        ),
        "gripper_positions": np.stack(
            [
                observation.policy_input["observation/gripper_position"]
                for observation in observations
            ]
        ),
        "prompts": np.asarray(
            [observation.policy_input["prompt"] for observation in observations]
        ),
        "source_ros_seconds": np.asarray(
            [
                [
                    observation.timing.source_ros_seconds[name]
                    for name in (
                        "wrist_image",
                        "exterior_image",
                        "joint_state",
                        "gripper_state",
                    )
                ]
                for observation in observations
            ],
            dtype=np.float64,
        ),
    }


def _observation_record(index: int, observation: Any) -> dict[str, Any]:
    policy_input = observation.policy_input
    return {
        "index": index,
        "schema": {
            key: (
                _array_contract(value)
                if isinstance(value, np.ndarray)
                else {"type": type(value).__name__, "value": value}
            )
            for key, value in policy_input.items()
        },
        "joint_position_rad": observation.joint_snapshot.position_rad.tolist(),
        "gripper": {
            "finger_position_m": (
                observation.gripper_snapshot.finger_position_m.tolist()
            ),
            "width_m": observation.gripper_snapshot.width_m,
            "maximum_width_m": (
                observation.gripper_snapshot.maximum_width_m
            ),
            "unclipped_closure": (
                observation.gripper_snapshot.unclipped_closure
            ),
            "closure": observation.gripper_snapshot.closure,
            "closure_clipped": (
                observation.gripper_snapshot.closure_clipped
            ),
        },
        "wrist_camera": _camera_record(observation.wrist_frame),
        "exterior_camera": _camera_record(observation.exterior_frame),
        "timing": dataclasses.asdict(observation.timing),
    }


def _camera_record(frame: Any) -> dict[str, Any]:
    return {
        "serial": frame.serial,
        "model": frame.model,
        "topic": frame.topic,
        "source_encoding": frame.source_encoding,
        "native_shape": list(frame.native_shape),
        "preprocessing": frame.preprocessing,
        "final": _array_contract(frame.image_rgb),
    }


def _provenance(
    args: Args,
    *,
    camera_inventory: list[dict[str, str]],
) -> dict[str, Any]:
    repository = args.repository_root.resolve()
    franka = args.franka_description_dir.resolve()
    igd = args.igd_control_dir.resolve()
    openpi = repository / "third_party/openpi"
    hand_xacro = franka / "end_effectors/common/franka_hand.xacro"
    return {
        "repository": {
            "path": str(repository),
            "branch": _git_text(repository, "branch", "--show-current"),
            "commit": _git_text(repository, "rev-parse", "HEAD"),
            "status": _git_text(repository, "status", "--short").splitlines(),
            "m1_commit": M1_COMMIT,
            "m2_commit": M2_COMMIT,
        },
        "openpi": {
            "path": str(openpi.resolve()),
            "commit": _git_text(openpi, "rev-parse", "HEAD"),
        },
        "franka_description": _external_git_identity(franka),
        "igd_fr3_control": _external_git_identity(igd),
        "gripper_maximum_opening_source": {
            "path": str(hand_xacro),
            "sha256": _sha256(hand_xacro),
            "finger_upper_limit_m": 0.04,
            "total_width_m": 0.08,
        },
        "ros_distro": _environment_value("ROS_DISTRO"),
        "camera_inventory": camera_inventory,
        "runtime": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": np.__version__,
        },
        "policy_checkpoint": POLICY_CHECKPOINT,
    }


def _external_git_identity(path: Path) -> dict[str, Any]:
    diff = _git_bytes(path, "diff", "--binary", "--no-ext-diff")
    return {
        "path": str(path),
        "commit": _git_text(path, "rev-parse", "HEAD"),
        "status": _git_text(path, "status", "--short").splitlines(),
        "local_diff_sha256": hashlib.sha256(diff).hexdigest(),
    }


def _write_capture(
    output_dir: Path,
    arrays: dict[str, np.ndarray],
    run_record: dict[str, Any],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=False)
    bundle_path = output_dir / "observation_bundle.npz"
    np.savez_compressed(bundle_path, **arrays)
    run_record["bundle"]["sha256"] = _sha256(bundle_path)
    destination = output_dir / "run.json"
    with destination.open("x", encoding="utf-8") as file:
        json.dump(run_record, file, indent=2, sort_keys=True, allow_nan=False)
        file.write("\n")


def _validate_args(args: Args) -> None:
    if args.output_dir.exists():
        raise FileExistsError(
            f"Output directory already exists: {args.output_dir}."
        )
    if not args.exterior_camera_serial.strip():
        raise ValueError(
            "--exterior-camera-serial is required. Only the wrist camera "
            "was present during M3 implementation; no image substitution "
            "is permitted."
        )
    if args.observation_count <= 0:
        raise ValueError("observation_count must be positive.")
    if args.timeout_seconds <= 0.0:
        raise ValueError("timeout_seconds must be positive.")
    if args.tf_wait_timeout_seconds <= 0.0:
        raise ValueError("tf_wait_timeout_seconds must be positive.")
    if args.maximum_tf_age_seconds <= 0.0:
        raise ValueError("maximum_tf_age_seconds must be positive.")
    for path in (
        args.repository_root,
        args.franka_description_dir,
        args.igd_control_dir,
    ):
        if not path.exists():
            raise FileNotFoundError(path)


def _git_text(path: Path, *arguments: str) -> str:
    return _git_bytes(path, *arguments).decode("utf-8").rstrip()


def _git_bytes(path: Path, *arguments: str) -> bytes:
    return subprocess.run(
        ("git", "-C", str(path), *arguments),
        check=True,
        stdout=subprocess.PIPE,
    ).stdout


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _array_contract(value: np.ndarray) -> dict[str, Any]:
    contract = {
        "shape": list(value.shape),
        "dtype": str(value.dtype),
    }
    if np.issubdtype(value.dtype, np.number):
        contract.update(
            {
                "minimum": float(np.min(value)) if value.size else None,
                "maximum": float(np.max(value)) if value.size else None,
            }
        )
    elif np.issubdtype(value.dtype, np.str_):
        lengths = np.char.str_len(value)
        contract.update(
            {
                "minimum_length": (
                    int(np.min(lengths)) if value.size else None
                ),
                "maximum_length": (
                    int(np.max(lengths)) if value.size else None
                ),
            }
        )
    return contract


def _json_compatible(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_compatible(item) for item in value]
    return value


def _environment_value(name: str) -> str | None:
    import os

    return os.environ.get(name)


def _utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def _parse_args() -> Args:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--exterior-camera-serial", required=True)
    parser.add_argument("--prompt", required=True)
    parser.add_argument("--wrist-camera-serial", default=WRIST_SERIAL)
    parser.add_argument(
        "--wrist-image-topic",
        default="/wrist/wrist_camera/color/image_raw",
    )
    parser.add_argument(
        "--exterior-image-topic",
        default="/exterior/exterior_camera/color/image_raw",
    )
    parser.add_argument("--joint-state-topic", default="/franka/joint_states")
    parser.add_argument(
        "--gripper-state-topic",
        default="/franka_gripper/joint_states",
    )
    parser.add_argument("--observation-count", type=int, default=5)
    parser.add_argument("--timeout-seconds", type=float, default=30.0)
    parser.add_argument(
        "--maximum-source-age-seconds",
        type=float,
        default=0.5,
    )
    parser.add_argument(
        "--maximum-cross-source-skew-seconds",
        type=float,
        default=0.25,
    )
    parser.add_argument("--base-frame", default="fr3_link0")
    parser.add_argument("--tcp-frame", default="fr3_hand_tcp")
    parser.add_argument(
        "--tf-wait-timeout-seconds",
        type=float,
        default=15.0,
    )
    parser.add_argument(
        "--maximum-tf-age-seconds",
        type=float,
        default=0.25,
    )
    parser.add_argument("--repository-root", type=Path, default=Path("."))
    parser.add_argument(
        "--franka-description-dir",
        type=Path,
        default=Path(
            "/home/hvl-robotics2404/franka_ros2_ws/src/"
            "franka_description"
        ),
    )
    parser.add_argument(
        "--igd-control-dir",
        type=Path,
        default=Path(
            "/home/hvl-robotics2404/franka_ros2_ws/src/igd_fr3_control"
        ),
    )
    parsed = parser.parse_args()
    return Args(**vars(parsed))


if __name__ == "__main__":
    main(_parse_args())
