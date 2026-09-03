#!/usr/bin/env python3
"""Log spnavd human input in TCP and base frames without actuation."""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
from typing import Any

import numpy as np

from saps.human_input.spnav import SpnavConfig
from saps.human_input.spnav import SpnavHumanInputBackend
from saps.physical.live_observation import ordered_fr3_joint_positions
from saps.physical.live_observation import ros_stamp_seconds
from saps.physical.fr3_kinematics import Fr3PinocchioKinematics
from saps.physical.ros_observation import RosLiveTransformReader


IGD_COMMIT = "1ecd52e310f069d855591ff69c17e5c3412e1722"


def main(args: argparse.Namespace) -> None:
    """Sample current-q frame transforms and spnavd events, log only."""

    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    if args.duration_seconds <= 0.0 or args.sample_rate_hz <= 0.0:
        raise ValueError("duration_seconds and sample_rate_hz must be positive.")
    if args.maximum_joint_age_seconds <= 0.0:
        raise ValueError("maximum_joint_age_seconds must be positive.")
    if args.tf_wait_timeout_seconds <= 0.0:
        raise ValueError("tf_wait_timeout_seconds must be positive.")
    if args.maximum_tf_age_seconds <= 0.0:
        raise ValueError("maximum_tf_age_seconds must be positive.")

    xacro_path = (
        args.franka_description_dir.resolve()
        / "robots/fr3/fr3.urdf.xacro"
    )
    kinematics = Fr3PinocchioKinematics.from_xacro(xacro_path)
    config = SpnavConfig(
        maximum_raw_value=args.maximum_raw_value,
        deadzone=args.deadzone,
        stale_timeout_seconds=args.stale_timeout_seconds,
        open_button=args.open_button,
        close_button=args.close_button,
        device_path=args.device_path,
    )
    backend = SpnavHumanInputBackend(config)

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("saps_physical_m3_spnav_subscriber")
    transform_reader = RosLiveTransformReader(
        node,
        target_frame=args.base_frame,
        source_frame=args.tcp_frame,
    )
    latest: dict[str, Any] = {
        "q": None,
        "source_ros_seconds": None,
        "receive_monotonic_seconds": None,
        "error": None,
        "message_count": 0,
    }

    def joint_callback(message: Any) -> None:
        try:
            latest["q"] = ordered_fr3_joint_positions(
                message.name,
                message.position,
            )
            latest["source_ros_seconds"] = ros_stamp_seconds(
                int(message.header.stamp.sec),
                int(message.header.stamp.nanosec),
            )
            latest["receive_monotonic_seconds"] = time.monotonic()
            latest["message_count"] += 1
            latest["error"] = None
        except (TypeError, ValueError) as error:
            latest["error"] = str(error)

    subscription = node.create_subscription(
        JointState,
        args.joint_state_topic,
        joint_callback,
        qos_profile_sensor_data,
    )

    records = []
    start_utc = _utc_now()
    try:
        initial_transform = transform_reader.wait_until_ready(
            timeout_seconds=args.tf_wait_timeout_seconds,
            maximum_age_seconds=args.maximum_tf_age_seconds,
        )
        deadline = time.monotonic() + args.joint_wait_timeout_seconds
        while latest["q"] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if latest["q"] is None:
            raise TimeoutError(
                "No valid FR3 JointState received for SpaceMouse frame "
                f"transformation on {args.joint_state_topic}; "
                f"last_error={latest['error']!r}."
            )

        backend.start()
        diagnostic_start = time.monotonic()
        period = 1.0 / args.sample_rate_hz
        while time.monotonic() - diagnostic_start < args.duration_seconds:
            cycle_start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            ros_now = node.get_clock().now().nanoseconds / 1e9
            joint_age = ros_now - latest["source_ros_seconds"]
            if joint_age < 0.0 or joint_age > args.maximum_joint_age_seconds:
                raise RuntimeError(
                    f"FR3 JointState age {joint_age:.6f}s is outside the "
                    "configured diagnostic range."
                )
            q = np.asarray(latest["q"], dtype=np.float64)
            live_transform = transform_reader.latest(
                maximum_age_seconds=args.maximum_tf_age_seconds,
            )
            pinocchio_transform = kinematics.forward_kinematics(q)
            base_rotation_tcp = live_transform.rotation_target_source
            rotation_disagreement = _rotation_disagreement_radians(
                base_rotation_tcp,
                pinocchio_transform[:3, :3],
            )
            sample = backend.sample(
                base_rotation_tcp=base_rotation_tcp,
                armed=False,
            )
            records.append(
                _sample_record(
                    len(records),
                    sample,
                    q=q,
                    live_transform=live_transform,
                    base_rotation_tcp=base_rotation_tcp,
                    pinocchio_rotation_tcp=pinocchio_transform[:3, :3],
                    rotation_disagreement_radians=rotation_disagreement,
                    joint_source_ros_seconds=latest["source_ros_seconds"],
                    joint_age_seconds=joint_age,
                )
            )
            print(
                f"raw={sample.last_raw_motion} "
                f"tcp={sample.tcp_frame_normalized.tolist()} "
                f"base={sample.base_frame_normalized.tolist()} "
                f"active={sample.human_input.motion_active} "
                f"stale={sample.human_input.stale_input} "
                f"buttons=({sample.human_input.open_button_pressed},"
                f"{sample.human_input.close_button_pressed})",
                flush=True,
            )
            remaining = period - (time.monotonic() - cycle_start)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        backend.close()
        node.destroy_node()
        rclpy.shutdown()

    actions = np.asarray(
        [record["base_frame_normalized"] for record in records],
        dtype=np.float64,
    )
    active_mask = np.asarray(
        [record["motion_active"] for record in records],
        dtype=bool,
    )
    record = {
        "schema_version": 1,
        "milestone": "physical_pi05_droid_m3",
        "diagnostic_scope": (
            "spnavd, FR3 JointState, /tf, and /tf_static subscribers only; "
            "no evdev open, ROS publisher, service client, action client, "
            "Servo command, robot command, or gripper command"
        ),
        "started_utc": start_utc,
        "completed_utc": _utc_now(),
        "provenance": _provenance(args, kinematics),
        "spnav_contract": {
            "daemon_socket": "/run/spnav.sock",
            "maximum_raw_value": config.maximum_raw_value,
            "deadzone": config.deadzone,
            "deadzone_semantics": (
                "component abs(value/max) < deadzone becomes zero; no "
                "rescaling and no clipping"
            ),
            "axis_mapping": [
                "tcp_x=-spnav_translation_z",
                "tcp_y=+spnav_translation_x",
                "tcp_z=+spnav_translation_y",
                "tcp_roll=-spnav_rotation_z",
                "tcp_pitch=+spnav_rotation_x",
                "tcp_yaw=+spnav_rotation_y",
            ],
            "stale_timeout_seconds": config.stale_timeout_seconds,
            "stale_behavior": "six motion dimensions become zero",
            "open_button": config.open_button,
            "close_button": config.close_button,
            "button_intent": "open=-1, none=0, close=+1; close priority",
            "device_path_inspected_only": config.device_path,
        },
        "frame_contract": {
            "source_frame": args.tcp_frame,
            "common_saps_frame": args.base_frame,
            "orientation_source": "live ROS TF lookup at each sample",
            "tf_readiness": transform_reader.readiness_record(),
            "initial_live_transform": initial_transform.as_dict(),
            "maximum_tf_age_seconds": args.maximum_tf_age_seconds,
            "maximum_tf_pinocchio_rotation_disagreement_radians": max(
                item["tf_pinocchio_rotation_disagreement_radians"]
                for item in records
            ),
            "equation": (
                "h_base = diag(R_base_tcp,R_base_tcp) h_tcp; both vectors "
                "refer to the TCP point, so no translational adjoint term"
            ),
            "normalization": (
                "dimensionless physical Cartesian scales s_t and s_r are "
                "unresolved; this diagnostic applies no physical SI scale"
            ),
            "servo_execution_scales_used": False,
        },
        "ros_contract": {
            "joint_state_topic": args.joint_state_topic,
            "joint_message_count": latest["message_count"],
            "joint_callback_error": latest["error"],
        },
        "distribution": {
            "all_samples": _motion_summary(actions),
            "active_samples": (
                _motion_summary(actions[active_mask])
                if np.any(active_mask)
                else None
            ),
            "active_sample_count": int(np.count_nonzero(active_mask)),
            "stale_sample_count": sum(
                record["stale_input"] for record in records
            ),
            "hidden_gain_tuning": False,
        },
        "samples": records,
        "actuation": {
            "published_topics": [],
            "called_services": [],
            "called_actions": [],
            "robot_commands_issued": 0,
        },
    }
    args.output_dir.mkdir(parents=True, exist_ok=False)
    output_path = args.output_dir / "spnav.json"
    with output_path.open("x", encoding="utf-8") as file:
        json.dump(record, file, indent=2, sort_keys=True, allow_nan=False)
        file.write("\n")
    print(f"Wrote M3 SpaceMouse evidence to {output_path}")


def _sample_record(
    index: int,
    sample: Any,
    *,
    q: np.ndarray,
    live_transform: Any,
    base_rotation_tcp: np.ndarray,
    pinocchio_rotation_tcp: np.ndarray,
    rotation_disagreement_radians: float,
    joint_source_ros_seconds: float,
    joint_age_seconds: float,
) -> dict[str, Any]:
    human = sample.human_input
    return {
        "index": index,
        "sample_monotonic_seconds": human.sample_monotonic_seconds,
        "raw_axes": list(sample.last_raw_motion),
        "mapped_axes": list(human.mapped_axes),
        "tcp_frame_normalized": sample.tcp_frame_normalized.tolist(),
        "base_frame_normalized": sample.base_frame_normalized.tolist(),
        "human_input_action": human.action.tolist(),
        "motion_active": human.motion_active,
        "connected": human.connected,
        "physical_device_connected": human.physical_device_connected,
        "stale_input": human.stale_input,
        "event_age_seconds": sample.event_age_seconds,
        "open_button_pressed": human.open_button_pressed,
        "close_button_pressed": human.close_button_pressed,
        "gripper_command": human.gripper_command,
        "button_events": [
            {
                "button": event.button,
                "pressed": event.pressed,
                "receive_monotonic_seconds": (
                    event.receive_monotonic_seconds
                ),
            }
            for event in sample.button_events
        ],
        "physical_device_error": human.physical_device_error,
        "joint_position_rad": q.tolist(),
        "joint_source_ros_seconds": joint_source_ros_seconds,
        "joint_age_seconds": joint_age_seconds,
        "live_transform": live_transform.as_dict(),
        "base_rotation_tcp": base_rotation_tcp.tolist(),
        "pinocchio_rotation_tcp": pinocchio_rotation_tcp.tolist(),
        "tf_pinocchio_rotation_disagreement_radians": (
            rotation_disagreement_radians
        ),
    }


def _rotation_disagreement_radians(
    first: np.ndarray,
    second: np.ndarray,
) -> float:
    relative = np.asarray(first, dtype=np.float64).T @ np.asarray(
        second,
        dtype=np.float64,
    )
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.arccos(cosine))


def _motion_summary(values: np.ndarray) -> dict[str, Any]:
    motion = np.asarray(values, dtype=np.float64).reshape(-1, 6)
    translation = np.linalg.norm(motion[:, :3], axis=1)
    rotation = np.linalg.norm(motion[:, 3:], axis=1)
    overall = np.linalg.norm(motion, axis=1)
    return {
        "sample_count": int(motion.shape[0]),
        "component_minimum": np.min(motion, axis=0).tolist(),
        "component_maximum": np.max(motion, axis=0).tolist(),
        "translation_norm": _numeric_summary(translation),
        "rotation_norm": _numeric_summary(rotation),
        "overall_motion_norm": _numeric_summary(overall),
        "component_fraction_above_unit_magnitude": float(
            np.mean(np.abs(motion) > 1.0)
        ),
        "action_fraction_with_any_component_above_unit_magnitude": float(
            np.mean(np.any(np.abs(motion) > 1.0, axis=1))
        ),
        "clipping": "none",
    }


def _numeric_summary(values: np.ndarray) -> dict[str, float]:
    return {
        "minimum": float(np.min(values)),
        "maximum": float(np.max(values)),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
    }


def _provenance(args: argparse.Namespace, kinematics: Any) -> dict[str, Any]:
    repository = args.repository_root.resolve()
    franka = args.franka_description_dir.resolve()
    igd = args.igd_control_dir.resolve()
    return {
        "repository": _git_identity(repository),
        "openpi_commit": _git_text(
            repository / "third_party/openpi",
            "rev-parse",
            "HEAD",
        ),
        "franka_description": _git_identity(franka),
        "igd_fr3_control": {
            **_git_identity(igd),
            "expected_commit": IGD_COMMIT,
            "mapping_source": (
                "igd_fr3_control/SpacemouseClass.py at the recorded commit"
            ),
        },
        "kinematics": {
            "backend": "Pinocchio",
            "version": kinematics.backend_version,
            "generated_urdf_sha256": kinematics.urdf_sha256,
        },
        "runtime": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": np.__version__,
        },
    }


def _git_identity(path: Path) -> dict[str, Any]:
    diff = _git_bytes(path, "diff", "--binary", "--no-ext-diff")
    return {
        "path": str(path),
        "commit": _git_text(path, "rev-parse", "HEAD"),
        "status": _git_text(path, "status", "--short").splitlines(),
        "local_diff_sha256": hashlib.sha256(diff).hexdigest(),
    }


def _git_text(path: Path, *arguments: str) -> str:
    return _git_bytes(path, *arguments).decode("utf-8").rstrip()


def _git_bytes(path: Path, *arguments: str) -> bytes:
    return subprocess.run(
        ("git", "-C", str(path), *arguments),
        check=True,
        stdout=subprocess.PIPE,
    ).stdout


def _utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration-seconds", type=float, default=10.0)
    parser.add_argument("--sample-rate-hz", type=float, default=50.0)
    parser.add_argument("--maximum-raw-value", type=float, default=500.0)
    parser.add_argument("--deadzone", type=float, default=0.3)
    parser.add_argument("--stale-timeout-seconds", type=float, default=0.25)
    parser.add_argument("--open-button", type=int, default=0)
    parser.add_argument("--close-button", type=int, default=1)
    parser.add_argument(
        "--device-path",
        default=(
            "/dev/input/by-id/"
            "usb-3Dconnexion_SpaceMouse_Wireless-event-joystick"
        ),
    )
    parser.add_argument("--joint-state-topic", default="/franka/joint_states")
    parser.add_argument(
        "--maximum-joint-age-seconds",
        type=float,
        default=0.25,
    )
    parser.add_argument(
        "--joint-wait-timeout-seconds",
        type=float,
        default=15.0,
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
        default=Path.home() / "franka_ros2_ws/src/franka_description",
    )
    parser.add_argument(
        "--igd-control-dir",
        type=Path,
        default=Path.home() / "franka_ros2_ws/src/igd_fr3_control",
    )
    return parser.parse_args()


if __name__ == "__main__":
    main(_parse_args())
