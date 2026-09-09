"""Read-only graph preflight and finite P0 process lifecycle."""

from __future__ import annotations

import os
from pathlib import Path
import platform
import subprocess
import sys
import time
from typing import Any

import cv2
import numpy as np

from saps.physical.live_observation import ObservationFreshness
from saps.physical.live_shadow import run_live_loop
from saps.physical.live_shadow import utc_now
from saps.physical.live_shadow import write_json
from saps.physical.live_shadow import ZERO_ACTUATION
from saps.physical.ros_observation import RosPhysicalObservationCollector
from saps.physical.shadow_config import load_shadow_config
from saps.physical.shadow_config import observation_contract
from saps.physical.shadow_config import OPENPI_COMMIT
from saps.policies.bounded_websocket import BoundedWebsocketClient
from saps.policies.openpi_droid import OpenPiDroidPolicy


class SubscriptionBoundary:
    """Expose only subscription and clock capabilities to the collector."""

    def __init__(self, node: Any) -> None:
        self.__node = node

    def create_subscription(self, *args: Any, **kwargs: Any) -> Any:
        return self.__node.create_subscription(*args, **kwargs)

    def get_clock(self) -> Any:
        return self.__node.get_clock()


def git_identity(path: Path) -> dict[str, Any]:
    """Record unrelated checkout state without requiring or making it clean."""

    if not path.is_dir():
        return {"path": str(path), "available": False}

    def git(*arguments: str) -> str:
        return subprocess.check_output(
            ["git", "-C", str(path), *arguments], text=True, timeout=10,
        ).strip()

    return {
        "path": str(path), "available": True,
        "branch": git("branch", "--show-current"),
        "commit": git("rev-parse", "HEAD"),
        "status": git("status", "--short").splitlines(),
        "dirty": bool(git("status", "--porcelain")),
    }


def validate_graph(node: Any, config: dict[str, Any]) -> dict[str, Any]:
    """Verify unique topic publishers; associate camera endpoints with nodes."""

    sources = {
        "joint_state": (config["robot"]["joint_state_topic"], "sensor_msgs/msg/JointState"),
        "gripper_state": (config["robot"]["gripper_state_topic"], "sensor_msgs/msg/JointState"),
        "wrist_camera": (config["wrist_camera"]["topic"], "sensor_msgs/msg/Image"),
        "exterior_camera": (config["exterior_camera"]["topic"], "sensor_msgs/msg/Image"),
    }
    result = {}
    for role, (topic, message_type) in sources.items():
        publishers = node.get_publishers_info_by_topic(topic)
        if len(publishers) != 1:
            raise RuntimeError(f"{topic} requires exactly one publisher; got {len(publishers)}.")
        publisher = publishers[0]
        name = publisher.node_namespace.rstrip("/") + "/" + publisher.node_name
        if publisher.topic_type != message_type:
            raise RuntimeError(f"Wrong message type for {topic}.")
        if role.endswith("camera") and name != config[role]["node"]:
            raise RuntimeError(f"Camera publisher {name} differs from configured {role} node.")
        result[role] = {
            "topic": topic, "node": name, "type": publisher.topic_type,
            "publisher_gid": list(publisher.endpoint_gid),
        }
    return result


def camera_serial_evidence(config: dict[str, Any]) -> dict[str, Any]:
    """Read camera serial parameters only; no robot service is contacted.

    ROS Image carries no device serial. A configured label alone cannot prove
    identity. Pair these read-only parameter results with graph endpoint
    binding; the RealSense driver selects the physical device by serial_no.
    """

    evidence = {}
    for role in ("wrist_camera", "exterior_camera"):
        camera = config[role]
        command = ["ros2", "param", "get", "--hide-type", camera["node"], "serial_no"]
        result = subprocess.run(
            command, check=True, capture_output=True, text=True, timeout=15,
        )
        raw = result.stdout.strip()
        # RealSense launch uses one leading underscore to prevent YAML from
        # interpreting the serial as an integer; the driver removes it.
        serial = raw[1:] if raw.startswith("_") else raw
        if serial != camera["serial"]:
            raise RuntimeError(f"{role} serial_no={raw!r}, expected {camera['serial']!r}.")
        evidence[role] = {
            "command": command, "raw_parameter": raw, "serial": serial,
            "read_utc": utc_now(), "identity_source": "RealSense serial_no parameter",
        }
    return evidence


def node_interface_evidence(node: Any) -> dict[str, Any]:
    """Inspect locally owned ROS interfaces, allowing only parameter events."""

    topics = [publisher.topic_name for publisher in node.publishers]
    services = [client.srv_name for client in node.clients]
    if any(topic != "/parameter_events" for topic in topics) or services:
        raise RuntimeError("Unexpected output/client interface in P0 subscriber node.")
    return {
        "published_topics": topics,
        "service_clients": services,
        "robot_command_publishers": 0,
    }


def run_shadow(args: Any) -> None:
    """Allocate a unique run, preflight, infer finitely, and always finalize."""

    if (not isinstance(args.prompt, str) or not args.prompt.strip()
            or args.requests <= 0 or not 0 <= args.policy_episode_seed <= 0x7fffffff
            or any(not np.isfinite(value) or value <= 0 for value in (
                args.observation_timeout, args.policy_timeout,
            ))):
        raise ValueError("Positive finite counts/timeouts and a valid episode seed are required.")
    config = load_shadow_config(args.config)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    record: dict[str, Any] = {
        "schema_version": 1, "milestone": "physical_pi05_droid_p0",
        "run_id": args.output_dir.name, "start_utc": utc_now(),
        "execution_enabled": False, "actuation": ZERO_ACTUATION,
        "config": config, "prompt": args.prompt,
        "policy_episode_seed": args.policy_episode_seed,
        "requested_requests": args.requests, "completed_requests": 0,
        "timeouts_seconds": {
            "observation": args.observation_timeout, "policy": args.policy_timeout,
        },
        "runtime": {
            "ros_distro": os.environ.get("ROS_DISTRO"), "python": sys.version,
            "platform": platform.platform(), "numpy": np.__version__, "opencv": cv2.__version__,
        },
    }
    node = None
    transport = None
    collector = None
    ros_initialized = False
    try:
        repository = Path(__file__).resolve().parents[3]
        record["provenance"] = {
            "repository": git_identity(repository),
            "openpi": git_identity(repository / "third_party/openpi"),
            "fr3_lab_stack": git_identity(args.lab_stack_dir),
            "igd_fr3_control": {
                **git_identity(args.igd_control_dir),
                "expected_branch": "commissioning/fr3-spacemouse",
            },
        }
        if record["provenance"]["openpi"]["commit"] != OPENPI_COMMIT:
            raise ValueError("OpenPI submodule does not match the frozen P0 pin.")
        write_json(args.output_dir / "start.json", record)
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image, JointState

        rclpy.init(args=[])
        ros_initialized = True
        node = rclpy.create_node(
            "saps_physical_pi05_shadow", enable_rosout=False,
            start_parameter_services=False,
        )
        # Depth one avoids replaying a backlog accumulated during inference.
        collector = RosPhysicalObservationCollector(
            SubscriptionBoundary(node), observation_contract(config), prompt=args.prompt,
            freshness=ObservationFreshness(**config["freshness"]),
            preserve_native_images=True,
            joint_state_type=JointState, image_type=Image,
            qos_profile=QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        record["initial_node_interfaces"] = node_interface_evidence(node)
        deadline = time.monotonic() + args.observation_timeout
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                record["initial_ros_graph"] = validate_graph(node, config)
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
        record["camera_identity"] = camera_serial_evidence(config)
        transport = BoundedWebsocketClient(args.host, args.port, args.policy_timeout)
        policy = OpenPiDroidPolicy(client=transport)
        run_live_loop(
            collector=collector, policy=policy, output_dir=args.output_dir,
            request_count=args.requests, policy_episode_seed=args.policy_episode_seed,
            observation_timeout=args.observation_timeout,
            spin_once=lambda: rclpy.spin_once(node, timeout_sec=0.05),
            ros_now=lambda: node.get_clock().now().nanoseconds / 1e9,
            config=config, record=record,
        )
        record["final_ros_graph"] = validate_graph(node, config)
        record["final_node_interfaces"] = node_interface_evidence(node)
        record["final_camera_identity"] = camera_serial_evidence(config)
        if record["final_ros_graph"] != record["initial_ros_graph"]:
            raise RuntimeError("Source publisher endpoints changed during the run.")
        record["termination_reason"] = "request_count_reached"
    except BaseException as error:
        record["termination_reason"] = (
            "interrupted" if isinstance(error, KeyboardInterrupt) else "error"
        )
        record["error"] = {"type": type(error).__name__, "message": str(error)}
        raise
    finally:
        # Finalize before cleanup so an interrupted cleanup cannot erase evidence.
        record["end_utc"] = utc_now()
        record["wall_clock_duration_seconds"] = time.monotonic() - started
        if collector is not None:
            record["source_rates"] = collector.source_rates()
            record["callback_errors"] = collector.errors
        try:
            write_json(args.output_dir / "run.json", record)
        finally:
            try:
                if transport is not None:
                    transport.close()
            finally:
                if node is not None:
                    node.destroy_node()
                if ros_initialized:
                    rclpy.shutdown()
