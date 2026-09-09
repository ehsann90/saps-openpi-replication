"""Live P1-A lifecycle: subscriptions, inference and candidate artifacts only."""

from __future__ import annotations

from pathlib import Path
import subprocess
import threading
import time
from typing import Any

import numpy as np

from saps.physical.discrete_verifier import emulate_chunk, limits_from_urdf
from saps.physical.live_observation import ObservationFreshness
from saps.physical.live_shadow import (
    CameraPairGate, observation_record, utc_now, validate_request, write_json,
    ZERO_ACTUATION,
)
from saps.physical.ros_observation import RosPhysicalObservationCollector
from saps.physical.shadow_audit import save_model_audit
from saps.physical.shadow_config import (
    load_shadow_config, observation_contract, OPENPI_COMMIT,
    POLICY_CONFIG, POLICY_CHECKPOINT,
)
from saps.physical.shadow_ros import (
    camera_serial_evidence, git_identity, node_interface_evidence, validate_graph,
)
from saps.physical.verifier_observation import ContinuousBoundary, spin_continuously
from saps.policies.bounded_websocket import BoundedWebsocketClient
from saps.policies.openpi_droid import OpenPiDroidPolicy


def inspect_controller(output: Path) -> str:
    """Read-only CLI queries; no controller switch or actuator clients."""
    queries = {
        "controllers": ["control", "list_controllers", "-v"],
        "hardware_interfaces": ["control", "list_hardware_interfaces"],
        "controller_parameters": ["param", "dump", "/fr3_arm_controller"],
        "manager_rate": ["param", "get", "/controller_manager", "update_rate"],
        "manager_limit_enforcement": ["param", "get", "/controller_manager", "enforce_command_limits"],
        "controller_endpoints": ["node", "info", "/fr3_arm_controller"],
        "command_topic": ["topic", "info", "/fr3_arm_controller/joint_trajectory", "-v"],
        "trajectory_action": ["action", "info", "/fr3_arm_controller/follow_joint_trajectory"],
        "robot_description": ["param", "get", "--hide-type",
                              "/robot_state_publisher", "robot_description"],
    }
    records = {}
    for name, args in queries.items():
        result = subprocess.run(["ros2", *args], text=True, capture_output=True,
                                timeout=20, check=False)
        records[name] = {"command": ["ros2", *args], "returncode": result.returncode,
                         "stdout": result.stdout, "stderr": result.stderr}
        if result.returncode:
            write_json(output / "controller_inspection.json", records)
            raise RuntimeError(f"Controller inspection failed: {name}")
    write_json(output / "controller_inspection.json", records)
    xml = records["robot_description"]["stdout"].strip()
    (output / "robot_description.urdf").write_text(xml, encoding="utf-8")
    return xml


def verify_requests(args: Any, config: dict[str, Any], collector: Any,
                    boundary: ContinuousBoundary, node: Any, policy: Any,
                    limits: dict[str, Any], record: dict[str, Any]) -> None:
    policy.validate_policy_identity(config_name=POLICY_CONFIG,
                                    checkpoint=POLICY_CHECKPOINT)
    metadata = policy.server_metadata
    if (metadata.get("saps_model_input_audit", {}).get("openpi_commit") != OPENPI_COMMIT
            or metadata.get("saps_seeded_sampling", {}).get("action_horizon") != 15):
        raise ValueError("Expected pinned pi05-DROID with 15-action horizon.")
    record["server_metadata"] = metadata
    gate = CameraPairGate()
    ros_now = lambda: node.get_clock().now().nanoseconds / 1e9
    snapshot = lambda: boundary.snapshot(collector)
    for index in range(args.requests):
        deadline = time.monotonic() + args.observation_timeout
        while True:
            state = snapshot()
            signature = state.latest_signature()
            if gate.accepts(signature) and not state.errors:
                try:
                    observation = state.assemble()
                    break
                except ValueError as error:
                    record["last_rejected_observation"] = str(error)
            if time.monotonic() >= deadline:
                raise TimeoutError("No fresh advancing physical observation.")
            time.sleep(0.005)
        gate.commit(signature)
        validate_request(observation.policy_input)
        for frame in (observation.wrist_frame, observation.exterior_frame):
            if (list(frame.native_shape) != config["native_images"]["shape"]
                    or frame.source_encoding.lower() != config["native_images"]["encoding"]):
                raise ValueError("Camera profile differs from P0 contract.")
        directory = args.output_dir / f"request_{index:04d}"
        directory.mkdir()
        # These immutable arrays survive later callback replacements.
        request_start, request_ros = time.monotonic(), ros_now()
        if not 0 <= request_ros - observation.timing.oldest_source_ros_seconds <= config["freshness"]["maximum_source_age_seconds"]:
            raise ValueError("Observation expired before inference.")
        try:
            response = policy.infer(observation.policy_input,
                                    policy_episode_seed=args.policy_episode_seed,
                                    replan_index=index, audit_model_input=index == 0)
        finally:
            response_end = time.monotonic()
            np.savez_compressed(directory / "observation.npz", **observation.policy_input)
            write_json(directory / "request.json", {
                **observation_record(observation), "replan_index": index,
                "policy_episode_seed": args.policy_episode_seed,
                "request_started_monotonic_seconds": request_start,
                "request_started_ros_seconds": request_ros,
                "call_ended_monotonic_seconds": response_end,
            })
            write_json(directory / "inference_continuity.json",
                       boundary.continuity(request_start, response_end))
        np.savez_compressed(directory / "actions.npz", actions=response.actions)
        write_json(directory / "response.json", {
            "client_round_trip_seconds": response.client_round_trip_seconds,
            "policy_timing": response.policy_timing,
            "server_timing": response.server_timing,
            "sampling_metadata": response.sampling_metadata,
            "response_completed_monotonic_seconds": response_end,
        })
        if response.actions.shape != (15, 8):
            raise ValueError("Expected [15,8] policy response.")
        if index == 0:
            write_json(directory / "model_audit.json", save_model_audit(
                response.model_input_audit, observation.policy_input,
                directory / "model_audit"))
        rows = emulate_chunk(
            response.actions, snapshot=snapshot, limits=limits,
            request_start=request_start, response_end=response_end,
            observation_ros=observation.timing.oldest_source_ros_seconds,
            ros_now=ros_now, max_state_age=args.max_state_age,
            max_action_age=args.max_action_age, max_lateness=args.max_lateness,
        )
        write_json(directory / "candidates.json", rows)
        record["completed_requests"] = index + 1
        record["candidate_actions"] += len(rows)
        record["accepted_candidates"] += sum(row["verifier_accepted"] for row in rows)
        print(f"P1-A chunk {index}: {len(rows)} candidates, "
              f"{sum(row['verifier_accepted'] for row in rows)} accepted by "
              "diagnostic gates, executed=0", flush=True)


def run_verifier(args: Any) -> None:
    if (args.requests <= 0 or args.requests > 100
            or not args.prompt.strip()
            or not 0 <= args.policy_episode_seed <= 0x7fffffff
            or any(not np.isfinite(v) or v <= 0 for v in (
                args.observation_timeout, args.policy_timeout,
                args.max_state_age, args.max_action_age, args.max_lateness))):
        raise ValueError("Require 1..100 requests, prompt, seed and finite positive bounds.")
    config = load_shadow_config(args.config)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    record = {"schema_version": 1, "milestone": "physical_pi05_droid_p1a",
              "start_utc": utc_now(), "run_id": args.output_dir.name,
              "actuation": dict(ZERO_ACTUATION), "execution_enabled": False,
              "config": config, "requested_requests": args.requests,
              "completed_requests": 0, "candidate_actions": 0,
              "accepted_candidates": 0, "frequency_hz": 15,
              "reference_horizon": 8,
              "diagnostic_gates_seconds": {
                  "state_age": args.max_state_age,
                  "action_age_since_request": args.max_action_age,
                  "schedule_lateness": args.max_lateness}}
    node = transport = collector = boundary = thread = None
    observation_started = None
    stop = threading.Event()
    initialized = False
    try:
        root = Path(__file__).resolve().parents[3]
        record["provenance"] = {"repository": git_identity(root),
                                "openpi": git_identity(root / "third_party/openpi")}
        if record["provenance"]["openpi"]["commit"] != OPENPI_COMMIT:
            raise ValueError("Unexpected OpenPI submodule.")
        write_json(args.output_dir / "start.json", record)
        limits = limits_from_urdf(inspect_controller(args.output_dir))
        write_json(args.output_dir / "joint_limits.json", limits)
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image, JointState

        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node("saps_physical_pi05_p1a", enable_rosout=False,
                                 start_parameter_services=False)
        boundary = ContinuousBoundary(node)
        collector = RosPhysicalObservationCollector(
            boundary, observation_contract(config), prompt=args.prompt,
            freshness=ObservationFreshness(**config["freshness"]),
            preserve_native_images=False, joint_state_type=JointState,
            image_type=Image, qos_profile=QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST))
        thread = threading.Thread(target=spin_continuously,
                                  args=(node, boundary, stop), daemon=True)
        observation_started = time.monotonic()
        thread.start()
        deadline = time.monotonic() + args.observation_timeout
        while True:
            try:
                record["initial_graph"] = validate_graph(node, config)
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)
        record["camera_identity"] = camera_serial_evidence(config)
        record["initial_node_interfaces"] = node_interface_evidence(node)
        transport = BoundedWebsocketClient(args.host, args.port, args.policy_timeout)
        verify_requests(args, config, collector, boundary, node,
                        OpenPiDroidPolicy(client=transport), limits, record)
        record["final_graph"] = validate_graph(node, config)
        record["final_node_interfaces"] = node_interface_evidence(node)
        if record["initial_graph"] != record["final_graph"]:
            raise RuntimeError("Physical source endpoints changed.")
        record["termination_reason"] = "request_count_reached"
    except BaseException as error:
        record["termination_reason"] = "interrupted" if isinstance(error, KeyboardInterrupt) else "error"
        record["error"] = {"type": type(error).__name__, "message": str(error)}
        raise
    finally:
        stop.set()
        if thread is not None:
            thread.join(timeout=5)
        ended = time.monotonic()
        record["end_utc"] = utc_now()
        record["wall_clock_duration_seconds"] = ended - started
        if boundary is not None and observation_started is not None:
            write_json(args.output_dir / "continuity.json", boundary.continuity(observation_started, ended))
        try:
            write_json(args.output_dir / "run.json", record)
        finally:
            if transport is not None:
                transport.close()
            if node is not None:
                node.destroy_node()
            if initialized:
                rclpy.shutdown()
