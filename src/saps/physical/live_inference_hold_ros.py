"""Separate C1-C1 execution lifecycle; P0 remains subscriber-only."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import threading
import time
from typing import Any
import uuid

import numpy as np

from saps.physical.discrete_verifier import limits_from_urdf
from saps.physical.droid_gripper import finalize_gripper_evidence
from saps.physical.live_inference_hold import (
    captured_records, controller_identity, finalize_hold_sequence,
    run_hold_sequence,
)
from saps.physical.live_observation import ObservationFreshness
from saps.physical.live_shadow import (
    CameraPairGate, prepare_live_request, utc_now, validate_live_policy, write_json,
)
from saps.physical.ros_observation import RosPhysicalObservationCollector
from saps.physical.shadow_config import (
    OPENPI_COMMIT, load_shadow_config, observation_contract,
)
from saps.physical.shadow_ros import (
    SubscriptionBoundary, camera_serial_evidence, git_identity,
    node_interface_evidence, validate_graph,
)
from saps.physical.streaming_ros import StreamingBoundary, pinned_timing_helpers
from saps.policies.bounded_websocket import BoundedWebsocketClient
from saps.policies.openpi_droid import OpenPiDroidPolicy

C1B_BASE_COMMIT = "dc5ce5e1ee038d793f7b672cc72a5fffce8d06c2"


def run_inference_hold(args: Any, *, policy_execution: bool = False) -> int:
    """Require explicit execution, preserve failure artifacts, never retry."""
    if (not args.execute or not args.prompt.strip()
            or not 0 <= args.policy_episode_seed <= 0x7fffffff
            or any(not np.isfinite(v) or v <= 0 for v in (
                args.observation_timeout, args.policy_timeout,
                args.application_confirmation_timeout,
            ))):
        raise ValueError("Explicit execution, prompt, seed and finite timeouts required")
    config = load_shadow_config(args.config)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    result: dict[str, Any] = {
        "schema_version": 1, "milestone": "physical_c1c1",
        "run": str(uuid.uuid4()), "status": "startup_failed", "rows": [],
        "start_utc": utc_now(), "policy_actions_executed": 0,
        "gripper_commands_issued": 0, "completed_requests": 0,
        "config": config, "prompt": args.prompt,
        "policy_episode_seed": args.policy_episode_seed,
        "timeouts_seconds": {
            "observation": args.observation_timeout,
            "policy": args.policy_timeout,
            "application_confirmation": args.application_confirmation_timeout,
        },
    }
    if policy_execution:
        result.update(milestone="physical_c1c2", warmup={}, episode={},
                      replans=[], termination={"task_outcome": "not_evaluated"},
                      runtime_health={}, warmup_requests=0,
                      main_policy_requests=0, completed_main_replans=0,
                      policy_actions_scheduled=0, terminal_holds_applied=0)
    boundary = node = executor = transport = thread = collector = None
    gripper = None
    enable_gripper = policy_execution and getattr(args, "enable_gripper", False)
    initialized = False
    analysis_start = 0
    expected_identity = None
    timing = None
    stop = threading.Event()
    observer_errors: list[str] = []
    try:
        root = Path(__file__).resolve().parents[3]
        if enable_gripper:
            from saps.physical.gripper_ros import gripper_timing_helpers
            timing, lab_identity = gripper_timing_helpers(args.lab_stack_dir)
        else:
            timing, lab_identity = pinned_timing_helpers(args.lab_stack_dir)
        provenance = {
            "repository": git_identity(root),
            "openpi": git_identity(root / "third_party/openpi"),
            "fr3_lab_stack": lab_identity,
            "igd_fr3_control": git_identity(args.igd_control_dir),
            "c1b_base_commit": C1B_BASE_COMMIT,
            "clock": timing.clock_provenance(),
        }
        result["provenance"] = provenance
        write_json(args.output_dir / "provenance.json", provenance)
        if (provenance["openpi"]["commit"] != OPENPI_COMMIT
                or provenance["openpi"]["dirty"]):
            raise ValueError("OpenPI must be clean at the frozen pin")
        subprocess.run(["git", "-C", str(root), "merge-base", "--is-ancestor",
                        C1B_BASE_COMMIT, "HEAD"], check=True, timeout=10)
        write_json(args.output_dir / "start.json", result)
        query = subprocess.run(
            ["ros2", "param", "get", "--hide-type",
             "/robot_state_publisher", "robot_description"],
            check=True, capture_output=True, text=True, timeout=20,
        )
        xml = query.stdout.strip()
        limits = limits_from_urdf(xml)
        (args.output_dir / "robot_description.urdf").write_text(xml)
        write_json(args.output_dir / "joint_limits.json", limits)

        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image, JointState

        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node("saps_physical_c1c2_observation" if policy_execution
                                 else "saps_physical_c1c1_observation",
                                 enable_rosout=False,
                                 start_parameter_services=False)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        collector = RosPhysicalObservationCollector(
            SubscriptionBoundary(node), observation_contract(config),
            prompt=args.prompt,
            freshness=ObservationFreshness(**config["freshness"]),
            preserve_native_images=True, joint_state_type=JointState,
            image_type=Image, qos_profile=QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        if enable_gripper:
            from saps.physical.gripper_ros import create_gripper
            gripper = create_gripper(
                node, collector, config, args.lab_stack_dir,
                speed=args.gripper_speed, timeout=args.gripper_timeout)
        allowed_service_clients = (
            frozenset({"/franka_gripper/stop"})
            if gripper is not None else frozenset()
        )
        deadline = time.monotonic() + args.observation_timeout
        while True:
            executor.spin_once(timeout_sec=0.05)
            try:
                result["initial_ros_graph"] = validate_graph(node, config)
                if gripper is not None and not gripper.hand.ready():
                    raise RuntimeError("Gripper move/grasp/stop endpoints unavailable")
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
        result["initial_observation_interfaces"] = node_interface_evidence(
            node, allowed_service_clients=allowed_service_clients,
        )
        result["camera_identity"] = camera_serial_evidence(config)
        transport = BoundedWebsocketClient(args.host, args.port, args.policy_timeout)
        policy = OpenPiDroidPolicy(client=transport)
        validate_live_policy(policy, result)
        boundary = StreamingBoundary(result["run"], provenance["clock"])
        time.sleep(2.0)  # C1-B discovery convention; no commands.
        with boundary.lock:
            analysis_start = len(boundary.records)
        # Capture a baseline controller activation before either hold.
        deadline = time.monotonic() + args.application_confirmation_timeout
        while True:
            try:
                expected_identity = controller_identity(
                    captured_records(boundary, analysis_start),
                )
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)
        result["expected_controller_identity"] = expected_identity
        if not policy_execution:
            observation, sample_dir = prepare_live_request(
                collector=collector, output_dir=args.output_dir, index=0,
                policy_episode_seed=args.policy_episode_seed,
                observation_timeout=args.observation_timeout,
                spin_once=lambda: executor.spin_once(timeout_sec=0.05),
                config=config, record=result, gate=CameraPairGate(),
            )

        def observe() -> None:
            try:
                while not stop.is_set():
                    executor.spin_once(timeout_sec=0.01)
            except Exception as error:
                observer_errors.append(f"{type(error).__name__}: {error}")

        # Transfer this executor to one background thread after assembly.
        # StreamingBoundary has its own executor thread; neither blocks on infer.
        thread = threading.Thread(target=observe, daemon=True)
        thread.start()
        if policy_execution:
            from saps.physical.policy_execution import run_policy_episode

            def check_runtime() -> None:
                if observer_errors or collector.errors or boundary.errors:
                    raise RuntimeError("Physical callback or boundary error")

            def check_observers() -> None:
                check_runtime()
                if validate_graph(node, config) != result["initial_ros_graph"]:
                    raise RuntimeError("Source publisher endpoints changed")
                time.sleep(.01)

            run_policy_episode(
                boundary=boundary, collector=collector, policy=policy,
                config=config, output_dir=args.output_dir, limits=limits,
                analyzer=timing.analyze, expected_identity=expected_identity,
                analysis_start=analysis_start,
                application_timeout=args.application_confirmation_timeout,
                observation_timeout=args.observation_timeout,
                policy_episode_seed=args.policy_episode_seed,
                warmup_policy_seed=args.warmup_policy_seed,
                max_replans=args.max_replans,
                max_executed_policy_chunks=args.max_executed_policy_chunks,
                stop_after_inference_replan=getattr(
                    args, "stop_after_inference_replan", None),
                spin_once=check_observers,
                check_runtime=check_runtime,
                ros_now=lambda: node.get_clock().now().nanoseconds / 1e9,
                result=result, gripper=gripper,
                gripper_transition_timeout=args.gripper_timeout,
            )
        else:
            run_hold_sequence(
                boundary=boundary, limits=limits, analyzer=timing.analyze,
                expected_identity=expected_identity, analysis_start=analysis_start,
                application_timeout=args.application_confirmation_timeout,
                observation_timeout=args.observation_timeout, result=result,
                inference_arguments=dict(
                    observation=observation, sample_dir=sample_dir, policy=policy,
                    index=0, policy_episode_seed=args.policy_episode_seed,
                    ros_now=lambda: node.get_clock().now().nanoseconds / 1e9,
                    config=config,
                ),
            )
        result["final_ros_graph"] = validate_graph(node, config)
        result["final_observation_interfaces"] = node_interface_evidence(
            node, allowed_service_clients=allowed_service_clients,
        )
        result["final_camera_identity"] = camera_serial_evidence(config)
        if result["final_ros_graph"] != result["initial_ros_graph"]:
            raise RuntimeError("Source publisher endpoints changed")
    except (Exception, KeyboardInterrupt) as error:
        result.update(status="failed", error=f"{type(error).__name__}: {error}")
    finally:
        cleanup_errors = []
        if gripper is not None:
            gripper.stop()
            deadline = time.monotonic() + args.gripper_timeout
            try:
                while not gripper.hand.settled and time.monotonic() < deadline:
                    if thread is not None and thread.is_alive():
                        time.sleep(.01)
                    else:
                        executor.spin_once(timeout_sec=.01)
            except Exception as error:
                cleanup_errors.append(f"gripper_cleanup: {error}")
            finalize_gripper_evidence(result, gripper)
            if not gripper.hand.settled or result["gripper"]["error"]:
                cleanup_errors.append("gripper_stop_unconfirmed_or_failed")
        stop.set()
        if thread is not None:
            thread.join(timeout=5)
            if thread.is_alive():
                cleanup_errors.append("observation_executor_did_not_stop")
        if boundary is not None:
            try:
                boundary.close()
            except RuntimeError as error:
                cleanup_errors.append(str(error))
            records = captured_records(boundary, 0)
            with (args.output_dir / "timing.jsonl").open("x") as output:
                for record in records:
                    output.write(json.dumps(record, allow_nan=False) + "\n")
            result["analysis_start_record"] = analysis_start
            try:
                if (policy_execution and expected_identity is not None
                        and result["rows"]):
                    from saps.physical.policy_execution import (
                        audit_inference_hold, validate_execution_delivery,
                    )
                    eligible = [row for row in result["rows"]
                                if row["safety_gate"]["accepted"]]
                    validation = validate_execution_delivery(
                        records[analysis_start:], eligible, timing.analyze,
                        expected_identity,
                    )
                    validation["safety_rejected_unpublished_targets"] = (
                        len(result["rows"]) - len(eligible))
                    result["runtime_health"]["final_delivery_validation"] = validation
                    if not validation["accepted"]:
                        result["status"] = "failed"
                    for replan in result["replans"]:
                        if "hold_audit" in replan["inference"]:
                            audit = audit_inference_hold(
                                records[analysis_start:], replan["pre_hold"],
                                replan["inference"],
                            )
                            replan["inference"]["hold_audit"] = audit
                            if not audit["accepted"]:
                                result["status"] = "failed"
                elif not policy_execution and expected_identity is not None:
                    finalize_hold_sequence(
                        records=records[analysis_start:], result=result,
                        analyzer=timing.analyze, expected_identity=expected_identity,
                    )
            except (KeyError, TypeError, ValueError, RuntimeError) as error:
                result["timing_analysis_error"] = str(error)
                result["status"] = "failed"
            result["boundary_errors"] = dict(boundary.errors)
            if boundary.errors:
                result["status"] = "failed"
        if collector is not None:
            result["source_rates"] = collector.source_rates()
            result["observation_callback_errors"] = dict(collector.errors)
            if collector.errors:
                result["status"] = "failed"
        result["observation_executor_errors"] = observer_errors
        result["cleanup_errors"] = cleanup_errors
        if cleanup_errors or observer_errors:
            result["status"] = "failed"
        try:
            if transport is not None:
                transport.close()
            if executor is not None:
                executor.shutdown(timeout_sec=5)
            if node is not None:
                node.destroy_node()
            if initialized:
                rclpy.shutdown()
        except Exception as error:
            cleanup_errors.append(str(error))
            result["status"] = "failed"
        result["end_utc"] = utc_now()
        if policy_execution:
            result["runtime_validation_status"] = (
                "passed" if result["status"] == "success" else "failed")
            if result["status"] != "success":
                result["termination"]["termination_reason"] = "runtime_abort"
        write_json(args.output_dir / "run.json", result)
    return 0 if result["status"] == "success" else 1
