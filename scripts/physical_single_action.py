#!/usr/bin/env python3
"""One live pi05-DROID action connected to the FR3 joint-target server.

This P1-B entry point is deliberately plan-only. It performs one policy
inference, selects action index zero, anchors the DROID delta to one fresh
post-inference FR3 joint sample, applies the SAPS safety gate, sends at most one
absolute target to /fr3_joint_target, and terminates. The joint-target server
must report execute=false; otherwise the run is rejected before any goal is
sent.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np

from saps.physical.discrete_verifier import limits_from_urdf
from saps.physical.live_observation import ObservationFreshness
from saps.physical.live_shadow import (
    CameraPairGate,
    observation_record,
    utc_now,
    validate_request,
    write_json,
)
from saps.physical.ros_observation import RosPhysicalObservationCollector
from saps.physical.shadow_audit import save_model_audit
from saps.physical.shadow_config import (
    OPENPI_COMMIT,
    POLICY_CHECKPOINT,
    POLICY_CONFIG,
    load_shadow_config,
    observation_contract,
)
from saps.physical.shadow_ros import (
    camera_serial_evidence,
    git_identity,
    node_interface_evidence,
    validate_graph,
)
from saps.physical.single_action import (
    ACTION_AGE,
    STATE_AGE,
    first_action_from_chunk,
    safety_gate,
)
from saps.physical.single_action_ros import JointTargetActionClient
from saps.physical.verifier_observation import (
    ContinuousBoundary,
    spin_continuously,
)
from saps.physical.verifier_ros import inspect_controller
from saps.policies.bounded_websocket import BoundedWebsocketClient
from saps.policies.openpi_droid import OpenPiDroidPolicy


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/physical_pi05_fr3.json"),
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--prompt", required=True)
    parser.add_argument(
        "--policy-episode-seed",
        type=int,
        default=20260827,
    )
    parser.add_argument("--observation-timeout", type=float, default=30.0)
    parser.add_argument("--policy-timeout", type=float, default=120.0)
    parser.add_argument("--joint-target-timeout", type=float, default=15.0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--lab-stack-dir",
        type=Path,
        default=Path.home() / "franka_ros2_ws/src/fr3_lab_stack",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help=(
            "Explicitly permit exactly one physical trajectory execution; "
            "the joint-target server must independently report execute=true."
        ),
    )
    return parser.parse_args()


def _wait_for_observation(
    *,
    collector: Any,
    boundary: ContinuousBoundary,
    timeout: float,
) -> Any:
    gate = CameraPairGate()
    deadline = time.monotonic() + timeout
    last_error = None

    while time.monotonic() < deadline:
        state = boundary.snapshot(collector)
        signature = state.latest_signature()
        if gate.accepts(signature) and not state.errors:
            try:
                observation = state.assemble()
            except ValueError as error:
                last_error = str(error)
            else:
                gate.commit(signature)
                return observation
        time.sleep(0.005)

    raise TimeoutError(
        "No fresh complete physical observation before timeout; "
        f"missing={collector.missing_sources()}, "
        f"errors={collector.errors}, last_assembly_error={last_error!r}."
    )


def _wait_for_post_inference_joint(
    *,
    collector: Any,
    boundary: ContinuousBoundary,
    previous_ros_seconds: float,
    response_end: float,
    request_start: float,
    timeout: float,
) -> Any:
    deadline = min(
        time.monotonic() + timeout,
        request_start + ACTION_AGE,
    )
    latest = None

    while time.monotonic() < deadline:
        state = boundary.snapshot(collector)
        joint = state.latest_joint
        latest = joint
        if (
            joint is not None
            and joint.stamp.ros_seconds > previous_ros_seconds
            and joint.stamp.receive_monotonic_seconds >= response_end
        ):
            return state
        time.sleep(0.002)

    stamp = None if latest is None else latest.stamp.ros_seconds
    raise TimeoutError(
        "No advancing post-inference FR3 joint sample before the policy "
        f"action expired; latest_joint_stamp={stamp}."
    )


def _plan_only_result_ok(result: dict[str, Any]) -> bool:
    evidence = result["server_evidence"]
    return (
        result["success"] is True
        and result["outcome"] == "plan_only_validated"
        and result["planning_error_code"] == 1
        and result["execution_attempted"] is False
        and evidence.get("execution_attempts") == 0
    )


def run(args: argparse.Namespace) -> None:
    if (
        not isinstance(args.prompt, str)
        or not args.prompt.strip()
        or not 0 <= args.policy_episode_seed <= 0x7FFFFFFF
        or any(
            not np.isfinite(value) or value <= 0
            for value in (
                args.observation_timeout,
                args.policy_timeout,
                args.joint_target_timeout,
            )
        )
    ):
        raise ValueError(
            "Require a prompt, valid policy seed, and finite positive timeouts."
        )

    config = load_shadow_config(args.config)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    record: dict[str, Any] = {
        "schema_version": 1,
        "milestone": "physical_pi05_droid_p1b_single_action",
        "run_id": args.output_dir.name,
        "start_utc": utc_now(),
        "execution_enabled": bool(args.execute),
        "selected_action_index": 0,
        "policy_actions_selected": 0,
        "policy_actions_executed": 0,
        "gripper_commands_issued": 0,
        "joint_target_goal_calls_started": 0,
        "joint_target_results_received": 0,
        "prompt": args.prompt,
        "policy_episode_seed": args.policy_episode_seed,
        "safety_thresholds": {
            "state_age_seconds": STATE_AGE,
            "action_age_seconds": ACTION_AGE,
        },
    }

    node = None
    collector = None
    boundary = None
    observation_thread = None
    policy_transport = None
    joint_target_client = None
    initialized = False
    stop = threading.Event()

    try:
        root = Path(__file__).resolve().parents[1]
        record["provenance"] = {
            "repository": git_identity(root),
            "openpi": git_identity(root / "third_party/openpi"),
            "fr3_lab_stack": git_identity(args.lab_stack_dir),
        }
        if record["provenance"]["openpi"]["commit"] != OPENPI_COMMIT:
            raise ValueError("Unexpected OpenPI submodule commit.")

        write_json(args.output_dir / "start.json", record)

        limits = limits_from_urdf(inspect_controller(args.output_dir))
        write_json(args.output_dir / "joint_limits.json", limits)

        import rclpy
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image, JointState

        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node(
            "saps_physical_pi05_p1b_single_action",
            enable_rosout=False,
            start_parameter_services=False,
        )
        boundary = ContinuousBoundary(node)
        collector = RosPhysicalObservationCollector(
            boundary,
            observation_contract(config),
            prompt=args.prompt,
            freshness=ObservationFreshness(**config["freshness"]),
            preserve_native_images=False,
            joint_state_type=JointState,
            image_type=Image,
            qos_profile=QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        observation_thread = threading.Thread(
            target=spin_continuously,
            args=(node, boundary, stop),
            daemon=True,
        )
        observation_thread.start()

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

        # Create and preflight the action boundary before inference so policy
        # age is not spent discovering a server. This still sends no goal.
        joint_target_client = JointTargetActionClient()
        server_execute = joint_target_client.server_execute_enabled(timeout=5.0)
        record["joint_target_server_execute"] = server_execute
        if server_execute != args.execute:
            raise RuntimeError(
                "P1-B execution-mode mismatch: "
                f"script --execute={args.execute}, "
                f"joint-target server execute={server_execute}."
            )

        policy_transport = BoundedWebsocketClient(
            args.host,
            args.port,
            args.policy_timeout,
        )
        policy = OpenPiDroidPolicy(client=policy_transport)
        policy.validate_policy_identity(
            config_name=POLICY_CONFIG,
            checkpoint=POLICY_CHECKPOINT,
        )
        metadata = policy.server_metadata
        if (
            metadata.get("saps_model_input_audit", {}).get("openpi_commit")
            != OPENPI_COMMIT
            or metadata.get("saps_seeded_sampling", {}).get("action_horizon")
            != 15
        ):
            raise ValueError(
                "Expected pinned pi05-DROID with a 15-action horizon."
            )
        record["server_metadata"] = metadata

        observation = _wait_for_observation(
            collector=collector,
            boundary=boundary,
            timeout=args.observation_timeout,
        )
        validate_request(observation.policy_input)
        for frame in (observation.wrist_frame, observation.exterior_frame):
            if (
                list(frame.native_shape) != config["native_images"]["shape"]
                or frame.source_encoding.lower()
                != config["native_images"]["encoding"]
            ):
                raise ValueError("Camera profile differs from the P0 contract.")

        np.savez_compressed(
            args.output_dir / "observation.npz",
            **observation.policy_input,
        )
        write_json(
            args.output_dir / "observation.json",
            observation_record(observation),
        )

        request_start = time.monotonic()
        response = policy.infer(
            observation.policy_input,
            policy_episode_seed=args.policy_episode_seed,
            replan_index=0,
            audit_model_input=True,
        )
        response_end = time.monotonic()

        action_age_at_response = response_end - request_start
        record["policy_action_age_at_response_seconds"] = action_age_at_response

        if action_age_at_response > ACTION_AGE:
            record["termination_reason"] = "action_expired_during_inference"
            print(
                "P1-B policy action expired during inference; no goal sent.",
                flush=True,
            )
            return

        action = first_action_from_chunk(response.actions)
        record["policy_actions_selected"] = 1
        np.savez_compressed(
            args.output_dir / "actions.npz",
            actions=response.actions,
            selected_action=action,
        )
        write_json(
            args.output_dir / "response.json",
            {
                "client_round_trip_seconds": response.client_round_trip_seconds,
                "policy_timing": response.policy_timing,
                "server_timing": response.server_timing,
                "sampling_metadata": response.sampling_metadata,
                "request_started_monotonic_seconds": request_start,
                "response_completed_monotonic_seconds": response_end,
                "selected_action_index": 0,
                "selected_raw_policy_action": action,
            },
        )
        write_json(
            args.output_dir / "model_audit.json",
            save_model_audit(
                response.model_input_audit,
                observation.policy_input,
                args.output_dir / "model_audit",
            ),
        )

        post_state = _wait_for_post_inference_joint(
            collector=collector,
            boundary=boundary,
            previous_ros_seconds=observation.joint_snapshot.stamp.ros_seconds,
            response_end=response_end,
            request_start=request_start,
            timeout=args.observation_timeout,
        )
        joint = post_state.latest_joint
        gate_monotonic = time.monotonic()
        gate_ros = node.get_clock().now().nanoseconds / 1e9
        readiness_reasons = []
        joint_error = post_state.errors.get("joint_state")
        if joint_error is not None:
            readiness_reasons.append(
                f"joint_state_callback_error: {joint_error}"
            )

        gate = safety_gate(
            action,
            joint.position_rad,
            limits,
            state_age=gate_ros - joint.stamp.ros_seconds,
            receive_age=(
                gate_monotonic - joint.stamp.receive_monotonic_seconds
            ),
            action_age=gate_monotonic - request_start,
            readiness_reasons=readiness_reasons,
        )
        gate["reference_joint_source_ros_seconds"] = joint.stamp.ros_seconds
        gate["reference_joint_receive_monotonic_seconds"] = (
            joint.stamp.receive_monotonic_seconds
        )
        gate["post_inference_callback_errors"] = dict(post_state.errors)
        write_json(args.output_dir / "safety.json", gate)
        record["safety_gate_accepted"] = bool(gate["accepted"])

        if not gate["accepted"]:
            record["termination_reason"] = "safety_gate_rejected"
            print("P1-B safety gate rejected action[0]; no goal sent.", flush=True)
            for reason in gate["rejection_reasons"]:
                print(f"  - {reason}", flush=True)
            return

        send_started = time.monotonic()
        send_action_age = send_started - request_start
        record["policy_action_age_at_goal_send_start_seconds"] = send_action_age

        if send_action_age > ACTION_AGE:
            record["termination_reason"] = "action_expired_before_goal_send"
            print(
                "P1-B action expired after safety approval; no goal sent.",
                flush=True,
            )
            return

        request_id = f"{args.output_dir.name}-action0"
        record["joint_target_goal_calls_started"] = 1

        result = joint_target_client.send_once(
            request_id=request_id,
            reference_q=joint.position_rad,
            target_q=gate["proposed_q_target_rad"],
            reference_ros_seconds=joint.stamp.ros_seconds,
            expect_execute=args.execute,
            timeout=args.joint_target_timeout,
        )

        send_finished = time.monotonic()
        record["joint_target_round_trip_seconds"] = (
            send_finished - send_started
        )
        record["joint_target_results_received"] = 1
        write_json(args.output_dir / "joint_target_result.json", result)

        if not args.execute:
            if not _plan_only_result_ok(result):
                raise RuntimeError(
                    "Joint-target server did not satisfy the P1-B "
                    "plan-only contract."
                )

            record["plan_only_validated"] = True
            record["termination_reason"] = "plan_only_validated"

            print(
                "P1-B real policy action[0] plan-only validation passed.",
                flush=True,
            )
            print(
                "raw action[0]: "
                f"{np.asarray(action, dtype=float).tolist()}",
                flush=True,
            )
            print(
                "q_target: "
                f"{np.asarray(gate['proposed_q_target_rad'], dtype=float).tolist()}",
                flush=True,
            )
            print(
                "execution_attempts: "
                f"{result['server_evidence']['execution_attempts']}",
                flush=True,
            )
            return

        target = np.asarray(
            gate["proposed_q_target_rad"],
            dtype=np.float64,
        )
        final_q = np.asarray(result["final_q"], dtype=np.float64)
        final_dq = np.asarray(result["final_dq"], dtype=np.float64)

        if (
            target.shape != (7,)
            or final_q.shape != (7,)
            or final_dq.shape != (7,)
            or not np.all(np.isfinite(target))
            or not np.all(np.isfinite(final_q))
            or not np.all(np.isfinite(final_dq))
        ):
            raise RuntimeError(
                "Invalid final execution state returned by joint-target server."
            )

        tracking_error = final_q - target
        max_tracking_error = float(np.max(np.abs(tracking_error)))
        max_final_speed = float(np.max(np.abs(final_dq)))

        evidence = result["server_evidence"]

        if not (
            result["success"] is True
            and result["outcome"] == "execution_succeeded"
            and result["planning_error_code"] == 1
            and result["execution_attempted"] is True
            and result["execution_action_status"] == 4
            and result["execution_error_code"] == 1
            and evidence.get("execution_attempts") == 1
            and max_tracking_error <= 0.01
            and max_final_speed <= 0.02
        ):
            raise RuntimeError(
                "Physical P1-B execution did not satisfy the "
                "single-action acceptance contract."
            )

        planned = evidence.get("planned", {})

        record["execution_accuracy"] = {
            "final_tracking_error_rad": tracking_error,
            "maximum_absolute_tracking_error_rad": max_tracking_error,
            "maximum_absolute_final_velocity_rad_s": max_final_speed,
        }

        record["timing_assessment"] = {
            "nominal_policy_frequency_hz": 15.0,
            "nominal_action_period_seconds": 1.0 / 15.0,
            "eight_action_window_seconds": 8.0 / 15.0,
            "joint_target_round_trip_seconds": (
                send_finished - send_started
            ),
            "moveit_planning_time_seconds": planned.get(
                "planning_time_s"
            ),
            "planned_trajectory_duration_seconds": planned.get(
                "duration_s"
            ),
        }

        record["policy_actions_executed"] = 1
        record["physical_execution_validated"] = True
        record["termination_reason"] = "execution_succeeded"

        print(
            "P1-B real policy action[0] physical execution passed.",
            flush=True,
        )
        print(
            "max final tracking error [rad]: "
            f"{max_tracking_error:.6f}",
            flush=True,
        )
        print(
            "max final |dq| [rad/s]: "
            f"{max_final_speed:.6f}",
            flush=True,
        )
        print(
            "joint-target round trip [s]: "
            f"{send_finished - send_started:.6f}",
            flush=True,
        )
        print(
            "planned trajectory duration [s]: "
            f"{planned.get('duration_s')}",
            flush=True,
        )

    except BaseException as error:
        record["termination_reason"] = (
            "interrupted" if isinstance(error, KeyboardInterrupt) else "error"
        )
        record["error"] = {
            "type": type(error).__name__,
            "message": str(error),
        }
        raise
    finally:
        stop.set()
        if observation_thread is not None:
            observation_thread.join(timeout=5.0)

        record["end_utc"] = utc_now()
        record["wall_clock_duration_seconds"] = time.monotonic() - started
        if collector is not None:
            record["source_rates"] = collector.source_rates()
            record["callback_errors"] = dict(collector.errors)

        try:
            write_json(args.output_dir / "run.json", record)
        finally:
            if policy_transport is not None:
                policy_transport.close()
            if joint_target_client is not None:
                joint_target_client.close()
            if node is not None:
                node.destroy_node()
            if initialized:
                rclpy.shutdown()


if __name__ == "__main__":
    run(parse_args())
