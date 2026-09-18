"""Supervised G1B gripper-only validation under the existing measured-q hold."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace
from typing import Any
import uuid

import numpy as np

from saps.physical.discrete_verifier import limits_from_urdf
from saps.physical.gripper_ros import create_gripper, gripper_timing_helpers
from saps.physical.live_inference_hold import (
    captured_records,
    controller_identity,
    validate_holds,
    wait_for_application,
)
from saps.physical.live_observation import (
    SourceStamp,
    gripper_snapshot_from_joint_state,
)
from saps.physical.live_shadow import write_json
from saps.physical.shadow_config import load_shadow_config
from saps.physical.shadow_ros import git_identity
from saps.physical.streaming_playback import prepare, selected_actions
from saps.physical.streaming_ros import (
    StreamingBoundary,
    validate_streaming_runtime_health,
)


def run_validation(args: Any) -> int:
    """Never infer policy or publish policy arm actions; retain failure traces."""
    import subprocess

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    if not args.execute:
        raise ValueError("Explicit supervised execution required")

    if args.mode not in {"grasp-release", "archived"}:
        raise ValueError(f"Unsupported validation mode: {args.mode}")

    if args.mode == "archived":
        prior = json.loads(
            (args.prior_validation / "run.json").read_text()
        )
        if (
            prior.get("status") != "passed"
            or prior.get("mode") != "grasp-release"
        ):
            raise ValueError(
                "Archived replay requires a passed grasp-release validation"
            )

    values = [0.0, 1.0, 0.0]
    if args.mode == "archived":
        with np.load(args.actions, allow_pickle=False) as data:
            values = selected_actions(data["actions"])[:, 7].tolist()

    config = load_shadow_config(args.config)
    maximum_width_m = (
        2.0 * config["robot"]["maximum_finger_position_m"]
    )

    # Fixed FR3 embodiment parameters for the first supervised G1B grasp test.
    # These are not policy outputs and do not depend on object geometry.
    gripper_speed_m_s = 0.1
    grasp_force_n = 20.0
    grasp_epsilon_inner_m = 0.001
    grasp_epsilon_outer_m = maximum_width_m
    grasp_dwell_seconds = 3.0
    width_tolerance_m = 0.002
    dwell_span_tolerance_m = 0.002
    lifecycle_timeout_seconds = 3.0

    timing, identity = gripper_timing_helpers(args.lab_stack_dir)
    args.output_dir.mkdir(parents=True, exist_ok=False)

    result: dict[str, Any] = dict(
        mode=args.mode,
        status="failed",
        run=str(uuid.uuid4()),
        rows=[],
        steps=[],
        policy_arm_actions=0,
        lab=identity,
        saps=git_identity(Path(__file__).resolve().parents[3]),
        gripper_contract=dict(
            policy_semantics="droid_binary_absolute_closure",
            open_primitive="move",
            close_primitive="grasp",
            maximum_width_m=maximum_width_m,
            speed_m_s=gripper_speed_m_s,
            grasp_width_m=0.0,
            grasp_force_n=grasp_force_n,
            grasp_epsilon_inner_m=grasp_epsilon_inner_m,
            grasp_epsilon_outer_m=grasp_epsilon_outer_m,
            grasp_dwell_seconds=grasp_dwell_seconds,
            width_tolerance_m=width_tolerance_m,
            dwell_span_tolerance_m=dwell_span_tolerance_m,
            object_width_supplied=False,
        ),
        lifecycle_timeout_seconds=lifecycle_timeout_seconds,
        retention_visual_confirmation_required=True,
    )

    if args.mode == "archived":
        result["actions_sha256"] = hashlib.sha256(
            args.actions.read_bytes()
        ).hexdigest()

    if args.prior_validation:
        result["prior_validation"] = str(
            args.prior_validation.resolve()
        )

    trace: list[dict[str, Any]] = []
    collector = SimpleNamespace(latest_gripper=None, errors=[])
    boundary = node = gripper = executor = None
    start = 0

    rclpy.init(args=[])
    try:
        xml = subprocess.check_output(
            [
                "ros2",
                "param",
                "get",
                "--hide-type",
                "/robot_state_publisher",
                "robot_description",
            ],
            text=True,
            timeout=20,
        ).strip()
        limits = limits_from_urdf(xml)

        node = rclpy.create_node("saps_g1b_gripper_validation")
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        def feedback(message: Any) -> None:
            received = time.monotonic()
            try:
                stamp = SourceStamp(
                    message.header.stamp.sec
                    + message.header.stamp.nanosec / 1e9,
                    received,
                )
                state = gripper_snapshot_from_joint_state(
                    message.name,
                    message.position,
                    stamp=stamp,
                    maximum_finger_position_m=config["robot"][
                        "maximum_finger_position_m"
                    ],
                )
                collector.latest_gripper = state
                trace.append(
                    dict(
                        receive_monotonic_seconds=received,
                        source_ros_seconds=stamp.ros_seconds,
                        width_m=state.width_m,
                        closure=state.closure,
                    )
                )
            except (ValueError, TypeError) as error:
                collector.errors.append(str(error))

        node.create_subscription(
            JointState,
            config["robot"]["gripper_state_topic"],
            feedback,
            qos_profile_sensor_data,
        )

        gripper = create_gripper(
            node,
            collector,
            config,
            args.lab_stack_dir,
            speed=gripper_speed_m_s,
            timeout=lifecycle_timeout_seconds,
            grasp_force_n=grasp_force_n,
            grasp_epsilon_inner_m=grasp_epsilon_inner_m,
            grasp_epsilon_outer_m=grasp_epsilon_outer_m,
        )

        boundary = StreamingBoundary(
            result["run"],
            timing.clock_provenance(),
        )

        deadline = time.monotonic() + 10.0
        while True:
            executor.spin_once(timeout_sec=0.01)
            try:
                state = boundary.snapshot()
                if (
                    state.readiness_reasons
                    or not gripper.hand.ready()
                ):
                    raise RuntimeError("hold_or_gripper_not_ready")
                gripper.measured()
                expected = controller_identity(
                    captured_records(boundary, 0)
                )
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise

        # Fresh reference and the existing zero-displacement safety/T4 gate.
        eligible = time.monotonic_ns()
        while state.receive_monotonic_ns < eligible:
            if (
                time.monotonic_ns() - eligible
                > 1_000_000_000
            ):
                raise TimeoutError(
                    "fresh_hold_reference_timeout"
                )
            time.sleep(0.001)
            state = boundary.snapshot()

        now = time.monotonic_ns()
        row = prepare(
            None,
            state,
            limits,
            now_ns=now,
            start_ns=now,
            scheduled_ns=now,
            kind="pre_inference_hold",
            action_index=None,
        )
        row["q_target"] = state.q.copy()
        result["rows"].append(row)

        if not row["safety_gate"]["accepted"]:
            raise RuntimeError("hold_safety_gate_rejected")

        with boundary.lock:
            start = len(boundary.records)

        boundary.publish(row, state, now)
        wait_for_application(
            boundary=boundary,
            analysis_start=start,
            rows=[row],
            analyzer=timing.analyze,
            expected_identity=expected,
            timeout=0.25,
        )

        def spin() -> None:
            executor.spin_once(timeout_sec=0.005)
            if collector.errors or boundary.errors:
                raise RuntimeError(
                    "validation_callback_error"
                )
            current = boundary.snapshot()
            if current.readiness_reasons:
                raise RuntimeError(
                    "; ".join(current.readiness_reasons)
                )
            gripper.check()

        def effective_request(
            step: dict[str, Any],
        ) -> dict[str, Any]:
            evidence = gripper.evidence()
            request = evidence["requests"][
                step["request_id"]
            ]
            owner = request.get(
                "duplicate_of",
                request["request_id"],
            )
            return evidence["requests"][owner]

        origin = time.monotonic()

        for index, value in enumerate(values):
            if args.mode == "archived":
                scheduled = origin + index / 15.0
                while time.monotonic() < scheduled:
                    spin()

            step = gripper.command(value)
            result["steps"].append(step)

            if args.mode == "archived":
                continue

            deadline = (
                time.monotonic()
                + lifecycle_timeout_seconds
            )

            while True:
                spin()
                request = effective_request(step)
                measured = gripper.measured()

                if request["disposition"] == "completed":
                    break

                if request["disposition"] in {
                    "failed",
                    "cancelled",
                }:
                    raise RuntimeError(
                        "gripper_command_did_not_complete: "
                        + request["disposition"]
                    )

                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        "gripper_command_completion_timeout"
                    )

            step["confirmed_measured"] = measured
            step[
                "confirmation_monotonic_seconds"
            ] = time.monotonic()

            if step["command_type"] == "move":
                if (
                    abs(
                        measured["width_m"]
                        - step["target_width_m"]
                    )
                    > width_tolerance_m
                ):
                    raise RuntimeError(
                        "gripper_open_width_not_confirmed"
                    )

            elif step["command_type"] == "grasp":
                dwell_start = time.monotonic()
                dwell_samples: list[
                    dict[str, Any]
                ] = []

                while (
                    time.monotonic() - dwell_start
                    < grasp_dwell_seconds
                ):
                    spin()
                    dwell_samples.append(
                        gripper.measured()
                    )

                if not dwell_samples:
                    raise RuntimeError(
                        "missing_grasp_dwell_samples"
                    )

                widths = [
                    sample["width_m"]
                    for sample in dwell_samples
                ]
                span = max(widths) - min(widths)

                step["grasp_dwell"] = dict(
                    duration_seconds=(
                        time.monotonic()
                        - dwell_start
                    ),
                    sample_count=len(dwell_samples),
                    initial_width_m=widths[0],
                    final_width_m=widths[-1],
                    minimum_width_m=min(widths),
                    maximum_width_m=max(widths),
                    width_span_m=span,
                )

                if span > dwell_span_tolerance_m:
                    raise RuntimeError(
                        "grasp_dwell_width_unstable"
                    )

            else:
                raise RuntimeError(
                    "unexpected_gripper_command_type: "
                    + step["command_type"]
                )

        if args.mode == "grasp-release":
            evidence = gripper.evidence()
            release_requests = [
                event
                for event in evidence["events"]
                if event["kind"]
                == "release_stop_requested"
            ]
            release_results = [
                event
                for event in evidence["events"]
                if event["kind"]
                == "release_stop_result"
            ]

            if len(release_requests) != 1:
                raise RuntimeError(
                    "expected_exactly_one_grasp_release_stop"
                )

            if (
                len(release_results) != 1
                or not release_results[0]["success"]
            ):
                raise RuntimeError(
                    "grasp_release_stop_not_confirmed"
                )

            result["release_stop_confirmed"] = True

        if args.mode == "archived":
            deadline = (
                time.monotonic()
                + lifecycle_timeout_seconds
            )

            while True:
                spin()
                final_step = result["steps"][-1]
                request = effective_request(final_step)
                measured = gripper.measured()

                if request["disposition"] in {
                    "failed",
                    "cancelled",
                }:
                    raise RuntimeError(
                        "archived_final_gripper_command_"
                        "did_not_complete: "
                        + request["disposition"]
                    )

                completed = (
                    request["disposition"] == "completed"
                    and gripper.hand.settled
                )

                if (
                    completed
                    and final_step["command_type"] == "move"
                    and abs(
                        measured["width_m"]
                        - final_step["target_width_m"]
                    )
                    > width_tolerance_m
                ):
                    completed = False

                if completed:
                    result["final_measured"] = measured
                    break

                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        "archived_final_gripper_state_timeout"
                    )

        # Drain async completions and the established arm evidence stream.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            spin()

        records = captured_records(boundary, start)
        result["hold_validation"] = validate_holds(
            records,
            [row],
            timing.analyze,
            expected,
        )
        result["arm_health"] = (
            validate_streaming_runtime_health(
                records,
                [row],
                hold_type="pre_inference_hold",
            )
        )

        # Verify the installed hold remained the same through every state sample.
        states = [
            record["data"]
            for record in records
            if (
                record["kind"] == "controller_state"
                and record["receive_monotonic_ns"]
                >= row["confirmation_monotonic_ns"]
            )
        ]
        result["hold_unchanged"] = bool(states) and all(
            len(sample) == 39
            and sample[38] == 1
            and sample[37]
            == row["timing"]["sequence"]
            and np.allclose(
                sample[14:21],
                row["q_target"],
                atol=1e-12,
                rtol=0,
            )
            for sample in states
        )

        if not (
            result["hold_validation"]["accepted"]
            and result["arm_health"]["accepted"]
            and result["hold_unchanged"]
        ):
            raise RuntimeError(
                "stationary_hold_evidence_failed"
            )

        result["status"] = "passed"

    except (Exception, KeyboardInterrupt) as error:
        result["error"] = (
            f"{type(error).__name__}: {error}"
        )

    finally:
        if gripper is not None:
            gripper.stop()
            deadline = time.monotonic() + 3.0
            try:
                while (
                    not gripper.hand.settled
                    and time.monotonic() < deadline
                ):
                    executor.spin_once(timeout_sec=0.01)
            except Exception as error:
                result["cleanup_error"] = str(error)
                result["status"] = "failed"

            result["gripper"] = gripper.evidence()

            if (
                not gripper.hand.settled
                or result["gripper"]["error"]
            ):
                result["status"] = "failed"

            for step in result["steps"]:
                request = result["gripper"][
                    "requests"
                ][step["request_id"]]
                step.update(request)

                owner = request.get(
                    "duplicate_of",
                    request["request_id"],
                )
                step["lifecycle_events"] = [
                    event
                    for event in result["gripper"][
                        "events"
                    ]
                    if event["request_id"] == owner
                ]

        if boundary is not None:
            try:
                boundary.close()
            except RuntimeError as error:
                result["boundary_cleanup_error"] = str(
                    error
                )
                result["status"] = "failed"

            with (
                args.output_dir / "arm_timing.jsonl"
            ).open("x") as output:
                for record in captured_records(
                    boundary,
                    0,
                ):
                    output.write(
                        json.dumps(
                            record,
                            allow_nan=False,
                        )
                        + "\n"
                    )

        write_json(
            args.output_dir / "gripper_feedback.json",
            trace,
        )
        write_json(
            args.output_dir / "run.json",
            result,
        )

        if executor is not None:
            executor.shutdown(timeout_sec=2.0)

        if node is not None:
            node.destroy_node()

        rclpy.shutdown()

    print(
        json.dumps(
            dict(
                status=result["status"],
                error=result.get("error"),
                output=str(args.output_dir),
            )
        )
    )
    return 0 if result["status"] == "passed" else 1
