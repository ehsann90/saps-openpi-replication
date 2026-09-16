"""ROS-only C1-B boundary for the pinned streaming forwarder contract."""

from __future__ import annotations

import json
from pathlib import Path
import threading
import time
from typing import Any
import types
import math
import numpy as np

from saps.physical.embodiment import FR3_JOINT_NAMES
from saps.physical.live_observation import (
    JointSnapshot, SourceStamp, ordered_fr3_joint_positions, ros_stamp_seconds,
)
from saps.physical.shadow_ros import git_identity
from saps.physical.single_action import ACTION_AGE, STATE_AGE
from saps.physical.streaming_playback import LAB_COMMIT, PERIOD_NS, MeasuredState

CONTROLLER = "fr3_streaming_joint_impedance_controller"
CONTROLLER_TYPE = "fr3_lab_stack/StreamingJointImpedanceController"
TARGET_TOPIC = "/fr3_streaming_joint_target"
JOINT_TOPIC = "/franka/joint_states"
FRANKA_STATE_TOPIC = "/franka_robot_state_broadcaster/robot_state"
FORWARDER = "fr3_streaming_joint_target_forwarder"


def pinned_timing_helpers(directory: Path) -> tuple[Any, dict[str, Any]]:
    """Read the pinned analyzer without generating bytecode in the reference tree."""
    identity = git_identity(directory)
    if identity.get("commit") != LAB_COMMIT or identity.get("dirty", True):
        raise ValueError("fr3_lab_stack must be clean at the exact C1-A2 pin.")
    path = directory / "fr3_lab_stack_runtime/timing_evidence.py"
    module = types.ModuleType("c1b_pinned_timing_evidence")
    exec(compile(path.read_text(), str(path), "exec"), module.__dict__)
    return module, identity


def correlate(records: list[dict[str, Any]], rows: list[dict[str, Any]],
              analyzer: Any) -> dict[str, Any]:
    """Use the unchanged C1-A2 joins; retain run/stamp identity on SAPS rows."""
    summary = analyzer(records)
    targets = {r["source_stamp_ns"]: r for r in summary["targets"]}
    for row in rows:
        stamp = row.get("source_stamp_ns")
        if stamp in targets:
            row["timing"] = targets[stamp]
            forwards = [r["data"] for r in records if r["kind"] == "forwarder"
                        and r["data"].get("source_frame_id") == row["run"]
                        and r["data"]["source_stamp_sec"] * 10**9
                        + r["data"]["source_stamp_nanosec"] == stamp]
            callbacks = [r["data"] for r in records if r["kind"] == "controller"
                         and r["data"].get("event") == "callback"
                         and r["data"].get("source_frame_id") == row["run"]
                         and r["data"].get("source_stamp_ns") == stamp]
            evidence = row["timing"]
            evidence.update(
                t0_ns=row.get("actual_publish_t0_ns"),
                t1_ns=None,
                t2_ns=None,
                t3_ns=None,
                t4_ns=None,
                forwarder_outcome=None,
                controller_callback_outcome=None,)
            if len(forwards) == 1:
                f = forwards[0]
                evidence.update(forwarder_outcome=f["outcome"],
                                t1_ns=f["server_receipt_monotonic_ns"],
                                t2_ns=f["publication_call_start_monotonic_ns"])
            if len(callbacks) == 1:
                evidence["t3_ns"] = callbacks[0]["t3_ns"]
                evidence["controller_callback_outcome"] = callbacks[0].get("outcome")
            # The upstream analyzer has already validated clock and identity
            # joins before producing this interval. Never mix clock domains.
            intervals = evidence.get("intervals_ns", {})
            if "t0_t4" in intervals:
                evidence["t4_ns"] = evidence["t0_ns"] + intervals["t0_t4"]
    return summary


def validate_c1b_delivery(
    rows: list[dict[str, Any]],
    summary: dict[str, Any],
) -> dict[str, Any]:
    """Require complete, ordered C1-B delivery for all eight actions plus hold."""

    reasons: list[str] = []

    expected_types = ["policy_action"] * 8 + ["terminal_hold"]
    expected_indices = list(range(8)) + [None]

    if len(rows) != 9:
        reasons.append(f"expected_9_rows_received_{len(rows)}")

    targets = summary.get("targets", [])
    if len(targets) != 9:
        reasons.append(
            f"expected_9_analyzed_targets_received_{len(targets)}"
        )

    source_stamps: list[int] = []
    sequences: list[int] = []
    identities: list[tuple[str, int]] = []
    application_times: list[int] = []

    for position, row in enumerate(rows[:9]):
        expected_type = expected_types[position]
        expected_index = expected_indices[position]

        if row.get("type") != expected_type:
            reasons.append(
                f"row_{position}_type_{row.get('type')!r}"
            )

        if row.get("action_index") != expected_index:
            reasons.append(
                f"row_{position}_action_index_"
                f"{row.get('action_index')!r}"
            )

        stamp = row.get("source_stamp_ns")
        if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0:
            reasons.append(f"row_{position}_invalid_source_stamp")
        else:
            source_stamps.append(stamp)

        timing = row.get("timing")
        if not isinstance(timing, dict):
            reasons.append(f"row_{position}_missing_timing")
            continue

        if timing.get("source_stamp_ns") != stamp:
            reasons.append(f"row_{position}_timing_stamp_mismatch")

        if timing.get("status") != "applied":
            reasons.append(
                f"row_{position}_status_{timing.get('status')!r}"
            )

        for field in (
            "forwarder_records",
            "callback_records",
            "application_records",
        ):
            if timing.get(field) != 1:
                reasons.append(
                    f"row_{position}_{field}_{timing.get(field)!r}"
                )

        if timing.get("forwarder_outcome") != "forwarded":
            reasons.append(
                f"row_{position}_forwarder_outcome_"
                f"{timing.get('forwarder_outcome')!r}"
            )

        if timing.get("controller_callback_outcome") != "accepted":
            reasons.append(
                f"row_{position}_callback_outcome_"
                f"{timing.get('controller_callback_outcome')!r}"
            )

        sequence = timing.get("sequence")
        if (
            not isinstance(sequence, int)
            or isinstance(sequence, bool)
            or sequence <= 0
        ):
            reasons.append(f"row_{position}_invalid_sequence")
        else:
            sequences.append(sequence)

        instance = timing.get("instance")
        activation = timing.get("activation")
        if (
            not isinstance(instance, str)
            or not instance
            or not isinstance(activation, int)
            or isinstance(activation, bool)
            or activation <= 0
        ):
            reasons.append(f"row_{position}_invalid_controller_identity")
        else:
            identities.append((instance, activation))

        try:
            target = np.asarray(row["q_target"], dtype=np.float64)
            desired = np.asarray(
                timing["q_desired"],
                dtype=np.float64,
            )
            if (
                target.shape != (7,)
                or desired.shape != (7,)
                or not np.isfinite(target).all()
                or not np.isfinite(desired).all()
                or not np.allclose(
                    target,
                    desired,
                    rtol=0.0,
                    atol=1e-12,
                )
            ):
                reasons.append(f"row_{position}_q_desired_mismatch")
        except (KeyError, TypeError, ValueError):
            reasons.append(f"row_{position}_invalid_target_evidence")

        intervals = timing.get("intervals_ns")
        if not isinstance(intervals, dict):
            reasons.append(f"row_{position}_missing_intervals")
        else:
            for name in (
                "t0_t1",
                "t1_t2",
                "t2_t3",
                "t3_t4",
                "t0_t4",
            ):
                value = intervals.get(name)
                if (
                    not isinstance(value, int)
                    or isinstance(value, bool)
                    or value < 0
                ):
                    reasons.append(
                        f"row_{position}_invalid_{name}"
                    )

        times = [
            timing.get("t0_ns"),
            timing.get("t1_ns"),
            timing.get("t2_ns"),
            timing.get("t3_ns"),
            timing.get("t4_ns"),
        ]
        if (
            any(
                not isinstance(value, int)
                or isinstance(value, bool)
                for value in times
            )
            or any(b < a for a, b in zip(times, times[1:]))
        ):
            reasons.append(f"row_{position}_invalid_event_order")
        else:
            application_times.append(times[-1])

    if len(source_stamps) == 9:
        if len(set(source_stamps)) != 9:
            reasons.append("source_stamps_not_unique")
        if any(
            b <= a for a, b in zip(source_stamps, source_stamps[1:])
        ):
            reasons.append("source_stamps_not_strictly_increasing")

    if len(sequences) == 9 and any(
        b != a + 1 for a, b in zip(sequences, sequences[1:])
    ):
        reasons.append("controller_sequences_not_consecutive")

    if len(identities) == 9 and len(set(identities)) != 1:
        reasons.append("controller_instance_or_activation_changed")

    if len(application_times) == 9 and any(
        b <= a
        for a, b in zip(application_times, application_times[1:])
    ):
        reasons.append("applications_not_strictly_ordered")

    gaps = summary.get("evidence_id_gaps", {})
    if any(value != 0 for value in gaps.values()):
        reasons.append("evidence_id_gap_observed")

    publication_errors = summary.get(
        "controller_publication_errors_cumulative",
        {},
    )
    if any(value != 0 for value in publication_errors.values()):
        reasons.append("controller_evidence_publication_error")

    overflow = summary.get("rt_overflow_cumulative", {})
    if any(
        counters.get("dropped_period_samples", 0) != 0
        or counters.get("dropped_application_records", 0) != 0
        for counters in overflow.values()
    ):
        reasons.append("controller_rt_evidence_overflow")

    for field in (
        "unmatched_forwarder_records",
        "unmatched_callback_records",
        "unmatched_application_records",
    ):
        if summary.get(field) != 0:
            reasons.append(f"{field}_nonzero")

    periods = summary.get("periods", [])
    if not periods or not any(
        isinstance(period.get("count"), int)
        and period["count"] > 0
        for period in periods
    ):
        reasons.append("no_controller_period_evidence")

    return {
        "accepted": not reasons,
        "reasons": reasons,
        "expected_targets": 9,
        "analyzed_targets": len(targets),
        "source_stamps": source_stamps,
        "controller_sequences": sequences,
        "controller_identity": (
            list(identities[0])
            if identities and len(set(identities)) == 1
            else None
        ),
    }


def validate_c1b_runtime_health(
    records: list[dict[str, Any]],
    rows: list[dict[str, Any]],
) -> dict[str, Any]:
    """Require healthy robot/controller evidence through the post-hold drain."""

    reasons: list[str] = []

    holds = [
        row for row in rows
        if row.get("type") == "terminal_hold"
    ]
    if len(holds) != 1:
        return {
            "accepted": False,
            "reasons": ["expected_exactly_one_terminal_hold"],
        }

    hold_t0 = holds[0].get("actual_publish_t0_ns")
    if (
        not isinstance(hold_t0, int)
        or isinstance(hold_t0, bool)
        or hold_t0 <= 0
    ):
        return {
            "accepted": False,
            "reasons": ["terminal_hold_missing_actual_t0"],
        }

    franka = [
        record for record in records
        if record.get("kind") == "franka_state"
    ]
    if not franka:
        reasons.append("no_franka_state_evidence")
    else:
        unhealthy = [
            record for record in franka
            if record.get("health_reasons")
        ]
        if unhealthy:
            reasons.append("franka_health_violation_observed")

        post_hold_franka = [
            record for record in franka
            if record.get("receive_monotonic_ns", -1) >= hold_t0
        ]
        if not post_hold_franka:
            reasons.append("no_post_hold_franka_state")
        elif post_hold_franka[-1].get("health_reasons"):
            reasons.append("final_franka_state_unhealthy")

    readiness = [
        record for record in records
        if record.get("kind") == "controller_readiness"
        and record.get("response_monotonic_ns", -1) >= hold_t0
    ]
    if not readiness:
        reasons.append("no_post_hold_controller_readiness")
    elif readiness[-1].get("readiness_reasons"):
        reasons.append("final_controller_readiness_failed")

    states = [
        record for record in records
        if record.get("kind") == "controller_state"
        and record.get("receive_monotonic_ns", -1) >= hold_t0
    ]
    if not states:
        reasons.append("no_post_hold_controller_state")
    else:
        data = states[-1].get("data")
        if (
            not isinstance(data, list)
            or len(data) != 39
            or not np.isfinite(data).all()
            or data[38] != 1
        ):
            reasons.append("final_controller_state_invalid_or_inactive")

    joints = [
        record for record in records
        if record.get("kind") == "measured_joint"
        and record.get("receive_monotonic_ns", -1) >= hold_t0
    ]
    if not joints:
        reasons.append("no_post_hold_measured_joint")

    return {
        "accepted": not reasons,
        "reasons": reasons,
        "franka_samples": len(franka),
        "franka_health_violation_count": (
            sum(bool(record.get("health_reasons")) for record in franka)
        ),
        "post_hold_franka_samples": (
            sum(
                record.get("receive_monotonic_ns", -1) >= hold_t0
                for record in franka
            )
        ),
        "post_hold_controller_readiness_samples": len(readiness),
        "post_hold_controller_state_samples": len(states),
        "post_hold_joint_samples": len(joints),
    }


def franka_health(message: Any) -> tuple[dict[str, Any], list[str]]:
    """Extract independent robot-health evidence used to admit a target."""

    reasons: list[str] = []

    current_errors = {
        name: bool(getattr(message.current_errors, name))
        for name in message.current_errors.get_fields_and_field_types()
    }

    collision = message.collision_indicators
    collision_values = [
        *[float(value) for value in collision.is_joint_collision],
        float(collision.is_cartesian_linear_collision.x),
        float(collision.is_cartesian_linear_collision.y),
        float(collision.is_cartesian_linear_collision.z),
        float(collision.is_cartesian_angular_collision.x),
        float(collision.is_cartesian_angular_collision.y),
        float(collision.is_cartesian_angular_collision.z),
    ]

    robot_mode = int(message.robot_mode)
    command_success_rate = float(message.control_command_success_rate)

    if robot_mode not in (
        int(message.ROBOT_MODE_IDLE),
        int(message.ROBOT_MODE_MOVE),
    ):
        reasons.append("franka_robot_mode_not_idle_or_move")

    if any(current_errors.values()):
        reasons.append("franka_current_errors_present")

    if (
        not all(math.isfinite(value) for value in collision_values)
        or any(collision_values)
    ):
        reasons.append("franka_collision_indicator_active_or_invalid")

    if not math.isfinite(command_success_rate):
        reasons.append("franka_control_command_success_rate_invalid")

    evidence = {
        "robot_mode": robot_mode,
        "current_errors": current_errors,
        "collision_values": collision_values,
        "control_command_success_rate": command_success_rate,
    }
    return evidence, reasons


class StreamingBoundary:
    """Background observations/evidence; main thread publishes one target per tick."""

    def __init__(self, run: str, clock: dict[str, Any]) -> None:
        import rclpy
        from controller_manager_msgs.srv import ListControllers
        from rclpy.qos import (
            DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray, String
        from franka_msgs.msg import FrankaRobotState

        self.rclpy, self.message_type = rclpy, JointState
        self.run, self.clock = run, clock
        self.lock = threading.RLock()
        self.records: list[dict[str, Any]] = []
        self.errors: dict[str, str] = {}
        self.joint: tuple[Any, int, int, Any] | None = None
        self.controller_state: tuple[list[float], int] | None = None
        self.franka_state: tuple[dict[str, Any], int, int, tuple[str, ...]] | None = None
        self.controller_readiness_reasons: tuple[str, ...] = ()
        self.active_dispatch_ns = 0
        self.pending = None
        self.stamps: set[int] = set()
        self.stop = threading.Event()
        self.node = rclpy.create_node(
            "saps_physical_c1b", enable_rosout=False,
            start_parameter_services=False,
        )
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.node.create_publisher(
            JointState, TARGET_TOPIC, target_qos,
        )
        self.node.create_subscription(
            JointState, JOINT_TOPIC, self._joint, qos_profile_sensor_data,
        )
        self.node.create_subscription(
            FrankaRobotState, FRANKA_STATE_TOPIC, self._franka, qos_profile_sensor_data
        )
        self.node.create_subscription(
            Float64MultiArray, f"/{CONTROLLER}/state", self._state,
            qos_profile_sensor_data,
        )
        for kind, topic in (
            ("forwarder", f"/{FORWARDER}/telemetry"),
            ("controller", f"/{CONTROLLER}/timing"),
        ):
            self.node.create_subscription(
                String, topic, lambda msg, k=kind: self._evidence(k, msg), 100,
            )
        self.service_type = ListControllers
        self.service = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers",
        )
        self.node.create_timer(0.1, self._poll)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _save(self, kind: str, **fields: Any) -> None:
        with self.lock:
            self.records.append(dict(kind=kind, run=self.run, **fields))

    def _joint(self, message: Any) -> None:
        received = time.monotonic_ns()
        try:
            seconds = ros_stamp_seconds(
                message.header.stamp.sec, message.header.stamp.nanosec,
            )
            ordered_fr3_joint_positions(message.name, message.position)
            # Preserve the original doubles after established shape/name checks.
            positions = dict(zip(message.name, message.position))
            q = np.array([positions[n] for n in FR3_JOINT_NAMES])
            q.setflags(write=False)
            joint = JointSnapshot(q, SourceStamp(seconds, received / 1e9))
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            dq = None
            if len(message.velocity) == len(message.name):
                velocities = dict(zip(message.name, message.velocity))
                dq = [float(velocities[n]) for n in FR3_JOINT_NAMES]
                if not np.isfinite(dq).all():
                    raise ValueError("Nonfinite measured joint velocity")
            with self.lock:
                if stamp <= 0 or (self.joint and stamp <= self.joint[1]):
                    raise ValueError("Nonpositive/nonadvancing measured joint stamp")
                self.joint = (joint, stamp, received, dq)
                self.errors.pop("joint", None)
            self._save("measured_joint", source_stamp_ns=stamp,
                       receive_monotonic_ns=received, q=q.tolist(), dq=dq)
        except (TypeError, ValueError) as error:
            with self.lock:
                self.errors["joint"] = str(error)

    def _franka(self, message: Any) -> None:
        received = time.monotonic_ns()

        try:
            stamp = (
                int(message.header.stamp.sec) * 10**9
                + int(message.header.stamp.nanosec)
            )
            if stamp <= 0:
                raise ValueError("Nonpositive Franka robot-state stamp")

            evidence, reasons = franka_health(message)

            with self.lock:
                self.franka_state = (
                    evidence,
                    stamp,
                    received,
                    tuple(reasons),
                )
                self.errors.pop("franka_state", None)

            self._save(
                "franka_state",
                source_stamp_ns=stamp,
                receive_monotonic_ns=received,
                evidence=evidence,
                health_reasons=reasons,
            )

        except (TypeError, ValueError, AttributeError) as error:
            with self.lock:
                self.errors["franka_state"] = str(error)

    def _state(self, message: Any) -> None:
        with self.lock:
            data = list(message.data)
            self.controller_state = (data, time.monotonic_ns())
            self._save("controller_state",
                       data=data if np.isfinite(data).all() else None,
                       invalid_data_repr=None if np.isfinite(data).all() else repr(data),
                       receive_monotonic_ns=self.controller_state[1])

    def _evidence(self, kind: str, message: Any) -> None:
        with self.lock:
            try:
                data = json.loads(message.data)
                self._save(kind, data=data, raw=message.data)
                if (data.get("source_frame_id") == self.run
                        and data.get("outcome") not in (None, "accepted", "forwarded")):
                    self.errors["target_evidence"] = str(data.get("outcome"))
            except (TypeError, ValueError) as error:
                self._save("invalid_evidence", source=kind, raw=message.data)
                self.errors[kind] = str(error)

    def _poll(self) -> None:
        with self.lock:
            if not self.service.service_is_ready():
                self.active_dispatch_ns = 0
                self.controller_readiness_reasons = (
                    "controller_manager_service_unavailable",
                )
                return
            if self.pending is not None:
                return
            dispatched = time.monotonic_ns()
            self.pending = self.service.call_async(self.service_type.Request())

        def done(future: Any) -> None:
            with self.lock:
                self.pending = None
                self.active_dispatch_ns = 0
                reasons: list[str] = []

                try:
                    response = future.result()
                    self.errors.pop("controller_service", None)

                    matches = [
                        controller
                        for controller in response.controller
                        if controller.name == CONTROLLER
                    ]

                    if len(matches) != 1:
                        reasons.append(
                            "streaming_controller_missing_or_duplicated"
                        )
                    else:
                        controller = matches[0]

                        if controller.state != "active":
                            reasons.append(
                                "streaming_controller_not_active"
                            )

                        if controller.type != CONTROLLER_TYPE:
                            reasons.append(
                                "unexpected_streaming_controller_type"
                            )

                        expected_claims = {
                            f"{joint}/effort"
                            for joint in FR3_JOINT_NAMES
                        }
                        claimed = set(controller.claimed_interfaces)

                        if claimed != expected_claims:
                            reasons.append(
                                "streaming_controller_effort_claims_mismatch"
                            )

                    # No other active controller may own or require an FR3 arm
                    # command interface. Gripper-only controllers remain allowed.
                    competing: list[str] = []

                    for controller in response.controller:
                        if (
                            controller.state != "active"
                            or controller.name == CONTROLLER
                        ):
                            continue

                        interfaces = (
                            list(controller.claimed_interfaces)
                            + list(controller.required_command_interfaces)
                        )

                        arm_interfaces = [
                            interface
                            for interface in interfaces
                            if interface.split("/", 1)[0] in FR3_JOINT_NAMES
                        ]

                        if arm_interfaces:
                            competing.append(controller.name)

                    if competing:
                        reasons.append(
                            "competing_active_arm_controller:"
                            + ",".join(sorted(competing))
                        )

                    if not reasons:
                        self.active_dispatch_ns = dispatched

                    self.controller_readiness_reasons = tuple(reasons)

                    self._save(
                        "controller_readiness",
                        states={
                            controller.name: controller.state
                            for controller in response.controller
                        },
                        types={
                            controller.name: controller.type
                            for controller in response.controller
                        },
                        claimed_interfaces={
                            controller.name: list(
                                controller.claimed_interfaces
                            )
                            for controller in response.controller
                        },
                        readiness_reasons=reasons,
                        request_monotonic_ns=dispatched,
                        response_monotonic_ns=time.monotonic_ns(),
                    )

                except Exception as error:
                    self.controller_readiness_reasons = (
                        f"controller_service: {error}",
                    )
                    self.errors["controller_service"] = str(error)
        self.pending.add_done_callback(done)

    def _spin(self) -> None:
        try:
            while not self.stop.is_set():
                self.rclpy.spin_once(self.node, timeout_sec=0.01)
        except Exception as error:
            with self.lock:
                self.errors["executor"] = str(error)

    def _readiness(self, now_ns: int, ros_ns: int) -> list[str]:
        reasons = [f"{k}: {v}" for k, v in self.errors.items()]
        if (self.active_dispatch_ns <= 0
                or not 0 <= now_ns - self.active_dispatch_ns <= 300_000_000):
            reasons.append("controller_active_evidence_missing_or_expired")

        reasons.extend(self.controller_readiness_reasons)
        if self.franka_state is None:
            reasons.append("franka_state_missing")
        else:
            _, source_stamp, received, health_reasons = (
                self.franka_state
            )

            source_age_ns = ros_ns - source_stamp
            receive_age_ns = now_ns - received

            if (
                source_age_ns < 0
                or source_age_ns > int(STATE_AGE * 1e9)
                or receive_age_ns < 0
                or receive_age_ns > int(STATE_AGE * 1e9)
            ):
                reasons.append("franka_state_stale_or_future")

            reasons.extend(health_reasons)

        if self.controller_state is None:
            reasons.append("controller_state_missing")
        else:
            data, received = self.controller_state
            if (len(data) != 39 or not np.isfinite(data).all()
                    or data[38] != 1
                    or not 0 <= now_ns - received <= 100_000_000
                    or not 0 <= ros_ns / 1e9 - data[35] <= STATE_AGE):
                reasons.append("controller_state_invalid_inactive_or_stale")

        endpoints = self.node.get_subscriptions_info_by_topic(TARGET_TOPIC)
        if (len(endpoints) != 1 or endpoints[0].node_name != FORWARDER):
            reasons.append("expected_exactly_one_forwarder_subscriber")

        target_publishers = self.node.get_publishers_info_by_topic(TARGET_TOPIC)
        if (len(target_publishers) != 1
            or target_publishers[0].node_name != self.node.get_name()):
            reasons.append("expected_c1b_as_only_streaming_target_publisher")

        publishers = self.node.get_publishers_info_by_topic(JOINT_TOPIC)
        if (len(publishers) != 1
                or publishers[0].node_name != "joint_state_broadcaster"):
            reasons.append("unexpected_joint_state_source")
        return reasons

    def snapshot(self) -> MeasuredState:
        with self.lock:
            if self.joint is None:
                raise RuntimeError("No measured FR3 joint state")
            joint, stamp, received, dq = self.joint
            ros_ns = self.node.get_clock().now().nanoseconds
            reasons = self._readiness(time.monotonic_ns(), ros_ns)
            return MeasuredState(joint.position_rad, stamp, received, ros_ns,
                                 tuple(reasons), dq)

    def publish(self, row: dict[str, Any], state: MeasuredState,
                start_ns: int) -> None:
        message = self.message_type()
        message.name = list(FR3_JOINT_NAMES)
        message.position = np.asarray(row["q_target"]).tolist()
        message.header.frame_id = self.run
        with self.lock:
            stamp = self.node.get_clock().now().nanoseconds
            now_ns = time.monotonic_ns()
            reasons = self._readiness(now_ns, stamp)
            age = (stamp - state.source_stamp_ns) / 1e9
            receive_age = (now_ns - state.receive_monotonic_ns) / 1e9
            if not (0 <= age <= STATE_AGE and 0 <= receive_age <= STATE_AGE):
                reasons.append("reference_expired_before_publication")
            if (row["type"] == "policy_action"
                    and not 0 <= (now_ns - start_ns) / 1e9 <= ACTION_AGE):
                reasons.append("playback_action_expired_before_publication")
            if stamp <= 0 or stamp in self.stamps:
                reasons.append("nonpositive_or_duplicate_source_stamp")
            row["publish_readiness_reasons"] = reasons
            if reasons:
                raise RuntimeError("; ".join(reasons))
            self.stamps.add(stamp)
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
            row.update(run=self.run, source_stamp_ns=stamp,
                       q_ref_age_at_publish_seconds=age,
                       q_ref_receive_age_at_publish_seconds=receive_age)
            # Graph inspection and Python scheduling may consume time. Recheck
            # the exact reference immediately before sampling publish-call T0.
            now_ns = time.monotonic_ns()
            ros_ns = self.node.get_clock().now().nanoseconds
            age = (ros_ns - state.source_stamp_ns) / 1e9
            receive_age = (now_ns - state.receive_monotonic_ns) / 1e9
            if not (0 <= age <= STATE_AGE and 0 <= receive_age <= STATE_AGE):
                raise RuntimeError("reference_expired_at_publication")
            if now_ns - row["scheduled_monotonic_ns"] >= PERIOD_NS:
                raise RuntimeError("missed_complete_interval_at_publication")
            row.update(q_ref_age_at_publish_seconds=age,
                       q_ref_receive_age_at_publish_seconds=receive_age)
            outcome = "returned"
            t0 = time.monotonic_ns()
            try:
                self.publisher.publish(message)
            except Exception as error:
                outcome = "publication_error"
                raise RuntimeError(f"Target publication failed: {error}") from error
            finally:
                end = time.monotonic_ns()
                row.update(actual_publish_t0_ns=t0,
                           schedule_lateness_ns=t0 - row["scheduled_monotonic_ns"],
                           publication_call_end_ns=end,
                           publication_call_outcome=outcome)
                self._save("sent", **self.clock, source_stamp_ns=stamp,
                           q=list(message.position), t0_ns=t0,
                           publication_call_end_ns=end,
                           publication_call_outcome=outcome)

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=5)
        if self.thread.is_alive():
            raise RuntimeError("Observation executor did not stop")
        self.node.destroy_node()
