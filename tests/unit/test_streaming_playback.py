"""Deterministic C1-B scheduling, independent anchoring and abort coverage."""

import unittest
from unittest.mock import Mock, patch

import numpy as np

from saps.physical.streaming_playback import (
    ACTION_COUNT, PERIOD_NS, MeasuredState, deadlines, playback,
    selected_actions, tick_offset_ns, wait_until,
)
from saps.physical.streaming_ros import (
    correlate, franka_health,
    validate_c1b_delivery,
    validate_c1b_runtime_health
    )
from types import SimpleNamespace


def valid_delivery_fixture():
    rows = []
    targets = []

    for index in range(9):
        stamp = 10_000 + index
        target = np.full(7, index * 0.01)
        t0 = 1_000_000 + index * 100_000
        timing = {
            "source_stamp_ns": stamp,
            "status": "applied",
            "forwarder_records": 1,
            "callback_records": 1,
            "application_records": 1,
            "instance": "controller-instance",
            "activation": 1234,
            "sequence": 20 + index,
            "q_desired": target.copy(),
            "forwarder_outcome": "forwarded",
            "controller_callback_outcome": "accepted",
            "t0_ns": t0,
            "t1_ns": t0 + 10,
            "t2_ns": t0 + 20,
            "t3_ns": t0 + 30,
            "t4_ns": t0 + 40,
            "intervals_ns": {
                "t0_t1": 10,
                "t1_t2": 10,
                "t2_t3": 10,
                "t3_t4": 10,
                "t0_t4": 40,
            },
        }
        rows.append({
            "type": (
                "terminal_hold"
                if index == 8
                else "policy_action"
            ),
            "action_index": None if index == 8 else index,
            "source_stamp_ns": stamp,
            "q_target": target,
            "timing": timing,
        })
        targets.append({
            "source_stamp_ns": stamp,
            "status": "applied",
        })

    summary = {
        "targets": targets,
        "controller_publication_errors_cumulative": {
            "controller-instance": 0,
        },
        "periods": [{
            "instance": "controller-instance",
            "activation": 1234,
            "count": 100,
        }],
        "evidence_id_gaps": {
            "controller:controller-instance": 0,
            "forwarder:forwarder-instance": 0,
        },
        "rt_overflow_cumulative": {
            "controller-instance": {
                "dropped_period_samples": 0,
                "dropped_application_records": 0,
            },
        },
        "unmatched_forwarder_records": 0,
        "unmatched_callback_records": 0,
        "unmatched_application_records": 0,
    }
    return rows, summary


def valid_runtime_health_fixture(hold_t0=1_800_000):
    controller_data = [0.0] * 39
    controller_data[38] = 1.0

    return [
        {
            "kind": "franka_state",
            "receive_monotonic_ns": 1_700_000,
            "health_reasons": [],
        },
        {
            "kind": "franka_state",
            "receive_monotonic_ns": 1_900_000,
            "health_reasons": [],
        },
        {
            "kind": "controller_readiness",
            "response_monotonic_ns": 1_900_000,
            "readiness_reasons": [],
        },
        {
            "kind": "controller_state",
            "receive_monotonic_ns": 1_900_000,
            "data": controller_data,
        },
        {
            "kind": "measured_joint",
            "receive_monotonic_ns": 1_900_000,
            "q": [0.0] * 7,
            "dq": [0.0] * 7,
        },
    ]


class FakeErrorState:
    def __init__(self, **values):
        self._values = values
        for name, value in values.items():
            setattr(self, name, value)

    def get_fields_and_field_types(self):
        return {name: "boolean" for name in self._values}


class FakeBoundary:
    def __init__(self):
        self.now = 1_000_000_000
        self.index = 0
        self.sent = []
        self.states = []
        self.stale_at = None
        self.invalid_at = None
        self.fail_at = None

    def wait(self, deadline):
        self.now = max(self.now, deadline)

    def snapshot(self):
        i = self.index
        self.index += 1
        state = MeasuredState(
            np.full(7, i * 0.01), self.now,
            self.now - (100_000_001 if i == self.stale_at else 0),
            self.now, ("inactive",) if i == self.invalid_at else (),
        )
        self.states.append(state)
        return state

    def publish(self, row, state, start):
        if row["action_index"] == self.fail_at and row["type"] == "policy_action":
            raise RuntimeError("publication failed")
        row.update(actual_publish_t0_ns=self.now,
                   schedule_lateness_ns=self.now - row["scheduled_monotonic_ns"])
        self.sent.append(row)
        self.now += 2_000_000  # Work takes time; next deadline must not drift.


class PlaybackTests(unittest.TestCase):
    def setUp(self):
        self.metric = patch("saps.physical.single_action.singularity_metric",
                            return_value={"rejected": False})
        self.metric.start()
        self.addCleanup(self.metric.stop)
        self.chunk = np.zeros((15, 8))
        self.chunk[:, :3] = [2, -2, .5]
        self.chunk[:, 7] = .75
        self.limits = {"lower_rad": np.full(7, -3.),
                       "upper_rad": np.full(7, 3.)}
        self.boundary = FakeBoundary()

    def run_playback(self, **kwargs):
        return playback(self.chunk, self.boundary, self.limits,
                        now=lambda: self.boundary.now,
                        wait=self.boundary.wait, **kwargs)

    def test_absolute_action_and_hold_deadlines(self):
        expected_offsets = (
            0,
            66_666_667,
            133_333_333,
            200_000_000,
            266_666_667,
            333_333_333,
            400_000_000,
            466_666_667,
            533_333_333,
        )

        ds = deadlines(123)
        self.assertEqual(
            tuple(deadline - 123 for deadline in ds),
            expected_offsets,
        )
        self.assertEqual(
            tuple(tick_offset_ns(k) for k in range(9)),
            expected_offsets,
        )

        # Regression: multiplying one rounded period must not reproduce the
        # eight-tick endpoint.
        self.assertNotEqual(
            tick_offset_ns(8),
            8 * PERIOD_NS,
        )

        result = self.run_playback()
        self.assertEqual(result["status"], "published")

        for row, offset in zip(result["rows"], expected_offsets):
            self.assertEqual(
                row["actual_publish_t0_ns"],
                10**9 + offset,
            )

        self.assertEqual(
            result["action0_to_terminal_hold_t0_ns"],
            expected_offsets[8],
        )
        self.assertEqual(result["rows"][-1]["type"], "terminal_hold")
        self.assertIsNone(result["rows"][-1]["action_index"])

        self.assertEqual(
            result["rows"][-1]["actual_publish_t0_ns"]
            - result["rows"][-2]["actual_publish_t0_ns"],
            expected_offsets[8] - expected_offsets[7],
        )

    def test_fresh_independent_mapping_clipping_and_logged_gripper(self):
        result = self.run_playback()
        for i, row in enumerate(result["rows"][:8]):
            expected = np.full(7, .01 * i) + np.array([.2, -.2, .1, 0, 0, 0, 0])
            np.testing.assert_array_equal(row["q_target"], expected)
            self.assertEqual(len(row["q_target"]), 7)
            self.assertEqual(row["raw_policy_action"][7], .75)
            self.assertNotIn("gripper_reference", row["safety_gate"])
        np.testing.assert_array_equal(result["rows"][-1]["q_target"], np.full(7, .08))
        self.assertEqual(result["gripper_commands_issued"], 0)

    def test_malformed_chunk_or_count_rejected_before_publish(self):
        for chunk in (np.zeros((8, 8)), np.zeros((15, 7)),
                      np.zeros((15, 8), dtype=int), np.full((15, 8), np.nan)):
            with self.assertRaises(ValueError):
                playback(chunk, self.boundary, self.limits)
        for count in (7, 9, True):
            with self.assertRaises(ValueError):
                selected_actions(self.chunk, count)
        self.assertEqual(self.boundary.sent, [])

    def test_stale_state_aborts_then_uses_own_fresh_hold_state(self):
        self.boundary.stale_at = 2
        result = self.run_playback()
        self.assertEqual(result["status"], "aborted_before_action_2")
        self.assertEqual([r["action_index"] for r in self.boundary.sent], [0, 1, None])
        self.assertEqual(self.boundary.sent[-1]["type"], "abort_hold")
        np.testing.assert_array_equal(self.boundary.sent[-1]["q_target"], np.full(7, .03))

    def test_safety_rejection_stops_policy_sequence(self):
        self.limits["upper_rad"][0] = .305
        result = self.run_playback()
        self.assertEqual(result["status"], "aborted_before_action_1")
        self.assertEqual([r["action_index"] for r in self.boundary.sent], [0, None])

    def test_readiness_rejection_aborts(self):
        self.boundary.invalid_at = 0
        result = self.run_playback()
        self.assertEqual(result["status"], "aborted_before_action_0")
        self.assertEqual(len(self.boundary.sent), 1)
        self.assertEqual(self.boundary.sent[0]["type"], "abort_hold")

    def test_unsafe_hold_is_not_published(self):
        self.limits["upper_rad"] = np.full(7, .05)
        result = self.run_playback()
        self.assertEqual(result["abort_hold"], "unsafe_or_failed_no_retry")
        self.assertEqual(self.boundary.sent, [])

    def test_publication_failure_has_no_retry_or_later_policy_action(self):
        self.boundary.fail_at = 3
        result = self.run_playback()
        self.assertEqual(result["status"], "aborted_before_action_3")
        self.assertEqual([r["action_index"] for r in self.boundary.sent], [0, 1, 2, None])

    def test_dry_run_never_calls_publish(self):
        self.boundary.publish = Mock(side_effect=AssertionError("ROS publication"))
        result = self.run_playback(dry_run=True)
        self.assertEqual(result["status"], "dry_run_complete")
        self.boundary.publish.assert_not_called()
        self.assertTrue(all(r["actual_publish_t0_ns"] is None for r in result["rows"]))

    def test_terminal_hold_rejection_is_not_retried(self):
        self.boundary.stale_at = 8
        result = self.run_playback()
        self.assertEqual(result["status"], "terminal_hold_failed")
        self.assertEqual(len(self.boundary.sent), ACTION_COUNT)
        self.assertEqual(self.boundary.index, 9)

    def test_selected_actions_are_copied_without_modification(self):
        selected = selected_actions(self.chunk)
        np.testing.assert_array_equal(selected, self.chunk[:8])
        self.assertFalse(selected.flags.writeable)
        self.chunk[0, 0] = 999
        self.assertEqual(selected[0, 0], 2)

    def test_wait_uses_absolute_deadline_after_early_wakeup(self):
        clock = [0]
        def sleep(seconds):
            clock[0] += max(100_000, int(seconds * 1e9 / 2))
        wait_until(1_000_000, now=lambda: clock[0], sleep=sleep)
        self.assertGreaterEqual(clock[0], 1_000_000)
        self.assertLess(clock[0], 1_100_000)

    def test_no_catchup_burst_after_missed_interval(self):
        self.boundary.wait = lambda deadline: setattr(self.boundary, "now", deadline + PERIOD_NS)
        result = self.run_playback()
        self.assertEqual(result["status"], "aborted_before_action_0")
        self.assertEqual([r["type"] for r in self.boundary.sent], ["abort_hold"])

    def test_correlation_preserves_identities_and_raw_records(self):
        records = [{"kind": "sent", "run": "unique-run", "source_stamp_ns": 123}]
        rows = [{"run": "unique-run", "source_stamp_ns": 123, "type": "terminal_hold"}]
        analyzer = Mock(return_value={"targets": [{"source_stamp_ns": 123, "status": "unmatched"}]})
        correlate(records, rows, analyzer)
        analyzer.assert_called_once_with(records)
        self.assertEqual(rows[0]["run"], "unique-run")
        self.assertEqual(rows[0]["timing"]["source_stamp_ns"], 123)
        self.assertEqual(records[0]["run"], "unique-run")


class StreamingBoundaryTests(unittest.TestCase):
    def setUp(self):
        import threading
        from types import SimpleNamespace
        from saps.physical.streaming_ros import StreamingBoundary
        self.boundary = object.__new__(StreamingBoundary)
        b = self.boundary
        b.lock = threading.RLock()
        b.run, b.clock = "run-identity", {}
        b.records, b.stamps, b.errors = [], set(), {}
        b.node = Mock()
        b.node.get_clock.return_value.now.return_value.nanoseconds = 1_000_000_000
        b._readiness = Mock(return_value=[])
        b.publisher = Mock()
        b.message_type = lambda: SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
        )
        b.franka_state = ({}, 1_000_000_000, 1_000_000_000, ())
        b.controller_readiness_reasons = ()
        b.active_dispatch_ns = 1_000_000_000
        b.controller_state = ([0.0] * 39, 1_000_000_000)
        b.controller_state[0][35] = 1.0
        b.controller_state[0][38] = 1.0
        self.state = MeasuredState(np.zeros(7), 990_000_000,
                                   990_000_000, 1_000_000_000)
        self.row = {"q_target": np.arange(7, dtype=float),
                    "type": "policy_action", "scheduled_monotonic_ns": 10**9}
        self.clock = patch("saps.physical.streaming_ros.time.monotonic_ns",
                           return_value=1_000_000_000)
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def test_publish_has_exact_seven_names_no_gripper_and_run_stamp(self):
        self.boundary.publish(self.row, self.state, 10**9)
        msg = self.boundary.publisher.publish.call_args.args[0]
        self.assertEqual(msg.name, [f"fr3_joint{i}" for i in range(1, 8)])
        self.assertEqual(msg.position, list(range(7)))
        self.assertEqual(msg.header.frame_id, "run-identity")
        self.assertEqual(msg.header.stamp.sec, 1)
        self.assertEqual(msg.header.stamp.nanosec, 0)
        self.assertEqual(self.boundary.records[0]["source_stamp_ns"], 10**9)
        self.assertEqual(self.row["actual_publish_t0_ns"], 10**9)

    def test_duplicate_source_stamp_never_published_twice(self):
        self.boundary.publish(self.row, self.state, 10**9)
        with self.assertRaisesRegex(RuntimeError, "duplicate"):
            self.boundary.publish(self.row, self.state, 10**9)
        self.assertEqual(self.boundary.publisher.publish.call_count, 1)

    def test_reference_expiring_at_boundary_is_not_published(self):
        stale = MeasuredState(np.zeros(7), 800_000_000, 800_000_000, 10**9)
        with self.assertRaisesRegex(RuntimeError, "expired"):
            self.boundary.publish(self.row, stale, 10**9)
        self.boundary.publisher.publish.assert_not_called()

    def test_controller_readiness_failure_never_publishes(self):
        self.boundary._readiness.return_value = ["inactive"]
        with self.assertRaisesRegex(RuntimeError, "inactive"):
            self.boundary.publish(self.row, self.state, 10**9)
        self.boundary.publisher.publish.assert_not_called()

    def test_failed_publish_keeps_t0_and_error_evidence_without_retry(self):
        self.boundary.publisher.publish.side_effect = RuntimeError("DDS failure")
        with self.assertRaisesRegex(RuntimeError, "DDS failure"):
            self.boundary.publish(self.row, self.state, 10**9)
        self.assertEqual(self.boundary.publisher.publish.call_count, 1)
        self.assertEqual(self.boundary.records[0]["publication_call_outcome"],
                         "publication_error")
        self.assertEqual(self.row["actual_publish_t0_ns"], 10**9)

    def test_late_boundary_does_not_emit_catchup_target(self):
        self.row["scheduled_monotonic_ns"] = 10**9 - PERIOD_NS
        with self.assertRaisesRegex(RuntimeError, "missed_complete_interval"):
            self.boundary.publish(self.row, self.state, 10**9)
        self.boundary.publisher.publish.assert_not_called()

    def test_franka_health_accepts_idle_without_errors_or_collision(self):
        message = SimpleNamespace(
            robot_mode=1,
            ROBOT_MODE_IDLE=1,
            ROBOT_MODE_MOVE=2,
            current_errors=FakeErrorState(
                joint_position_limits_violation=False,
                cartesian_position_limits_violation=False,
            ),
            collision_indicators=SimpleNamespace(
                is_joint_collision=[False] * 7,
                is_cartesian_linear_collision=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0,
                ),
                is_cartesian_angular_collision=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0,
                ),
            ),
            control_command_success_rate=1.0,
        )

        evidence, reasons = franka_health(message)

        self.assertEqual(reasons, [])
        self.assertEqual(evidence["robot_mode"], 1)
        self.assertFalse(any(evidence["current_errors"].values()))

    def test_franka_health_rejects_error_and_collision(self):
        message = SimpleNamespace(
            robot_mode=2,
            ROBOT_MODE_IDLE=1,
            ROBOT_MODE_MOVE=2,
            current_errors=FakeErrorState(
                joint_position_limits_violation=True,
            ),
            collision_indicators=SimpleNamespace(
                is_joint_collision=[False, True] + [False] * 5,
                is_cartesian_linear_collision=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0,
                ),
                is_cartesian_angular_collision=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0,
                ),
            ),
            control_command_success_rate=1.0,
        )

        _, reasons = franka_health(message)

        self.assertIn("franka_current_errors_present", reasons)
        self.assertIn(
            "franka_collision_indicator_active_or_invalid",
            reasons,
        )

    def test_delivery_validation_accepts_complete_nine_target_chain(self):
        rows, summary = valid_delivery_fixture()

        result = validate_c1b_delivery(rows, summary)

        self.assertTrue(result["accepted"])
        self.assertEqual(result["reasons"], [])
        self.assertEqual(
            result["controller_sequences"],
            list(range(20, 29)),
        )

    def test_delivery_validation_rejects_missing_application_or_loss(self):
        rows, summary = valid_delivery_fixture()

        rows[4]["timing"]["status"] = "accepted_application_unresolved"
        rows[4]["timing"]["application_records"] = 0
        summary["evidence_id_gaps"]["controller:controller-instance"] = 1

        result = validate_c1b_delivery(rows, summary)

        self.assertFalse(result["accepted"])
        self.assertIn("row_4_status_'accepted_application_unresolved'", result["reasons"])
        self.assertIn("row_4_application_records_0", result["reasons"])
        self.assertIn("evidence_id_gap_observed", result["reasons"])

    def test_runtime_health_accepts_healthy_post_hold_evidence(self):
        rows, _ = valid_delivery_fixture()
        rows[-1]["actual_publish_t0_ns"] = 1_800_000

        result = validate_c1b_runtime_health(
            valid_runtime_health_fixture(),
            rows,
        )

        self.assertTrue(result["accepted"])
        self.assertEqual(result["reasons"], [])

    def test_runtime_health_rejects_transient_franka_violation(self):
        rows, _ = valid_delivery_fixture()
        rows[-1]["actual_publish_t0_ns"] = 1_800_000

        records = valid_runtime_health_fixture()
        records.insert(
            1,
            {
                "kind": "franka_state",
                "receive_monotonic_ns": 1_750_000,
                "health_reasons": [
                    "franka_collision_indicator_active_or_invalid"
                ],
            },
        )

        result = validate_c1b_runtime_health(records, rows)

        self.assertFalse(result["accepted"])
        self.assertIn(
            "franka_health_violation_observed",
            result["reasons"],
        )

if __name__ == "__main__":
    unittest.main()
