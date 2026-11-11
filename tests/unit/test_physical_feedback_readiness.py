"""Non-actuating checks for the November 11 feedback failure conditions."""

from __future__ import annotations

from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from saps.physical.streaming_ros import (
    FORWARDER, JOINT_TOPIC, StreamingBoundary,
)


class FeedbackReadinessTests(unittest.TestCase):
    def setUp(self) -> None:
        self.boundary = object.__new__(StreamingBoundary)
        boundary = self.boundary
        boundary.errors = {}
        boundary.active_dispatch_ns = 10_000_000_000
        boundary.controller_readiness_reasons = ()
        boundary.franka_state = ({}, 100_000_000_000,
                                 10_000_000_000, ())
        data = [0.0] * 39
        data[35], data[38] = 100.0, 1.0
        boundary.controller_state = (data, 10_000_000_000)
        boundary.node = Mock()
        boundary.node.get_name.return_value = "saps_physical_c1b"
        boundary.node.get_subscriptions_info_by_topic.return_value = [
            SimpleNamespace(node_name=FORWARDER)]
        boundary.node.get_publishers_info_by_topic.side_effect = lambda topic: [
            SimpleNamespace(node_name=("joint_state_broadcaster"
                if topic == JOINT_TOPIC else "saps_physical_c1b"))]

    def reasons(self, delay_ns: int = 0) -> list[str]:
        return self.boundary._readiness(10_000_000_000 + delay_ns,
                                        100_000_000_000 + delay_ns)

    def test_normal_feedback_and_exact_age_boundary_are_ready(self) -> None:
        self.assertEqual(self.reasons(), [])
        self.assertEqual(self.reasons(100_000_000), [])

    def test_shared_callback_delay_rejects_both_states_while_active(self) -> None:
        self.assertEqual(self.boundary.controller_state[0][38], 1.0)
        self.assertCountEqual(self.reasons(108_000_000), [
            "franka_state_stale_or_future",
            "controller_state_invalid_inactive_or_stale",
        ])

    def test_recent_callback_does_not_make_old_source_stamp_fresh(self) -> None:
        # Run B received a Franka callback just before the failing snapshot,
        # but its source stamp still predated the delivery gap.
        self.boundary.franka_state = ({}, 100_000_000_000,
                                     10_107_500_000, ())
        self.assertIn("franka_state_stale_or_future",
                      self.reasons(108_000_000))

    def test_future_source_stamps_are_rejected_in_their_ros_domain(self) -> None:
        self.boundary.franka_state = ({}, 100_000_000_001,
                                     10_000_000_000, ())
        self.boundary.controller_state[0][35] = 100.001
        self.assertCountEqual(self.reasons(), [
            "franka_state_stale_or_future",
            "controller_state_invalid_inactive_or_stale",
        ])

    def test_controller_inactive_and_missing_evidence_fail_closed(self) -> None:
        self.boundary.controller_state[0][38] = 0.0
        self.assertEqual(self.reasons(), [
            "controller_state_invalid_inactive_or_stale"])
        self.boundary.controller_state = None
        self.assertEqual(self.reasons(), ["controller_state_missing"])

    def test_refreshed_states_restore_snapshot_readiness(self) -> None:
        self.assertTrue(self.reasons(108_000_000))
        self.boundary.franka_state = ({}, 100_108_000_000,
                                     10_108_000_000, ())
        self.boundary.controller_state[0][35] = 100.108
        self.boundary.controller_state = (
            self.boundary.controller_state[0], 10_108_000_000)
        self.assertEqual(self.reasons(108_000_000), [])


if __name__ == "__main__":
    unittest.main()
