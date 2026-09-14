"""Focused unit tests for P1-B single-action safety and ROS-boundary helpers."""

from __future__ import annotations

import unittest
from unittest.mock import Mock, patch

import numpy as np

from saps.physical.single_action import (
    first_action_from_chunk,
    safety_gate,
)
from saps.physical.single_action_ros import (
    JointTargetActionClient,
    _validate_server_evidence,
    split_ros_time,
)


class SingleActionSafetyTest(unittest.TestCase):
    def setUp(self) -> None:
        self.limits = {
            "lower_rad": np.full(7, -3.0),
            "upper_rad": np.full(7, 3.0),
        }
        self.acceptable_metric = {
            "condition_number": 8.0,
            "singular_values": np.ones(6),
            "base": "fr3_link0",
            "tip": "fr3_link8",
            "lower_threshold": 17.0,
            "hard_threshold": 30.0,
            "rejected": False,
        }

    def test_first_action_requires_native_chunk_and_copies_action_zero(self) -> None:
        actions = np.arange(120, dtype=np.float32).reshape(15, 8)
        selected = first_action_from_chunk(actions)

        np.testing.assert_allclose(selected, actions[0])
        self.assertEqual(selected.shape, (8,))
        self.assertFalse(selected.flags.writeable)

        original = float(selected[0])
        actions[0, 0] = -999.0
        self.assertEqual(float(selected[0]), original)

    def test_first_action_rejects_wrong_or_nonfinite_chunk(self) -> None:
        for actions in (
            np.zeros((8, 8), dtype=np.float32),
            np.zeros((15, 7), dtype=np.float32),
            np.zeros((15, 8), dtype=np.int64),
            np.full((15, 8), np.nan, dtype=np.float32),
        ):
            with self.assertRaises(ValueError):
                first_action_from_chunk(actions)

    @patch("saps.physical.single_action.singularity_metric")
    def test_nominal_gate_returns_authoritative_target(self, metric) -> None:
        metric.return_value = self.acceptable_metric
        action = np.array([2.0, -2.0, 0.5, 0, 0, 0, 0, 0.75])
        q = np.zeros(7)

        evidence = safety_gate(
            action,
            q,
            self.limits,
            state_age=0.01,
            receive_age=0.01,
            action_age=0.2,
            readiness_reasons=[],
        )

        self.assertTrue(evidence["accepted"])
        np.testing.assert_allclose(
            evidence["proposed_q_target_rad"],
            [0.2, -0.2, 0.1, 0, 0, 0, 0],
        )
        self.assertEqual(evidence["raw_gripper_policy_value"], 0.75)
        self.assertNotIn("gripper_reference", evidence)
        self.assertEqual(metric.call_count, 2)

    @patch("saps.physical.single_action.singularity_metric")
    def test_stale_state_and_action_are_rejected(self, metric) -> None:
        metric.return_value = self.acceptable_metric
        evidence = safety_gate(
            np.zeros(8),
            np.zeros(7),
            self.limits,
            state_age=0.1001,
            receive_age=0.1001,
            action_age=1.0001,
            readiness_reasons=[],
        )
        self.assertFalse(evidence["accepted"])
        self.assertEqual(len(evidence["rejection_reasons"]), 3)

    @patch("saps.physical.single_action.singularity_metric")
    def test_joint_margin_is_independent_rejection(self, metric) -> None:
        metric.return_value = self.acceptable_metric
        q = np.zeros(7)
        q[0] = 2.75
        action = np.zeros(8)
        action[0] = 0.5

        accepted = safety_gate(
            action,
            q,
            self.limits,
            state_age=0.01,
            receive_age=0.01,
            action_age=0.2,
            readiness_reasons=[],
        )
        self.assertTrue(accepted["accepted"])

        action[0] = 1.0
        rejected = safety_gate(
            action,
            q,
            self.limits,
            state_age=0.01,
            receive_age=0.01,
            action_age=0.2,
            readiness_reasons=[],
        )
        self.assertFalse(rejected["accepted"])
        self.assertTrue(
            any(
                "joint1_upper_margin=" in reason
                for reason in rejected["rejection_reasons"]
            )
        )

    @patch("saps.physical.single_action.singularity_metric")
    def test_singularity_rejection_is_recorded(self, metric) -> None:
        metric.return_value = {
            **self.acceptable_metric,
            "condition_number": 18.0,
            "rejected": True,
        }
        evidence = safety_gate(
            np.zeros(8),
            np.zeros(7),
            self.limits,
            state_age=0.01,
            receive_age=0.01,
            action_age=0.2,
            readiness_reasons=[],
        )
        self.assertFalse(evidence["accepted"])
        self.assertEqual(
            sum(
                "singularity_condition=" in reason
                for reason in evidence["rejection_reasons"]
            ),
            2,
        )


class SingleActionRosBoundaryTest(unittest.TestCase):
    def test_split_ros_time(self) -> None:
        self.assertEqual(split_ros_time(12.25), (12, 250_000_000))
        self.assertEqual(split_ros_time(1.9999999996), (2, 0))

        for value in (-1.0, np.inf, np.nan):
            with self.assertRaises(ValueError):
                split_ros_time(value)


    def test_wait_future_uses_private_executor(self) -> None:
        client = object.__new__(JointTargetActionClient)
        client._executor = Mock()

        future = Mock()
        future.done.return_value = True
        future.exception.return_value = None
        future.result.return_value = "result"

        result = client._wait_future(
            future,
            timeout=2.5,
            label="test future",
        )

        self.assertEqual(result, "result")
        client._executor.spin_until_future_complete.assert_called_once_with(
            future,
            timeout_sec=2.5,
        )

    def test_server_evidence_preserves_reference_and_target(self) -> None:
        reference = np.arange(7, dtype=float) / 10
        target = reference + 0.01
        evidence = {
            "reference_q": reference.tolist(),
            "requested_absolute_target": target.tolist(),
            "execution_attempted": False,
            "execution_attempts": 0,
        }

        _validate_server_evidence(
            reference_q=reference,
            target_q=target,
            evidence=evidence,
            expect_execute=False,
        )

        changed = dict(evidence)
        changed["requested_absolute_target"] = (target + 0.001).tolist()
        with self.assertRaises(RuntimeError):
            _validate_server_evidence(
                reference_q=reference,
                target_q=target,
                evidence=changed,
                expect_execute=False,
            )

    def test_plan_only_evidence_cannot_contain_execution(self) -> None:
        reference = np.zeros(7)
        target = np.full(7, 0.01)
        evidence = {
            "reference_q": reference.tolist(),
            "requested_absolute_target": target.tolist(),
            "execution_attempted": True,
            "execution_attempts": 1,
        }

        with self.assertRaises(RuntimeError):
            _validate_server_evidence(
                reference_q=reference,
                target_q=target,
                evidence=evidence,
                expect_execute=False,
            )

    def test_more_than_one_execution_attempt_is_rejected(self) -> None:
        reference = np.zeros(7)
        target = np.full(7, 0.01)
        evidence = {
            "reference_q": reference.tolist(),
            "requested_absolute_target": target.tolist(),
            "execution_attempted": True,
            "execution_attempts": 2,
        }

        with self.assertRaises(RuntimeError):
            _validate_server_evidence(
                reference_q=reference,
                target_q=target,
                evidence=evidence,
                expect_execute=True,
            )


if __name__ == "__main__":
    unittest.main()
