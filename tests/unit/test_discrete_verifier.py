"""Discrete measured-state targets, timing gates and subscription isolation."""
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest

import numpy as np

from saps.physical.discrete_verifier import candidate_target, emulate_chunk, limits_from_urdf
from saps.physical.embodiment import FR3_JOINT_NAMES
from saps.physical.verifier_observation import ContinuousBoundary


class DiscreteVerifierTest(unittest.TestCase):
    def setUp(self):
        self.limits = {"lower_rad": np.full(7, -2.0), "upper_rad": np.full(7, 2.0)}

    def test_measured_anchor_and_component_clipping(self):
        raw = np.array([2, -2, .5, -.5, 0, .2, -.2, .5])
        q = np.arange(7) / 10
        row = candidate_target(raw, q, self.limits)
        np.testing.assert_allclose(row["delta_q_rad"], [.2, -.2, .1, -.1, 0, .04, -.04])
        np.testing.assert_allclose(row["proposed_q_target_rad"], q + row["delta_q_rad"])
        self.assertEqual(row["gripper_reference"], "open")
        self.assertFalse(row["safe_to_execute"])
        self.assertEqual(raw[0], 2)
        raw[-1] = .50001
        self.assertEqual(candidate_target(raw, q, self.limits)["gripper_reference"], "closed")

    def test_limits_boundaries_and_invalid_values(self):
        self.assertTrue(candidate_target(np.zeros(8), np.full(7, 2), self.limits)["position_gate_accepted"])
        row = candidate_target(np.ones(8), np.full(7, 1.9), self.limits)
        self.assertFalse(row["position_gate_accepted"])
        self.assertAlmostEqual(row["minimum_target_margin_rad"], -.1)
        for raw in ([0] * 7, [np.nan] * 8, [np.inf] * 8):
            with self.assertRaises(ValueError):
                candidate_target(raw, np.zeros(7), self.limits)

    def test_urdf_named_order_and_validation(self):
        xml = '<robot>' + ''.join(
            f'<joint name="{name}" type="revolute"><limit lower="-{i+1}" upper="{i+1}"/></joint>'
            for i, name in reversed(list(enumerate(FR3_JOINT_NAMES)))) + '</robot>'
        limits = limits_from_urdf(xml)
        np.testing.assert_equal(limits["upper_rad"], np.arange(1, 8))
        with self.assertRaises(ValueError):
            limits_from_urdf('<robot/>')

    def schedule(self, late=False, stale=False):
        clock = [10.0]
        samples = []
        def sleep(delay):
            clock[0] += delay
            if late:
                clock[0] += .2
        def snapshot():
            q = np.full(7, len(samples) * .01)
            samples.append(q)
            stamp = SimpleNamespace(ros_seconds=clock[0] - (1 if stale else 0),
                                    receive_monotonic_seconds=clock[0])
            return SimpleNamespace(latest_joint=SimpleNamespace(position_rad=q, stamp=stamp), errors={})
        rows = emulate_chunk(np.ones((15, 8)), snapshot=snapshot, limits=self.limits,
                             request_start=9.9, response_end=10, observation_ros=9.85,
                             ros_now=lambda: clock[0], monotonic=lambda: clock[0], sleep=sleep,
                             max_state_age=.1, max_action_age=1, max_lateness=1/15)
        return rows, clock[0]

    def test_first_eight_use_fresh_measurement_not_previous_target(self):
        rows, end = self.schedule()
        self.assertEqual(len(rows), 8)
        for i, row in enumerate(rows):
            np.testing.assert_allclose(row["proposed_q_target_rad"], .2 + i * .01)
            self.assertTrue(row["verifier_accepted"])
            self.assertAlmostEqual(row["intended_monotonic_seconds"], 10 + i/15)
        self.assertAlmostEqual(end, 10 + 8/15)

    def test_stale_and_late_candidates_are_rejected(self):
        for row in self.schedule(stale=True)[0]:
            self.assertIn("stale_or_future_joint_state", row["rejection_reasons"])
        rows = self.schedule(late=True)[0]
        self.assertTrue(all(not r["verifier_accepted"] for r in rows))
        self.assertIn("expired_policy_action", rows[-1]["rejection_reasons"])

    def test_continuity_boundaries_include_no_messages(self):
        boundary = ContinuousBoundary(None)
        boundary.events = {"camera": [(1.1, 3), (1.2, 3)], "arm": []}
        evidence = boundary.continuity(1, 2)
        self.assertEqual(evidence["camera"]["nonadvancing_source_intervals"], 1)
        self.assertEqual(evidence["arm"]["maximum_receive_gap_including_boundaries_seconds"], 1)

    def test_snapshot_isolated_while_callback_can_advance(self):
        boundary = ContinuousBoundary(None)
        collector = SimpleNamespace(latest_joint=1, errors={}, rate_state={})
        snapshot = boundary.snapshot(collector)
        def callback():
            with boundary.lock:
                collector.latest_joint = 2
                collector.errors["arm"] = "invalid"
        thread = threading.Thread(target=callback)
        thread.start()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(snapshot.latest_joint, 1)
        self.assertEqual(snapshot.errors, {})
        self.assertEqual(boundary.snapshot(collector).latest_joint, 2)

    def test_make_workspace_default_is_exported_after_definition(self):
        root = Path(__file__).resolve().parents[2]
        makefile = (root / "Makefile").read_text()
        self.assertLess(makefile.index("FRANKA_ROS2_INSTALL ?="),
                        makefile.index("export FRANKA_ROS2_INSTALL"))

    def test_runtime_has_no_actuator_construction(self):
        import ast
        root = Path(__file__).resolve().parents[2]
        forbidden = {"create_publisher", "create_client", "ActionClient", "publish",
                     "send_goal_async", "call_async"}
        for name in ("discrete_verifier.py", "verifier_observation.py", "verifier_ros.py"):
            tree = ast.parse((root / "src/saps/physical" / name).read_text())
            calls = {getattr(node.func, "attr", getattr(node.func, "id", ""))
                     for node in ast.walk(tree) if isinstance(node, ast.Call)}
            self.assertFalse(calls & forbidden)


if __name__ == '__main__':
    unittest.main()
