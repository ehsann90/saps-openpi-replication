"""C1-C2 full-loop fixtures: real inference contracts, no ROS or hardware."""

from __future__ import annotations

import dataclasses
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from saps.physical.policy_execution import run_policy_episode, TaskOutcome
from saps.physical.shadow_config import load_shadow_config
from saps.policies.openpi_droid import OpenPiDroidPolicy
from test_live_inference_hold import FakeBoundary, fake_analyze, IDENTITY
from test_physical_live_shadow import FakeClient, FakeCollector, CONFIG_PATH


class Boundary(FakeBoundary):
    def __init__(self) -> None:
        super().__init__()
        self.errors = {}
        self.sequence = 19
        self.target = np.zeros(7)
        self.fail_action = None
        self.frozen_receive = False

    def snapshot(self):
        state = super().snapshot()
        if self.frozen_receive:
            state = dataclasses.replace(state, receive_monotonic_ns=1)
        return state

    def publish(self, row, state, start):
        if row["type"] == "policy_action" and row["action_index"] == self.fail_action:
            raise RuntimeError("injected action failure")
        super().publish(row, dataclasses.replace(state, q=row["q_target"]), start)
        row["schedule_lateness_ns"] = self.now - row["scheduled_monotonic_ns"]
        self.sequence += 1
        self.target = np.array(row["q_target"])

    def sleep(self, seconds):
        # Advance even the scheduler's final spin, without real sleeping.
        super().sleep(max(seconds, .000001))
        for record in self.records[-4:]:
            if record["kind"] == "controller_state":
                record["data"][14:21] = self.target.tolist()
                record["data"][37] = self.sequence


class Collector(FakeCollector):
    def __init__(self, boundary):
        super().__init__()
        self.boundary = boundary
        self.old_source = None

    def spin(self):
        self.boundary.sleep(.01)
        super().spin()

    def latest_signature(self):
        return (self.boundary.now / 1e9,) * 4

    def assemble(self):
        observation = super().assemble()
        changes = {}
        for name in ("wrist_frame", "exterior_frame", "joint_snapshot", "gripper_snapshot"):
            source = getattr(observation, name)
            stamp = self.boundary.now / 1e9
            if name == self.old_source:
                stamp = 1.
            changes[name] = dataclasses.replace(
                source, stamp=dataclasses.replace(source.stamp, ros_seconds=stamp))
        changes["timing"] = dataclasses.replace(
            observation.timing, oldest_source_ros_seconds=self.boundary.now / 1e9)
        return dataclasses.replace(observation, **changes)


class EpisodeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.output = Path(self.tmp.name)
        self.boundary = Boundary()
        self.collector = Collector(self.boundary)
        self.client = FakeClient()
        self.policy = OpenPiDroidPolicy(client=self.client)
        self.result = {}
        self.metric = patch("saps.physical.single_action.singularity_metric",
                            return_value={"rejected": False})
        self.metric.start()
        self.addCleanup(self.metric.stop)
        self.original_infer = self.client.infer
        self.main_hook = lambda: None

        def infer(request):
            before = len([r for r in self.boundary.records if r["kind"] == "sent"])
            self.boundary.sleep(.02)
            if request["policy_episode_seed"] == 20260827:
                pre = self.result["replans"][-1]["pre_hold"]
                self.assertTrue(pre["application_confirmation"]["accepted"])
                self.main_hook()
            self.assertEqual(before, len([r for r in self.boundary.records
                                          if r["kind"] == "sent"]))
            return self.original_infer(request)
        self.client.infer = infer

    def run_episode(self, cap=1, provider=lambda _: TaskOutcome.CONTINUE,
                    max_replans=3, seed=20260917):
        run_policy_episode(
            boundary=self.boundary, collector=self.collector, policy=self.policy,
            config=load_shadow_config(CONFIG_PATH), output_dir=self.output,
            limits={"lower_rad": np.full(7, -3.), "upper_rad": np.full(7, 3.)},
            analyzer=fake_analyze, expected_identity=IDENTITY, analysis_start=0,
            application_timeout=.05, observation_timeout=.05,
            policy_episode_seed=20260827, warmup_policy_seed=seed,
            max_replans=max_replans, max_executed_policy_chunks=cap,
            spin_once=self.collector.spin, ros_now=lambda: self.boundary.now / 1e9,
            result=self.result, outcome_provider=provider,
            now=lambda: self.boundary.now, sleep=self.boundary.sleep,
        )

    def test_first_test_cap_and_separate_latency(self):
        self.run_episode()
        self.assertEqual(self.result["status"], "success", self.result.get("error"))
        self.assertEqual(self.result["warmup_requests"], 1)
        self.assertEqual(self.result["main_policy_requests"], 1)
        self.assertEqual(self.result["completed_main_replans"], 1)
        self.assertEqual(self.result["policy_actions_executed"], 8)
        self.assertEqual(self.result["terminal_holds_applied"], 1)
        self.assertEqual(self.result["gripper_commands_issued"], 0)
        self.assertEqual(self.result["termination"], {
            "termination_reason": "test_chunk_limit_reached", "task_outcome": "continue"})
        self.assertIn("client_round_trip_seconds", self.result["warmup"])
        self.assertIn("client_round_trip_seconds",
                      self.result["replans"][0]["inference"]["response"])
        self.assertEqual([(r["policy_episode_seed"], r["replan_index"],
                           r.get("audit_model_input", False)) for r in self.client.requests],
                         [(20260917, 0, False), (20260827, 0, True)])

    def test_continue_repeats_indices_seed_holds_and_only_first_audit(self):
        self.run_episode(cap=3)
        self.assertEqual(self.result["status"], "success", self.result.get("error"))
        self.assertEqual(self.result["warmup_requests"], 1)
        self.assertEqual([r["replan_index"] for r in self.client.requests], [0, 0, 1, 2])
        self.assertEqual([r["policy_episode_seed"] for r in self.client.requests[1:]],
                         [20260827] * 3)
        self.assertEqual([r.get("audit_model_input", False) for r in self.client.requests],
                         [False, True, False, False])
        previous_terminal = None
        for replan in self.result["replans"]:
            pre = replan["pre_hold"]
            if previous_terminal:
                self.assertGreater(pre["actual_publish_t0_ns"],
                                   previous_terminal["confirmation_monotonic_ns"])
            self.assertTrue(replan["inference"]["hold_audit"]["accepted"])
            request = replan["policy_observation"]["request"]
            barrier = replan["pre_hold_source_barrier_ros_seconds"]
            stamps = [request["cameras"][role]["stamp"]["ros_seconds"]
                      for role in ("wrist", "exterior")]
            stamps += [request["joint_stamp"]["ros_seconds"],
                       request["gripper"]["stamp"]["ros_seconds"]]
            self.assertTrue(all(stamp > barrier for stamp in stamps))
            self.assertGreater(barrier,
                               self.result["post_warmup_source_barrier_ros_seconds"])
            previous_terminal = replan["terminal_hold"]

    def test_fresh_independent_anchors_exact_deadlines_and_terminal(self):
        self.run_episode()
        replan = self.result["replans"][0]
        self.assertEqual([r["action_index"] for r in replan["actions"]], list(range(8)))
        origin = replan["schedule_origin_monotonic_ns"]
        previous_stamp = replan["pre_hold"]["q_ref_source_stamp_ns"]
        for k, row in enumerate(replan["actions"] + [replan["terminal_hold"]]):
            self.assertEqual(row["scheduled_monotonic_ns"], origin + (k * 10**9 + 7) // 15)
            self.assertGreaterEqual(row["q_ref_receive_monotonic_ns"],
                                    row["preparation_eligible_monotonic_ns"])
            self.assertGreater(row["q_ref_source_stamp_ns"], previous_stamp)
            previous_stamp = row["q_ref_source_stamp_ns"]
            if k < 8:
                np.testing.assert_allclose(row["q_target"], row["q_ref"] +
                    .2 * np.clip(row["raw_policy_action"][:7], -1, 1), atol=1e-7)
            else:
                np.testing.assert_array_equal(row["q_target"], row["q_ref"])
        self.assertLess(replan["inference"]["response_completion_monotonic_ns"],
                        replan["actions"][0]["actual_publish_t0_ns"])

    def test_each_post_hold_source_must_advance(self):
        for source in ("wrist_frame", "exterior_frame", "joint_snapshot", "gripper_snapshot"):
            with self.subTest(source=source):
                # Inject only after warm-up, at the pre-hold publication.
                with tempfile.TemporaryDirectory() as tmp:
                    self.output = Path(tmp)
                    self.boundary = Boundary()
                    self.collector = Collector(self.boundary)
                    original = self.boundary.publish
                    def publish(row, state, start):
                        original(row, state, start)
                        if row["type"] == "pre_inference_hold":
                            self.collector.old_source = source
                    self.boundary.publish = publish
                    self.client.requests.clear()
                    self.run_episode()
                    self.assertEqual(self.result["main_policy_requests"], 0)
                    self.assertEqual(self.result["policy_actions_executed"], 0)

    def test_terminal_task_outcomes_stop_before_next_inference(self):
        for outcome in (TaskOutcome.SUCCESS, TaskOutcome.FAILURE, TaskOutcome.ABORT):
            with self.subTest(outcome=outcome), tempfile.TemporaryDirectory() as tmp:
                self.output = Path(tmp)
                self.boundary = Boundary()
                self.collector = Collector(self.boundary)
                self.run_episode(cap=None, provider=lambda _: outcome)
                self.assertEqual(self.result["completed_main_replans"], 1)
                self.assertEqual(self.result["termination"]["task_outcome"], outcome.value)
                self.assertEqual(self.result["main_policy_requests"], 1)

    def test_mid_chunk_failure_suppresses_later_actions_and_holds(self):
        self.boundary.fail_action = 3
        self.run_episode(cap=3)
        self.assertEqual(self.result["policy_actions_executed"], 3)
        self.assertEqual(self.result["main_policy_requests"], 1)
        self.assertEqual(self.result["rows"][-1]["type"], "abort_hold")
        self.assertEqual(self.result["failure_hold"], "applied", self.result)

    def test_unconfirmed_terminal_never_replans(self):
        self.boundary.missing.add("terminal_hold")
        self.run_episode(cap=3)
        self.assertEqual(self.result["main_policy_requests"], 1)
        self.assertEqual(self.result["status"], "failed")
        self.assertEqual(self.result["rows"][-1]["type"], "terminal_hold")

    def test_unconfirmed_pre_hold_never_infers(self):
        self.boundary.missing.add("pre_inference_hold")
        self.run_episode()
        self.assertEqual(self.result["main_policy_requests"], 0)
        self.assertEqual(self.result["policy_actions_executed"], 0)

    def test_seed_equality_fails_before_any_request(self):
        with self.assertRaises(ValueError):
            self.run_episode(seed=20260827)
        self.assertEqual(self.client.requests, [])

    def test_hard_bound_is_explicit_limit_not_task_failure(self):
        self.run_episode(cap=None, max_replans=1)
        self.assertEqual(self.result["termination"], {
            "termination_reason": "safety_replan_limit_reached", "task_outcome": "continue"})

    def test_main_invalid_response_restores_hold_without_actions(self):
        self.main_hook = lambda: setattr(self.client, "shape", (14, 8))
        self.run_episode()
        self.assertEqual(self.result["policy_actions_executed"], 0)
        self.assertEqual(self.result["failure_hold"], "applied")

    def test_frozen_receive_time_cannot_supply_action_zero(self):
        self.main_hook = lambda: setattr(self.boundary, "frozen_receive", True)
        self.run_episode()
        self.assertEqual(self.result["policy_actions_executed"], 0)
        self.assertEqual(self.result["status"], "failed")

    def test_changed_installed_hold_blocks_all_actions(self):
        def change():
            next(r for r in reversed(self.boundary.records)
                 if r["kind"] == "controller_state")["data"][14] += .1
        self.main_hook = change
        self.run_episode()
        self.assertEqual(self.result["policy_actions_executed"], 0)
        self.assertIn("inference_hold_evidence_failed", self.result["error"])

    def test_response_seed_index_and_finite_fail_closed(self):
        original = self.original_infer
        for defect in ("seed", "index", "nan", "timeout"):
            with self.subTest(defect=defect), tempfile.TemporaryDirectory() as tmp:
                self.output = Path(tmp)
                self.boundary = Boundary()
                self.collector = Collector(self.boundary)
                def invalid(request):
                    response = original(request)
                    if request["policy_episode_seed"] == 20260827:
                        if defect == "seed":
                            response["saps_sampling"]["policy_episode_seed"] += 1
                        elif defect == "index":
                            response["saps_sampling"]["replan_index"] += 1
                        elif defect == "nan":
                            response["actions"][0, 0] = np.nan
                        else:
                            raise TimeoutError("policy timeout")
                    return response
                self.original_infer = invalid
                self.run_episode(cap=3)
                self.assertEqual(self.result["main_policy_requests"], 1)
                self.assertEqual(self.result["policy_actions_executed"], 0)
                self.assertEqual(self.result["failure_hold"], "applied")

    def test_health_failure_during_inference_stops_before_action_zero(self):
        def unhealthy():
            self.boundary.records.append({"kind": "franka_state",
                "health_reasons": ["collision"],
                "receive_monotonic_ns": self.boundary.now})
        self.main_hook = unhealthy
        self.run_episode(cap=3)
        self.assertEqual(self.result["main_policy_requests"], 1)
        self.assertEqual(self.result["policy_actions_executed"], 0)

    def test_late_drain_health_failure_prevents_outcome_and_replan(self):
        original = self.boundary.sleep
        def sleep(seconds):
            original(seconds)
            if seconds == 2.:
                self.boundary.records.append({"kind": "franka_state",
                    "health_reasons": ["collision"],
                    "receive_monotonic_ns": self.boundary.now})
        self.boundary.sleep = sleep
        self.run_episode(cap=3)
        self.assertEqual(self.result["main_policy_requests"], 1)
        self.assertEqual(self.result["completed_main_replans"], 0)
        self.assertEqual(self.result["replans"][0]["outcome_after_chunk"], "not_evaluated")

    def test_missed_tick_does_not_burst_remaining_actions(self):
        original = self.boundary.publish
        def publish(row, state, start):
            original(row, state, start)
            if row["action_index"] == 1:
                self.boundary.sleep(.2)
        self.boundary.publish = publish
        self.run_episode(cap=3)
        self.assertEqual(self.result["policy_actions_executed"], 2)
        self.assertEqual(self.result["main_policy_requests"], 1)
        self.assertEqual(self.result["rows"][-1]["type"], "abort_hold")

    def test_missing_action_application_prevents_another_chunk(self):
        self.boundary.missing.add("policy_action")
        self.run_episode(cap=3)
        self.assertEqual(self.result["main_policy_requests"], 1)
        self.assertEqual(self.result["status"], "failed")
        self.assertEqual(self.result["policy_actions_published"], 8)
        self.assertEqual(self.result["policy_actions_executed"], 0)


if __name__ == "__main__":
    unittest.main()
