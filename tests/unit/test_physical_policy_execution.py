"""C1-C2 full-loop fixtures: real inference contracts, no ROS or hardware."""

from __future__ import annotations

import dataclasses
import copy
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from saps.physical import policy_execution as execution
from saps.physical.policy_execution import (
    execution_evidence_snapshot, run_policy_episode, TaskOutcome,
)
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
        for record in self.pending:
            for sample in record["data"]["samples"]:
                sample["application"]["sequence"] = self.sequence
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


def analyze(records):
    """Keep controller sequences stable when analyzing a target-local slice."""
    summary = fake_analyze(records)
    sequences = {
        sample["application"]["source_stamp_ns"]:
        sample["application"]["sequence"]
        for record in records if record["kind"] == "controller"
        and record["data"].get("event") == "samples"
        for sample in record["data"]["samples"] if "application" in sample
    }
    for target in summary["targets"]:
        if target["source_stamp_ns"] in sequences:
            target["sequence"] = sequences[target["source_stamp_ns"]]
    return summary


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
                    max_replans=3, seed=20260917, analyzer=analyze):
        run_policy_episode(
            boundary=self.boundary, collector=self.collector, policy=self.policy,
            config=load_shadow_config(CONFIG_PATH), output_dir=self.output,
            limits={"lower_rad": np.full(7, -3.), "upper_rad": np.full(7, 3.)},
            analyzer=analyzer, expected_identity=IDENTITY, analysis_start=0,
            application_timeout=.25, observation_timeout=.05,
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

    def test_terminal_confirmation_cost_excludes_large_prior_telemetry(self):
        original = self.boundary.publish
        def publish(row, state, start):
            original(row, state, start)
            if row["action_index"] == 7:
                self.boundary.records.extend(
                    {"kind": "measured_joint", "historical": True}
                    for _ in range(20000))
        self.boundary.publish = publish

        def costly_analyze(records):
            # Deterministic cost model: full history exceeds the fixed 250 ms
            # deadline, while the cursor-local window remains cheap.
            historical = sum(r.get("historical", False) for r in records)
            self.boundary.now += historical * 15000
            return analyze(records)

        self.run_episode(cap=2, analyzer=costly_analyze)
        self.assertEqual(self.result["status"], "success", self.result.get("error"))
        for replan in self.result["replans"]:
            terminal = replan["terminal_hold"]
            self.assertEqual(terminal["application_confirmation"]["analyzed_targets"], 1)
            self.assertLess(terminal["confirmation_monotonic_ns"] -
                            terminal["actual_publish_t0_ns"], 250_000_000)
            self.assertTrue(replan["delivery_validation"]["accepted"])

    def test_chunk_audit_precedes_new_pre_hold_and_replan_one(self):
        original = self.boundary.publish
        def publish(row, state, start):
            if row["type"] == "pre_inference_hold" and len(self.result["replans"]) == 2:
                first = self.result["replans"][0]
                self.assertTrue(first["terminal_hold"]["application_confirmation"]["accepted"])
                self.assertEqual(first["terminal_hold"]["application_confirmation"]["analyzed_targets"], 1)
                self.assertTrue(first["delivery_validation"]["accepted"])
                self.assertEqual(first["delivery_validation"]["analyzed_targets"], 10)
                self.assertEqual(len(self.client.requests), 2)
            original(row, state, start)
        self.boundary.publish = publish
        self.run_episode(cap=2)
        self.assertEqual(self.result["status"], "success", self.result.get("error"))
        self.assertEqual(self.client.requests[-1]["replan_index"], 1)
        second = self.result["replans"][1]
        self.assertLess(second["pre_hold"]["confirmation_monotonic_ns"],
                        second["inference"]["request_start_monotonic_ns"])

    def test_invalid_latest_evidence_never_replans(self):
        for defect in ("identity", "stamp", "q", "duplicate", "run",
                       "forwarder_duplicate", "callback_duplicate", "t0"):
            with self.subTest(defect=defect), tempfile.TemporaryDirectory() as tmp:
                self.output = Path(tmp)
                self.boundary = Boundary()
                self.collector = Collector(self.boundary)
                original = self.boundary.publish
                def publish(row, state, start):
                    original(row, state, start)
                    if row["type"] != "terminal_hold":
                        return
                    batch = self.boundary.pending[-1]
                    application = batch["data"]["samples"][0]["application"]
                    if defect == "identity":
                        batch["data"]["instance"] = "wrong-controller"
                    elif defect == "stamp":
                        application["source_stamp_ns"] += 1
                    elif defect == "q":
                        application["q_desired"][0] += .1
                    elif defect == "duplicate":
                        self.boundary.pending.append(copy.deepcopy(batch))
                    elif defect == "run":
                        self.boundary.records[-1]["data"]["source_frame_id"] = "wrong-run"
                    elif defect == "t0":
                        self.boundary.records[-3]["t0_ns"] += 1
                    else:
                        offset = -2 if defect == "forwarder_duplicate" else -1
                        self.boundary.records.append(
                            copy.deepcopy(self.boundary.records[offset]))
                self.boundary.publish = publish
                self.run_episode(cap=2)
                self.assertEqual(self.result["status"], "failed")
                self.assertEqual(self.result["failed_phase"], "terminal_hold")
                self.assertEqual(self.result["main_policy_requests"], 1)
                self.assertIn("hold_application_confirmation_timeout", self.result["error"])

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

    def slow_post_chunk_analyzer(self, *, recover):
        self.boundary.records.extend(
            {"kind": "measured_joint", "historical": True}
            for _ in range(20000))
        stale_since = None
        refreshed = False
        original_snapshot = self.boundary.snapshot
        original_sleep = self.boundary.sleep

        def snapshot():
            state = original_snapshot()
            if stale_since is not None and not refreshed:
                state = dataclasses.replace(state, readiness_reasons=(
                    "controller_active_evidence_missing_or_expired",))
            return state

        def sleep(seconds):
            nonlocal refreshed
            original_sleep(seconds)
            # Model the background controller-manager response arriving after
            # the audit releases execution, without issuing a robot command.
            if (recover and stale_since is not None
                    and self.boundary.now - stale_since >= 20_000_000):
                refreshed = True

        self.boundary.snapshot = snapshot
        self.boundary.sleep = sleep

        def analyzer(records):
            nonlocal stale_since, refreshed
            timing = self.result["replans"][-1].get("post_chunk_timing", {})
            if ("evidence_snapshot_end_monotonic_ns" in timing
                    and "validation_completion_monotonic_ns" not in timing):
                self.boundary.now += sum(
                    r.get("historical", False) for r in records) * 20000
                stale_since = self.boundary.now
                refreshed = False
            return analyze(records)
        return analyzer

    def test_slow_audit_reacquires_readiness_before_next_pre_hold(self):
        analyzer = self.slow_post_chunk_analyzer(recover=True)
        original_publish = self.boundary.publish
        def publish(row, state, start):
            if row["type"] == "pre_inference_hold" and len(self.result["replans"]) == 2:
                barrier = self.result["replans"][0]["readiness_reacquisition"]
                self.assertTrue(barrier["accepted"])
                self.assertGreaterEqual(barrier["checks"], 3)
                self.assertFalse(state.readiness_reasons)
            original_publish(row, state, start)
        self.boundary.publish = publish
        self.run_episode(cap=2, analyzer=analyzer)
        self.assertEqual(self.result["status"], "success", self.result.get("error"))
        first, second = self.result["replans"]
        timing = first["post_chunk_timing"]
        self.assertGreater(timing["validation_completion_monotonic_ns"] -
                           timing["evidence_snapshot_end_monotonic_ns"], 300_000_000)
        ordered = [timing[key] for key in (
            "drain_completion_monotonic_ns",
            "evidence_snapshot_start_monotonic_ns",
            "evidence_snapshot_end_monotonic_ns",
            "validation_completion_monotonic_ns")]
        barrier = first["readiness_reacquisition"]
        ordered += [barrier["start_monotonic_ns"], barrier["end_monotonic_ns"],
                    second["pre_hold"]["actual_publish_t0_ns"],
                    second["inference"]["request_start_monotonic_ns"]]
        self.assertEqual(ordered, sorted(ordered))
        self.assertEqual(self.client.requests[-1]["replan_index"], 1)

    def test_readiness_timeout_issues_no_command_or_next_inference(self):
        self.run_episode(cap=2, analyzer=self.slow_post_chunk_analyzer(recover=False))
        self.assertEqual(self.result["status"], "failed")
        self.assertEqual(self.result["failed_phase"], "readiness_reacquisition")
        self.assertIn("readiness_reacquisition_timeout", self.result["error"])
        self.assertEqual(self.result["main_policy_requests"], 1)
        self.assertEqual(len(self.result["rows"]), 10)
        self.assertEqual(self.result["rows"][-1]["type"], "terminal_hold")
        barrier = self.result["replans"][0]["readiness_reacquisition"]
        self.assertFalse(barrier["accepted"])
        self.assertEqual(barrier["end_monotonic_ns"] -
                         barrier["start_monotonic_ns"], 250_000_000)
        self.assertGreater(barrier["checks"], 1)
        self.assertIn("controller_active_evidence_missing_or_expired",
                      barrier["last_readiness_reasons"])

    def test_post_chunk_validators_share_one_detached_snapshot(self):
        seen = []
        originals = [execution.validate_execution_delivery,
                     execution.audit_inference_hold,
                     execution.validate_streaming_runtime_health]
        def wrapper(original):
            def validate(records, *args, **kwargs):
                timing = self.result["replans"][-1].get("post_chunk_timing", {})
                if ("evidence_snapshot_end_monotonic_ns" in timing
                        and "validation_completion_monotonic_ns" not in timing):
                    seen.append(records)
                    self.assertFalse(any(r.get("late_append") for r in records))
                    self.assertIsNot(records[0], self.boundary.records[0])
                    self.boundary.records.append(
                        {"kind": "measured_joint", "late_append": True})
                return original(records, *args, **kwargs)
            return validate
        with patch.object(execution, "validate_execution_delivery", wrapper(originals[0])), \
             patch.object(execution, "audit_inference_hold", wrapper(originals[1])), \
             patch.object(execution, "validate_streaming_runtime_health", wrapper(originals[2])), \
             patch.object(execution, "execution_evidence_snapshot",
                          wraps=execution_evidence_snapshot) as capture:
            self.run_episode()
        self.assertEqual(self.result["status"], "success", self.result.get("error"))
        self.assertEqual(len(seen), 3)
        self.assertTrue(all(records is seen[0] for records in seen))
        # Two inference audits, one shared post-chunk snapshot, final audit.
        self.assertEqual(capture.call_count, 4)


class EvidenceSnapshotTests(unittest.TestCase):
    def test_deepcopy_releases_lock_and_excludes_concurrent_appends(self):
        boundary = SimpleNamespace(lock=threading.Lock(), records=[])
        test = self
        class Probe:
            def __deepcopy__(self, memo):
                acquired = boundary.lock.acquire(blocking=False)
                test.assertTrue(acquired, "deep copy held the boundary lock")
                try:
                    boundary.records.append({"late": True})
                finally:
                    boundary.lock.release()
                return {"nested": [1]}
        boundary.records.extend([{"skip": True}, {"probe": Probe()}])
        snapshot = execution_evidence_snapshot(boundary, 1)
        self.assertEqual(snapshot, [{"probe": {"nested": [1]}}])
        self.assertEqual(len(boundary.records), 3)


if __name__ == "__main__":
    unittest.main()
