"""Deterministic C1-C1 sequencing and evidence tests; no ROS or live server."""

from __future__ import annotations

import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np

# Existing P0 fixtures use sibling imports under unittest discovery.
with patch.object(sys, "path", [str(Path(__file__).parent), *sys.path]):
    import test_physical_live_shadow as shadow
    from test_streaming_playback import valid_delivery_fixture

from saps.physical.live_inference_hold import (
    ABORT_HOLD, POST_HOLD, PRE_HOLD, finalize_hold_sequence, run_hold_sequence,
    validate_holds,
)
from saps.physical.live_inference_hold_ros import run_inference_hold
from saps.physical.live_shadow import (
    CameraPairGate, prepare_live_request, validate_live_policy,
)
from saps.physical.shadow_config import load_shadow_config
from saps.physical.streaming_playback import MeasuredState
from saps.policies.openpi_droid import OpenPiDroidPolicy

IDENTITY = ("controller-instance", 1234)


class FakeBoundary:
    def __init__(self) -> None:
        self.now = 1_000_000_000
        self.run = "unique-run"
        self.lock = threading.RLock()
        self.records = [self.batch()]
        self.pending = []
        self.missing = set()
        self.readiness = ()
        self.counter = 0
        self.frozen_joint = False

    def batch(self, application=None) -> dict:
        sample = {"activation": IDENTITY[1], "period_ns": 1_000_000}
        if application is not None:
            sample["application"] = application
        return {"kind": "controller", "data": {
            "event": "samples", "instance": IDENTITY[0], "samples": [sample],
        }}

    def snapshot(self) -> MeasuredState:
        self.counter += 1
        stamp = 1_000_000_000 if self.frozen_joint else self.now
        return MeasuredState(np.full(7, self.counter * .01), stamp,
                             self.now, self.now, self.readiness)

    def publish(self, row, state, start) -> None:
        self.now += 100
        stamp = self.now
        row.update(run=self.run, source_stamp_ns=stamp,
                   actual_publish_t0_ns=self.now,
                   publication_call_outcome="returned")
        self.records.append(dict(kind="sent", run=self.run,
                                 source_stamp_ns=stamp, q=state.q.tolist(),
                                 t0_ns=self.now, publication_call_outcome="returned"))
        self.records.append({"kind": "forwarder", "data": {
            "source_frame_id": self.run,
            "source_stamp_sec": stamp // 10**9,
            "source_stamp_nanosec": stamp % 10**9,
            "outcome": "forwarded", "server_receipt_monotonic_ns": self.now + 10,
            "publication_call_start_monotonic_ns": self.now + 20,
        }})
        self.records.append({"kind": "controller", "data": {
            "event": "callback", "source_frame_id": self.run,
            "source_stamp_ns": stamp, "outcome": "accepted",
            "t3_ns": self.now + 30,
        }})
        if row["type"] not in self.missing:
            self.pending.append(self.batch({
                "source_stamp_ns": stamp, "q_desired": state.q.tolist(),
                "t4_ns": self.now + 40,
            }))

    def sleep(self, seconds) -> None:
        self.now += int(seconds * 1e9)
        self.records.extend(self.pending)
        self.pending.clear()
        state = [0.] * 39
        state[38] = 1.
        self.records.extend([
            dict(kind="franka_state", receive_monotonic_ns=self.now,
                 health_reasons=[]),
            dict(kind="controller_state", receive_monotonic_ns=self.now, data=state),
            dict(kind="controller_readiness", response_monotonic_ns=self.now,
                 readiness_reasons=[]),
            dict(kind="measured_joint", receive_monotonic_ns=self.now),
        ])


def fake_analyze(records) -> dict:
    """Fake only the external analyzer's schema; real correlate joins T1/T3."""
    _, summary = valid_delivery_fixture()
    summary["targets"] = []
    applications = [s["application"] for r in records
                    if r["kind"] == "controller"
                    and r["data"].get("event") == "samples"
                    for s in r["data"]["samples"] if "application" in s]
    for index, sent in enumerate(r for r in records if r["kind"] == "sent"):
        found = [a for a in applications
                 if a["source_stamp_ns"] == sent["source_stamp_ns"]]
        target = dict(source_stamp_ns=sent["source_stamp_ns"],
                      status="applied" if len(found) == 1 else "unresolved",
                      forwarder_records=1, callback_records=1,
                      application_records=len(found), instance=IDENTITY[0],
                      activation=IDENTITY[1], sequence=20 + index)
        if found:
            target.update(q_desired=found[0]["q_desired"], intervals_ns={
                "t0_t1": 10, "t1_t2": 10, "t2_t3": 10, "t3_t4": 10,
                "t0_t4": found[0]["t4_ns"] - sent["t0_ns"],
            })
        summary["targets"].append(target)
    return summary


class HoldSequenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.output = Path(self.directory.name)
        metric = patch("saps.physical.single_action.singularity_metric",
                       return_value={"rejected": False})
        metric.start()
        self.addCleanup(metric.stop)
        self.boundary = FakeBoundary()
        self.result = {}
        self.client = shadow.FakeClient()
        self.policy = OpenPiDroidPolicy(client=self.client)
        self.config = load_shadow_config(shadow.CONFIG_PATH)
        collector = shadow.FakeCollector()
        validate_live_policy(self.policy, self.result)
        self.observation, self.sample = prepare_live_request(
            collector=collector, output_dir=self.output, index=0,
            policy_episode_seed=17, observation_timeout=1,
            spin_once=collector.spin, config=self.config, record=self.result,
            gate=CameraPairGate(),
        )
        self.original_infer = self.client.infer

        def infer(request):
            self.assertEqual(len(self.sent()), 1)
            self.assertEqual(self.result["rows"][0]["type"], PRE_HOLD)
            self.assertTrue(self.result["rows"][0]["application_confirmation"]["accepted"])
            self.assertLessEqual(self.result["rows"][0]["timing"]["t4_ns"],
                                 self.result["policy_inference"]["request_start_monotonic_ns"])
            self.boundary.sleep(.2)
            self.assertEqual(len(self.sent()), 1)
            return self.original_infer(request)
        self.client.infer = infer

    def sent(self) -> list:
        return [r for r in self.boundary.records if r["kind"] == "sent"]

    def run_sequence(self) -> bool:
        run_hold_sequence(
            boundary=self.boundary,
            limits={"lower_rad": np.full(7, -3.), "upper_rad": np.full(7, 3.)},
            analyzer=fake_analyze, expected_identity=IDENTITY, analysis_start=0,
            application_timeout=.05, observation_timeout=.05,
            result=self.result, now=lambda: self.boundary.now,
            sleep=self.boundary.sleep, inference_arguments=dict(
                observation=self.observation, sample_dir=self.sample,
                policy=self.policy, index=0, policy_episode_seed=17,
                ros_now=lambda: 10.03, config=self.config,
            ),
        )
        return finalize_hold_sequence(records=self.boundary.records,
                                      result=self.result, analyzer=fake_analyze,
                                      expected_identity=IDENTITY)

    def assert_zero_actions(self) -> None:
        self.assertEqual(self.result["policy_actions_executed"], 0)
        self.assertEqual(self.result["gripper_commands_issued"], 0)
        for row in self.result["rows"]:
            self.assertIsNone(row["raw_policy_action"])
            np.testing.assert_array_equal(row["q_target"], row["q_ref"])

    def test_confirm_before_infer_no_publication_and_save_native_chunk(self) -> None:
        self.assertTrue(self.run_sequence(), self.result)
        self.assertEqual(len(self.client.requests), 1)
        self.assertEqual(self.client.requests[0]["replan_index"], 0)
        self.assertEqual(self.client.requests[0]["policy_episode_seed"], 17)
        self.assertEqual([r["type"] for r in self.result["rows"]], [PRE_HOLD, POST_HOLD])
        with np.load(self.sample / "actions.npz") as archive:
            np.testing.assert_array_equal(archive["actions"],
                                          np.linspace(-2, 2, 120).reshape(15, 8))
        pre, post = self.result["rows"]
        self.assertGreater(post["q_ref_source_stamp_ns"], pre["q_ref_source_stamp_ns"])
        self.assertFalse(np.array_equal(pre["q_ref"], post["q_ref"]))
        self.assertTrue(self.result["publication_audit"]["no_publication_during_inference"])
        for filename in ("request.json", "response.json", "request_timing.json",
                         "model_audit.json", "native_rgb.npz", "observation.npz"):
            self.assertTrue((self.sample / filename).is_file())
        self.assert_zero_actions()

    def test_missing_pre_application_never_calls_inference(self) -> None:
        self.boundary.missing.add(PRE_HOLD)
        self.assertFalse(self.run_sequence())
        self.assertEqual(self.client.requests, [])
        self.assertEqual(len(self.sent()), 1)
        self.assert_zero_actions()

    def test_pre_readiness_failure_never_publishes_or_infers(self) -> None:
        self.boundary.readiness = ("unhealthy",)
        self.assertFalse(self.run_sequence())
        self.assertEqual(self.sent(), [])
        self.assertEqual(self.client.requests, [])

    def test_wrong_shape_saved_but_never_executed(self) -> None:
        self.client.shape = (14, 8)
        self.assertFalse(self.run_sequence())
        self.assertEqual(self.result["failure_hold"], "applied")
        self.assertEqual(len(self.client.requests), 1)
        self.assertTrue((self.sample / "actions.npz").is_file())
        self.assert_zero_actions()

    def test_nonfinite_policy_response_cannot_execute(self) -> None:
        original = self.original_infer
        def invalid(request):
            response = original(request)
            response["actions"][0, 0] = np.nan
            return response
        self.original_infer = invalid
        self.assertFalse(self.run_sequence())
        self.assertEqual(self.result["failure_hold"], "applied")
        self.assert_zero_actions()

    def test_inference_exception_attempts_one_failure_hold(self) -> None:
        self.client.error = TimeoutError("inference timeout")
        self.assertFalse(self.run_sequence())
        self.assertEqual([r["type"] for r in self.result["rows"]], [PRE_HOLD, ABORT_HOLD])
        self.assertEqual(self.result["failure_hold"], "applied")
        self.assertEqual(len(self.client.requests), 1)
        self.assert_zero_actions()

    def test_unsafe_failure_hold_is_not_published_or_retried(self) -> None:
        def fail(request):
            self.boundary.readiness = ("collision",)
            raise RuntimeError("failed")
        self.original_infer = fail
        self.assertFalse(self.run_sequence())
        self.assertEqual(len(self.sent()), 1)
        self.assertEqual(self.result["failure_hold"], "unsafe_or_unconfirmed_no_retry")
        self.assert_zero_actions()

    def test_failure_hold_application_timeout_has_no_retry(self) -> None:
        self.client.error = RuntimeError("server")
        self.boundary.missing.add(ABORT_HOLD)
        self.assertFalse(self.run_sequence())
        self.assertEqual(len(self.sent()), 2)
        self.assertEqual(self.result["failure_hold"], "unsafe_or_unconfirmed_no_retry")

    def test_post_hold_application_failure_fails_without_retry(self) -> None:
        self.boundary.missing.add(POST_HOLD)
        self.assertFalse(self.run_sequence())
        self.assertEqual(len(self.sent()), 2)
        self.assertNotIn("failure_hold", self.result)
        self.assert_zero_actions()

    def test_post_hold_requires_advancing_measured_stamp(self) -> None:
        self.boundary.frozen_joint = True
        self.assertFalse(self.run_sequence())
        self.assertEqual(len(self.sent()), 1)

    def test_expired_observation_prevents_policy_submission(self) -> None:
        self.config["freshness"]["maximum_source_age_seconds"] = .001
        self.assertFalse(self.run_sequence())
        self.assertEqual(self.client.requests, [])
        self.assert_zero_actions()

    def test_late_extra_publication_invalidates_final_acceptance(self) -> None:
        self.assertTrue(self.run_sequence())
        self.boundary.records.append(dict(
            kind="sent", run="foreign-run", source_stamp_ns=999,
            q=[0.] * 7, publication_call_outcome="returned",
            t0_ns=self.result["policy_inference"]["request_start_monotonic_ns"] + 10,
        ))
        self.assertFalse(finalize_hold_sequence(
            records=self.boundary.records, result=self.result,
            analyzer=fake_analyze, expected_identity=IDENTITY,
        ))
        self.assertFalse(self.result["publication_audit"]["no_publication_during_inference"])

    def test_mismatched_run_stamp_target_and_controller_are_rejected(self) -> None:
        self.assertTrue(self.run_sequence())
        for kind in ("run", "stamp", "target", "controller", "duplicate", "loss"):
            with self.subTest(kind=kind):
                records = copy.deepcopy(self.boundary.records)
                rows = copy.deepcopy(self.result["rows"])
                analyzer = fake_analyze
                if kind == "run":
                    next(r for r in records if r["kind"] == "forwarder")["data"]["source_frame_id"] = "wrong"
                elif kind == "stamp":
                    next(r for r in records if r["kind"] == "controller" and r["data"]["event"] == "callback")["data"]["source_stamp_ns"] = 999
                elif kind == "target":
                    rows[0]["q_target"][0] += .1
                elif kind == "controller":
                    records[0]["data"]["instance"] = "restarted"
                elif kind == "duplicate":
                    records.append(copy.deepcopy(next(r for r in records if r["kind"] == "forwarder")))
                else:
                    def analyzer(records):
                        result = fake_analyze(records)
                        result["evidence_id_gaps"]["controller:controller-instance"] = 1
                        return result
                self.assertFalse(validate_holds(records, rows, analyzer, IDENTITY)["accepted"])

    def test_health_evidence_must_extend_beyond_post_hold_t4(self) -> None:
        self.assertTrue(self.run_sequence())
        post_t0 = self.result["rows"][-1]["actual_publish_t0_ns"]
        for record in self.boundary.records:
            for key in ("receive_monotonic_ns", "response_monotonic_ns"):
                if record.get(key, 0) > post_t0:
                    record[key] = post_t0 + 1
        self.result["status"] = "holds_applied"
        self.assertFalse(finalize_hold_sequence(
            records=self.boundary.records, result=self.result,
            analyzer=fake_analyze, expected_identity=IDENTITY,
        ))
        self.assertFalse(self.result["runtime_health_validation"]["accepted"])

    def test_wrong_expected_controller_blocks_inference(self) -> None:
        original = fake_analyze
        def wrong_controller(records):
            summary = original(records)
            for target in summary["targets"]:
                target["activation"] += 1
            return summary
        with patch(__name__ + ".fake_analyze", side_effect=wrong_controller):
            self.assertFalse(self.run_sequence())
        self.assertEqual(self.client.requests, [])
        self.assertEqual(len(self.sent()), 1)

    def test_health_violation_during_drain_fails(self) -> None:
        self.assertTrue(self.run_sequence())
        self.boundary.records.append(dict(kind="franka_state", health_reasons=["collision"],
                                          receive_monotonic_ns=self.boundary.now))
        self.assertFalse(finalize_hold_sequence(records=self.boundary.records,
                                               result=self.result, analyzer=fake_analyze,
                                               expected_identity=IDENTITY))


class LifecycleTests(unittest.TestCase):
    def test_startup_failure_is_saved_and_output_is_exclusive(self) -> None:
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as tmp:
            args = SimpleNamespace(
                execute=True, prompt="pick", policy_episode_seed=17,
                observation_timeout=1., policy_timeout=1.,
                application_confirmation_timeout=1., config=shadow.CONFIG_PATH,
                output_dir=Path(tmp) / "exclusive", lab_stack_dir=Path(tmp),
            )
            with patch("saps.physical.live_inference_hold_ros.pinned_timing_helpers",
                       side_effect=ValueError("wrong pin")):
                self.assertEqual(run_inference_hold(args), 1)
            record = json.loads((args.output_dir / "run.json").read_text())
            self.assertEqual(record["policy_actions_executed"], 0)
            self.assertEqual(record["gripper_commands_issued"], 0)
            with self.assertRaises(FileExistsError):
                run_inference_hold(args)


if __name__ == "__main__":
    unittest.main()
