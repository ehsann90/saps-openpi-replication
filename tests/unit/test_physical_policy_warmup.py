"""Warm-up lifecycle contracts using real adapters and deterministic fixtures."""

from __future__ import annotations

import ast
import dataclasses
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from saps.physical.live_shadow import infer_live_request
from saps.physical.policy_warmup import (
    PHYSICAL_WARMUP_POLICY_SEED, PolicyWarmup, prepare_warmed_observation,
)
from saps.physical.shadow_config import load_shadow_config
from saps.policies.openpi_droid import OpenPiDroidPolicy
from test_physical_live_shadow import CONFIG_PATH, ROOT, FakeClient, FakeCollector


class AdvancingCollector(FakeCollector):
    def latest_signature(self) -> tuple:
        delta = (self.index - 1) * .11
        return (10 + delta, 10.02 + delta, 10.01 + delta, 10.015 + delta)

    def assemble(self) -> object:
        observation = super().assemble()
        frames = [dataclasses.replace(
            frame, stamp=dataclasses.replace(
                frame.stamp, ros_seconds=stamp,
            ),
        ) for frame, stamp in zip(
            (observation.wrist_frame, observation.exterior_frame),
            self.latest_signature()[:2],
        )]
        states = [dataclasses.replace(
            state, stamp=dataclasses.replace(state.stamp, ros_seconds=stamp),
        ) for state, stamp in zip(
            (observation.joint_snapshot, observation.gripper_snapshot),
            self.latest_signature()[2:],
        )]
        return dataclasses.replace(
            observation, wrist_frame=frames[0], exterior_frame=frames[1],
            joint_snapshot=states[0], gripper_snapshot=states[1],
        )


class WarmupTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        self.client = FakeClient()
        self.policy = OpenPiDroidPolicy(client=self.client)
        self.collector = AdvancingCollector()
        self.warmup = PolicyWarmup(
            warmup_policy_seed=PHYSICAL_WARMUP_POLICY_SEED,
            policy_episode_seed=20260827,
        )
        self.record = {"completed_requests": 0, "policy_actions_executed": 0,
                       "gripper_commands_issued": 0}
        self.config = load_shadow_config(CONFIG_PATH)

    def prepare(self, **kwargs: object) -> tuple:
        return prepare_warmed_observation(
            warmup=self.warmup, collector=self.collector, policy=self.policy,
            output_dir=self.path, observation_timeout=1,
            spin_once=self.collector.spin, ros_now=lambda: 10.1,
            config=self.config, record=self.record, **kwargs,
        )

    def test_once_discarded_then_real_seed_zero_with_audit(self) -> None:
        with patch.object(self.policy, "infer", wraps=self.policy.infer) as infer:
            observation, sample_dir = self.prepare()
            self.assertIs(infer.call_args.kwargs["audit_model_input"], False)
            self.assertEqual(len(self.client.requests), 1)
            warm = self.record["warmup"]
            self.assertTrue(warm["completed"])
            self.assertTrue(warm["response_discarded"])
            self.assertEqual(warm["request_count"], 1)
            self.assertEqual(warm["actions_returned_shape"], [15, 8])
            self.assertEqual(warm["policy_seed"], 20260917)
            self.assertTrue(warm["excluded_from_episode_timing"])
            for key in ("actions_executed", "policy_actions_executed",
                        "gripper_commands_issued", "target_publications"):
                self.assertEqual(warm[key], 0)
            self.assertEqual(self.record["completed_requests"], 0)
            self.assertEqual(self.record["policy_actions_executed"], 0)
            self.assertEqual(self.record["gripper_commands_issued"], 0)
            self.assertEqual(self.record["first_real_replan_index"], 0)
            self.assertFalse((self.path / "warmup/request_0000/model_audit").exists())
            self.assertFalse((sample_dir / "response.json").exists())
            request = json.loads((sample_dir / "request.json").read_text())
            self.assertEqual(request["policy_episode_seed"], 20260827)
            self.assertEqual(request["replan_index"], 0)
            for key in ("client_round_trip_seconds", "server_timing", "policy_timing",
                        "observation_age_at_request_seconds",
                        "observation_age_at_response_seconds",
                        "request_started_monotonic_seconds",
                        "response_completed_monotonic_seconds"):
                self.assertIn(key, warm)
                self.assertNotIn(key, self.record)
            # A fixture-only real request exercises the unchanged default audit.
            infer_live_request(
                observation=observation, sample_dir=sample_dir, policy=self.policy,
                index=0, policy_episode_seed=20260827, ros_now=lambda: 10.1,
                config=self.config,
            )
            self.assertIs(infer.call_args.kwargs["audit_model_input"], True)
        self.assertEqual([r["policy_episode_seed"] for r in self.client.requests],
                         [20260917, 20260827])
        self.assertEqual([r["replan_index"] for r in self.client.requests], [0, 0])
        self.assertTrue((sample_dir / "model_audit.json").exists())
        self.assertEqual(self.record["completed_requests"], 0)
        with self.assertRaisesRegex(RuntimeError, "already attempted"):
            self.prepare()
        with self.assertRaisesRegex(RuntimeError, "already attempted"):
            self.warmup.run(
                observation=observation, sample_dir=sample_dir, policy=self.policy,
                ros_now=lambda: 10.1, config=self.config,
            )
        self.assertEqual(len(self.client.requests), 2)

    def test_equal_and_invalid_seeds_rejected_before_submission(self) -> None:
        for warm, main in ((20260827, 20260827), (20260917, 20260917),
                           (-1, 0), (1, 2**31), (True, 2)):
            with self.subTest(warm=warm, main=main), self.assertRaises(ValueError):
                PolicyWarmup(warmup_policy_seed=warm, policy_episode_seed=main)
        self.warmup.warmup_policy_seed = self.warmup.policy_episode_seed
        with self.assertRaisesRegex(ValueError, "must differ"):
            self.prepare()
        self.assertEqual(self.client.requests, [])
        self.assertEqual(self.record["warmup"]["request_count"], 0)

    def assert_failed_once(self) -> None:
        with self.assertRaises((ValueError, RuntimeError, TypeError, KeyError,
                                TimeoutError)):
            self.prepare()
        self.assertEqual(len(self.client.requests), 1)
        self.assertFalse(self.warmup.record["completed"])
        self.assertEqual(self.warmup.record["request_count"], 1)
        self.assertFalse((self.path / "request_0000").exists())
        self.assertEqual(self.record["completed_requests"], 0)
        self.assertEqual(self.warmup.record["target_publications"], 0)
        self.assertEqual(self.warmup.record["gripper_commands_issued"], 0)
        with self.assertRaises(RuntimeError):
            self.prepare()
        self.assertEqual(len(self.client.requests), 1)

    def test_transport_failure_has_no_retry_or_real_observation(self) -> None:
        self.client.error = TimeoutError("transport")
        self.assert_failed_once()
        self.assertTrue((self.path / "warmup/request_0000/request_timing.json").exists())

    def test_bad_responses_fail_closed(self) -> None:
        def change(result: dict, case: str) -> object:
            if case == "malformed":
                return []
            if case == "missing":
                del result["actions"]
            elif case in ("nan", "inf"):
                result["actions"][0, 0] = float(case)
            elif case == "shape":
                result["actions"] = np.zeros((10, 8))
            elif case in ("policy_episode_seed", "replan_index", "protocol_version"):
                result["saps_sampling"][case] += 1
            elif case == "digest":
                result["saps_sampling"]["noise_sha256"] = "wrong"
            elif case == "timing":
                del result["policy_timing"]
            return result

        for case in ("malformed", "missing", "nan", "inf", "shape",
                     "policy_episode_seed", "replan_index", "protocol_version",
                     "digest", "timing"):
            with self.subTest(case=case):
                self.setUp()
                original = self.client.infer
                self.client.infer = lambda req: change(original(req), case)
                self.assert_failed_once()

    def test_wrong_server_identity_prevents_request(self) -> None:
        for section, key in (("saps_seeded_sampling", "policy_config_name"),
                             ("saps_seeded_sampling", "policy_checkpoint"),
                             ("saps_model_input_audit", "openpi_commit"),
                             ("saps_seeded_sampling", "action_horizon")):
            with self.subTest(key=key):
                self.setUp()
                metadata = self.client.get_server_metadata()
                metadata[section][key] = "wrong"
                self.client.get_server_metadata = lambda: metadata
                with self.assertRaises((ValueError, RuntimeError)):
                    self.prepare()
                self.assertEqual(self.client.requests, [])
                self.assertEqual(self.warmup.record["request_count"], 0)

    def test_both_camera_stamps_advance_before_return(self) -> None:
        signatures = iter(((10, 10.02, 10.01, 10.015),
                           (10, 10.02, 10.02, 10.025),
                           (10.001, 10.02, 10.02, 10.025),
                           (10.101, 10.121, 10.11, 10.115)))
        current = [None]
        self.collector.spin = lambda: current.__setitem__(0, next(signatures))
        self.collector.latest_signature = lambda: current[0]
        observation, sample_dir = self.prepare()
        warm = json.loads((self.path / "warmup/request_0000/request.json").read_text())
        post = json.loads((sample_dir / "request.json").read_text())
        self.assertEqual(post["duplicate_or_nonadvancing_pair_checks"], 2)
        self.assertTrue(all(b > a for a, b in zip(
            warm["camera_pair_source_stamps"], post["camera_pair_source_stamps"],
        )))
        self.assertEqual(observation.wrist_frame.stamp.ros_seconds, 10.101)
        self.assertEqual(len(self.client.requests), 1)

    def test_each_source_must_strictly_exceed_response_barrier(self) -> None:
        for source_index in range(4):
            for old_stamp in (10.09, 10.1):
                with self.subTest(source=source_index, stamp=old_stamp):
                    self.setUp()
                    stale = [10.11, 10.12, 10.13, 10.14]
                    stale[source_index] = old_stamp
                    signatures = iter((
                        (10, 10.02, 10.01, 10.015), tuple(stale),
                        (10.11, 10.12, 10.13, 10.14),
                    ))
                    current = [None]
                    self.collector.spin = lambda: current.__setitem__(
                        0, next(signatures),
                    )
                    self.collector.latest_signature = lambda: current[0]
                    original = self.collector.assemble

                    def assemble() -> object:
                        observation = original()
                        # All messages arrived after inference, even when their
                        # source timestamps are older than completion.
                        changes = {}
                        for name in ("wrist_frame", "exterior_frame",
                                     "joint_snapshot", "gripper_snapshot"):
                            source = getattr(observation, name)
                            changes[name] = dataclasses.replace(
                                source, stamp=dataclasses.replace(
                                    source.stamp, receive_monotonic_seconds=100,
                                ),
                            )
                        return dataclasses.replace(observation, **changes)

                    self.collector.assemble = assemble
                    observation, _ = self.prepare()
                    self.assertEqual(
                        self.record["post_warmup_source_barrier_ros_seconds"], 10.1,
                    )
                    self.assertIn("barrier", self.record["last_rejected_observation"])
                    for name in ("wrist_frame", "exterior_frame",
                                 "joint_snapshot", "gripper_snapshot"):
                        self.assertGreater(getattr(observation, name).stamp.ros_seconds,
                                           10.1)
                    self.assertEqual(len(self.client.requests), 1)

    def test_queued_pre_barrier_sources_timeout_without_real_request(self) -> None:
        self.collector.latest_signature = lambda: (
            (10, 10.02, 10.01, 10.015) if self.collector.index == 1
            else (10.09, 10.09, 10.09, 10.09)
        )
        ticks = iter(i * .1 for i in range(100))
        with self.assertRaises(TimeoutError):
            self.prepare(monotonic=lambda: next(ticks))
        self.assertTrue(self.warmup.record["completed"])
        self.assertEqual(len(self.client.requests), 1)
        self.assertEqual(self.record["completed_requests"], 0)
        self.assertFalse((self.path / "request_0000").exists())

    def test_post_observation_failure_prevents_real_inference(self) -> None:
        for failure in ("same_pair", "stale_state"):
            with self.subTest(failure=failure):
                self.setUp()
                original = self.client.infer

                def infer(request: dict) -> dict:
                    response = original(request)
                    if failure == "same_pair":
                        self.collector.spin = lambda: None
                        # prepare() captures spin before inference.
                        self.collector.latest_signature = lambda: (10, 10.02, 10.01, 10.015)
                    else:
                        self.collector.stale = True
                    return response

                self.client.infer = infer
                ticks = iter(i * .1 for i in range(100))
                with self.assertRaises(TimeoutError):
                    self.prepare(monotonic=lambda: next(ticks))
                self.assertTrue(self.warmup.record["completed"])
                self.assertEqual(len(self.client.requests), 1)
                self.assertNotIn("post_warmup_observation_acquired", self.record)
                self.assertEqual(self.record["completed_requests"], 0)
                self.assertFalse((self.path / "request_0000").exists())

    def test_expired_warmup_observation_prevents_submission(self) -> None:
        self.collector.stale = True
        ticks = iter(i * .1 for i in range(100))
        with self.assertRaises(TimeoutError):
            self.prepare(monotonic=lambda: next(ticks))
        self.assertEqual(self.client.requests, [])

    def test_no_command_capabilities_in_warmup_path(self) -> None:
        for name in ("policy_warmup.py", "shadow_ros.py", "live_shadow.py"):
            tree = ast.parse((ROOT / "src/saps/physical" / name).read_text())
            calls = [n.func.attr for n in ast.walk(tree)
                     if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
            for forbidden in ("publish", "create_publisher", "create_client",
                              "create_action_client", "send_goal_async"):
                self.assertNotIn(forbidden, calls)
            self.assertNotIn("StreamingBoundary", ast.dump(tree))

    def test_make_target_exports_overrides_and_sources_environment(self) -> None:
        result = subprocess.run(
            ["make", "-n", "physical-c1c2-warmup", "PHYSICAL_RUN_ID=test",
             "PHYSICAL_PROMPT=pick", "PHYSICAL_WARMUP_POLICY_SEED=42",
             "DROID_POLICY_SEED=42"], cwd=ROOT, check=True,
            capture_output=True, text=True,
        )
        for text in ("/opt/ros/jazzy/setup.bash", "FRANKA_ROS2_INSTALL/setup.bash",
                     "third_party/openpi/packages/openpi-client/src",
                     "physical_policy_warmup.py", "--warmup-policy-seed",
                     "--policy-episode-seed"):
            self.assertIn(text, result.stdout)


if __name__ == "__main__":
    unittest.main()
