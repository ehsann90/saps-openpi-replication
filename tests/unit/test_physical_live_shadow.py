"""Deterministic P0 contract and failure tests without ROS or policy services."""

from __future__ import annotations

import ast
import dataclasses
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from saps.physical.live_observation import make_camera_frame
from saps.physical.live_shadow import CameraPairGate, run_live_loop, validate_request
from saps.physical.shadow_audit import IMAGE_MASKS, save_model_audit
from saps.physical.shadow_config import load_shadow_config, observation_contract, OPENPI_COMMIT
from saps.physical.shadow_ros import (
    SubscriptionBoundary, validate_graph, camera_serial_evidence, run_shadow,
    node_interface_evidence,
)
from saps.policies.model_input_audit import array_evidence, infer_with_model_audit
from saps.policies.openpi_droid import OpenPiDroidPolicy
from saps.policies.bounded_websocket import BoundedWebsocketClient
import test_physical_live_observation as observation_tests


ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / "configs/physical_pi05_fr3.json"


def make_observation() -> object:
    observation = observation_tests.ObservationAssemblyTest()._assemble()
    frames = []
    for frame in (observation.wrist_frame, observation.exterior_frame):
        frames.append(make_camera_frame(
            np.full((720, 1280, 3), 37 if frame.serial == "wrist" else 211, np.uint8),
            stamp=frame.stamp, serial=frame.serial, topic=frame.topic,
            model=frame.model, source_encoding="rgb8", preserve_native_image=True,
        ))
    policy_input = dict(observation.policy_input)
    policy_input["observation/wrist_image_left"] = frames[0].image_rgb
    policy_input["observation/exterior_image_1_left"] = frames[1].image_rgb
    return dataclasses.replace(observation, wrist_frame=frames[0], exterior_frame=frames[1],
                               policy_input=policy_input)


def audit_fixture(policy_input: dict) -> dict:
    images = {name: np.zeros((224, 224, 3), np.uint8) for name in IMAGE_MASKS}
    images["base_0_rgb"][:] = 211
    images["left_wrist_0_rgb"][:] = 37
    return {
        "schema_version": 1, "boundary": "fixture sampler boundary",
        "prompt": policy_input["prompt"],
        "canonical_arrays": {key: array_evidence(value) for key, value in policy_input.items()
                             if isinstance(value, np.ndarray)},
        "transformed_images": images,
        "model_images": {name: value[None].astype(np.float32) / 255 * 2 - 1
                         for name, value in images.items()},
        "image_masks": {name: np.array([mask]) for name, mask in IMAGE_MASKS.items()},
        "state": np.zeros((1, 32), np.float32),
        "tokenized_prompt": np.zeros((1, 200), np.int32),
        "tokenized_prompt_mask": np.ones((1, 200), np.bool_),
    }


class FakeClient:
    def __init__(self) -> None:
        self.requests = []
        self.shape = (15, 8)
        self.error = None

    def get_server_metadata(self) -> dict:
        return {
            "saps_seeded_sampling": {
                "policy_config_name": "pi05_droid",
                "policy_checkpoint": "gs://openpi-assets/checkpoints/pi05_droid",
                "action_horizon": 15,
            },
            "saps_model_input_audit": {"schema_version": 1, "openpi_commit": OPENPI_COMMIT},
        }

    def infer(self, request: dict) -> dict:
        self.requests.append(request)
        if self.error:
            raise self.error
        result = {
            "actions": np.linspace(-2, 2, np.prod(self.shape), dtype=np.float64).reshape(self.shape),
            "saps_sampling": {
                "policy_episode_seed": request["policy_episode_seed"],
                "replan_index": request["replan_index"], "protocol_version": 1,
                "noise_sha256": "a" * 64,
            },
            "policy_timing": {"infer_ms": 7}, "server_timing": {"infer_ms": 9},
        }
        if request.get("audit_model_input"):
            result["saps_model_input_audit"] = audit_fixture(request["observation"])
        return result


class FakeCollector:
    def __init__(self) -> None:
        self.observation = make_observation()
        self.index = 0
        self.errors = {}
        self.stale = False

    def latest_signature(self) -> tuple:
        return (10 + self.index * .001, 10.02 + self.index * .001, 10.01, 10.015)

    def assemble(self) -> object:
        if self.stale:
            raise ValueError("stale")
        return self.observation

    def missing_sources(self) -> tuple:
        return ()

    def source_rates(self) -> dict:
        return {"wrist_image": {"message_count": self.index}}

    def spin(self) -> None:
        self.index += 1


class ConfigTest(unittest.TestCase):
    def test_current_config_roles_topics_and_historical_config(self) -> None:
        config = load_shadow_config(CONFIG_PATH)
        contract = observation_contract(config)
        self.assertEqual(contract.wrist_camera.serial, "342222073510")
        self.assertEqual(contract.exterior_camera.serial, "244222076317")
        self.assertEqual(contract.wrist_camera.topic, "/camera/wrist_camera/color/image_raw")
        self.assertEqual(contract.exterior_camera.topic, "/camera/external_camera/color/image_raw")
        self.assertEqual(contract.joint_state_topic, "/franka/joint_states")
        self.assertEqual(contract.gripper_state_topic, "/franka_gripper/joint_states")
        historical = json.loads((ROOT / "configs/physical_m3.json").read_text())
        self.assertEqual(historical["wrist_camera"]["ros_topic"], "/wrist/wrist_camera/color/image_raw")

    def test_invalid_configs_rejected(self) -> None:
        for section, key, value in (
            (None, "execution_enabled", True),
            ("policy", "action_shape", [10, 8]),
            ("policy", "checkpoint", "other"),
            ("wrist_camera", "serial", "244222076317"),
            ("wrist_camera", "topic", "/camera/external_camera/color/image_raw"),
            ("robot", "maximum_finger_position_m", .05),
        ):
            with self.subTest(section=section, key=key), tempfile.TemporaryDirectory() as tmp:
                config = load_shadow_config(CONFIG_PATH)
                (config if section is None else config[section])[key] = value
                path = Path(tmp) / "config.json"
                path.write_text(json.dumps(config))
                with self.assertRaises(ValueError):
                    load_shadow_config(path)

    def test_native_16_by_9_preserved_without_crop(self) -> None:
        observation = make_observation()
        frame = observation.wrist_frame
        self.assertIn("no crop (native 16:9)", frame.preprocessing)
        self.assertIn("INTER_AREA", frame.preprocessing)
        self.assertEqual(frame.image_rgb.shape, (180, 320, 3))
        self.assertEqual(frame.native_image_rgb.shape, (720, 1280, 3))
        self.assertTrue(np.all(frame.image_rgb == 37))
        validate_request(observation.policy_input)
        invalid = dict(observation.policy_input, extra=1)
        with self.assertRaises(ValueError):
            validate_request(invalid)


class PairGateTest(unittest.TestCase):
    def test_both_cameras_must_strictly_advance(self) -> None:
        gate = CameraPairGate()
        self.assertFalse(gate.accepts(None))
        self.assertTrue(gate.accepts((1, 2, 3, 4)))
        gate.commit((1, 2, 3, 4))
        for signature in ((1, 2, 4, 5), (1.1, 2, 4, 5), (.9, 2.1, 4, 5)):
            self.assertFalse(gate.accepts(signature))
        self.assertEqual(gate.rejected_checks, 3)
        self.assertTrue(gate.accepts((1.1, 2.1, 4, 5)))


class LoopTest(unittest.TestCase):
    def run_loop(self, path: Path, client: FakeClient, collector: FakeCollector,
                 **kwargs: object) -> dict:
        record = {}
        run_live_loop(
            collector=collector, policy=OpenPiDroidPolicy(client=client),
            output_dir=path, request_count=2, policy_episode_seed=17,
            observation_timeout=1, spin_once=collector.spin, ros_now=lambda: 10.1,
            config=load_shadow_config(CONFIG_PATH), record=record, **kwargs,
        )
        return record

    def test_two_live_requests_preserve_raw_actions_seed_and_audit(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            client = FakeClient()
            record = self.run_loop(path, client, FakeCollector())
            self.assertEqual(record["completed_requests"], 2)
            self.assertEqual([r["replan_index"] for r in client.requests], [0, 1])
            self.assertEqual([r.get("audit_model_input", False) for r in client.requests], [True, False])
            self.assertTrue(all(r["policy_episode_seed"] == 17 for r in client.requests))
            result = json.loads((path / "request_0000/response.json").read_text())
            self.assertEqual(result["action"]["shape"], [15, 8])
            self.assertEqual(result["action"]["dtype"], "float64")
            self.assertEqual(result["sampling_metadata"]["noise_sha256"], "a" * 64)
            self.assertEqual(result["policy_actions_executed"], 0)
            self.assertAlmostEqual(result["observation_age_at_request_seconds"], .1)
            with np.load(path / "request_0000/actions.npz", allow_pickle=False) as bundle:
                self.assertEqual(bundle["actions"][0, 0], -2)
                self.assertEqual(bundle["actions"][-1, -1], 2)
            self.assertTrue((path / "request_0000/model_audit/base_0_rgb.png").is_file())
            self.assertFalse((path / "request_0001/model_audit").exists())

    def test_unexpected_horizon_saved_and_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            client = FakeClient()
            client.shape = (10, 8)
            with self.assertRaisesRegex(ValueError, "15, 8"):
                self.run_loop(Path(tmp), client, FakeCollector())
            self.assertTrue((Path(tmp) / "request_0000/actions.npz").exists())
            self.assertEqual(len(client.requests), 1)

    def test_stale_observations_time_out_without_inference(self) -> None:
        collector = FakeCollector()
        collector.stale = True
        client = FakeClient()
        ticks = iter((0, .1, .2, 2))
        with tempfile.TemporaryDirectory() as tmp, self.assertRaises(TimeoutError):
            self.run_loop(Path(tmp), client, collector, monotonic=lambda: next(ticks))
        self.assertEqual(client.requests, [])

    def test_policy_failure_preserves_request_and_call_timing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            client = FakeClient()
            client.error = TimeoutError("delayed response")
            with self.assertRaises(TimeoutError):
                self.run_loop(Path(tmp), client, FakeCollector())
            self.assertTrue((Path(tmp) / "request_0000/request.json").exists())
            self.assertTrue((Path(tmp) / "request_0000/request_timing.json").exists())

    def test_wrong_server_pin_prevents_any_request(self) -> None:
        client = FakeClient()
        metadata = client.get_server_metadata()
        metadata["saps_model_input_audit"]["openpi_commit"] = "wrong"
        client.get_server_metadata = lambda: metadata
        with tempfile.TemporaryDirectory() as tmp, self.assertRaises(ValueError):
            self.run_loop(Path(tmp), client, FakeCollector())
        self.assertEqual(client.requests, [])


class TransportTest(unittest.TestCase):
    def test_connection_and_receive_timeouts_are_finite(self) -> None:
        socket = Mock()
        socket.recv.return_value = b"response"
        connect = Mock(return_value=socket)
        codec = SimpleNamespace(
            unpackb=lambda value: {"received": True},
            Packer=lambda: SimpleNamespace(pack=lambda value: b"request"),
        )
        modules = {
            "openpi_client": SimpleNamespace(msgpack_numpy=codec),
            "websockets.sync.client": SimpleNamespace(connect=connect),
        }
        with patch.dict(sys.modules, modules):
            client = BoundedWebsocketClient("127.0.0.1", 8000, 2.5)
            self.assertEqual(connect.call_args.kwargs["open_timeout"], 2.5)
            self.assertEqual(client.infer({}), {"received": True})
            socket.recv.assert_called_with(timeout=2.5)
            socket.recv.side_effect = TimeoutError("delayed")
            with self.assertRaises(TimeoutError):
                client.infer({})
            client.close()
            socket.close.assert_called_once()

    def test_handshake_error_closes_connection(self) -> None:
        socket = Mock()
        socket.recv.side_effect = TimeoutError("handshake")
        with patch.dict(sys.modules, {
            "openpi_client": SimpleNamespace(msgpack_numpy=object()),
            "websockets.sync.client": SimpleNamespace(connect=lambda *a, **k: socket),
        }):
            with self.assertRaises(TimeoutError):
                BoundedWebsocketClient("127.0.0.1", 8000, 1)
        socket.close.assert_called_once()


class AuditTest(unittest.TestCase):
    def test_actual_pinned_droid_inputs_mapping_and_placeholder(self) -> None:
        # Execute the pinned PI05 branch verbatim, isolating heavy imports.
        # The validated LIBERO test image runs Python 3.8, so remove the
        # unsupported match syntax and other branches before parsing. No
        # mapping statements are reproduced independently in this test.
        source = ROOT / "third_party/openpi/src/openpi/policies/droid_policy.py"
        text = source.read_text()
        text = text[text.index("@dataclasses.dataclass"):text.rindex("@dataclasses.dataclass")]
        match_start = text.index("        match self.model_type:")
        branch_start = text.index("\n", text.index("case _model.ModelType.PI0 | _model.ModelType.PI05:")) + 1
        branch_end = text.index("            case", branch_start)
        branch = "\n".join(line[8:] for line in text[branch_start:branch_end].splitlines())
        text = text[:match_start] + branch + text[text.index("\n        inputs ="):]
        tree = ast.parse(text)
        node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "DroidInputs")

        def strict_zip(left: tuple, right: tuple, *, strict: bool) -> object:
            # Python 3.8 compatibility for upstream's zip(strict=True).
            self.assertTrue(strict)
            self.assertEqual(len(left), len(right))
            return zip(left, right)

        namespace = {
            "dataclasses": dataclasses, "np": np,
            "transforms": SimpleNamespace(DataTransformFn=object),
            "_model": SimpleNamespace(ModelType=SimpleNamespace(PI0=0, PI05=1, PI0_FAST=2)),
            "_parse_image": np.asarray,
            "zip": strict_zip,
        }
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), namespace)
        observation = make_observation().policy_input
        transformed = namespace["DroidInputs"](model_type=1)(observation)
        np.testing.assert_array_equal(transformed["image"]["base_0_rgb"], observation["observation/exterior_image_1_left"])
        np.testing.assert_array_equal(transformed["image"]["left_wrist_0_rgb"], observation["observation/wrist_image_left"])
        self.assertEqual(transformed["image_mask"], IMAGE_MASKS)
        self.assertFalse(transformed["image"]["right_wrist_0_rgb"].any())

    def test_audit_hook_calls_originals_once_and_restores_on_error(self) -> None:
        inputs = make_observation().policy_input
        audit = audit_fixture(inputs)
        calls = []

        def transform(value: dict) -> dict:
            calls.append("transform")
            return {"image": audit["transformed_images"]}

        def sampler(rng: object, observation: object, **kwargs: object) -> dict:
            calls.append("sample")
            self.assertIs(kwargs["noise"], noise)
            return {"actions": np.zeros((15, 8))}

        policy = SimpleNamespace(_input_transform=transform, _sample_actions=sampler)
        model = SimpleNamespace(
            images=audit["model_images"], image_masks=audit["image_masks"],
            state=audit["state"], tokenized_prompt=audit["tokenized_prompt"],
            tokenized_prompt_mask=audit["tokenized_prompt_mask"],
        )

        def infer(value: dict, *, noise: np.ndarray) -> dict:
            policy._input_transform(value)
            return policy._sample_actions(None, model, noise=noise)

        policy.infer = infer
        noise = np.ones((15, 32), np.float32)
        result = infer_with_model_audit(policy, inputs, noise=noise)
        self.assertEqual(calls, ["transform", "sample"])
        self.assertIn("saps_model_input_audit", result)
        self.assertIs(policy._input_transform, transform)
        self.assertIs(policy._sample_actions, sampler)
        policy.infer = lambda *args, **kwargs: (_ for _ in ()).throw(ValueError("failed"))
        with self.assertRaises(ValueError):
            infer_with_model_audit(policy, inputs, noise=noise)
        self.assertIs(policy._input_transform, transform)
        self.assertIs(policy._sample_actions, sampler)

    def test_audit_rejects_mask_or_request_mismatch(self) -> None:
        inputs = make_observation().policy_input
        for field in ("image_masks", "prompt"):
            audit = audit_fixture(inputs)
            if field == "prompt":
                audit[field] = "wrong instruction"
            else:
                audit[field]["right_wrist_0_rgb"] = np.array([True])
            with tempfile.TemporaryDirectory() as tmp, self.assertRaises(ValueError):
                save_model_audit(audit, inputs, Path(tmp) / "audit")


class RosPreflightTest(unittest.TestCase):
    def test_owned_node_interfaces_reject_command_publishers_and_clients(self) -> None:
        node = SimpleNamespace(
            publishers=[SimpleNamespace(topic_name="/parameter_events")], clients=[],
        )
        self.assertEqual(node_interface_evidence(node)["robot_command_publishers"], 0)
        node.publishers.append(SimpleNamespace(topic_name="/servo_node/delta_twist_cmds"))
        with self.assertRaises(RuntimeError):
            node_interface_evidence(node)
        node.publishers = []
        node.clients = [SimpleNamespace(srv_name="/robot/move")]
        with self.assertRaises(RuntimeError):
            node_interface_evidence(node)

    def test_subscription_boundary_has_no_command_capabilities(self) -> None:
        boundary = SubscriptionBoundary(observation_tests.FakeRosNode())
        for name in ("create_publisher", "create_client", "create_service", "create_action_client"):
            self.assertFalse(hasattr(boundary, name))
        for name in ("live_shadow.py", "shadow_ros.py"):
            source = (ROOT / "src/saps/physical" / name).read_text()
            tree = ast.parse(source)
            calls = [n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call)
                     and isinstance(n.func, ast.Attribute)]
            self.assertNotIn("create_publisher", calls)
            self.assertNotIn("create_client", calls)
            self.assertNotIn("project", calls)

    def test_serial_parameter_binding_and_mismatch(self) -> None:
        config = load_shadow_config(CONFIG_PATH)
        with patch("saps.physical.shadow_ros.subprocess.run") as run:
            run.side_effect = [SimpleNamespace(stdout="_342222073510\n"),
                               SimpleNamespace(stdout="_244222076317\n")]
            record = camera_serial_evidence(config)
            self.assertEqual(record["wrist_camera"]["serial"], "342222073510")
            self.assertEqual(run.call_count, 2)
            run.side_effect = [SimpleNamespace(stdout="_244222076317\n")]
            with self.assertRaises(RuntimeError):
                camera_serial_evidence(config)

    def test_graph_rejects_swapped_or_duplicate_publishers(self) -> None:
        config = load_shadow_config(CONFIG_PATH)
        publishers = {}
        for role in ("wrist_camera", "exterior_camera"):
            camera = config[role]
            publishers[camera["topic"]] = [SimpleNamespace(
                node_namespace="/camera", node_name=camera["node"].split("/")[-1],
                topic_type="sensor_msgs/msg/Image", endpoint_gid=[1],
            )]
        for key in ("joint_state_topic", "gripper_state_topic"):
            publishers[config["robot"][key]] = [SimpleNamespace(
                node_namespace="/", node_name="state_source",
                topic_type="sensor_msgs/msg/JointState", endpoint_gid=[2],
            )]
        node = SimpleNamespace(get_publishers_info_by_topic=lambda topic: publishers[topic])
        self.assertEqual(len(validate_graph(node, config)), 4)
        topic = config["wrist_camera"]["topic"]
        publishers[topic][0].node_name = "external_camera"
        with self.assertRaises(RuntimeError):
            validate_graph(node, config)
        publishers[topic] *= 2
        with self.assertRaises(RuntimeError):
            validate_graph(node, config)

    def test_preflight_failure_finalizes_and_run_cannot_be_reused(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = SimpleNamespace(
                requests=2, policy_episode_seed=17, observation_timeout=1,
                policy_timeout=1, config=CONFIG_PATH, output_dir=Path(tmp) / "run",
                prompt="pick", lab_stack_dir=Path(tmp), igd_control_dir=Path(tmp),
            )
            with patch("saps.physical.shadow_ros.git_identity", side_effect=RuntimeError("test failure")):
                with self.assertRaises(RuntimeError):
                    run_shadow(args)
            record = json.loads((args.output_dir / "run.json").read_text())
            self.assertEqual(record["termination_reason"], "error")
            self.assertEqual(record["actuation"]["policy_actions_executed"], 0)
            with self.assertRaises(FileExistsError):
                run_shadow(args)


if __name__ == "__main__":
    unittest.main()
