"""Tests for pure live FR3 observation contracts."""

from __future__ import annotations

import unittest

import numpy as np

from saps.physical.embodiment import FR3_JOINT_NAMES
from saps.physical.live_observation import assemble_physical_policy_observation
from saps.physical.live_observation import CURRENT_FR3_FINGER_JOINT_NAMES
from saps.physical.live_observation import decode_ros_rgb_image
from saps.physical.live_observation import FR3_FINGER_JOINT_NAMES
from saps.physical.live_observation import gripper_closure_from_width
from saps.physical.live_observation import gripper_snapshot_from_joint_state
from saps.physical.live_observation import JointSnapshot
from saps.physical.live_observation import make_camera_frame
from saps.physical.live_observation import ObservationFreshness
from saps.physical.live_observation import ordered_fr3_joint_positions
from saps.physical.live_observation import preprocess_policy_rgb
from saps.physical.live_observation import SourceStamp
from saps.physical.live_observation import validate_camera_identities
from saps.physical.ros_observation import RosCameraContract
from saps.physical.ros_observation import RosObservationContract
from saps.physical.ros_observation import RosPhysicalObservationCollector
from saps.physical.ros_observation import rotation_matrix_from_quaternion_xyzw
from saps.policies.openpi_droid import DROID_POLICY_INPUT_KEYS


def stamp(ros_seconds: float) -> SourceStamp:
    return SourceStamp(
        ros_seconds=ros_seconds,
        receive_monotonic_seconds=ros_seconds + 100.0,
    )


def camera(serial: str, ros_seconds: float) -> object:
    return make_camera_frame(
        np.zeros((360, 640, 3), dtype=np.uint8),
        stamp=stamp(ros_seconds),
        serial=serial,
        model="D435i",
        topic=f"/{serial}/color/image_raw",
        source_encoding="rgb8",
    )


class JointStateContractTest(unittest.TestCase):
    def test_joint_state_is_ordered_by_name_as_float32(self) -> None:
        names = tuple(reversed(FR3_JOINT_NAMES))
        positions = tuple(float(index) for index in reversed(range(7)))

        ordered = ordered_fr3_joint_positions(names, positions)

        self.assertEqual(ordered.dtype, np.float32)
        self.assertEqual(ordered.shape, (7,))
        np.testing.assert_array_equal(ordered, np.arange(7))

    def test_missing_duplicate_and_unexpected_joints_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "missing"):
            ordered_fr3_joint_positions(
                FR3_JOINT_NAMES[:-1],
                np.zeros(6),
            )
        duplicate = list(FR3_JOINT_NAMES)
        duplicate[-1] = duplicate[0]
        with self.assertRaisesRegex(ValueError, "duplicate"):
            ordered_fr3_joint_positions(duplicate, np.zeros(7))
        unexpected = list(FR3_JOINT_NAMES)
        unexpected[-1] = "other_joint"
        with self.assertRaisesRegex(ValueError, "unexpected"):
            ordered_fr3_joint_positions(unexpected, np.zeros(7))

    def test_nonfinite_or_misaligned_joint_state_is_rejected(self) -> None:
        values = np.zeros(7)
        values[3] = np.nan
        with self.assertRaisesRegex(ValueError, "finite"):
            ordered_fr3_joint_positions(FR3_JOINT_NAMES, values)
        with self.assertRaisesRegex(ValueError, "equal-length"):
            ordered_fr3_joint_positions(FR3_JOINT_NAMES, np.zeros(6))


class GripperContractTest(unittest.TestCase):
    def test_physical_width_maps_to_canonical_closure(self) -> None:
        self.assertEqual(
            gripper_closure_from_width(0.08, maximum_width_m=0.08),
            0.0,
        )
        self.assertEqual(
            gripper_closure_from_width(0.0, maximum_width_m=0.08),
            1.0,
        )
        self.assertEqual(
            gripper_closure_from_width(0.02, maximum_width_m=0.08),
            0.75,
        )

    def test_gripper_uses_both_name_ordered_finger_positions(self) -> None:
        snapshot = gripper_snapshot_from_joint_state(
            tuple(reversed(FR3_FINGER_JOINT_NAMES)),
            (0.03, 0.01),
            stamp=stamp(1.0),
            maximum_finger_position_m=0.04,
        )

        np.testing.assert_allclose(snapshot.finger_position_m, [0.01, 0.03])
        self.assertEqual(snapshot.width_m, 0.04)
        self.assertEqual(snapshot.maximum_width_m, 0.08)
        self.assertEqual(snapshot.unclipped_closure, 0.5)
        self.assertEqual(snapshot.closure, 0.5)
        self.assertFalse(snapshot.closure_clipped)

    def test_current_and_legacy_finger_names_have_identical_semantics(
        self,
    ) -> None:
        for names in (
            FR3_FINGER_JOINT_NAMES,
            CURRENT_FR3_FINGER_JOINT_NAMES,
        ):
            with self.subTest(names=names):
                snapshot = gripper_snapshot_from_joint_state(
                    tuple(reversed(names)),
                    (0.03, 0.01),
                    stamp=stamp(1.0),
                    maximum_finger_position_m=0.04,
                )
                self.assertEqual(snapshot.joint_names, tuple(reversed(names)))
                np.testing.assert_allclose(
                    snapshot.finger_position_m,
                    [0.01, 0.03],
                )
                self.assertEqual(snapshot.width_m, 0.04)
                self.assertEqual(snapshot.closure, 0.5)

    def test_mixed_missing_duplicate_and_unexpected_names_are_rejected(
        self,
    ) -> None:
        invalid = (
            (("_finger_joint1", "fr3_finger_joint2"), (0.01, 0.03)),
            (("fr3_finger_joint1",), (0.01,)),
            (("fr3_finger_joint1", "fr3_finger_joint1"), (0.01, 0.03)),
            (("fr3_finger_joint1", "other_joint"), (0.01, 0.03)),
        )
        for names, positions in invalid:
            with self.subTest(names=names):
                with self.assertRaises(ValueError):
                    gripper_snapshot_from_joint_state(
                        names,
                        positions,
                        stamp=stamp(1.0),
                        maximum_finger_position_m=0.04,
                    )

    def test_measured_open_boundary_overshoot_is_explicitly_clipped(self) -> None:
        snapshot = gripper_snapshot_from_joint_state(
            ("_finger_joint1", "_finger_joint2"),
            (0.04002251848578453, 0.04002251848578453),
            stamp=stamp(1.0),
            maximum_finger_position_m=0.04,
        )

        self.assertGreater(snapshot.width_m, snapshot.maximum_width_m)
        self.assertLess(snapshot.unclipped_closure, 0.0)
        self.assertEqual(snapshot.closure, 0.0)
        self.assertTrue(snapshot.closure_clipped)

    def test_nonnegative_width_closure_is_clipped_to_unit_interval(self) -> None:
        self.assertEqual(
            gripper_closure_from_width(0.081, maximum_width_m=0.08),
            0.0,
        )


class ImageContractTest(unittest.TestCase):
    def test_bgr_with_padded_rows_decodes_as_rgb(self) -> None:
        rows = np.asarray(
            [
                [1, 2, 3, 4, 5, 6, 99, 99],
                [7, 8, 9, 10, 11, 12, 99, 99],
            ],
            dtype=np.uint8,
        )

        decoded = decode_ros_rgb_image(
            height=2,
            width=2,
            encoding="bgr8",
            step=8,
            data=rows.tobytes(),
        )

        np.testing.assert_array_equal(decoded[0, 0], [3, 2, 1])
        np.testing.assert_array_equal(decoded[1, 1], [12, 11, 10])

    def test_unsupported_encoding_and_bad_buffer_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "rgb8 or bgr8"):
            decode_ros_rgb_image(
                height=2,
                width=2,
                encoding="mono8",
                step=2,
                data=b"\0" * 4,
            )
        with self.assertRaisesRegex(ValueError, "expected"):
            decode_ros_rgb_image(
                height=2,
                width=2,
                encoding="rgb8",
                step=6,
                data=b"\0" * 11,
            )

    def test_preprocessing_crops_without_distortion_and_is_deterministic(self) -> None:
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[:, :, 0] = np.arange(640, dtype=np.uint16) % 256

        first, operation = preprocess_policy_rgb(image)
        second, _ = preprocess_policy_rgb(image)

        self.assertEqual(first.shape, (180, 320, 3))
        self.assertEqual(first.dtype, np.uint8)
        self.assertIn("center-crop height 480->360", operation)
        np.testing.assert_array_equal(first, second)

    def test_image_shape_and_dtype_are_strict(self) -> None:
        with self.assertRaisesRegex(TypeError, "uint8"):
            preprocess_policy_rgb(np.zeros((10, 20, 3), dtype=np.float32))
        with self.assertRaisesRegex(ValueError, "shape"):
            preprocess_policy_rgb(np.zeros((10, 20), dtype=np.uint8))


class ObservationAssemblyTest(unittest.TestCase):
    def _assemble(
        self,
        *,
        wrist_seconds: float = 10.00,
        exterior_seconds: float = 10.02,
        joint_seconds: float = 10.01,
        gripper_seconds: float = 10.015,
        assembly_seconds: float = 10.05,
        freshness: ObservationFreshness = ObservationFreshness(),
    ) -> object:
        joint_snapshot = JointSnapshot(
            position_rad=ordered_fr3_joint_positions(
                FR3_JOINT_NAMES,
                np.linspace(-0.3, 0.3, 7),
            ),
            stamp=stamp(joint_seconds),
        )
        gripper_snapshot = gripper_snapshot_from_joint_state(
            FR3_FINGER_JOINT_NAMES,
            (0.02, 0.02),
            stamp=stamp(gripper_seconds),
            maximum_finger_position_m=0.04,
        )
        return assemble_physical_policy_observation(
            exterior_frame=camera("exterior", exterior_seconds),
            wrist_frame=camera("wrist", wrist_seconds),
            joint_snapshot=joint_snapshot,
            gripper_snapshot=gripper_snapshot,
            prompt="pick up the object",
            assembly_ros_seconds=assembly_seconds,
            assembly_monotonic_seconds=assembly_seconds + 100.0,
            freshness=freshness,
        )

    def test_exact_droid_schema_shapes_and_timing_are_assembled(self) -> None:
        observation = self._assemble()

        self.assertEqual(tuple(observation.policy_input), DROID_POLICY_INPUT_KEYS)
        self.assertEqual(
            observation.policy_input[
                "observation/exterior_image_1_left"
            ].shape,
            (180, 320, 3),
        )
        self.assertEqual(
            observation.policy_input["observation/joint_position"].dtype,
            np.float32,
        )
        self.assertEqual(
            observation.policy_input["observation/gripper_position"].tolist(),
            [0.5],
        )
        self.assertAlmostEqual(
            observation.timing.cross_source_skew_seconds,
            0.02,
        )
        self.assertEqual(
            set(observation.timing.source_age_seconds),
            {"wrist_image", "exterior_image", "joint_state", "gripper_state"},
        )

    def test_stale_source_and_excessive_skew_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "stale"):
            self._assemble(
                wrist_seconds=9.0,
                freshness=ObservationFreshness(
                    maximum_source_age_seconds=0.5,
                    maximum_cross_source_skew_seconds=2.0,
                ),
            )
        with self.assertRaisesRegex(ValueError, "skew"):
            self._assemble(
                wrist_seconds=9.9,
                freshness=ObservationFreshness(
                    maximum_source_age_seconds=1.0,
                    maximum_cross_source_skew_seconds=0.05,
                ),
            )

    def test_missing_or_duplicate_camera_identity_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "required"):
            validate_camera_identities(
                wrist_serial="342222073510",
                exterior_serial="",
            )
        with self.assertRaisesRegex(ValueError, "differ"):
            validate_camera_identities(
                wrist_serial="342222073510",
                exterior_serial="342222073510",
            )


class FakeClockNow:
    nanoseconds = 10_100_000_000


class FakeClock:
    def now(self) -> FakeClockNow:
        return FakeClockNow()


class FakeRosNode:
    def __init__(self) -> None:
        self.callbacks = {}
        self.operations = []

    def create_subscription(self, message_type, topic, callback, qos):
        self.operations.append(("subscription", topic, message_type, qos))
        self.callbacks[topic] = callback
        return object()

    def get_clock(self) -> FakeClock:
        return FakeClock()


class FakeStamp:
    sec = 10
    nanosec = 50_000_000


class FakeHeader:
    stamp = FakeStamp()


class FakeMessage:
    header = FakeHeader()


class RosBoundaryTest(unittest.TestCase):
    def test_mocked_boundary_subscribes_only_and_assembles(self) -> None:
        node = FakeRosNode()
        contract = RosObservationContract(
            joint_state_topic="/arm",
            gripper_state_topic="/hand",
            wrist_camera=RosCameraContract(
                role="wrist",
                serial="wrist",
                model="D435i",
                topic="/wrist",
            ),
            exterior_camera=RosCameraContract(
                role="exterior",
                serial="exterior",
                model="D435",
                topic="/exterior",
            ),
        )
        collector = RosPhysicalObservationCollector(
            node,
            contract,
            prompt="pick up the object",
            freshness=ObservationFreshness(),
            joint_state_type=object,
            image_type=bytes,
            qos_profile="sensor",
        )

        arm = FakeMessage()
        arm.name = tuple(reversed(FR3_JOINT_NAMES))
        arm.position = tuple(float(value) for value in reversed(range(7)))
        node.callbacks["/arm"](arm)
        hand = FakeMessage()
        hand.name = FR3_FINGER_JOINT_NAMES
        hand.position = (0.02, 0.02)
        node.callbacks["/hand"](hand)
        image = FakeMessage()
        image.height = 2
        image.width = 2
        image.encoding = "rgb8"
        image.step = 6
        image.data = bytes(range(12))
        node.callbacks["/wrist"](image)
        node.callbacks["/exterior"](image)

        observation = collector.assemble()

        self.assertEqual(len(node.operations), 4)
        self.assertTrue(
            all(operation[0] == "subscription" for operation in node.operations)
        )
        np.testing.assert_array_equal(
            observation.joint_snapshot.position_rad,
            np.arange(7),
        )
        self.assertEqual(observation.gripper_snapshot.closure, 0.5)
        self.assertEqual(collector.missing_sources(), ())
        self.assertEqual(
            collector.source_rates()["wrist_image"]["message_count"],
            1,
        )

    def test_ros_quaternion_conversion_is_a_proper_rotation(self) -> None:
        rotation = rotation_matrix_from_quaternion_xyzw(
            [1.0, 0.0, 0.0, 0.0]
        )

        np.testing.assert_allclose(
            rotation,
            np.diag([1.0, -1.0, -1.0]),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            rotation.T @ rotation,
            np.eye(3),
            atol=1e-12,
        )
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0)

    def test_invalid_ros_quaternion_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "shape"):
            rotation_matrix_from_quaternion_xyzw([0.0, 0.0, 1.0])
        with self.assertRaisesRegex(ValueError, "nonzero"):
            rotation_matrix_from_quaternion_xyzw(np.zeros(4))


if __name__ == "__main__":
    unittest.main()
