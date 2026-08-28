"""Subscriber-only ROS 2 boundary for live physical observations."""

from __future__ import annotations

import dataclasses
import time
from typing import Any

import numpy as np

from saps.physical.live_observation import assemble_physical_policy_observation
from saps.physical.live_observation import decode_ros_rgb_image
from saps.physical.live_observation import gripper_snapshot_from_joint_state
from saps.physical.live_observation import JointSnapshot
from saps.physical.live_observation import make_camera_frame
from saps.physical.live_observation import ObservationFreshness
from saps.physical.live_observation import ordered_fr3_joint_positions
from saps.physical.live_observation import PhysicalPolicyObservation
from saps.physical.live_observation import ros_stamp_seconds
from saps.physical.live_observation import SourceStamp


@dataclasses.dataclass(frozen=True)
class RosCameraContract:
    """Explicit physical identity and topic for one camera role."""

    role: str
    serial: str
    model: str
    topic: str

    def __post_init__(self) -> None:
        for field_name in ("role", "serial", "model", "topic"):
            value = getattr(self, field_name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"camera {field_name} must be non-empty.")


@dataclasses.dataclass(frozen=True)
class RosObservationContract:
    """Read-only ROS topics and physical conversion constants."""

    joint_state_topic: str
    gripper_state_topic: str
    wrist_camera: RosCameraContract
    exterior_camera: RosCameraContract
    maximum_finger_position_m: float = 0.04

    def __post_init__(self) -> None:
        if not self.joint_state_topic.strip():
            raise ValueError("joint_state_topic must be non-empty.")
        if not self.gripper_state_topic.strip():
            raise ValueError("gripper_state_topic must be non-empty.")
        if self.wrist_camera.role != "wrist":
            raise ValueError("wrist_camera role must be 'wrist'.")
        if self.exterior_camera.role != "exterior":
            raise ValueError("exterior_camera role must be 'exterior'.")
        if self.wrist_camera.serial == self.exterior_camera.serial:
            raise ValueError("wrist and exterior camera serials must differ.")
        if self.maximum_finger_position_m <= 0.0:
            raise ValueError("maximum_finger_position_m must be positive.")


@dataclasses.dataclass(frozen=True)
class LiveTransformSnapshot:
    """One fresh transform sampled from the live ROS TF tree."""

    target_frame: str
    source_frame: str
    translation_m: np.ndarray
    quaternion_xyzw: np.ndarray
    rotation_target_source: np.ndarray
    source_ros_seconds: float
    age_seconds: float
    receive_monotonic_seconds: float

    def as_dict(self) -> dict[str, Any]:
        """Return an explicit JSON-compatible transform record."""

        return {
            "target_frame": self.target_frame,
            "source_frame": self.source_frame,
            "translation_m": self.translation_m.tolist(),
            "quaternion_xyzw": self.quaternion_xyzw.tolist(),
            "rotation_target_source": self.rotation_target_source.tolist(),
            "source_ros_seconds": self.source_ros_seconds,
            "age_seconds": self.age_seconds,
            "receive_monotonic_seconds": self.receive_monotonic_seconds,
        }


class RosLiveTransformReader:
    """Subscriber-only TF reader with an explicit startup-readiness wait."""

    def __init__(
        self,
        node: Any,
        *,
        target_frame: str,
        source_frame: str,
    ) -> None:
        if not target_frame.strip() or not source_frame.strip():
            raise ValueError("TF target and source frames must be non-empty.")
        if target_frame == source_frame:
            raise ValueError("TF target and source frames must differ.")

        from tf2_ros import Buffer
        from tf2_ros import TransformListener

        self.node = node
        self.target_frame = target_frame
        self.source_frame = source_frame
        # Omitting node prevents Buffer from creating the optional frame-graph
        # service; TransformListener creates only /tf and /tf_static subscribers.
        self.buffer = Buffer()
        self.listener = TransformListener(
            self.buffer,
            node,
            spin_thread=False,
        )
        self.wait_attempts = 0
        self.wait_failed_attempts = 0
        self.wait_elapsed_seconds: float | None = None
        self.first_wait_error: str | None = None

    def wait_until_ready(
        self,
        *,
        timeout_seconds: float,
        maximum_age_seconds: float,
    ) -> LiveTransformSnapshot:
        """Spin subscriptions until a fresh transform exists or time out."""

        if timeout_seconds <= 0.0 or maximum_age_seconds <= 0.0:
            raise ValueError("TF wait timeout and maximum age must be positive.")

        from tf2_ros import TransformException

        start = time.monotonic()
        deadline = start + timeout_seconds
        last_error: str | None = None
        while time.monotonic() < deadline:
            self.wait_attempts += 1
            _spin_once(self.node, 0.05)
            try:
                snapshot = self.latest(
                    maximum_age_seconds=maximum_age_seconds,
                )
            except (TransformException, RuntimeError, ValueError) as error:
                last_error = str(error)
                if self.first_wait_error is None:
                    self.first_wait_error = last_error
                self.wait_failed_attempts += 1
                continue
            self.wait_elapsed_seconds = time.monotonic() - start
            return snapshot
        self.wait_elapsed_seconds = time.monotonic() - start
        raise TimeoutError(
            f"Timed out waiting for fresh TF {self.target_frame} <- "
            f"{self.source_frame}; last_error={last_error!r}."
        )

    def latest(self, *, maximum_age_seconds: float) -> LiveTransformSnapshot:
        """Return the newest transform and reject future or stale stamps."""

        if maximum_age_seconds <= 0.0:
            raise ValueError("maximum_age_seconds must be positive.")

        from rclpy.time import Time

        message = self.buffer.lookup_transform(
            self.target_frame,
            self.source_frame,
            Time(),
        )
        stamp_seconds = ros_stamp_seconds(
            int(message.header.stamp.sec),
            int(message.header.stamp.nanosec),
        )
        now_seconds = self.node.get_clock().now().nanoseconds / 1e9
        age_seconds = now_seconds - stamp_seconds
        if age_seconds < 0.0 or age_seconds > maximum_age_seconds:
            raise RuntimeError(
                f"TF age {age_seconds:.6f}s is outside [0, "
                f"{maximum_age_seconds:.6f}]s."
            )
        transform = message.transform
        translation = np.asarray(
            [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ],
            dtype=np.float64,
        )
        quaternion = np.asarray(
            [
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(translation)):
            raise ValueError("TF translation must be finite.")
        return LiveTransformSnapshot(
            target_frame=self.target_frame,
            source_frame=self.source_frame,
            translation_m=translation,
            quaternion_xyzw=quaternion,
            rotation_target_source=rotation_matrix_from_quaternion_xyzw(
                quaternion
            ),
            source_ros_seconds=stamp_seconds,
            age_seconds=age_seconds,
            receive_monotonic_seconds=time.monotonic(),
        )

    def readiness_record(self) -> dict[str, Any]:
        """Return startup-race evidence for the current reader."""

        return {
            "wait_attempts": self.wait_attempts,
            "failed_attempts_before_ready": self.wait_failed_attempts,
            "wait_elapsed_seconds": self.wait_elapsed_seconds,
            "first_wait_error": self.first_wait_error,
        }


def rotation_matrix_from_quaternion_xyzw(
    quaternion_xyzw: Any,
) -> np.ndarray:
    """Convert one finite nonzero ROS quaternion to a proper rotation."""

    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    if quaternion.shape != (4,):
        raise ValueError(
            "quaternion_xyzw must have shape (4,), received "
            f"{quaternion.shape}."
        )
    if not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion_xyzw must be finite.")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 0.0:
        raise ValueError("quaternion_xyzw must be nonzero.")
    x, y, z, w = quaternion / norm
    rotation = np.asarray(
        [
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ],
            [
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ],
            [
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )
    rotation.setflags(write=False)
    return rotation


def _spin_once(node: Any, timeout_seconds: float) -> None:
    import rclpy

    rclpy.spin_once(node, timeout_sec=timeout_seconds)


@dataclasses.dataclass
class SourceRateState:
    """Receive and source-stamp history for one subscriber."""

    message_count: int = 0
    first_source_ros_seconds: float | None = None
    latest_source_ros_seconds: float | None = None
    first_receive_monotonic_seconds: float | None = None
    latest_receive_monotonic_seconds: float | None = None

    def update(self, stamp: SourceStamp) -> None:
        self.message_count += 1
        if self.first_source_ros_seconds is None:
            self.first_source_ros_seconds = stamp.ros_seconds
            self.first_receive_monotonic_seconds = (
                stamp.receive_monotonic_seconds
            )
        self.latest_source_ros_seconds = stamp.ros_seconds
        self.latest_receive_monotonic_seconds = (
            stamp.receive_monotonic_seconds
        )

    def as_dict(self) -> dict[str, Any]:
        source_duration = _duration(
            self.first_source_ros_seconds,
            self.latest_source_ros_seconds,
        )
        receive_duration = _duration(
            self.first_receive_monotonic_seconds,
            self.latest_receive_monotonic_seconds,
        )
        intervals = max(0, self.message_count - 1)
        return {
            "message_count": self.message_count,
            "first_source_ros_seconds": self.first_source_ros_seconds,
            "latest_source_ros_seconds": self.latest_source_ros_seconds,
            "source_duration_seconds": source_duration,
            "source_rate_hz": (
                intervals / source_duration
                if source_duration is not None and source_duration > 0.0
                else None
            ),
            "receive_duration_seconds": receive_duration,
            "receive_rate_hz": (
                intervals / receive_duration
                if receive_duration is not None and receive_duration > 0.0
                else None
            ),
        }


class RosPhysicalObservationCollector:
    """Collect latest ROS messages without any publishers or clients."""

    SOURCE_NAMES = (
        "joint_state",
        "gripper_state",
        "wrist_image",
        "exterior_image",
    )

    def __init__(
        self,
        node: Any,
        contract: RosObservationContract,
        *,
        prompt: str,
        freshness: ObservationFreshness,
        joint_state_type: Any | None = None,
        image_type: Any | None = None,
        qos_profile: Any | None = None,
    ) -> None:
        if (
            joint_state_type is None
            or image_type is None
            or qos_profile is None
        ):
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image
            from sensor_msgs.msg import JointState

            joint_state_type = JointState
            image_type = Image
            qos_profile = qos_profile_sensor_data

        if not prompt.strip():
            raise ValueError("prompt must be non-empty.")
        self.node = node
        self.contract = contract
        self.prompt = prompt
        self.freshness = freshness
        self.latest_joint: JointSnapshot | None = None
        self.latest_gripper: Any | None = None
        self.latest_wrist: Any | None = None
        self.latest_exterior: Any | None = None
        self.errors: dict[str, str] = {}
        self.rate_state = {
            name: SourceRateState() for name in self.SOURCE_NAMES
        }
        self._subscriptions = (
            node.create_subscription(
                joint_state_type,
                contract.joint_state_topic,
                self._joint_callback,
                qos_profile,
            ),
            node.create_subscription(
                joint_state_type,
                contract.gripper_state_topic,
                self._gripper_callback,
                qos_profile,
            ),
            node.create_subscription(
                image_type,
                contract.wrist_camera.topic,
                lambda message: self._camera_callback(
                    message,
                    contract.wrist_camera,
                ),
                qos_profile,
            ),
            node.create_subscription(
                image_type,
                contract.exterior_camera.topic,
                lambda message: self._camera_callback(
                    message,
                    contract.exterior_camera,
                ),
                qos_profile,
            ),
        )

    def _source_stamp(self, message: Any) -> SourceStamp:
        return SourceStamp(
            ros_seconds=ros_stamp_seconds(
                int(message.header.stamp.sec),
                int(message.header.stamp.nanosec),
            ),
            receive_monotonic_seconds=time.monotonic(),
        )

    def _joint_callback(self, message: Any) -> None:
        source = "joint_state"
        try:
            stamp = self._source_stamp(message)
            position = ordered_fr3_joint_positions(
                message.name,
                message.position,
            )
            self.latest_joint = JointSnapshot(
                position_rad=position,
                stamp=stamp,
            )
            self.rate_state[source].update(stamp)
            self.errors.pop(source, None)
        except (TypeError, ValueError) as error:
            self.errors[source] = str(error)

    def _gripper_callback(self, message: Any) -> None:
        source = "gripper_state"
        try:
            stamp = self._source_stamp(message)
            self.latest_gripper = gripper_snapshot_from_joint_state(
                message.name,
                message.position,
                stamp=stamp,
                maximum_finger_position_m=(
                    self.contract.maximum_finger_position_m
                ),
            )
            self.rate_state[source].update(stamp)
            self.errors.pop(source, None)
        except (TypeError, ValueError) as error:
            self.errors[source] = str(error)

    def _camera_callback(
        self,
        message: Any,
        camera: RosCameraContract,
    ) -> None:
        source = f"{camera.role}_image"
        try:
            stamp = self._source_stamp(message)
            image_rgb = decode_ros_rgb_image(
                height=int(message.height),
                width=int(message.width),
                encoding=str(message.encoding).lower(),
                step=int(message.step),
                data=message.data,
            )
            frame = make_camera_frame(
                image_rgb,
                stamp=stamp,
                serial=camera.serial,
                model=camera.model,
                topic=camera.topic,
                source_encoding=str(message.encoding),
            )
            if camera.role == "wrist":
                self.latest_wrist = frame
            else:
                self.latest_exterior = frame
            self.rate_state[source].update(stamp)
            self.errors.pop(source, None)
        except (TypeError, ValueError) as error:
            self.errors[source] = str(error)

    def missing_sources(self) -> tuple[str, ...]:
        """Return source roles that have not produced one valid message."""

        latest = {
            "joint_state": self.latest_joint,
            "gripper_state": self.latest_gripper,
            "wrist_image": self.latest_wrist,
            "exterior_image": self.latest_exterior,
        }
        return tuple(name for name, value in latest.items() if value is None)

    def latest_signature(self) -> tuple[float, ...] | None:
        """Return source stamps used to prevent duplicate assemblies."""

        if self.missing_sources():
            return None
        return (
            self.latest_wrist.stamp.ros_seconds,
            self.latest_exterior.stamp.ros_seconds,
            self.latest_joint.stamp.ros_seconds,
            self.latest_gripper.stamp.ros_seconds,
        )

    def assemble(self) -> PhysicalPolicyObservation:
        """Assemble the latest complete observation with fresh ROS time."""

        missing = self.missing_sources()
        if missing:
            raise RuntimeError(f"missing physical sources: {list(missing)}")
        assembly_ros = self.node.get_clock().now().nanoseconds / 1e9
        return assemble_physical_policy_observation(
            exterior_frame=self.latest_exterior,
            wrist_frame=self.latest_wrist,
            joint_snapshot=self.latest_joint,
            gripper_snapshot=self.latest_gripper,
            prompt=self.prompt,
            assembly_ros_seconds=assembly_ros,
            assembly_monotonic_seconds=time.monotonic(),
            freshness=self.freshness,
        )

    def source_rates(self) -> dict[str, dict[str, Any]]:
        """Return measured callback rates for all four sources."""

        return {
            name: state.as_dict()
            for name, state in self.rate_state.items()
        }


def _duration(first: float | None, latest: float | None) -> float | None:
    if first is None or latest is None:
        return None
    return latest - first
