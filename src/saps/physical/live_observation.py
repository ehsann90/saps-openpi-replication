"""Pure contracts for assembling live FR3 DROID observations."""

from __future__ import annotations

from collections.abc import Sequence
import dataclasses
from typing import Any

import cv2
import numpy as np

from saps.physical.embodiment import FR3_JOINT_NAMES
from saps.policies.openpi_droid import prepare_droid_observation


POLICY_IMAGE_HEIGHT = 180
POLICY_IMAGE_WIDTH = 320
FR3_FINGER_JOINT_NAMES = (
    "_finger_joint1",
    "_finger_joint2",
)
CURRENT_FR3_FINGER_JOINT_NAMES = (
    "fr3_finger_joint1",
    "fr3_finger_joint2",
)
ACCEPTED_FR3_FINGER_JOINT_NAME_PAIRS = (
    FR3_FINGER_JOINT_NAMES,
    CURRENT_FR3_FINGER_JOINT_NAMES,
)


@dataclasses.dataclass(frozen=True)
class SourceStamp:
    """ROS source and local receive times for one physical sample."""

    ros_seconds: float
    receive_monotonic_seconds: float

    def __post_init__(self) -> None:
        _finite_nonnegative(self.ros_seconds, "ros_seconds")
        _finite_nonnegative(
            self.receive_monotonic_seconds,
            "receive_monotonic_seconds",
        )


@dataclasses.dataclass(frozen=True)
class JointSnapshot:
    """One name-ordered seven-joint FR3 state."""

    position_rad: np.ndarray
    stamp: SourceStamp


@dataclasses.dataclass(frozen=True)
class GripperSnapshot:
    """One physical Franka Hand width and canonical closure."""

    joint_names: tuple[str, ...]
    finger_position_m: np.ndarray
    width_m: float
    maximum_width_m: float
    unclipped_closure: float
    closure: float
    closure_clipped: bool
    stamp: SourceStamp


@dataclasses.dataclass(frozen=True)
class CameraFrame:
    """One explicitly identified RGB camera frame."""

    image_rgb: np.ndarray
    stamp: SourceStamp
    serial: str
    model: str
    topic: str
    source_encoding: str
    native_shape: tuple[int, int, int]
    preprocessing: str


@dataclasses.dataclass(frozen=True)
class ObservationFreshness:
    """Configurable M3 diagnostic thresholds, not execution limits."""

    maximum_source_age_seconds: float = 0.5
    maximum_cross_source_skew_seconds: float = 0.25

    def __post_init__(self) -> None:
        _finite_positive(
            self.maximum_source_age_seconds,
            "maximum_source_age_seconds",
        )
        _finite_nonnegative(
            self.maximum_cross_source_skew_seconds,
            "maximum_cross_source_skew_seconds",
        )


@dataclasses.dataclass(frozen=True)
class ObservationTiming:
    """Auditable timing for one asynchronously assembled observation."""

    assembly_ros_seconds: float
    assembly_monotonic_seconds: float
    source_ros_seconds: dict[str, float]
    source_age_seconds: dict[str, float]
    receive_age_seconds: dict[str, float]
    oldest_source_ros_seconds: float
    newest_source_ros_seconds: float
    cross_source_skew_seconds: float


@dataclasses.dataclass(frozen=True)
class PhysicalPolicyObservation:
    """Canonical pi05-DROID input plus physical acquisition metadata."""

    policy_input: dict[str, Any]
    timing: ObservationTiming
    joint_snapshot: JointSnapshot
    gripper_snapshot: GripperSnapshot
    wrist_frame: CameraFrame
    exterior_frame: CameraFrame


def ros_stamp_seconds(seconds: int, nanoseconds: int) -> float:
    """Convert a ROS builtin time pair to finite non-negative seconds."""

    if not isinstance(seconds, int) or isinstance(seconds, bool):
        raise TypeError("ROS stamp seconds must be an integer.")
    if not isinstance(nanoseconds, int) or isinstance(nanoseconds, bool):
        raise TypeError("ROS stamp nanoseconds must be an integer.")
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("ROS stamp fields are outside their valid ranges.")
    return float(seconds) + float(nanoseconds) / 1_000_000_000.0


def ordered_fr3_joint_positions(
    names: Sequence[str],
    positions: Sequence[float],
) -> np.ndarray:
    """Order an exact FR3 arm JointState by name, never array position."""

    values = _strict_named_positions(
        names,
        positions,
        expected_names=FR3_JOINT_NAMES,
        state_name="FR3 arm JointState",
    )
    return _readonly(values, dtype=np.float32)


def gripper_snapshot_from_joint_state(
    names: Sequence[str],
    positions: Sequence[float],
    *,
    stamp: SourceStamp,
    maximum_finger_position_m: float,
) -> GripperSnapshot:
    """Convert the two physical finger positions into closure in [0, 1]."""

    maximum_finger = _finite_positive(
        maximum_finger_position_m,
        "maximum_finger_position_m",
    )
    actual_names = tuple(names)
    duplicate_names = sorted(
        {name for name in actual_names if actual_names.count(name) > 1}
    )
    if duplicate_names:
        raise ValueError(
            "Franka Hand JointState contains duplicate joints: "
            f"{duplicate_names}."
        )
    recognized = [
        pair
        for pair in ACCEPTED_FR3_FINGER_JOINT_NAME_PAIRS
        if set(actual_names) == set(pair)
    ]
    if len(recognized) != 1:
        raise ValueError(
            "Franka Hand JointState must contain exactly one recognized "
            f"finger pair; received {list(actual_names)}."
        )
    fingers = _strict_named_positions(
        names,
        positions,
        expected_names=recognized[0],
        state_name="Franka Hand JointState",
    )
    maximum_width = 2.0 * maximum_finger
    width = float(np.sum(fingers))
    unclipped_closure = 1.0 - width / maximum_width
    closure = gripper_closure_from_width(width, maximum_width_m=maximum_width)
    return GripperSnapshot(
        joint_names=actual_names,
        finger_position_m=_readonly(fingers, dtype=np.float32),
        width_m=width,
        maximum_width_m=maximum_width,
        unclipped_closure=unclipped_closure,
        closure=closure,
        closure_clipped=closure != unclipped_closure,
        stamp=stamp,
    )


def gripper_closure_from_width(
    width_m: float,
    *,
    maximum_width_m: float,
) -> float:
    """Map physical width to zero-open, one-closed with explicit clipping."""

    width = _finite_nonnegative(width_m, "width_m")
    maximum = _finite_positive(maximum_width_m, "maximum_width_m")
    return float(np.clip(1.0 - width / maximum, 0.0, 1.0))


def decode_ros_rgb_image(
    *,
    height: int,
    width: int,
    encoding: str,
    step: int,
    data: bytes | bytearray | memoryview,
) -> np.ndarray:
    """Decode an rgb8/bgr8 ROS Image, including padded row strides."""

    if height <= 0 or width <= 0:
        raise ValueError("ROS image height and width must be positive.")
    if encoding not in {"rgb8", "bgr8"}:
        raise ValueError(
            "M3 RGB input supports only explicit rgb8 or bgr8 encodings; "
            f"received {encoding!r}."
        )
    packed_step = width * 3
    if step < packed_step:
        raise ValueError(
            f"ROS image step {step} is smaller than packed RGB row "
            f"width {packed_step}."
        )
    flat = np.frombuffer(data, dtype=np.uint8)
    required = height * step
    if flat.size != required:
        raise ValueError(
            f"ROS image data has {flat.size} bytes; expected {required}."
        )
    rows = flat.reshape(height, step)
    image = rows[:, :packed_step].reshape(height, width, 3)
    if encoding == "bgr8":
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


def preprocess_policy_rgb(image_rgb: np.ndarray) -> tuple[np.ndarray, str]:
    """Centre-crop to 16:9 and resize deterministically to 180 by 320."""

    image = np.asarray(image_rgb)
    if image.dtype != np.uint8:
        raise TypeError(
            f"RGB image must have dtype uint8, received {image.dtype}."
        )
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(
            "RGB image must have shape [height, width, 3], received "
            f"{image.shape}."
        )
    height, width = image.shape[:2]
    if height <= 0 or width <= 0:
        raise ValueError("RGB image dimensions must be positive.")

    target_ratio = POLICY_IMAGE_WIDTH / POLICY_IMAGE_HEIGHT
    source_ratio = width / height
    if source_ratio > target_ratio:
        crop_width = int(np.floor(height * target_ratio))
        left = (width - crop_width) // 2
        cropped = image[:, left : left + crop_width]
        crop_description = (
            f"center-crop width {width}->{crop_width} at x={left}"
        )
    elif source_ratio < target_ratio:
        crop_height = int(np.floor(width / target_ratio))
        top = (height - crop_height) // 2
        cropped = image[top : top + crop_height, :]
        crop_description = (
            f"center-crop height {height}->{crop_height} at y={top}"
        )
    else:
        cropped = image
        crop_description = "no crop (native 16:9)"

    interpolation = (
        cv2.INTER_AREA
        if (
            cropped.shape[0] > POLICY_IMAGE_HEIGHT
            or cropped.shape[1] > POLICY_IMAGE_WIDTH
        )
        else cv2.INTER_LINEAR
    )
    resized = cv2.resize(
        cropped,
        (POLICY_IMAGE_WIDTH, POLICY_IMAGE_HEIGHT),
        interpolation=interpolation,
    )
    operation = (
        f"{crop_description}; resize {cropped.shape[1]}x"
        f"{cropped.shape[0]}->320x180 with "
        f"{'INTER_AREA' if interpolation == cv2.INTER_AREA else 'INTER_LINEAR'}"
    )
    return np.ascontiguousarray(resized, dtype=np.uint8), operation


def make_camera_frame(
    image_rgb: np.ndarray,
    *,
    stamp: SourceStamp,
    serial: str,
    model: str,
    topic: str,
    source_encoding: str,
) -> CameraFrame:
    """Validate identity and preprocess one explicitly selected camera."""

    for field_name, value in (
        ("serial", serial),
        ("model", model),
        ("topic", topic),
        ("source_encoding", source_encoding),
    ):
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"camera {field_name} must be non-empty.")
    native = np.asarray(image_rgb)
    processed, operation = preprocess_policy_rgb(native)
    return CameraFrame(
        image_rgb=_readonly(processed, dtype=np.uint8),
        stamp=stamp,
        serial=serial,
        model=model,
        topic=topic,
        source_encoding=source_encoding,
        native_shape=tuple(int(value) for value in native.shape),
        preprocessing=operation,
    )


def validate_camera_identities(
    *,
    wrist_serial: str,
    exterior_serial: str,
) -> None:
    """Require two distinct explicit physical camera serials."""

    if not wrist_serial.strip():
        raise ValueError("wrist camera serial must be explicit.")
    if not exterior_serial.strip():
        raise ValueError(
            "exterior camera serial is required; a missing camera cannot "
            "be replaced with the wrist stream."
        )
    if wrist_serial == exterior_serial:
        raise ValueError("wrist and exterior camera serials must differ.")


def assemble_physical_policy_observation(
    *,
    exterior_frame: CameraFrame,
    wrist_frame: CameraFrame,
    joint_snapshot: JointSnapshot,
    gripper_snapshot: GripperSnapshot,
    prompt: str,
    assembly_ros_seconds: float,
    assembly_monotonic_seconds: float,
    freshness: ObservationFreshness,
) -> PhysicalPolicyObservation:
    """Assemble and reject stale or excessively skewed physical inputs."""

    validate_camera_identities(
        wrist_serial=wrist_frame.serial,
        exterior_serial=exterior_frame.serial,
    )
    assembly_ros = _finite_nonnegative(
        assembly_ros_seconds,
        "assembly_ros_seconds",
    )
    assembly_monotonic = _finite_nonnegative(
        assembly_monotonic_seconds,
        "assembly_monotonic_seconds",
    )
    sources = {
        "wrist_image": wrist_frame.stamp,
        "exterior_image": exterior_frame.stamp,
        "joint_state": joint_snapshot.stamp,
        "gripper_state": gripper_snapshot.stamp,
    }
    source_ros = {
        name: stamp.ros_seconds for name, stamp in sources.items()
    }
    source_age = {
        name: assembly_ros - stamp.ros_seconds
        for name, stamp in sources.items()
    }
    receive_age = {
        name: assembly_monotonic - stamp.receive_monotonic_seconds
        for name, stamp in sources.items()
    }
    future = {
        name: age for name, age in source_age.items() if age < 0.0
    }
    if future:
        raise ValueError(f"physical source timestamps are in the future: {future}")
    negative_receive = {
        name: age for name, age in receive_age.items() if age < 0.0
    }
    if negative_receive:
        raise ValueError(
            "physical receive timestamps are after assembly: "
            f"{negative_receive}"
        )
    stale = {
        name: age
        for name, age in source_age.items()
        if age > freshness.maximum_source_age_seconds
    }
    if stale:
        raise ValueError(f"stale physical observation sources: {stale}")
    oldest = min(source_ros.values())
    newest = max(source_ros.values())
    skew = newest - oldest
    if skew > freshness.maximum_cross_source_skew_seconds:
        raise ValueError(
            f"physical source skew {skew:.6f}s exceeds configured "
            f"maximum {freshness.maximum_cross_source_skew_seconds:.6f}s."
        )

    policy_input = prepare_droid_observation(
        exterior_image=exterior_frame.image_rgb,
        wrist_image=wrist_frame.image_rgb,
        joint_position=joint_snapshot.position_rad,
        gripper_position=np.asarray(
            [gripper_snapshot.closure],
            dtype=np.float32,
        ),
        prompt=prompt,
    )
    timing = ObservationTiming(
        assembly_ros_seconds=assembly_ros,
        assembly_monotonic_seconds=assembly_monotonic,
        source_ros_seconds=source_ros,
        source_age_seconds=source_age,
        receive_age_seconds=receive_age,
        oldest_source_ros_seconds=oldest,
        newest_source_ros_seconds=newest,
        cross_source_skew_seconds=skew,
    )
    return PhysicalPolicyObservation(
        policy_input=policy_input,
        timing=timing,
        joint_snapshot=joint_snapshot,
        gripper_snapshot=gripper_snapshot,
        wrist_frame=wrist_frame,
        exterior_frame=exterior_frame,
    )


def _strict_named_positions(
    names: Sequence[str],
    positions: Sequence[float],
    *,
    expected_names: tuple[str, ...],
    state_name: str,
) -> np.ndarray:
    actual_names = tuple(names)
    values = np.asarray(positions)
    if values.ndim != 1 or len(actual_names) != values.size:
        raise ValueError(
            f"{state_name} names and positions must be equal-length "
            "one-dimensional arrays."
        )
    duplicate_names = sorted(
        {name for name in actual_names if actual_names.count(name) > 1}
    )
    if duplicate_names:
        raise ValueError(
            f"{state_name} contains duplicate joints: {duplicate_names}."
        )
    expected = set(expected_names)
    actual = set(actual_names)
    missing = sorted(expected - actual)
    unexpected = sorted(actual - expected)
    if missing or unexpected:
        raise ValueError(
            f"{state_name} joint set mismatch; missing={missing}, "
            f"unexpected={unexpected}."
        )
    if not np.issubdtype(values.dtype, np.number):
        raise TypeError(f"{state_name} positions must be numeric.")
    numeric = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(numeric)):
        raise ValueError(f"{state_name} positions must be finite.")
    # Length equality is checked above; avoid zip(strict=...) because the
    # validated LIBERO runtime uses an older Python built-in contract.
    by_name = dict(zip(actual_names, numeric))
    return np.asarray([by_name[name] for name in expected_names])


def _finite_nonnegative(value: float, field_name: str) -> float:
    result = float(value)
    if not np.isfinite(result) or result < 0.0:
        raise ValueError(f"{field_name} must be finite and non-negative.")
    return result


def _finite_positive(value: float, field_name: str) -> float:
    result = float(value)
    if not np.isfinite(result) or result <= 0.0:
        raise ValueError(f"{field_name} must be finite and positive.")
    return result


def _readonly(value: np.ndarray, *, dtype: Any) -> np.ndarray:
    result = np.array(value, dtype=dtype, copy=True, order="C")
    result.setflags(write=False)
    return result
