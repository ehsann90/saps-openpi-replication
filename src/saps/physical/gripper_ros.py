"""G1B assembly for the frozen committed Franka Hand runtime."""

from __future__ import annotations

import hashlib
from pathlib import Path
import runpy
import subprocess
import time
from typing import Any

from saps.physical.droid_gripper import DroidGripper
from saps.physical.shadow_ros import git_identity
from saps.physical.streaming_playback import LAB_COMMIT


GRIPPER_LAB_COMMIT = "4bb6cdc58839dcdd94acbe1633b8f361676a4eb6"

EXTENSION_FILES = {
    "CMakeLists.txt",
    "package.xml",
    "docs/franka_hand.md",
    "fr3_lab_stack_runtime/franka_hand.py",
    "test/test_franka_hand.py",
}

# Fixed embodiment parameters physically validated in G1B. They do not depend
# on object width or task-specific geometry.
DEFAULT_GRASP_FORCE_N = 20.0
DEFAULT_GRASP_EPSILON_INNER_M = 0.001


def gripper_timing_helpers(directory: Path, config: dict[str, Any] | None = None) -> tuple[Any, dict[str, Any]]:
    """Require the exact clean G1B hand commit; retain the frozen arm baseline."""
    if config is not None and config.get('hardware_profile') == 'zed_robotiq_2f85':
        from saps.physical.hardware_provenance import hardware_timing_helpers
        return hardware_timing_helpers(directory)
    from types import SimpleNamespace

    def git(*args: str) -> bytes:
        return subprocess.check_output(
            ["git", "-C", str(directory), *args],
            timeout=10,
        )

    head = git("rev-parse", "HEAD").decode().strip()
    if head != GRIPPER_LAB_COMMIT:
        raise ValueError(
            "Gripper-enabled runtime requires frozen fr3_lab_stack commit "
            + GRIPPER_LAB_COMMIT
            + "; found "
            + head
        )

    status = git("status", "--porcelain").decode().splitlines()
    if status:
        raise ValueError(
            "Gripper-enabled runtime requires a clean fr3_lab_stack checkout: "
            + str(status)
        )

    identity = git_identity(directory)
    identity["arm_baseline"] = LAB_COMMIT
    identity["gripper_commit"] = GRIPPER_LAB_COMMIT
    identity["extension_sha256"] = {
        name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
        for name in sorted(EXTENSION_FILES)
        if (directory / name).is_file()
    }

    module = runpy.run_path(
        str(directory / "fr3_lab_stack_runtime/timing_evidence.py")
    )
    return SimpleNamespace(**module), identity


def create_gripper(
    node: Any,
    collector: Any,
    config: dict[str, Any],
    directory: Path,
    *,
    speed: float,
    timeout: float,
    grasp_force_n: float | None = DEFAULT_GRASP_FORCE_N,
    grasp_epsilon_inner_m: float | None = DEFAULT_GRASP_EPSILON_INNER_M,
    grasp_epsilon_outer_m: float | None = None,
) -> DroidGripper:
    owner_instance = None
    def measured() -> dict[str, Any]:
        nonlocal owner_instance
        if getattr(collector, 'errors', {}).get('gripper_state'):
            raise RuntimeError('invalid_gripper_feedback: ' + collector.errors['gripper_state'])
        state = collector.latest_gripper
        if state is None:
            raise RuntimeError("missing_gripper_state")

        receive_age = (
            time.monotonic()
            - state.stamp.receive_monotonic_seconds
        )
        source_age = (
            node.get_clock().now().nanoseconds / 1e9
            - state.stamp.ros_seconds
        )
        bound = config["freshness"]["maximum_source_age_seconds"]

        if (
            not 0 <= receive_age <= bound
            or not 0 <= source_age <= bound
        ):
            raise RuntimeError("stale_gripper_state")

        metadata = getattr(state, 'feedback_metadata', None)
        if config.get('integration_stage') == 'c4':
            if (not metadata or metadata.get('serial_exclusive') is not True
                    or metadata.get('commands_authorized') is not True
                    or not metadata.get('server_instance')):
                raise RuntimeError('C4 gripper owner/command authorization evidence missing')
            if owner_instance is None:
                owner_instance = metadata['server_instance']
            elif owner_instance != metadata['server_instance']:
                raise RuntimeError('C4 gripper server identity changed')
        return dict(
            feedback_metadata=metadata,
            width_m=state.width_m,
            closure=state.closure,
            source_ros_seconds=state.stamp.ros_seconds,
            receive_monotonic_seconds=(
                state.stamp.receive_monotonic_seconds
            ),
        )

    if config.get('hardware_profile') == 'zed_robotiq_2f85':
        from saps.physical.robotiq_hand import RosRobotiqHand
        from saps.physical.robotiq import calibrated_width
        settings = config['gripper']
        # Validate calibration before constructing any command clients.
        calibrated_width(settings['calibration']['open_position'], settings['calibration'])
        maximum = settings['calibration']['maximum_width_m']
        hand = RosRobotiqHand(node, namespace=settings['namespace'],
                              measured=measured, timeout=timeout)
        return DroidGripper(hand, maximum_width_m=maximum, measured=measured)
    module = runpy.run_path(
        str(directory / "fr3_lab_stack_runtime/franka_hand.py")
    )
    maximum = 2 * config["robot"]["maximum_finger_position_m"]

    if grasp_epsilon_outer_m is None:
        grasp_epsilon_outer_m = maximum

    hand = module["RosFrankaHand"](
        node,
        maximum_width=maximum,
        speed=speed,
        timeout=timeout,
    )

    return DroidGripper(
        hand,
        maximum_width_m=maximum,
        measured=measured,
        grasp_force_n=grasp_force_n,
        grasp_epsilon_inner_m=grasp_epsilon_inner_m,
        grasp_epsilon_outer_m=grasp_epsilon_outer_m,
    )
