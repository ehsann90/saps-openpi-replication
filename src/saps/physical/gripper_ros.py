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


def gripper_timing_helpers(directory: Path) -> tuple[Any, dict[str, Any]]:
    """Require the exact clean G1B hand commit; retain the frozen arm baseline."""
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

    def measured() -> dict[str, Any]:
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

        return dict(
            width_m=state.width_m,
            closure=state.closure,
            source_ros_seconds=state.stamp.ros_seconds,
            receive_monotonic_seconds=(
                state.stamp.receive_monotonic_seconds
            ),
        )

    return DroidGripper(
        hand,
        maximum_width_m=maximum,
        measured=measured,
        grasp_force_n=grasp_force_n,
        grasp_epsilon_inner_m=grasp_epsilon_inner_m,
        grasp_epsilon_outer_m=grasp_epsilon_outer_m,
    )
