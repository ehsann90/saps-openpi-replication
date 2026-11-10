"""Explicit source layouts for the migrated and preserved ROS integrations."""
from __future__ import annotations

import os
from pathlib import Path


def default_lab_stack_directory() -> Path:
    """Keep checkout identity explicit and overridable for frozen protocols."""
    workspace = Path(os.environ.get(
        'FRANKA_ROS2_WS', str(Path.home() / 'franka_ehsan_ros2_ws')))
    return Path(os.environ.get(
        'FRANKA_LAB_STACK_DIR', str(workspace / 'src/franka_droid_stack')))


def lab_runtime_file(directory: Path, filename: str) -> Path:
    """Resolve known layouts; reject ambiguous or unavailable source files."""
    paths = [
        directory / 'franka_droid_stack/franka_droid_stack_runtime' / filename,
        directory / 'fr3_lab_stack_runtime' / filename,
    ]
    found = [path for path in paths if path.is_file()]
    if len(found) != 1:
        raise ValueError(f'Expected one ROS integration source for {filename}: {found}')
    return found[0]


def lab_package_directory(directory: Path) -> Path:
    """Return the renamed package directory within its Git repository."""
    return directory / 'franka_droid_stack'
