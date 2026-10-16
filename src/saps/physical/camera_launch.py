"""Launch RealSense RGB streams from the execution camera configuration."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from saps.physical.shadow_config import load_shadow_config


def realsense_arguments(camera: dict[str, Any]) -> dict[str, str]:
    """Bind device selection and publisher identity to one configured role."""
    if (camera.get("serial_parameter", "serial_no") != "serial_no"
            or camera.get("serial_parameter_prefix", "_") != "_"):
        raise ValueError("This launcher requires RealSense serial_no identity")
    namespace, name = camera["node"].rsplit("/", 1)
    if not name or camera["topic"] != camera["node"] + "/color/image_raw":
        raise ValueError("RealSense topic must be <camera node>/color/image_raw")
    return {
        "camera_namespace": namespace.lstrip("/"),
        "camera_name": name,
        "serial_no": "_" + camera["serial"],
        "enable_color": "true",
        "rgb_camera.color_profile": "1280x720x30",
        "rgb_camera.color_format": "RGB8",
        "enable_depth": "false",
        "enable_gyro": "false",
        "enable_accel": "false",
    }


def generate_launch_description() -> Any:
    """Load one explicit config at launch time; start camera drivers only."""
    from launch import LaunchDescription
    from launch.actions import (
        DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
        OpaqueFunction,
    )
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare

    def launch_cameras(context: Any) -> list[Any]:
        config = load_shadow_config(
            Path(LaunchConfiguration("config").perform(context))
        )
        driver = PathJoinSubstitution([
            FindPackageShare("realsense2_camera"), "launch", "rs_launch.py",
        ])
        # Validate both cameras before starting either driver.
        arguments = [realsense_arguments(config[role]) for role in (
            "wrist_camera", "exterior_camera",
        )]
        return [GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(driver),
                launch_arguments=values.items(),
            ),
        ]) for values in arguments]

    return LaunchDescription([
        DeclareLaunchArgument("config", description="Physical runtime JSON"),
        OpaqueFunction(function=launch_cameras),
    ])
