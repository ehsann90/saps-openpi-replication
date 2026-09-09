"""Current P0 interface contract, independent of historical M3 provenance."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from saps.physical.live_observation import ObservationFreshness
from saps.physical.ros_observation import RosCameraContract
from saps.physical.ros_observation import RosObservationContract


OPENPI_COMMIT = "15a9616a00943ada6c20a0f158e3adb39df2ccac"
POLICY_CONFIG = "pi05_droid"
POLICY_CHECKPOINT = "gs://openpi-assets/checkpoints/pi05_droid"
ACTION_SHAPE = (15, 8)


def load_shadow_config(path: Path) -> dict[str, Any]:
    """Reject incompatible policy/state/image semantics before subscribing."""

    config = json.loads(path.read_text(encoding="utf-8"))
    if config["schema_version"] != 1 or config["execution_enabled"] is not False:
        raise ValueError("P0 requires schema 1 and execution_enabled=false.")
    policy = config["policy"]
    expected = {
        "config": POLICY_CONFIG, "checkpoint": POLICY_CHECKPOINT,
        "openpi_commit": OPENPI_COMMIT, "action_shape": list(ACTION_SHAPE),
        "reference_future_open_loop_horizon": 8,
    }
    if policy != expected:
        raise ValueError("P0 policy identity and native action contract are frozen.")
    robot = config["robot"]
    if (robot["base_frame"], robot["tcp_frame"],
            robot["maximum_finger_position_m"]) != (
                "fr3_link0", "fr3_hand_tcp", 0.04):
        raise ValueError("P0 frame and gripper conventions are frozen.")
    for key, role, serial in (
        ("wrist_camera", "wrist", "342222073510"),
        ("exterior_camera", "exterior", "244222076317"),
    ):
        camera = config[key]
        if (camera["role"], camera["serial"]) != (role, serial):
            raise ValueError("P0 camera serial-role mapping is frozen.")
        if not camera["node"].startswith("/"):
            raise ValueError("Camera node must be fully qualified.")
    wrist, exterior = config["wrist_camera"], config["exterior_camera"]
    if wrist["topic"] == exterior["topic"] or wrist["node"] == exterior["node"]:
        raise ValueError("P0 requires distinct camera topics and nodes.")
    native, client = config["native_images"], config["client_images"]
    if native != {
        "shape": [720, 1280, 3], "encoding": "rgb8",
        "expected_rate_hz": 30, "hardware_synchronized": False,
    }:
        raise ValueError("P0 native camera profile must be 1280x720 RGB8/30.")
    if (client["shape"], client["dtype"], client["color_order"]) != (
        [180, 320, 3], "uint8", "RGB",
    ):
        raise ValueError("P0 client images must be 320x180 RGB uint8.")
    observation_contract(config)
    ObservationFreshness(**config["freshness"])
    return config


def observation_contract(config: dict[str, Any]) -> RosObservationContract:
    """Reuse the established ROS adapter with explicitly configured roles."""

    def camera(key: str) -> RosCameraContract:
        return RosCameraContract(**{
            name: config[key][name] for name in ("role", "serial", "model", "topic")
        })

    return RosObservationContract(
        joint_state_topic=config["robot"]["joint_state_topic"],
        gripper_state_topic=config["robot"]["gripper_state_topic"],
        wrist_camera=camera("wrist_camera"),
        exterior_camera=camera("exterior_camera"),
        maximum_finger_position_m=config["robot"]["maximum_finger_position_m"],
    )
