"""Current P0 interface contract, independent of historical M3 provenance."""

from __future__ import annotations

import json
import math
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
    new_hardware = config.get('hardware_profile') == 'zed_robotiq_2f85'
    c4 = config.get('integration_stage') == 'c4'
    if config.get('integration_stage') not in (None, 'c4') or c4 and not new_hardware:
        raise ValueError('Unknown physical integration stage')
    if new_hardware:
        if (robot['base_frame'] != 'fr3_link0'
                or robot['tcp_frame'] is not None):
            raise ValueError('Robotiq TCP remains unresolved; no Franka TCP substitution allowed')
        gripper = config['gripper']
        if gripper['backend'] != 'robotiq_2f85' or gripper['state_topic'] != robot['gripper_state_topic']:
            raise ValueError('Inconsistent Robotiq backend/topics')
        if not gripper['namespace'].startswith('/') or not gripper['node'].startswith('/'):
            raise ValueError('Robotiq names must be fully qualified')
        for key in ('speed_register', 'force_register', 'open_command', 'close_command'):
            if type(gripper[key]) is not int or not 0 <= gripper[key] <= 255:
                raise ValueError(f'Invalid Robotiq {key}')
        if gripper['open_command'] >= gripper['close_command']:
            raise ValueError('Robotiq register direction must be zero-open, 255-closed')
    elif config.get('hardware_profile') is not None:
        raise ValueError('Unknown physical hardware profile')
    if not new_hardware and (robot["base_frame"], robot["tcp_frame"],
            robot["maximum_finger_position_m"]) != (
                "fr3_link0", "fr3_hand_tcp", 0.04):
        raise ValueError("P0 frame and gripper conventions are frozen.")
    for key, role in (
        ("wrist_camera", "wrist"),
        ("exterior_camera", "exterior"),
    ):
        camera = config[key]
        if camera["role"] != role:
            raise ValueError("Camera roles must match their configured slots.")
        for name in ("node", "topic"):
            value = camera[name]
            if not isinstance(value, str) or not value.startswith("/"):
                raise ValueError(f"Camera {name} must be fully qualified.")
        parameter = camera.get("serial_parameter", "serial_no")
        prefix = camera.get("serial_parameter_prefix", "_")
        if not isinstance(parameter, str) or not parameter.strip():
            raise ValueError("Camera serial_parameter must be non-empty.")
        if not isinstance(prefix, str):
            raise ValueError("Camera serial_parameter_prefix must be a string.")
    wrist, exterior = config["wrist_camera"], config["exterior_camera"]
    if wrist["topic"] == exterior["topic"] or wrist["node"] == exterior["node"]:
        raise ValueError("P0 requires distinct camera topics and nodes.")
    native, client = config["native_images"], config["client_images"]
    if not new_hardware and native != {
        "shape": [720, 1280, 3], "encoding": "rgb8",
        "expected_rate_hz": 30, "hardware_synchronized": False,
    }:
        raise ValueError("P0 native camera profile must be 1280x720 RGB8/30.")
    if new_hardware:
        if (native['shape'] not in ([720, 1280, 3], [1080, 1920, 3], [1242, 2208, 3], [376, 672, 3])
                or native['encoding'] not in ('rgb8', 'bgr8', 'rgba8', 'bgra8')
                or not math.isfinite(native['expected_rate_hz'])
                or native['expected_rate_hz'] <= 0 or native['hardware_synchronized'] is not False):
            raise ValueError('Unsupported explicit ZED native image profile')
        for key, model in (('wrist_camera', 'zedm'), ('exterior_camera', 'zed2i' if c4 else 'zed2')):
            camera = config[key]
            name = 'wrist_camera' if key == 'wrist_camera' else 'external_camera'
            if (not camera['serial'].isdecimal() or int(camera['serial']) <= 0
                    or camera['driver'] != 'zed' or camera['camera_model'] != model
                    or camera['node'] != f'/{name}/zed_node'
                    or camera['topic'] != camera['node'] + ('/rgb/color/rect/image' if c4 else '/rgb/image_rect_color')
                    or camera['serial_parameter'] != 'general.serial_number'
                    or camera['serial_parameter_prefix'] != ''
                    or camera['orientation'] != 'native_no_flip_rotate_mirror'):
                raise ValueError('ZED identity, topic, orientation or role mismatch')
        zed = config['zed']
        profiles = {'HD720': [720,1280,3], 'HD1080': [1080,1920,3],
                    'HD2K': [1242,2208,3], 'VGA': [376,672,3]}
        if (profiles.get(zed['resolution']) != native['shape']
                or zed['fps'] != native['expected_rate_hz']):
            raise ValueError('ZED launch and observation profiles must agree')
    if (client["shape"], client["dtype"], client["color_order"]) != (
        [180, 320, 3], "uint8", "RGB",
    ):
        raise ValueError("P0 client images must be 320x180 RGB uint8.")
    if c4:
        if config['camera_interface'] != dict(
                selection='latest_advancing', qos='reliable', history_depth=2,
                source_clock='SYSTEM_CLOCK', preserve_historical_freshness=True):
            raise ValueError('C4 requires explicit RELIABLE latest-frame camera semantics')
        if config['freshness'] != dict(maximum_source_age_seconds=.5,
                                       maximum_cross_source_skew_seconds=.25):
            raise ValueError('C4 retains historical freshness limits; C3 thresholds are not rollout defaults')
        if (config['wrist_camera']['serial'], config['exterior_camera']['serial']) != ('15330503', '32919961'):
            raise ValueError('C4 requires the approved camera identities in their fixed roles')
        if (config['zed']['sdk_version'], config['zed']['wrapper_version']) != ('5.5.1', '5.5.0'):
            raise ValueError('C4 requires the commissioned ZED software versions')
        if config['native_images']['shape'] != [720,1280,3] or config['native_images']['encoding'] != 'bgra8':
            raise ValueError('C4 requires native HD720 BGRA8')
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
        gripper_owner_required=config.get('integration_stage') == 'c4',
        gripper_config=config.get('gripper') if config.get('hardware_profile') == 'zed_robotiq_2f85' else None,
        native_image_profile=config['native_images'] if config.get('hardware_profile') == 'zed_robotiq_2f85' else None,
    )
