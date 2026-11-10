"""Read-only C4 hardware evidence. No actuator clients or serial connections."""
from __future__ import annotations

import copy
import hashlib
import json
import os
import re
from pathlib import Path
import subprocess
import time
from typing import Any

from saps.physical.hardware_provenance import require_supported_arm_geometry
from saps.physical.ros_packages import lab_package_directory
from franka_robotiq.robotiq import calibrated_width


class HardwareReadinessError(RuntimeError):
    """Preserve effective camera evidence even when preflight fails."""

    def __init__(self, message: str, evidence: dict) -> None:
        self.evidence = evidence
        super().__init__(message)


def prepare_profile(hardware_path: Path, output: Path) -> None:
    """Copy operator-owned gripper settings unchanged into a NEW C4 profile."""
    root = Path(__file__).resolve().parents[3]
    config = json.loads((root / 'configs/physical_pi05_fr3_c4.json').read_text())
    hardware = json.loads(hardware_path.read_text())
    config['gripper'] = copy.deepcopy(hardware['gripper'])
    # Carry operator-supplied tool evidence forward without applying SDK settings
    # or replacing it with the template's incomplete catalog assembly.
    if 'mounted_tool' in hardware:
        config['mounted_tool'] = copy.deepcopy(hardware['mounted_tool'])
    config['robot']['gripper_state_topic'] = config['gripper']['state_topic']
    # No geometry flag or calibration endpoint/verification is synthesized.
    from saps.physical.shadow_config import load_shadow_config
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(config, stream, indent=2)
        stream.write('\n')
    load_shadow_config(output)


def camera_qos(config: dict) -> Any:
    """Apply RELIABLE only to cameras; preserve sensor-state QoS separately."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    c4 = config.get('integration_stage') == 'c4'
    return QoSProfile(depth=2 if c4 else 1,
                      reliability=(ReliabilityPolicy.RELIABLE if c4 else ReliabilityPolicy.BEST_EFFORT),
                      history=HistoryPolicy.KEEP_LAST)


def camera_evidence(node: Any, config: dict) -> dict:
    """Reuse the audited lab C3 validator, including the depth-NONE exception."""
    from franka_zed_cameras.camera_validation import inspect_sources, provenance
    sources = inspect_sources(node)
    versions = provenance()
    for role, key in (('wrist', 'wrist_camera'), ('external', 'exterior_camera')):
        source = sources[role]
        if not source['valid']:
            raise HardwareReadinessError(
                f'C4 camera {role} checks failed: {source.get("failed_checks", source)}',
                dict(sources=sources, software=versions))
        camera = config[key]
        params = source['parameters']
        if (params['general.serial_number'] != int(camera['serial'])
                or params['general.camera_model'] != camera['camera_model']):
            raise HardwareReadinessError('C4 camera configuration and effective identity disagree',
                                         dict(sources=sources, software=versions))
    if versions.get('sdk_version_from_headers') != config['zed']['sdk_version']:
        raise HardwareReadinessError('C4 ZED SDK header version mismatch', dict(sources=sources, software=versions))
    for package in ('zed_wrapper', 'zed_components', 'zed_msgs'):
        if versions.get('packages', {}).get(package, {}).get('version') != config['zed']['wrapper_version']:
            raise HardwareReadinessError('C4 ZED package version mismatch: ' + package, dict(sources=sources, software=versions))
    return dict(sources=sources, software=versions,
                identity_scope='configured driver serial plus unique publisher, not image-header identity')


def gripper_readiness(node: Any, collector: Any, config: dict, *, commands: bool) -> dict:
    """Inspect existing status/ROS ownership, never open serial or call services."""
    state = collector.latest_gripper
    if state is None or collector.errors.get('gripper_state'):
        raise RuntimeError('C4 gripper feedback unavailable or invalid')
    bound = config['freshness']['maximum_source_age_seconds']
    ages = (time.monotonic() - state.stamp.receive_monotonic_seconds,
            node.get_clock().now().nanoseconds / 1e9 - state.stamp.ros_seconds)
    if not all(0 <= age <= bound for age in ages):
        raise RuntimeError('C4 stale gripper feedback')
    metadata = state.feedback_metadata or {}
    settings = config['gripper']
    calibrated_width(metadata['actual_position'], settings['calibration'])
    if (metadata.get('serial_exclusive') is not True
            or type(metadata.get('serial_owner_pid')) is not int
            or metadata['serial_owner_pid'] <= 0 or not metadata.get('server_instance')):
        raise RuntimeError('C4 serial-owner evidence unavailable; use the current single Robotiq server')
    namespace, node_name = settings['node'].rsplit('/', 1)
    namespace = namespace or '/'
    owners = [(n, ns) for n, ns in node.get_node_names_and_namespaces()
              if ns.rstrip('/') + '/' + n == settings['node']]
    if len(owners) != 1:
        raise RuntimeError('C4 requires exactly one Robotiq server node')
    services = dict(node.get_service_names_and_types_by_node(node_name, namespace))
    expected = {settings['namespace'] + '/' + kind for kind in ('open', 'close', 'stop')}
    available = {name for name, types in services.items() if types == ['std_srvs/srv/Trigger']}
    if commands and (not metadata.get('commands_authorized') or not expected <= available
                     or metadata.get('command_busy') is not False):
        raise RuntimeError('C4 authorized gripper command services unavailable')
    return dict(
        server=settings['node'], server_instance=metadata['server_instance'],
        serial_owner_pid=metadata['serial_owner_pid'], serial_exclusive=True,
        port=settings['port'], slave=settings['slave'],
        commands_authorized=metadata.get('commands_authorized', False),
        command_services=sorted(expected & available),
        calibration=settings['calibration'], measured_register=metadata['actual_position'],
        closure=state.closure, contact_status=metadata.get('object_status'),
        commands_checked=commands, serial_opened_by_rollout=False,
        width_limitation='Motor-position estimate; nominal reference width is not a calibrated fingertip distance.')


def execution_blockers(config: dict) -> list[str]:
    blockers = []
    try:
        require_supported_arm_geometry(config, joint_space=True)
    except ValueError as error:
        blockers.append(str(error))
    return blockers


def controller_evidence(node: Any, manager: str) -> dict:
    """Read controller status and forwarder identity only; never switch/load."""
    result = subprocess.run(['ros2', 'control', 'list_controllers',
                             '--controller-manager', manager],
                            capture_output=True, text=True, check=True, timeout=10)
    lines = re.sub(r'\x1b\[[0-9;]*m', '', result.stdout).splitlines()
    required = 'fr3_streaming_joint_impedance_controller'
    selected = [line for line in lines if line.split() and line.split()[0] == required]
    if len(selected) != 1 or 'fr3_lab_stack/StreamingJointImpedanceController' not in selected[0] or selected[0].split()[-1] != 'active':
        raise RuntimeError('C4 streaming impedance controller is not uniquely active')
    from saps.physical.streaming_ros import TARGET_TOPIC, FORWARDER
    endpoints = node.get_subscriptions_info_by_topic(TARGET_TOPIC)
    if len(endpoints) != 1 or endpoints[0].node_name != FORWARDER or endpoints[0].node_namespace not in ('', '/'):
        raise RuntimeError('C4 streaming target must have exactly one forwarder subscriber')
    return dict(controller_manager=manager, controllers=result.stdout,
                target_topic=TARGET_TOPIC, subscriber=FORWARDER,
                limitation='Graph/activity evidence only; streaming health, T4 and safety supervision remain mandatory at execution.')


def evidence_file(path: Path) -> dict:
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def workspace_evidence(directory: Path) -> dict:
    """Fail on a different overlay; do not silently source a second workspace."""
    from ament_index_python.packages import get_package_share_directory
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        raise RuntimeError('C4 requires the unified ROS 2 Jazzy environment')
    share = Path(get_package_share_directory('franka_droid_stack'))
    source = (share / 'package.xml').resolve().parent
    if source != lab_package_directory(directory).resolve():
        raise RuntimeError(f'C4 lab overlay mismatch: {source} != {directory}')
    gripper_share = Path(get_package_share_directory('franka_robotiq'))
    gripper_source = (gripper_share / 'package.xml').resolve().parent
    if gripper_source != (directory / 'franka_robotiq').resolve():
        raise RuntimeError(f'C4 gripper overlay mismatch: {gripper_source} != {directory}')
    camera_share = Path(get_package_share_directory('franka_zed_cameras'))
    camera_source = (camera_share / 'package.xml').resolve().parent
    if camera_source != (directory / 'franka_zed_cameras').resolve():
        raise RuntimeError(f'C4 camera overlay mismatch: {camera_source} != {directory}')
    return dict(ros_distro='jazzy', package_share=str(share), source_directory=str(source))


def source_integrity_evidence(directory: Path) -> dict:
    """Explain the existing pin mismatch without updating or bypassing it."""
    manifest_path = Path(__file__).resolve().parents[3] / 'configs/zed_robotiq_lab_sources.json'
    expected = json.loads(manifest_path.read_text())['files_sha256']
    raw = subprocess.check_output(['git', '-C', str(directory), 'ls-files',
                                   '--cached', '--others', '--exclude-standard', '-z'], timeout=10)
    actual = {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
              for name in raw.decode().split('\0') if name and (directory / name).is_file()}
    return dict(manifest=evidence_file(manifest_path),
                matches_reviewed_manifest=actual == expected,
                changed=sorted(name for name in actual.keys() & expected.keys() if actual[name] != expected[name]),
                added=sorted(actual.keys() - expected.keys()),
                missing=sorted(expected.keys() - actual.keys()))
