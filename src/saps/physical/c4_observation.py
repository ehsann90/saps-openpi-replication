"""C4 observation/readiness/shadow workflows. No physical command interfaces."""
from __future__ import annotations

import dataclasses
import json
from pathlib import Path
import time
from typing import Any, Callable

import cv2
import numpy as np

from saps.physical.c4_readiness import (
    camera_evidence, camera_qos, controller_evidence, evidence_file,
    execution_blockers, gripper_readiness, workspace_evidence,
)
from saps.physical.rollout_provenance import (
    capture_provenance, readiness_report, require_qualified_sources, validate_mode,
)
from saps.physical.discrete_verifier import candidate_target, limits_from_urdf
from saps.physical.droid_gripper import droid_gripper_decision
from saps.physical.live_observation import ObservationFreshness
from saps.physical.live_shadow import (
    CameraPairGate, infer_live_request, prepare_live_request, utc_now,
    validate_live_policy, write_json, ZERO_ACTUATION,
)
from saps.physical.policy_warmup import PolicyWarmup, prepare_warmed_observation
from saps.physical.ros_observation import RosPhysicalObservationCollector
from saps.physical.shadow_config import OPENPI_COMMIT, load_shadow_config, observation_contract
from saps.physical.shadow_ros import (
    SubscriptionBoundary, node_interface_evidence, validate_graph,
)
from saps.physical.single_action import first_action_from_chunk
from saps.policies.bounded_websocket import BoundedWebsocketClient
from saps.policies.openpi_droid import OpenPiDroidPolicy


def save_images(observation: Any, directory: Path) -> dict:
    """Save exact native decoded RGB and policy RGB for operator role review."""
    evidence = {}
    for role, frame in (('wrist', observation.wrist_frame), ('external', observation.exterior_frame)):
        for stage, image in (('native_decoded_rgb', frame.native_image_rgb),
                             ('policy_rgb', frame.image_rgb)):
            if image is None or image.dtype != np.uint8 or image.ndim != 3 or image.shape[-1] != 3:
                raise ValueError('C4 requires native and processed RGB uint8 evidence')
            ok, encoded = cv2.imencode('.png', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            if not ok:
                raise RuntimeError('C4 PNG encoding failed')
            path = directory / f'{role}_{stage}.png'
            with path.open('xb') as stream:
                stream.write(encoded.tobytes())
            evidence[role + '_' + stage] = dict(evidence_file(path), shape=list(image.shape),
                                                dtype=str(image.dtype))
        evidence[role + '_timing'] = dict(
            source_ros_seconds=frame.stamp.ros_seconds,
            receive_monotonic_seconds=frame.stamp.receive_monotonic_seconds,
            preprocessing_seconds=frame.preprocessing_seconds,
            source_encoding=frame.source_encoding, frame_id=frame.source_frame_id,
            preprocessing=frame.preprocessing)
    write_json(directory / 'image_evidence.json', evidence)
    return evidence


def prediction_preview(actions: np.ndarray, observation: Any, config: dict,
                       limits: dict | None = None) -> dict:
    """Inspect raw predictions without creating executable targets or commands."""
    first_action_from_chunk(actions)
    rows = []
    previous = None
    transitions = 0
    for index, action in enumerate(actions):
        decision = droid_gripper_decision(float(action[7]))
        closed = bool(decision.binary_closure)
        if previous is not None and previous != closed:
            transitions += 1
        previous = closed
        row = dict(index=index, raw_action=action,
                   gripper=dataclasses.asdict(decision), intent='CLOSE' if closed else 'OPEN',
                   requested_register=config['gripper']['close_command' if closed else 'open_command'],
                   inside_existing_execution_window=index < 8,
                   arm_coordinates_outside_clip_range=bool(np.any(np.abs(action[:7]) > 1)),
                   gripper_outside_unit_interval=not 0 <= float(action[7]) <= 1)
        if limits is not None:
            row['snapshot_position_feasibility'] = candidate_target(
                action, observation.joint_snapshot.position_rad, limits)
        rows.append(row)
    return dict(rows=rows, predicted_gripper_transitions=transitions,
                first_intent=rows[0]['intent'], policy_actions_executed=0,
                gripper_commands_issued=0, safe_to_execute=False,
                reference='observation snapshot only; execution must acquire fresh measured q per tick',
                joint_limits_evaluated=limits is not None,
                unevaluated=['fresh execution reference', 'controller/forwarder delivery',
                             'tool geometry', 'collision/workspace clearances',
                             'velocity/acceleration supervision', 'physical grasp outcome'])


def shadow_requests(
    *, collector: Any, policy: Any, config: dict, directory: Path,
    record: dict, requests: int, observation_timeout: float,
    spin_once: Callable, ros_now: Callable, warmup_seed: int,
    episode_seed: int, limits: dict | None = None,
) -> None:
    """Reuse the discarded warm-up and all-four-source barrier before index 0."""
    if type(requests) is not int or not 1 <= requests <= 100:
        raise ValueError('C4 requires 1..100 finite shadow requests')
    record['observed_server_metadata'] = policy.server_metadata
    validate_live_policy(policy, record)
    warmup = PolicyWarmup(warmup_policy_seed=warmup_seed, policy_episode_seed=episode_seed)
    observation, sample_dir = prepare_warmed_observation(
        warmup=warmup, collector=collector, policy=policy, output_dir=directory,
        observation_timeout=observation_timeout, spin_once=spin_once,
        ros_now=ros_now, config=config, record=record)
    gate = CameraPairGate()
    for index in range(requests):
        if index:
            observation, sample_dir = prepare_live_request(
                collector=collector, output_dir=directory, index=index,
                policy_episode_seed=episode_seed, observation_timeout=observation_timeout,
                spin_once=spin_once, config=config, record=record, gate=gate)
        else:
            gate.commit((observation.wrist_frame.stamp.ros_seconds,
                         observation.exterior_frame.stamp.ros_seconds))
        try:
            infer_live_request(observation=observation, sample_dir=sample_dir, policy=policy,
                               index=index, policy_episode_seed=episode_seed,
                               ros_now=ros_now, config=config)
        finally:
            save_images(observation, sample_dir)
        with np.load(sample_dir / 'actions.npz', allow_pickle=False) as bundle:
            preview = prediction_preview(bundle['actions'], observation, config, limits)
        write_json(sample_dir / 'predicted_commands.json', preview)
        record['completed_requests'] = index + 1


def run(args: Any) -> int:
    """Bounded subscriber-only ROS lifecycle; execution blockers are never lifted."""
    config = load_shadow_config(args.config)
    if config.get('integration_stage') != 'c4':
        raise ValueError('Use an explicit C4 profile; historical profiles remain on their entry points')
    provenance_mode = getattr(args, 'provenance_mode', 'development')
    validate_mode(provenance_mode, config)
    if (args.mode not in ('observations', 'shadow', 'readiness')
            or type(args.samples) is not int or not 1 <= args.samples <= 100
            or not args.prompt.strip()
            or any(type(seed) is not int or not 0 <= seed <= 0x7fffffff
                   for seed in (args.policy_episode_seed, args.warmup_policy_seed))
            or any(not np.isfinite(v) or v <= 0 for v in (args.observation_timeout, args.policy_timeout))):
        raise ValueError('C4 requires a valid mode, prompt, 1..100 samples and finite timeouts')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    record = dict(schema_version=1, milestone='physical_fr3_c4', mode=args.mode,
                  start_utc=utc_now(), config=config, actuation=ZERO_ACTUATION,
                  policy_actions_executed=0, gripper_commands_issued=0,
                  completed_requests=0, status='startup_failed',
                  physical_rollout_readiness='UNVERIFIED', blockers=execution_blockers(config),
                  camera_selection='latest advancing, historical 500ms age / 250ms cross-source skew',
                  visual_role_confirmation='pending researcher inspection',
                  provenance_mode=provenance_mode,
                  unconfirmed_execution_gates=['independent workspace/collision and velocity/acceleration protection',
                      'validated controller frames/load', 'measured usable opening width',
                      'robot-native safety and E-stop/operator supervision',
                      'fresh streaming health and target-application evidence'],
                  prompt=args.prompt, policy_episode_seed=args.policy_episode_seed,
                  warmup_policy_seed=args.warmup_policy_seed)
    node = transport = collector = None
    initialized = False
    started = time.monotonic()
    try:
        root = Path(__file__).resolve().parents[3]
        record['provenance'] = capture_provenance(
            root, args.lab_stack_dir, args.igd_control_dir, args.config, config)
        record['readiness'] = readiness_report(config, record['provenance'])
        record['source_integrity'] = dict(record['provenance']['manifest'])
        record['source_integrity']['matches_reviewed_manifest'] = record['source_integrity']['matches']
        write_json(args.output_dir / 'provenance.json', record['provenance'])
        if provenance_mode == 'strict':
            require_qualified_sources(record['provenance'])
        if record['provenance']['openpi'].get('commit') != OPENPI_COMMIT or record['provenance']['openpi'].get('dirty', True):
            raise ValueError('C4 requires the unchanged clean pinned OpenPI checkout')
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        record['workspace'] = workspace_evidence(args.lab_stack_dir)
        record['readiness']['observation']['status'] = 'checking'
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node('saps_physical_c4', enable_rosout=False,
                                 start_parameter_services=False)
        collector = RosPhysicalObservationCollector(
            SubscriptionBoundary(node), observation_contract(config), prompt=args.prompt,
            freshness=ObservationFreshness(**config['freshness']), preserve_native_images=True,
            qos_profile=qos_profile_sensor_data, camera_qos_profile=camera_qos(config))
        deadline = time.monotonic() + args.observation_timeout
        while True:
            rclpy.spin_once(node, timeout_sec=.05)
            try:
                record['initial_graph'] = validate_graph(node, config)
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
        record['interfaces'] = node_interface_evidence(node)
        record['camera_identity'] = camera_evidence(node, config)
        preflight = args.output_dir / 'observations'
        preflight.mkdir()
        gate = CameraPairGate()
        for index in range(args.samples if args.mode == 'observations' else 1):
            acquire = time.monotonic()
            observation, sample_dir = prepare_live_request(
                collector=collector, output_dir=preflight, index=index,
                policy_episode_seed=args.policy_episode_seed,
                observation_timeout=args.observation_timeout,
                spin_once=lambda: rclpy.spin_once(node, timeout_sec=.01),
                config=config, record=record, gate=gate)
            record['last_acquisition_and_evidence_seconds'] = time.monotonic() - acquire
            save_images(observation, sample_dir)
            record['gripper_readiness'] = gripper_readiness(
                node, collector, config, commands=args.mode == 'readiness')
            record['completed_observations'] = index + 1
            record['readiness']['observation']['status'] = 'passed'
            record['provenance']['effective_hardware'] = dict(
                status='read_only_observation_evidence', camera_identity=record['camera_identity'],
                gripper=record['gripper_readiness'],
                limitation='Configured controller/tool/load and independent protections remain unverified')
        limits = None
        if args.robot_description is not None:
            xml = args.robot_description.read_text()
            limits = limits_from_urdf(xml)
            write_json(args.output_dir / 'joint_limits.json', limits)
            record['robot_description'] = evidence_file(args.robot_description)
        if args.mode == 'readiness':
            record['controller_and_forwarder'] = controller_evidence(node, args.controller_manager)
        if args.mode in ('shadow', 'readiness'):
            record['readiness']['policy_inference']['status'] = 'checking'
            transport = BoundedWebsocketClient(args.host, args.port, args.policy_timeout)
            policy = OpenPiDroidPolicy(client=transport)
            record['observed_server_metadata'] = policy.server_metadata
            validate_live_policy(policy, record)
            record['readiness']['policy_inference']['status'] = 'identity_verified_inference_not_tested'
            if args.mode == 'shadow':
                destination = args.output_dir / 'shadow'
                destination.mkdir()
                shadow_requests(collector=collector, policy=policy, config=config,
                                directory=destination, record=record, requests=args.samples,
                                observation_timeout=args.observation_timeout,
                                spin_once=lambda: rclpy.spin_once(node, timeout_sec=.01),
                                ros_now=lambda: node.get_clock().now().nanoseconds / 1e9,
                                warmup_seed=args.warmup_policy_seed,
                                episode_seed=args.policy_episode_seed, limits=limits)
                record['readiness']['policy_inference']['status'] = 'passed'
        record['final_graph'] = validate_graph(node, config)
        record['final_camera_identity'] = camera_evidence(node, config)
        if record['final_graph'] != record['initial_graph']:
            raise RuntimeError('C4 source ownership changed during validation')
        record['status'] = 'passed_non_actuating_checks'
    except (Exception, KeyboardInterrupt) as error:
        record.update(status='failed', error=f'{type(error).__name__}: {error}')
        if 'readiness' in record:
            for stage in ('observation', 'policy_inference'):
                state = record['readiness'][stage]
                if state['status'] == 'checking' or (stage == 'policy_inference' and args.mode == 'shadow'
                                                    and state['status'] == 'identity_verified_inference_not_tested'):
                    state.update(status='failed', reason=record['error'])
        if hasattr(error, 'evidence'):
            record['readiness_failure_evidence'] = error.evidence
    finally:
        if collector is not None:
            record['rates'] = collector.source_rates()
            record['callback_errors'] = collector.errors
            record['last_gripper_status'] = collector.latest_gripper_status
            record['last_camera_metadata'] = collector.last_camera_metadata
        for cleanup in (lambda: transport.close() if transport is not None else None,
                        lambda: node.destroy_node() if node is not None else None,
                        lambda: rclpy.try_shutdown() if initialized else None):
            try:
                cleanup()
            except Exception as error:
                record.setdefault('cleanup_errors', []).append(str(error))
                record['status'] = 'failed'
        record['duration_seconds'] = time.monotonic() - started
        if 'readiness' in record:
            write_json(args.output_dir / 'readiness.json', record['readiness'])
            write_json(args.output_dir / 'effective_hardware.json', record['provenance']['effective_hardware'])
        write_json(args.output_dir / 'run.json', record)
        print(json.dumps(dict(status=record['status'], physical_rollout_readiness='UNVERIFIED',
                              blockers=record['blockers'], output=str(args.output_dir))))
    return 0 if record['status'] == 'passed_non_actuating_checks' else 1
