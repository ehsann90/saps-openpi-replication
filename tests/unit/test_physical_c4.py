"""C4 end-to-end observations and seeded shadow using fake ROS/server only."""
import copy
import dataclasses
import json
from pathlib import Path
import tempfile
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from saps.physical import c4_observation as c4
from saps.physical import c4_readiness as readiness
from saps.physical.embodiment import FR3_JOINT_NAMES
from saps.physical.hardware_provenance import require_supported_arm_geometry
from saps.physical.shadow_config import load_shadow_config, observation_contract
from saps.physical.ros_observation import RosPhysicalObservationCollector
from saps.physical.live_observation import ObservationFreshness
from test_physical_live_shadow import FakeClient

ROOT = Path(__file__).resolve().parents[2]


def profile():
    c = json.loads((ROOT / 'configs/physical_pi05_fr3_c4.json').read_text())
    c['gripper']['port'] = '/dev/serial/by-id/OFFLINE_FIXTURE'
    c['gripper']['calibration'] = dict(verified=True, evidence='offline fixture only',
        open_position=3, closed_position=227, maximum_width_m=.08)
    return c


class Node:
    def __init__(self, config):
        self.config = config
        self.now = 10.
        self.callbacks = {}
        self.qos = {}
        self.publishers = self.clients = []
        self.destroy_node = Mock()
        self.owner = 'offline-instance'
        self.owner_pid = 123
        self.bad_profile = False
        self.stale = False
        self.position = 102
        self.fault = 0

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=round(self.now * 1e9)))

    def create_subscription(self, kind, topic, callback, qos):
        self.callbacks[topic] = callback
        self.qos[topic] = qos
        return object()

    def get_node_names_and_namespaces(self):
        return [('saps_robotiq', '/'), ('saps_physical_c4', '/')]

    def get_service_names_and_types_by_node(self, name, namespace):
        return [(f'/robotiq/{kind}', ['std_srvs/srv/Trigger']) for kind in ('open', 'close', 'stop')]

    def get_publishers_info_by_topic(self, topic):
        for key in ('wrist_camera', 'exterior_camera'):
            if topic == self.config[key]['topic']:
                ns, name = self.config[key]['node'].rsplit('/', 1)
                return [NS(node_namespace=ns, node_name=name, topic_type='sensor_msgs/msg/Image', endpoint_gid=b'image')]
        name = 'saps_robotiq' if topic == '/robotiq/status' else 'joint_state_broadcaster'
        kind = 'std_msgs/msg/String' if name == 'saps_robotiq' else 'sensor_msgs/msg/JointState'
        return [NS(node_namespace='/', node_name=name, topic_type=kind, endpoint_gid=b'state')]

    def spin(self, *args, **kwargs):
        self.now += .11
        source = self.now - (1 if self.stale else .02)
        stamp = NS(sec=int(source), nanosec=round((source - int(source)) * 1e9))
        for key, color in [('wrist_camera', 37), ('exterior_camera', 211)]:
            image = np.full((720, 1280, 4), color, dtype=np.uint8)
            image[:, :, 3] = 255
            self.callbacks[self.config[key]['topic']](NS(
                header=NS(stamp=stamp, frame_id=key + '_optical'),
                height=720, width=1280, step=5120,
                encoding='mono8' if self.bad_profile else 'bgra8', data=image.tobytes()))
        self.callbacks['/franka/joint_states'](NS(header=NS(stamp=stamp),
                                                name=FR3_JOINT_NAMES, position=[0.] * 7))
        settings = self.config['gripper']
        status = dict(activated=True, activation_state=3, fault=self.fault, ready=self.fault == 0,
                      actual_position=self.position, requested_position=255,
                      object_status=2, object_detected=True, source_ros_seconds=source,
                      serial_exclusive=True, serial_owner_pid=self.owner_pid,
                      server_instance=self.owner, commands_authorized=False, command_busy=False,
                      **{key: settings[key] for key in ('port', 'slave', 'speed_register',
                          'force_register', 'open_command', 'close_command')})
        self.callbacks[settings['state_topic']](NS(data=json.dumps(status)))


def ros_modules(node):
    ros, qos, messages = ModuleType('rclpy'), ModuleType('rclpy.qos'), ModuleType('sensor_msgs.msg')
    ros.init, ros.try_shutdown = Mock(), Mock()
    ros.create_node = Mock(return_value=node)
    ros.spin_once = node.spin
    qos.qos_profile_sensor_data = 'sensor_best_effort'
    qos.QoSProfile = lambda **kwargs: NS(**kwargs)
    qos.ReliabilityPolicy = NS(RELIABLE='reliable', BEST_EFFORT='best_effort')
    qos.HistoryPolicy = NS(KEEP_LAST='keep_last')
    messages.Image = messages.JointState = NS
    strings = ModuleType('std_msgs.msg')
    strings.String = NS
    return {'rclpy': ros, 'rclpy.qos': qos, 'sensor_msgs': ModuleType('sensor_msgs'),
            'sensor_msgs.msg': messages, 'std_msgs': ModuleType('std_msgs'), 'std_msgs.msg': strings}


class C4Tests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = profile()
        self.config_path = self.root / 'config.json'
        self.config_path.write_text(json.dumps(self.config))
        self.node = Node(self.config)
        self.modules = ros_modules(self.node)

    def collector(self):
        with patch.dict('sys.modules', self.modules):
            return RosPhysicalObservationCollector(self.node, observation_contract(self.config),
                prompt='offline fixture', freshness=ObservationFreshness(), preserve_native_images=True,
                joint_state_type=NS, image_type=NS, qos_profile='sensor_best_effort',
                camera_qos_profile='reliable_depth_2')

    def test_profile_keeps_historical_limits_and_policy(self):
        c = load_shadow_config(self.config_path)
        old = load_shadow_config(ROOT / 'configs/physical_pi05_fr3.json')
        self.assertEqual(c['policy'], old['policy'])
        self.assertEqual(c['freshness'], old['freshness'])
        self.assertEqual(c['exterior_camera']['camera_model'], 'zed2i')
        self.assertEqual(c['camera_interface']['selection'], 'latest_advancing')
        for change in [('camera_interface','qos','best_effort'), ('freshness','maximum_source_age_seconds',.1),
                       ('wrist_camera','serial','32919961'), ('exterior_camera','camera_model','zed2')]:
            bad = copy.deepcopy(c)
            bad[change[0]][change[1]] = change[2]
            self.config_path.write_text(json.dumps(bad))
            with self.assertRaises(ValueError):
                load_shadow_config(self.config_path)

    def test_prepare_profile_preserves_calibration_speed_force_and_original(self):
        raw = self.config_path.read_bytes()
        output = self.root / 'new_profile.json'
        readiness.prepare_profile(self.config_path, output)
        saved = load_shadow_config(output)
        self.assertEqual(saved['gripper'], self.config['gripper'])
        self.assertEqual(self.config_path.read_bytes(), raw)
        with self.assertRaises(FileExistsError):
            readiness.prepare_profile(self.config_path, output)
        with self.assertRaises(ValueError):
            require_supported_arm_geometry(saved)

    def test_prepare_profile_preserves_operator_tool_evidence_without_setters(self):
        hardware = json.loads(self.config_path.read_text())
        hardware['mounted_tool'] = dict(assembly_verified=False,
                                       evidence='operator tool evidence fixture')
        self.config_path.write_text(json.dumps(hardware))
        before = self.config_path.read_bytes()
        output = self.root / 'tool_profile.json'
        with patch('subprocess.run', side_effect=AssertionError('no robot commands')):
            readiness.prepare_profile(self.config_path, output)
        prepared = load_shadow_config(output)
        self.assertEqual(prepared['mounted_tool'], hardware['mounted_tool'])
        self.assertFalse(prepared['execution_enabled'])
        self.assertFalse(prepared['geometry']['execution_ready'])
        self.assertEqual(self.config_path.read_bytes(), before)

    def test_complete_bgra_observation_and_png_roles_without_serial_owner(self):
        with patch('franka_robotiq.robotiq.RobotiqRtu', side_effect=AssertionError('second serial owner')):
            collector = self.collector()
            self.node.spin()
            observation = collector.assemble()
            self.assertEqual(observation.policy_input['observation/wrist_image_left'].shape, (180,320,3))
            self.assertEqual(observation.policy_input['observation/wrist_image_left'].dtype, np.uint8)
            self.assertAlmostEqual(float(observation.policy_input['observation/gripper_position'][0]), 99 / 224)
            c4.save_images(observation, self.root)
        self.assertTrue(np.all(cv2.imread(str(self.root / 'wrist_policy_rgb.png')) == 37))
        self.assertTrue(np.all(cv2.imread(str(self.root / 'external_policy_rgb.png')) == 211))
        self.assertEqual(self.node.qos[self.config['wrist_camera']['topic']], 'reliable_depth_2')
        self.assertEqual(self.node.qos['/franka/joint_states'], 'sensor_best_effort')
        self.assertGreaterEqual(observation.wrist_frame.preprocessing_seconds, 0)

    def test_owner_change_is_latched_and_fault_or_stale_feedback_rejected(self):
        collector = self.collector()
        self.node.spin()
        collector.assemble()
        self.node.owner = 'replacement-instance'
        self.node.spin()
        with self.assertRaisesRegex(RuntimeError, 'identity changed'):
            collector.assemble()
        self.node.owner = 'offline-instance'
        self.node.spin()
        with self.assertRaisesRegex(RuntimeError, 'identity changed'):
            collector.assemble()

    def test_bad_images_faults_stale_and_uncalibrated_block_observation(self):
        for kind in ('bad_profile', 'stale', 'fault', 'calibration'):
            with self.subTest(kind=kind):
                self.node = Node(self.config)
                self.modules = ros_modules(self.node)
                collector = self.collector()
                if kind == 'calibration':
                    self.config['gripper']['calibration']['verified'] = False
                elif kind == 'fault':
                    self.node.fault = 9
                else:
                    setattr(self.node, kind, True)
                self.node.spin()
                with self.assertRaises((RuntimeError, ValueError)):
                    collector.assemble()
                self.config['gripper']['calibration']['verified'] = True

    def test_warmed_shadow_uses_fake_server_and_never_calls_actuators(self):
        collector = self.collector()
        client = FakeClient()
        policy = c4.OpenPiDroidPolicy(client=client)
        record = dict(completed_requests=0)
        directory = self.root / 'shadow'
        directory.mkdir()
        with patch('franka_robotiq.robotiq.RobotiqRtu', side_effect=AssertionError('serial opened')):
            c4.shadow_requests(collector=collector, policy=policy, config=self.config,
                directory=directory, record=record, requests=2, observation_timeout=2,
                spin_once=self.node.spin, ros_now=lambda:self.node.now,
                warmup_seed=20260917, episode_seed=20260827)
        self.assertEqual(len(client.requests), 3)
        self.assertEqual([r['policy_episode_seed'] for r in client.requests], [20260917,20260827,20260827])
        self.assertEqual([r['replan_index'] for r in client.requests], [0,0,1])
        self.assertTrue(record['post_warmup_observation_acquired'])
        self.assertTrue(record['warmup']['response_discarded'])
        self.assertEqual(record['completed_requests'], 2)
        preview = json.loads((directory / 'request_0000/predicted_commands.json').read_text())
        self.assertEqual(preview['policy_actions_executed'], 0)
        self.assertFalse(preview['safe_to_execute'])
        self.assertEqual(len(preview['rows']),15)
        self.assertTrue((directory / 'request_0000/actions.npz').exists())
        self.assertFalse(self.node.clients)

    def test_wrong_policy_identity_blocks_before_requests(self):
        collector = self.collector()
        client = FakeClient()
        client.get_server_metadata = lambda: dict(saps_seeded_sampling=dict(
            policy_config_name='pi05_droid', policy_checkpoint='fine_tuned_other_checkpoint'))
        with self.assertRaises(RuntimeError):
            c4.shadow_requests(collector=collector, policy=c4.OpenPiDroidPolicy(client=client),
                config=self.config, directory=self.root, record={}, requests=1,
                observation_timeout=1, spin_once=self.node.spin, ros_now=lambda:self.node.now,
                warmup_seed=20260917, episode_seed=20260827)
        self.assertFalse(client.requests)

    def test_readiness_requires_services_and_owner_identity(self):
        collector = self.collector()
        self.node.spin()
        result = readiness.gripper_readiness(self.node,collector,self.config,commands=False)
        self.assertFalse(result['serial_opened_by_rollout'])
        with self.assertRaisesRegex(RuntimeError, 'authorized'):
            readiness.gripper_readiness(self.node,collector,self.config,commands=True)
        self.node.get_node_names_and_namespaces = lambda: [('saps_robotiq','/')] * 2
        with self.assertRaisesRegex(RuntimeError, 'exactly one'):
            readiness.gripper_readiness(self.node,collector,self.config,commands=False)

    def test_nonfinite_actions_and_threshold_are_unchanged(self):
        collector = self.collector()
        self.node.spin()
        observation = collector.assemble()
        actions = np.zeros((15,8), dtype=float)
        actions[0,7], actions[1,7] = .5, .500001
        preview = c4.prediction_preview(actions,observation,self.config)
        self.assertEqual([r['intent'] for r in preview['rows'][:2]], ['OPEN','CLOSE'])
        self.assertEqual([r['requested_register'] for r in preview['rows'][:2]], [0,255])
        actions[2,0] = np.nan
        with self.assertRaises(ValueError):
            c4.prediction_preview(actions,observation,self.config)

    def test_observation_only_ros_lifecycle_does_not_create_policy_client(self):
        args = NS(config=self.config_path, mode='observations', samples=1, prompt='offline fixture',
                  output_dir=self.root/'run', observation_timeout=2, policy_timeout=1,
                  policy_episode_seed=20260827, warmup_policy_seed=20260917,
                  lab_stack_dir=ROOT, igd_control_dir=ROOT, robot_description=None,
                  host='127.0.0.1', port=8000, controller_manager='/controller_manager')
        provenance = dict(repository={'dirty':True}, lab={'dirty':True},
                          openpi={'commit':c4.OPENPI_COMMIT,'dirty':False},
                          manifest={'matches':False, 'changed':['fixture']},
                          qualification={'status':'unqualified_sources','blockers':['dirty fixture']})
        with patch.dict('sys.modules', self.modules), patch.object(c4,'camera_evidence',return_value={'verified':True}), patch.object(
                c4,'workspace_evidence',return_value={'source':'offline'}), patch.object(
                c4,'capture_provenance',return_value=provenance), patch.object(
                c4,'BoundedWebsocketClient',side_effect=AssertionError('policy contacted')):
            self.assertEqual(c4.run(args),0)
        result = json.loads((args.output_dir/'run.json').read_text())
        self.assertEqual(result['physical_rollout_readiness'],'UNVERIFIED')
        self.assertEqual(result['blockers'], [])
        self.assertEqual(result['gripper_commands_issued'],0)
        self.assertEqual(result['readiness']['observation']['status'], 'passed')
        self.assertEqual(result['readiness']['policy_inference']['status'], 'not_checked')
        self.assertEqual(result['readiness']['physical_runtime']['status'], 'unverified')
        self.assertEqual(result['readiness']['reproducible_experiment_qualification']['status'], 'not_qualified')
        self.assertTrue(json.loads((args.output_dir/'provenance.json').read_text())['repository']['dirty'])
        self.assertNotIn('dirty fixture', ' '.join(result['blockers']))
        self.modules['rclpy'].try_shutdown.assert_called_once()
        self.node.destroy_node.assert_called_once()

    def test_dirty_development_shadow_passes_without_experiment_qualification(self):
        args = NS(config=self.config_path, mode='shadow', samples=1, prompt='offline fixture',
                  output_dir=self.root/'dirty_shadow', observation_timeout=2, policy_timeout=1,
                  policy_episode_seed=20260827, warmup_policy_seed=20260917,
                  lab_stack_dir=ROOT, igd_control_dir=ROOT, robot_description=None,
                  host='127.0.0.1', port=8000, controller_manager='/controller_manager',
                  provenance_mode='development')
        provenance = dict(repository={'dirty':True},lab={'dirty':True},
                          openpi={'commit':c4.OPENPI_COMMIT,'dirty':False},
                          manifest={'matches':False},
                          qualification={'status':'unqualified_sources','blockers':['dirty code']})
        client = FakeClient()
        transport = Mock()
        policy = c4.OpenPiDroidPolicy(client=client)
        with patch.dict('sys.modules',self.modules), \
                patch.object(c4,'capture_provenance',return_value=provenance), \
                patch.object(c4,'workspace_evidence',return_value={'fixture':True}), \
                patch.object(c4,'camera_evidence',return_value={'fixture':True}), \
                patch.object(c4,'BoundedWebsocketClient',return_value=transport), \
                patch.object(c4,'OpenPiDroidPolicy',return_value=policy):
            self.assertEqual(c4.run(args),0)
        result = json.loads((args.output_dir/'run.json').read_text())
        self.assertEqual(len(client.requests),2)  # One discarded warm-up and one main request.
        self.assertEqual(result['readiness']['observation']['status'],'passed')
        self.assertEqual(result['readiness']['policy_inference']['status'],'passed')
        self.assertEqual(result['readiness']['physical_runtime']['status'],'unverified')
        self.assertEqual(result['readiness']['reproducible_experiment_qualification']['status'],'not_qualified')
        self.assertEqual(result['policy_actions_executed'],0)
        self.assertEqual(result['gripper_commands_issued'],0)
        self.assertFalse(self.node.clients)


class C4EvidenceTests(unittest.TestCase):
    def test_source_manifest_differences_are_reported_without_repinning(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'new.py').write_text('changed source')
            manifest = ROOT/'configs/zed_robotiq_lab_sources.json'
            before = manifest.read_bytes()
            with patch.object(readiness.subprocess,'check_output',return_value=b'new.py\0'):
                report = readiness.source_integrity_evidence(root)
            self.assertFalse(report['matches_reviewed_manifest'])
            self.assertEqual(report['added'],['new.py'])
            self.assertEqual(manifest.read_bytes(),before)

    def test_camera_checks_reuse_lab_contract_and_reject_wrong_effective_identity(self):
        module = ModuleType('franka_zed_cameras.camera_validation')
        c = profile()
        sources = {role:dict(valid=True, parameters={
            'general.serial_number':int(c[key]['serial']),
            'general.camera_model':c[key]['camera_model']})
            for role,key in [('wrist','wrist_camera'),('external','exterior_camera')]}
        module.inspect_sources = lambda node:sources
        module.provenance = lambda:dict(sdk_version_from_headers='5.5.1',
            packages={key:dict(version='5.5.0') for key in ('zed_wrapper','zed_components','zed_msgs')})
        with patch.dict('sys.modules', {'franka_zed_cameras':ModuleType('franka_zed_cameras'),
                'franka_zed_cameras.camera_validation':module}):
            self.assertIn('sources',readiness.camera_evidence(object(),c))
            sources['wrist']['parameters']['general.serial_number'] = 999
            with self.assertRaisesRegex(RuntimeError,'identity disagree'):
                readiness.camera_evidence(object(),c)

    def test_controller_check_is_read_only_and_rejects_inactive_or_multiple_owners(self):
        node = NS(get_subscriptions_info_by_topic=lambda topic:[NS(
            node_name='fr3_streaming_joint_target_forwarder', node_namespace='/')])
        response = NS(stdout='\x1b[92mfr3_streaming_joint_impedance_controller fr3_lab_stack/StreamingJointImpedanceController active\x1b[0m\n')
        with patch.object(readiness.subprocess,'run',return_value=response) as call:
            readiness.controller_evidence(node,'/controller_manager')
            self.assertEqual(call.call_args.args[0],['ros2','control','list_controllers',
                             '--controller-manager','/controller_manager'])
            response.stdout = response.stdout.replace(' active',' inactive')
            with self.assertRaises(RuntimeError):
                readiness.controller_evidence(node,'/controller_manager')
            response.stdout = response.stdout.replace(' inactive',' active')
            node.get_subscriptions_info_by_topic = lambda topic:[]
            with self.assertRaises(RuntimeError):
                readiness.controller_evidence(node,'/controller_manager')

    def test_geometry_flag_does_not_authorize_c4_execution(self):
        c = profile()
        c['geometry']['execution_ready'] = True
        self.assertEqual(readiness.execution_blockers(c), [])



if __name__ == '__main__':
    unittest.main()
