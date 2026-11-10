"""Read-only settling and persistent observation tests with fake devices/ROS."""
from __future__ import annotations
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from saps.physical.robotiq import RobotiqStatus
from saps.physical.robotiq_node import ObservationError, main, read_observation
from test_physical_robotiq_activation import Clock, Device, status


def calibration() -> dict:
    return dict(verified=True, evidence='OFFLINE fixture, not physical evidence',
                open_position=3, closed_position=228, maximum_width_m=.085)


def settings() -> dict:
    return dict(speed_register=64, force_register=32, open_command=0,
                close_command=255, command_timeout_seconds=.1,
                port='/dev/serial/by-id/OFFLINE_FIXTURE', slave=9,
                baudrate=115200, serial_timeout_seconds=.2,
                namespace='/robotiq', state_topic='/robotiq/status',
                calibration=calibration())


def measured(position: int, *, fault: int = 0) -> RobotiqStatus:
    # A CLOSE request and contact must not replace actual measured position.
    return RobotiqStatus(True, True, 3, 2, fault, 255, position, 7)


class StandaloneObservationTests(unittest.TestCase):
    def observe(self, device: Device, cal: dict | None = None) -> dict:
        clock = Clock()
        return read_observation(device, calibration() if cal is None else cal,
                                timeout=.1, now=clock.now, sleep=clock.sleep)

    def test_initial_watchdog_fault_settles_using_reads_only(self) -> None:
        device = Device([measured(102, fault=9), measured(102)])
        result = self.observe(device)
        self.assertTrue(result['success'])
        self.assertEqual(result['attempts'], 2)
        self.assertAlmostEqual(result['policy_closure'], 99 / 225)
        self.assertTrue(result['object_detected'])
        self.assertEqual(device.actions, [])

    def test_open_close_and_intermediate_measurements(self) -> None:
        for position, expected in ((3, 0.), (228, 1.), (102, 99 / 225)):
            with self.subTest(position=position):
                result = self.observe(Device([measured(position)]))
                self.assertAlmostEqual(result['policy_closure'], expected)
                self.assertAlmostEqual(result['estimated_width_m'], .085 * (1 - expected))
                self.assertEqual(result['requested_position'], 255)

    def test_persistent_nonready_or_watchdog_is_bounded(self) -> None:
        for last in (status(state=0), measured(102, fault=9)):
            device = Device([last])
            with self.subTest(last=last), self.assertRaises(ObservationError) as caught:
                self.observe(device)
            report = caught.exception.report
            self.assertEqual(report['last_status'], last.as_dict())
            self.assertEqual(report['reason'], 'feedback_not_ready')
            self.assertLessEqual(report['elapsed_seconds'], .100001)
            self.assertFalse(report['success'])
            self.assertEqual(device.actions, [])

    def test_timeout_or_communication_failure_is_not_retried(self) -> None:
        for error in (TimeoutError('read timeout'), OSError('CRC/communication failure')):
            device = Device([measured(102, fault=9), error])
            with self.subTest(error=error), self.assertRaises(ObservationError) as caught:
                self.observe(device)
            report = caught.exception.report
            self.assertEqual(report['reason'], 'communication_error')
            self.assertEqual(report['attempts'], 2)
            self.assertEqual(report['last_status']['fault'], 9)
            self.assertIn(str(error), report['error'])
            self.assertEqual(device.actions, [])

    def test_nonwatchdog_fault_fails_without_waiting(self) -> None:
        device = Device([measured(102, fault=12)])
        with self.assertRaises(ObservationError) as caught:
            self.observe(device)
        self.assertEqual(caught.exception.report['reason'], 'gripper_fault')
        self.assertEqual(caught.exception.report['attempts'], 1)
        self.assertEqual(device.actions, [])

    def test_missing_or_unverified_calibration_remains_blocked(self) -> None:
        for cal in ({}, dict(calibration(), verified=False),
                    dict(calibration(), evidence=None), {'verified': True, 'evidence': 'incomplete'}):
            device = Device([measured(102)])
            with self.subTest(cal=cal), self.assertRaises(ObservationError) as caught:
                self.observe(device, cal)
            self.assertEqual(caught.exception.report['reason'], 'invalid_calibration')
            self.assertTrue(caught.exception.report['last_status']['ready'])
            self.assertEqual(device.actions, [])


class CliLifecycleTests(unittest.TestCase):
    def run_cli(self, device: Device, *, mode: str = 'serve',
                authorize: bool = False, spin_error: BaseException | None = None,
                already_shutdown: bool = False, setup_error: bool = False,
                executor_error: bool = False, expect_failure: bool = False) -> tuple:
        ros = ModuleType('rclpy')
        groups = ModuleType('rclpy.callback_groups')
        executors = ModuleType('rclpy.executors')
        messages = ModuleType('std_msgs.msg')
        services = ModuleType('std_srvs.srv')
        class ExternalShutdownException(Exception):
            pass
        executors.ExternalShutdownException = ExternalShutdownException
        node, executor = Mock(), Mock()
        ros.init = Mock()
        ros.create_node = Mock(side_effect=RuntimeError('node creation failed')) if setup_error else Mock(return_value=node)
        ros.shutdown = Mock(side_effect=AssertionError('duplicate unsafe shutdown'))
        ros.context_ok = True
        ros.shutdown_count = 0
        def try_shutdown() -> None:
            if ros.context_ok:
                ros.shutdown_count += 1
                ros.context_ok = False
        ros.try_shutdown = Mock(side_effect=try_shutdown)
        def spin() -> None:
            if already_shutdown:
                ros.context_ok = False
            if spin_error:
                raise spin_error
            if already_shutdown:
                raise ExternalShutdownException()
        executor.spin.side_effect = spin
        if executor_error:
            executor.shutdown.side_effect = RuntimeError('executor cleanup failed')
        executors.MultiThreadedExecutor = Mock(return_value=executor)
        groups.ReentrantCallbackGroup = Mock()
        messages.String = Mock()
        services.Trigger = Mock()
        modules = {'rclpy': ros, 'rclpy.callback_groups': groups,
                   'rclpy.executors': executors, 'std_msgs': ModuleType('std_msgs'),
                   'std_msgs.msg': messages, 'std_srvs': ModuleType('std_srvs'),
                   'std_srvs.srv': services}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps(dict(gripper=settings())))
            argv = ['robotiq_node', '--config', str(path), '--mode', mode]
            if authorize:
                argv.append('--authorize-gripper')
            clock, output = Clock(), io.StringIO()
            def observe(device: Device, cal: dict, **kwargs: object) -> dict:
                return read_observation(device, cal, **kwargs,
                                        now=clock.now, sleep=clock.sleep)
            with patch.dict(sys.modules, modules), patch('sys.argv', argv), patch(
                    'saps.physical.robotiq_node.RobotiqRtu', return_value=device
            ) as owner, patch('saps.physical.robotiq_node.read_observation', side_effect=observe), redirect_stdout(output):
                if setup_error or executor_error:
                    with self.assertRaises(RuntimeError):
                        main()
                elif expect_failure:
                    with self.assertRaises(SystemExit) as caught:
                        main()
                    self.assertEqual(caught.exception.code, 1)
                else:
                    main()
                owner.assert_called_once()
            self.assertTrue(device.closed)
            self.assertEqual(device.actions, [9, 1] if mode == 'open' else [])
            return ros, node, executor, output.getvalue()

    def test_normal_shutdown(self) -> None:
        ros, node, executor, _ = self.run_cli(Device([]))
        self.assertEqual(ros.shutdown_count, 1)
        node.destroy_node.assert_called_once()
        executor.shutdown.assert_called_once()
        ros.shutdown.assert_not_called()
        node.create_service.assert_not_called()

    def test_keyboard_interrupt_with_live_or_already_shutdown_context(self) -> None:
        for closed in (False, True):
            with self.subTest(closed=closed):
                ros, node, _, _ = self.run_cli(Device([]), spin_error=KeyboardInterrupt(), already_shutdown=closed)
                self.assertEqual(ros.shutdown_count, 0 if closed else 1)
                node.destroy_node.assert_called_once()
                ros.shutdown.assert_not_called()

    def test_external_context_shutdown(self) -> None:
        ros, node, _, _ = self.run_cli(Device([]), already_shutdown=True)
        self.assertEqual(ros.shutdown_count, 0)
        node.destroy_node.assert_called_once()

    def test_cleanup_after_setup_or_executor_failure(self) -> None:
        for setup in (True, False):
            ros, _, _, _ = self.run_cli(Device([]), setup_error=setup, executor_error=not setup)
            self.assertEqual(ros.shutdown_count, 1)

    def test_authorization_alone_does_not_cause_shutdown_writes(self) -> None:
        self.run_cli(Device([]), authorize=True, spin_error=KeyboardInterrupt())

    def test_observation_failure_prints_structured_diagnostic_and_closes(self) -> None:
        _, _, _, output = self.run_cli(Device([TimeoutError('read timeout')]), mode='observation', expect_failure=True)
        result = json.loads(output)
        self.assertEqual(result['reason'], 'communication_error')
        self.assertIsNone(result['last_status'])

    def test_observation_success_and_single_serial_owner(self) -> None:
        _, _, _, output = self.run_cli(Device([measured(102, fault=9), measured(102)]), mode='observation')
        self.assertTrue(json.loads(output)['success'])

    def test_open_retains_authorized_stop_on_exit(self) -> None:
        class MotionDevice(Device):
            def output(self, *, action: int, **fields: int) -> None:
                if action == 9:
                    self.asserted_fields = fields
                self.actions.append(action)
        final = RobotiqStatus(True, True, 3, 3, 0, 0, 3, 0)
        device = MotionDevice([measured(3), final])
        self.run_cli(device, mode='open', authorize=True)
        self.assertEqual(device.asserted_fields, dict(position=0, speed=64, force=32))


    def test_connection_failure_reports_unknown_status_without_retry(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps(dict(gripper=settings())))
            output = io.StringIO()
            with patch('sys.argv', ['robotiq_node', '--config', str(path), '--mode', 'observation']), patch(
                    'saps.physical.robotiq_node.RobotiqRtu', side_effect=OSError('port already owned')
            ) as owner, redirect_stdout(output), self.assertRaises(SystemExit) as caught:
                main()
            self.assertEqual(caught.exception.code, 1)
            owner.assert_called_once()
            result = json.loads(output.getvalue())
            self.assertEqual(result['reason'], 'connection_error')
            self.assertIsNone(result['last_status'])
            self.assertEqual(result['attempts'], 0)

    def test_interrupt_during_motion_retains_stop_and_serial_cleanup(self) -> None:
        class InterruptedMotion(Device):
            def read_status(self) -> RobotiqStatus:
                if self.actions:
                    raise KeyboardInterrupt()
                return measured(3)
            def output(self, *, action: int, **fields: int) -> None:
                self.actions.append(action)
        self.run_cli(InterruptedMotion([]), mode='open', authorize=True)


class PersistentAdapterTests(unittest.TestCase):
    def setUp(self) -> None:
        from saps.physical.embodiment import FR3_JOINT_NAMES
        from saps.physical.live_observation import ObservationFreshness
        from saps.physical.ros_observation import (
            RosCameraContract, RosObservationContract,
            RosPhysicalObservationCollector,
        )
        from test_physical_live_observation import FakeMessage, FakeRosNode
        self.owner_patch = patch('saps.physical.robotiq.RobotiqRtu', side_effect=AssertionError('competing Modbus owner'))
        self.owner = self.owner_patch.start()
        self.addCleanup(self.owner_patch.stop)
        self.clock_patch = patch('saps.physical.ros_observation.time.monotonic', return_value=1000.)
        self.clock = self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        messages = ModuleType('std_msgs.msg')
        messages.String = NS
        self.config = settings()
        contract = RosObservationContract(
            joint_state_topic='/arm', gripper_state_topic='/robotiq/status',
            wrist_camera=RosCameraContract('wrist', 'wrist', 'zedm', '/wrist'),
            exterior_camera=RosCameraContract('exterior', 'exterior', 'zed2', '/exterior'),
            gripper_config=self.config)
        self.node = FakeRosNode()
        with patch.dict(sys.modules, {'std_msgs': ModuleType('std_msgs'), 'std_msgs.msg': messages}):
            self.collector = RosPhysicalObservationCollector(
                self.node, contract, prompt='offline fixture',
                freshness=ObservationFreshness(), joint_state_type=object,
                image_type=bytes, qos_profile='sensor')
        arm = FakeMessage()
        arm.name, arm.position = FR3_JOINT_NAMES, [0.] * 7
        self.node.callbacks['/arm'](arm)
        image = FakeMessage()
        image.height = image.width = 2
        image.encoding, image.step, image.data = 'rgb8', 6, bytes(12)
        self.node.callbacks['/wrist'](image)
        self.node.callbacks['/exterior'](image)

    def feed(self, position: int = 102, **changes: object) -> None:
        record = dict(measured(position).as_dict(), source_ros_seconds=10.05,
                      **{key: self.config[key] for key in (
                          'port', 'slave', 'speed_register', 'force_register',
                          'open_command', 'close_command')})
        record.update(changes)
        self.node.callbacks['/robotiq/status'](NS(data=json.dumps(record)))

    def test_measured_conversion_and_contact_are_separate(self) -> None:
        for position, expected in ((3, 0.), (228, 1.), (102, 99 / 225)):
            with self.subTest(position=position):
                self.feed(position)
                observation = self.collector.assemble()
                self.assertAlmostEqual(float(observation.policy_input['observation/gripper_position'][0]), expected)
                self.assertTrue(observation.gripper_snapshot.feedback_metadata['object_detected'])
                self.assertEqual(observation.gripper_snapshot.feedback_metadata['requested_position'], 255)
        self.owner.assert_not_called()
        self.assertEqual(len(self.node.operations), 4)
        self.assertTrue(all(op[0] == 'subscription' for op in self.node.operations))

    def test_invalid_latest_feedback_cannot_reuse_previous_sample(self) -> None:
        for changes in ({'fault': 9}, {'error': 'serial timeout'},
                        {'ready': False}, {'activated': False},
                        {'activation_state': 0}, {'port': 'different adapter'}):
            with self.subTest(changes=changes):
                self.feed()
                self.collector.assemble()
                self.feed(**changes)
                with self.assertRaisesRegex(RuntimeError, 'invalid_gripper_feedback'):
                    self.collector.assemble()
        self.owner.assert_not_called()

    def test_missing_or_unverified_calibration_rejects_assembly(self) -> None:
        for cal in ({}, dict(calibration(), verified=False)):
            with self.subTest(cal=cal):
                self.config['calibration'] = calibration()
                self.feed()
                self.config['calibration'] = cal
                self.feed()
                with self.assertRaisesRegex(RuntimeError, 'invalid_gripper_feedback'):
                    self.collector.assemble()

    def test_missing_feedback_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, 'missing physical sources'):
            self.collector.assemble()

    def test_stale_source_and_receive_feedback_are_rejected(self) -> None:
        self.feed(source_ros_seconds=9.)
        with self.assertRaisesRegex(ValueError, 'stale'):
            self.collector.assemble()
        self.feed()
        self.clock.return_value = 1001.
        with self.assertRaisesRegex(ValueError, 'stale gripper feedback receive'):
            self.collector.assemble()
        self.owner.assert_not_called()

    def test_single_owner_serial_configuration_uses_exclusive_open(self) -> None:
        # Inspect the actual boundary without allowing a real serial connection.
        self.owner_patch.stop()
        from saps.physical.robotiq import RobotiqRtu
        serial = ModuleType('serial')
        serial.Serial = Mock()
        with patch.dict(sys.modules, {'serial': serial}):
            device = RobotiqRtu('/dev/serial/by-id/OFFLINE_FIXTURE')
            self.assertTrue(serial.Serial.call_args.kwargs['exclusive'])
            device.close()
            serial.Serial.return_value.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
