"""Offline activation recovery and CLI cleanup; no serial or ROS connection."""
from __future__ import annotations
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from franka_robotiq.robotiq import RobotiqStatus
from franka_robotiq.robotiq_node import ActivationError, main, reset_activate


def status(*, activated: bool = True, state: int = 0,
           fault: int = 0) -> RobotiqStatus:
    return RobotiqStatus(activated, False, state, 0, fault, 0, 42, 0)


class Clock:
    def __init__(self) -> None:
        self.seconds = 0.

    def now(self) -> float:
        return self.seconds

    def sleep(self, seconds: float) -> None:
        self.seconds += seconds


class Device:
    def __init__(self, states: list, *, fail_action: int | None = None) -> None:
        self.states = iter(states)
        self.last = None
        self.fail_action = fail_action
        self.actions = []
        self.calls = []
        self.closed = False

    def read_status(self) -> RobotiqStatus:
        self.last = next(self.states, self.last)
        if isinstance(self.last, Exception):
            raise self.last
        self.calls.append(('read', self.last))
        return self.last

    def output(self, *, action: int) -> None:
        self.actions.append(action)
        self.calls.append(('write', action))
        if action == self.fail_action:
            raise OSError('FC16 acknowledgement failed')

    def stop(self) -> None:
        # Mirrors the real stop method: an unintended stop would be another 1.
        self.output(action=1)

    def close(self) -> None:
        self.closed = True


class ActivationTests(unittest.TestCase):
    def run_sequence(self, device: Device) -> dict:
        clock = Clock()
        return reset_activate(device, lock=threading.RLock(), timeout=.1,
                              now=clock.now, sleep=clock.sleep)

    def test_successful_recovery_records_initial_reset_and_final_state(self) -> None:
        initial = status(fault=9)
        reset = status(activated=False)
        device = Device([initial, status(), reset,
                         status(state=1, fault=5), status(state=3)])
        report = self.run_sequence(device)
        self.assertTrue(report['success'])
        self.assertEqual(report['initial_state'], initial.as_dict())
        self.assertTrue(report['final_state']['ready'])
        self.assertEqual(device.actions, [0, 1])
        # The rising edge follows observed reset, not merely a write ACK.
        self.assertEqual(device.calls[3], ('read', reset))
        self.assertEqual(device.calls[4], ('write', 1))
        phases = [row['phase'] for row in report['transitions']]
        self.assertIn('reset_acknowledged', phases)
        self.assertIn('reset_confirmed', phases)
        self.assertEqual(phases[-1], 'activation_confirmed')

    def test_reset_write_ack_failure_never_sets_activation(self) -> None:
        device = Device([status()], fail_action=0)
        with self.assertRaises(ActivationError) as caught:
            self.run_sequence(device)
        self.assertEqual(device.actions, [0])
        self.assertIn('acknowledgement failed', caught.exception.report['error'])
        self.assertFalse(caught.exception.report['success'])

    def test_reset_feedback_timeout_never_sets_activation(self) -> None:
        device = Device([status()])  # gACT never clears despite the FC16 ACK.
        with self.assertRaises(ActivationError) as caught:
            self.run_sequence(device)
        self.assertEqual(device.actions, [0])
        self.assertIn('reset feedback timeout', caught.exception.report['error'])

    def test_activation_timeout_does_not_retry(self) -> None:
        device = Device([status(), status(activated=False), status(state=1)])
        with self.assertRaises(ActivationError) as caught:
            self.run_sequence(device)
        self.assertEqual(device.actions, [0, 1])
        self.assertIn('activation feedback timeout', caught.exception.report['error'])
        self.assertEqual(caught.exception.report['final_state']['activation_state'], 1)

    def test_faults_abort_and_keep_last_feedback(self) -> None:
        for fault in (8, 9, 12, 13, 15):
            with self.subTest(fault=fault):
                device = Device([status(), status(activated=False),
                                 status(state=1, fault=fault)])
                with self.assertRaises(ActivationError) as caught:
                    self.run_sequence(device)
                self.assertEqual(device.actions, [0, 1])
                self.assertEqual(caught.exception.report['final_state']['fault'], fault)
                self.assertIn(f'fault 0x{fault:02x}', caught.exception.report['error'])

    def test_reset_fault_does_not_issue_activation(self) -> None:
        device = Device([status(fault=9), status(activated=False, fault=12)])
        with self.assertRaises(ActivationError):
            self.run_sequence(device)
        self.assertEqual(device.actions, [0])

    def test_communication_failure_preserves_trace(self) -> None:
        device = Device([status(), status(activated=False),
                         TimeoutError('Modbus response timeout')])
        with self.assertRaises(ActivationError) as caught:
            self.run_sequence(device)
        self.assertEqual(device.actions, [0, 1])
        self.assertIn('Modbus response timeout', caught.exception.report['error'])
        self.assertFalse(caught.exception.report['final_state']['activated'])

    def test_priority_notices_are_not_ready(self) -> None:
        device = Device([status(), status(activated=False, fault=7),
                         status(state=3, fault=5)])
        with self.assertRaises(ActivationError) as caught:
            self.run_sequence(device)
        self.assertIn('activation feedback timeout', caught.exception.report['error'])

    def test_one_deadline_covers_reset_and_activation(self) -> None:
        clock = Clock()
        class SlowReset(Device):
            def read_status(self) -> RobotiqStatus:
                value = super().read_status()
                clock.seconds += .04
                return value
        device = SlowReset([status(), status(activated=False), status(state=1)])
        with self.assertRaises(ActivationError):
            reset_activate(device, lock=threading.RLock(), timeout=.1,
                           now=clock.now, sleep=clock.sleep)
        self.assertLess(clock.seconds, .16)
        self.assertEqual(device.actions, [0, 1])


class ActivationCliTests(unittest.TestCase):
    def invoke(self, device: Device, *, authorize: bool = True,
               mode: str = 'reset-activate') -> dict:
        config = dict(gripper=dict(
            speed_register=64, force_register=32, open_command=0,
            close_command=255, command_timeout_seconds=.1,
            port='/dev/serial/by-id/OFFLINE_FIXTURE', slave=9,
            baudrate=115200, serial_timeout_seconds=.2))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps(config))
            argv = ['robotiq_node', '--config', str(path), '--mode', mode]
            if authorize:
                argv.append('--authorize-gripper')
            output = io.StringIO()
            clock = Clock()
            def activation(device: Device, **kwargs: object) -> dict:
                return reset_activate(device, **kwargs,
                                      now=clock.now, sleep=clock.sleep)
            with patch('sys.argv', argv), patch(
                    'franka_robotiq.robotiq_node.RobotiqRtu', return_value=device
            ) as constructor, patch(
                    'franka_robotiq.robotiq_node.reset_activate', side_effect=activation
            ), patch('franka_robotiq.robotiq_node.time.monotonic', clock.now), patch(
                    'franka_robotiq.robotiq_node.time.sleep', clock.sleep
            ), redirect_stdout(output):
                if not authorize:
                    with self.assertRaises(SystemExit) as caught:
                        main()
                    self.assertEqual(caught.exception.code, 2)
                    constructor.assert_not_called()
                    return {}
                try:
                    main()
                except SystemExit as error:
                    self.assertEqual(error.code, 1)
            return json.loads(output.getvalue())

    def test_explicit_authorization_required_before_connection(self) -> None:
        self.invoke(Device([]), authorize=False)

    def test_successful_cli_exit_does_not_write_stop_or_reset(self) -> None:
        device = Device([status(), status(activated=False), status(state=3)])
        self.assertTrue(self.invoke(device)['success'])
        self.assertEqual(device.actions, [0, 1])
        self.assertTrue(device.closed)

    def test_failed_cli_exit_does_not_reactivate(self) -> None:
        for device in [Device([status()], fail_action=0),
                       Device([status(), status(activated=False), status(state=1)]),
                       Device([status(), status(activated=False), status(fault=12)])]:
            with self.subTest(device=device):
                report = self.invoke(device)
                self.assertFalse(report['success'])
                self.assertTrue(report['error'])
                self.assertIn(device.actions, ([0], [0, 1]))
                self.assertTrue(device.closed)

    def test_existing_activate_exit_does_not_repeat_activation(self) -> None:
        device = Device([status(activated=False), status(state=3)])
        self.assertTrue(self.invoke(device, mode='activate')['success'])
        self.assertEqual(device.actions, [1])
        self.assertTrue(device.closed)


if __name__ == '__main__':
    unittest.main()
