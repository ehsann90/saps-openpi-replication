"""Hardware-independent register, feedback and asynchronous command contracts."""
from concurrent.futures import Future
import json
from types import SimpleNamespace
import unittest

import numpy as np

from saps.physical.droid_gripper import DroidGripper, droid_gripper_decision
from saps.physical.live_observation import decode_ros_rgb_image
from franka_robotiq.robotiq import calibrated_width, crc16, decode_status, RobotiqRtu
from saps.physical.robotiq_hand import RobotiqHand
from saps.physical.hardware_provenance import require_supported_arm_geometry


class Transport:
    def __init__(self):
        self.calls = []

    def call(self, kind):
        future = Future()
        self.calls.append((kind, future))
        return future

    def ready(self):
        return True


def respond(future, **status):
    future.set_result(SimpleNamespace(success=True, message=json.dumps(status)))


class RobotiqTests(unittest.TestCase):
    def setUp(self):
        self.calibration = dict(verified=True, evidence='fixture measured endpoints',
                                open_position=10, closed_position=230,
                                maximum_width_m=.08)

    def test_crc_matches_manufacturer_read_request(self):
        self.assertEqual(crc16(bytes.fromhex('09 03 07 D0 00 03')), bytes.fromhex('04 0E'))

    def test_modbus_errors_do_not_retry(self):
        class SerialFixture:
            def __init__(self, response):
                self.response = bytearray(response)
                self.writes = []
            def reset_input_buffer(self):
                pass
            def write(self, data):
                self.writes.append(data)
                return len(data)
            def read(self, count):
                data = bytes(self.response[:count])
                del self.response[:count]
                return data
        exception = bytes.fromhex('09 83 02')
        for response, error in [(b'', TimeoutError),
                                (exception + crc16(exception), OSError),
                                (bytes.fromhex('09 03 06 F9 00 00 FF E6 04 00 00'), OSError)]:
            device = object.__new__(RobotiqRtu)
            device.slave = 9
            device.serial = SerialFixture(response)
            with self.assertRaises(error):
                device.read_status()
            self.assertEqual(len(device.serial.writes), 1)

    def test_status_distinguishes_contact_from_position_reached(self):
        contact = decode_status(bytes([0xB9, 0, 0, 255, 90, 7]))
        reached = decode_status(bytes([0xF9, 0, 0, 255, 230, 4]))
        self.assertTrue(contact.ready)
        self.assertTrue(contact.as_dict()['object_detected'])
        self.assertFalse(reached.as_dict()['object_detected'])
        self.assertEqual(contact.actual_position, 90)

    def test_calibrated_measured_feedback_and_missing_calibration(self):
        self.assertEqual(calibrated_width(10, self.calibration), (.08, 0.))
        self.assertEqual(calibrated_width(230, self.calibration), (0., 1.))
        width, closure = calibrated_width(120, self.calibration)
        self.assertAlmostEqual(width, .04)
        self.assertEqual(closure, .5)
        with self.assertRaises(ValueError):
            calibrated_width(100, dict(self.calibration, verified=False))
        with self.assertRaises(ValueError):
            calibrated_width(100, dict(self.calibration, closed_position=10))

    def test_rgba_and_bgra_with_padded_rows(self):
        for encoding, pixel in [('rgba8', [1, 2, 3, 99]), ('bgra8', [3, 2, 1, 99])]:
            image = decode_ros_rgb_image(height=2, width=1, encoding=encoding,
                                         step=6, data=bytes(pixel + [0, 0]) * 2)
            np.testing.assert_array_equal(image, np.array([[[1, 2, 3]], [[1, 2, 3]]], dtype=np.uint8))
            self.assertTrue(image.flags.c_contiguous)

    def test_binary_boundary_and_no_initial_commands(self):
        transport = Transport()
        hand = RobotiqHand(transport, measured=lambda: dict(closure=.4), timeout=3.)
        hand.tick()
        self.assertEqual(transport.calls, [])
        gripper = DroidGripper(hand, maximum_width_m=.08, measured=lambda: dict(closure=.4))
        self.assertEqual(droid_gripper_decision(.5).binary_closure, 0.)
        gripper.command(.5)
        hand.tick()
        self.assertEqual(transport.calls[0][0], 'open')
        self.assertEqual(gripper.command(.2)['disposition'], 'duplicate')
        gripper.command(.50001)
        hand.tick()
        self.assertEqual([x[0] for x in transport.calls], ['open', 'stop'])
        respond(transport.calls[1][1], go_to=False)
        hand.tick()
        self.assertEqual(len(transport.calls), 2)  # Wait for old command result too.
        respond(transport.calls[0][1], cancelled=True)
        hand.tick()
        self.assertEqual(transport.calls[-1][0], 'close')

    def test_feedback_failure_stops_once_and_latches(self):
        transport = Transport()
        hand = RobotiqHand(transport, measured=lambda: {}, timeout=3.)
        hand.request_binary(True)
        hand.tick()
        def stale():
            raise RuntimeError('stale_gripper_state')
        hand.measured = stale
        hand.tick()
        hand.tick()
        self.assertEqual([x[0] for x in transport.calls], ['close', 'stop'])
        with self.assertRaises(RuntimeError):
            hand.check()

    def test_invalid_serial_device_rejected_before_connection(self):
        with self.assertRaises(ValueError):
            RobotiqRtu('/dev/ttyUSB0')

    def test_geometry_cannot_be_approved_by_config_flag(self):
        with self.assertRaises(ValueError):
            require_supported_arm_geometry(dict(hardware_profile='zed_robotiq_2f85',
                                                geometry={'execution_ready': True}))
        require_supported_arm_geometry({})


class C4LifecycleRegressionTests(unittest.TestCase):
    def hand(self):
        self.transport = Transport()
        return RobotiqHand(self.transport, measured=lambda: {'closure': .4}, timeout=3.)

    def test_close_contact_then_open_and_duplicates(self):
        hand = self.hand()
        hand.request_binary(True)
        self.assertEqual(hand.request_binary(True)['disposition'], 'duplicate')
        hand.tick()
        respond(self.transport.calls[0][1], success=True, object_status=2,
                actual_position=102, requested_position=255)
        hand.tick()
        self.assertEqual(hand.evidence()['requests'][0]['measured_after']['actual_position'],102)
        self.assertEqual(hand.request_binary(True)['disposition'],'duplicate')
        self.assertEqual(len(self.transport.calls),1)
        hand.request_binary(False)
        hand.tick()
        self.assertEqual(self.transport.calls[-1][0],'open')
        respond(self.transport.calls[-1][1], success=True, object_status=3, actual_position=3)
        hand.tick()
        self.assertEqual(hand.request_binary(False)['disposition'],'duplicate')
        self.assertNotIn('grasp_success',hand.evidence())

    def test_close_to_open_reversal_waits_for_both_results(self):
        hand = self.hand()
        hand.request_binary(True)
        hand.tick()
        hand.request_binary(False)
        hand.tick()
        self.assertEqual([kind for kind,future in self.transport.calls],['close','stop'])
        respond(self.transport.calls[0][1], cancelled=True)
        hand.tick()
        self.assertEqual(len(self.transport.calls),2)
        respond(self.transport.calls[1][1], go_to=False)
        hand.tick()
        self.assertEqual(self.transport.calls[-1][0],'open')

    def test_external_cancellation_is_a_failure_not_a_suppressed_command(self):
        hand = self.hand()
        hand.request_binary(True)
        hand.tick()
        respond(self.transport.calls[0][1], cancelled=True)
        hand.tick()
        self.assertEqual(hand.evidence()['requests'][0]['disposition'],'failed')
        self.assertEqual(self.transport.calls[-1][0],'stop')
        with self.assertRaisesRegex(RuntimeError,'cancelled_outside'):
            hand.request_binary(True)

    def test_rejection_or_transport_timeout_stops_once(self):
        for error in (TimeoutError('serial timeout'), RuntimeError('fault during motion')):
            hand = self.hand()
            hand.request_binary(True)
            hand.tick()
            self.transport.calls[0][1].set_exception(error)
            hand.tick()
            hand.tick()
            self.assertEqual([kind for kind,future in self.transport.calls],['close','stop'])
            with self.assertRaises(RuntimeError):
                hand.check()

    def test_explicit_stop_and_shutdown_do_not_open(self):
        hand = self.hand()
        hand.request_binary(True)
        hand.tick()
        hand.stop()
        hand.tick()
        respond(self.transport.calls[1][1], go_to=False)
        respond(self.transport.calls[0][1], cancelled=True)
        hand.tick()
        self.assertTrue(hand.settled)
        self.assertEqual([kind for kind,future in self.transport.calls],['close','stop'])


if __name__ == '__main__':
    unittest.main()
