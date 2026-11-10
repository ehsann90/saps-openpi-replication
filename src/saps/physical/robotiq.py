"""Isolated Robotiq 2F Modbus RTU protocol; no connection on import.

Register map: Robotiq 2F-85/140 instruction manual, section 4 (RS-485).
Position feedback is a motor register, not a direct width sensor.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import struct
from typing import Any


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return struct.pack('<H', crc)


@dataclass(frozen=True)
class RobotiqStatus:
    activated: bool
    go_to: bool
    activation_state: int
    object_status: int
    fault: int
    requested_position: int
    actual_position: int
    current_10ma: int

    @property
    def ready(self) -> bool:
        return self.activated and self.activation_state == 3 and self.fault == 0

    def as_dict(self) -> dict[str, Any]:
        return dict(asdict(self), ready=self.ready,
                    object_detected=self.object_status in (1, 2),
                    motion_complete=self.go_to and self.object_status != 0,
                    measurement_source='motor_position_register')


def decode_status(data: bytes) -> RobotiqStatus:
    if len(data) != 6:
        raise ValueError('Robotiq status must contain six register bytes')
    return RobotiqStatus(bool(data[0] & 1), bool(data[0] & 8),
                         (data[0] >> 4) & 3, (data[0] >> 6) & 3,
                         data[2] & 15, data[3], data[4], data[5])


def calibrated_width(position: int, calibration: dict[str, Any]) -> tuple[float, float]:
    """Explicit two-point estimate; refuse missing or unverified calibration."""
    if (calibration.get('verified') is not True
            or not calibration.get('evidence')):
        raise ValueError('Robotiq width calibration is pending physical validation')
    opened, closed = calibration['open_position'], calibration['closed_position']
    maximum = calibration['maximum_width_m']
    if (type(position) is not int or not 0 <= position <= 255
            or type(opened) is not int or type(closed) is not int
            or not 0 <= opened < closed <= 255
            or not isinstance(maximum, (int, float))
            or not math.isfinite(maximum) or not 0 < maximum <= .085):
        raise ValueError('Invalid measured Robotiq calibration or feedback')
    closure = min(1., max(0., (position - opened) / (closed - opened)))
    return maximum * (1. - closure), closure


class RobotiqRtu:
    """One bounded transaction, no automatic retry or activation.

    Caller serializes access and owns lifecycle. Opening the adapter can toggle
    adapter control lines; it does not send a gripper output-register write.
    """

    def __init__(self, port: str, *, slave: int = 9, baudrate: int = 115200,
                 timeout: float = .2) -> None:
        if not port.startswith('/dev/serial/by-id/'):
            raise ValueError('Use an inspected /dev/serial/by-id/ device identity')
        if type(slave) is not int or not 1 <= slave <= 247:
            raise ValueError('Invalid Modbus slave address')
        if not math.isfinite(timeout) or not 0 < timeout <= 1:
            raise ValueError('Serial timeout must be finite in (0,1] seconds')
        import serial
        self.slave = slave
        self.serial = serial.Serial(port, baudrate=baudrate, bytesize=8,
                                    parity='N', stopbits=1, timeout=timeout,
                                    write_timeout=timeout, exclusive=True)

    def _exchange(self, payload: bytes, length: int) -> bytes:
        request = bytes([self.slave]) + payload
        self.serial.reset_input_buffer()
        wire = request + crc16(request)
        if self.serial.write(wire) != len(wire):
            raise OSError('Incomplete Modbus write')
        header = self.serial.read(3)
        if len(header) != 3:
            raise TimeoutError('Modbus response header timeout')
        size = 5 if header[1] & 0x80 else length
        response = header + self.serial.read(size - 3)
        if len(response) != size:
            raise TimeoutError('Modbus response timeout')
        if response[-2:] != crc16(response[:-2]):
            raise OSError('Modbus CRC mismatch')
        if response[0] != self.slave:
            raise OSError('Unexpected Modbus device address')
        if response[1] & 0x80:
            raise OSError(f'Modbus exception {response[2]}')
        if response[1] != payload[0]:
            raise OSError('Unexpected Modbus function')
        return response

    def read_status(self) -> RobotiqStatus:
        response = self._exchange(bytes.fromhex('03 07 D0 00 03'), 11)
        if response[2] != 6:
            raise OSError('Invalid Modbus status byte count')
        return decode_status(response[3:9])

    def output(self, *, action: int, position: int = 0,
               speed: int = 0, force: int = 0) -> None:
        if any(type(v) is not int or not 0 <= v <= 255
               for v in (action, position, speed, force)):
            raise ValueError('Output registers must be bytes')
        payload = bytes.fromhex('10 03 E8 00 03 06') + bytes(
            [action, 0, 0, position, speed, force])
        response = self._exchange(payload, 8)
        if response[2:6] != bytes.fromhex('03 E8 00 03'):
            raise OSError('Invalid Modbus write acknowledgement')

    def stop(self) -> None:
        # rACT remains set, rGTO cleared. Never use emergency auto-release rATR.
        self.output(action=1)

    def close(self) -> None:
        self.serial.close()
