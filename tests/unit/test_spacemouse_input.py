"""Tests for native SpaceMouse discovery, processing, and safety."""

from __future__ import annotations

import errno
import struct
import unittest

import numpy as np

from saps.human_input.spacemouse import ABS_RX
from saps.human_input.spacemouse import ABS_RY
from saps.human_input.spacemouse import ABS_RZ
from saps.human_input.spacemouse import ABS_X
from saps.human_input.spacemouse import ABS_Y
from saps.human_input.spacemouse import ABS_Z
from saps.human_input.spacemouse import DEFAULT_DEVICE_GLOB
from saps.human_input.spacemouse import DeviceInfo
from saps.human_input.spacemouse import EV_ABS
from saps.human_input.spacemouse import EV_KEY
from saps.human_input.spacemouse import FALLBACK_DEVICE_GLOB
from saps.human_input.spacemouse import process_spacemouse_axes
from saps.human_input.spacemouse import select_spacemouse_device
from saps.human_input.spacemouse import SpaceMouseBackend
from saps.human_input.spacemouse import SpaceMouseConfig
from saps.human_input.spacemouse import SpaceMouseState
from saps.human_input.spacemouse import SpaceMouseUnavailableError
from saps.human_input.web_operator import BrowserOperatorServer


REQUIRED_AXES = {ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ}
INPUT_EVENT = struct.Struct("llHHi")


class FakeLinuxBoundary:
    """Deterministic fake for the native Linux input boundary."""

    def __init__(self) -> None:
        self.paths: dict[str, set[int]] = {}
        self.names: dict[str, str] = {}
        self.glob_results: dict[str, list[str]] = {}
        self.fd_paths: dict[int, str] = {}
        self.next_fd = 10
        self.opened_paths: list[str] = []
        self.closed_fds: list[int] = []
        self.grab_calls: list[tuple[int, bool]] = []
        self.read_results: list[bytes | BaseException] = []
        self.now = 10.0
        self.fail_grab = False

    def add_device(
        self,
        path: str,
        *,
        axes: set[int],
        name: str = "test device",
    ) -> None:
        self.paths[path] = set(axes)
        self.names[path] = name

    def glob(self, pattern: str) -> list[str]:
        return list(self.glob_results.get(pattern, []))

    def open(self, path: str) -> int:
        if path not in self.paths:
            raise FileNotFoundError(errno.ENOENT, "missing", path)
        file_descriptor = self.next_fd
        self.next_fd += 1
        self.fd_paths[file_descriptor] = path
        self.opened_paths.append(path)
        return file_descriptor

    def close(self, file_descriptor: int) -> None:
        self.closed_fds.append(file_descriptor)
        self.fd_paths.pop(file_descriptor, None)

    def read(self, file_descriptor: int, size: int) -> bytes:
        del file_descriptor, size
        if not self.read_results:
            raise BlockingIOError(errno.EAGAIN, "empty")
        result = self.read_results.pop(0)
        if isinstance(result, BaseException):
            raise result
        return result

    def grab(self, file_descriptor: int, enabled: bool) -> None:
        self.grab_calls.append((file_descriptor, enabled))
        if enabled and self.fail_grab:
            raise OSError(errno.EBUSY, "busy")

    def device_name(self, file_descriptor: int) -> str:
        return self.names[self.fd_paths[file_descriptor]]

    def axis_codes(self, file_descriptor: int) -> frozenset[int]:
        return frozenset(self.paths[self.fd_paths[file_descriptor]])

    def axis_range(
        self,
        file_descriptor: int,
        axis_code: int,
    ) -> tuple[int, int]:
        del file_descriptor, axis_code
        return -350, 350

    def realpath(self, path: str) -> str:
        return path.replace("by-id/device", "event9")

    def monotonic(self) -> float:
        return self.now


def event(
    event_type: int,
    code: int,
    value: int,
    *,
    seconds: int = 100,
    microseconds: int = 250000,
) -> bytes:
    return INPUT_EVENT.pack(
        seconds,
        microseconds,
        event_type,
        code,
        value,
    )


def state(
    *,
    connected: bool = True,
    motion: tuple[float, ...] = (0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
    stale: bool = False,
    open_pressed: bool = False,
    close_pressed: bool = False,
) -> SpaceMouseState:
    return SpaceMouseState(
        connected=connected,
        device_name="3Dconnexion SpaceMouse Wireless",
        device_path="/dev/input/by-id/device",
        raw_axes=(350, 0, 0, 0, 0, 0),
        mapped_axes=(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        final_motion=motion,
        motion_active=any(motion),
        stale=stale,
        open_button_pressed=open_pressed,
        close_button_pressed=close_pressed,
        native_event_timestamp_seconds=100.25,
        last_event_monotonic_seconds=10.0,
        error=None,
    )


class FakeSpaceMouseBackend:
    def __init__(self, samples: list[SpaceMouseState]) -> None:
        self.config = SpaceMouseConfig()
        self.samples = samples
        self.started = False
        self.closed = False

    def start(self) -> DeviceInfo:
        self.started = True
        return DeviceInfo(
            path="/dev/input/by-id/device",
            name="3Dconnexion SpaceMouse Wireless",
            axis_codes=frozenset(REQUIRED_AXES),
            axis_ranges=((-350, 350),) * 6,
        )

    def sample(self) -> SpaceMouseState:
        if len(self.samples) > 1:
            return self.samples.pop(0)
        return self.samples[0]

    def close(self) -> None:
        self.closed = True


class SpaceMouseProcessingTest(unittest.TestCase):
    def test_normalization_clipping_and_zero(self) -> None:
        config = SpaceMouseConfig(
            translation_gain=1.0,
            rotation_gain=1.0,
            deadzone=0.0,
        )
        processed = process_spacemouse_axes(
            [175, -350, 700, 0, 0, 0],
            config,
        )
        np.testing.assert_allclose(
            processed.final_motion,
            [0.5, -1.0, 1.0, 0.0, 0.0, 0.0],
        )
        np.testing.assert_allclose(
            process_spacemouse_axes([0] * 6, config).final_motion,
            np.zeros(6),
        )

    def test_rescaled_deadzone(self) -> None:
        config = SpaceMouseConfig(
            translation_gain=1.0,
            rotation_gain=1.0,
            deadzone=0.2,
        )
        processed = process_spacemouse_axes(
            [70, 210, -210, 0, 0, 0],
            config,
        )
        np.testing.assert_allclose(
            processed.final_motion[:3],
            [0.0, 0.5, -0.5],
            atol=1e-7,
        )

    def test_mapping_signs_and_gains(self) -> None:
        config = SpaceMouseConfig(
            translation_gain=0.5,
            rotation_gain=0.25,
            deadzone=0.0,
            axis_mapping=(
                "ABS_Z",
                "ABS_Y",
                "ABS_X",
                "ABS_RZ",
                "ABS_RY",
                "ABS_RX",
            ),
            axis_signs=(-1.0, 1.0, 1.0, -1.0, 1.0, 1.0),
        )
        processed = process_spacemouse_axes(
            [350, 175, -350, 350, 175, -350],
            config,
        )
        np.testing.assert_allclose(
            processed.mapped_axes,
            [1.0, 0.5, 1.0, 1.0, 0.5, 1.0],
        )
        np.testing.assert_allclose(
            processed.final_motion,
            [0.5, 0.25, 0.5, 0.25, 0.125, 0.25],
        )

    def test_axis_scales_and_disabled_axes(self) -> None:
        config = SpaceMouseConfig(
            translation_gain=1.0,
            rotation_gain=1.0,
            deadzone=0.0,
            axis_scales=(2.0, 0.5, 1.0, 1.5, 1.0, 1.0),
            axis_enabled=(True, True, False, True, False, True),
        )
        processed = process_spacemouse_axes(
            [175, 175, 350, 175, 350, 350],
            config,
        )
        np.testing.assert_allclose(
            processed.final_motion,
            [1.0, 0.25, 0.0, 0.75, 0.0, 1.0],
        )


class SpaceMouseDiscoveryTest(unittest.TestCase):
    def test_explicit_path_is_selected_first(self) -> None:
        boundary = FakeLinuxBoundary()
        boundary.add_device("/custom/event", axes=REQUIRED_AXES)
        boundary.add_device("/dev/input/event1", axes=REQUIRED_AXES)
        boundary.glob_results[FALLBACK_DEVICE_GLOB] = [
            "/dev/input/event1"
        ]
        selected = select_spacemouse_device(
            explicit_path="/custom/event",
            boundary=boundary,
        )
        self.assertEqual(selected.path, "/custom/event")
        self.assertEqual(boundary.opened_paths, ["/custom/event"])

    def test_by_id_precedes_capability_fallback(self) -> None:
        boundary = FakeLinuxBoundary()
        boundary.add_device(
            "/dev/input/by-id/incomplete-event-joystick",
            axes={ABS_X},
        )
        boundary.add_device(
            "/dev/input/by-id/device-event-joystick",
            axes=REQUIRED_AXES,
        )
        boundary.add_device("/dev/input/event4", axes=REQUIRED_AXES)
        boundary.glob_results[DEFAULT_DEVICE_GLOB] = [
            "/dev/input/by-id/incomplete-event-joystick",
            "/dev/input/by-id/device-event-joystick",
        ]
        boundary.glob_results[FALLBACK_DEVICE_GLOB] = [
            "/dev/input/event4"
        ]
        selected = select_spacemouse_device(boundary=boundary)
        self.assertEqual(
            selected.path,
            "/dev/input/by-id/device-event-joystick",
        )
        self.assertNotIn("/dev/input/event4", boundary.opened_paths)

    def test_node_without_all_six_axes_is_rejected(self) -> None:
        boundary = FakeLinuxBoundary()
        boundary.add_device(
            "/custom/event",
            axes=REQUIRED_AXES - {ABS_RZ},
            name="3Dconnexion SpaceMouse Wireless",
        )
        with self.assertRaisesRegex(
            SpaceMouseUnavailableError,
            "missing required axes",
        ):
            select_spacemouse_device(
                explicit_path="/custom/event",
                boundary=boundary,
            )


class SpaceMouseBackendTest(unittest.TestCase):
    def make_backend(
        self,
        *,
        config: SpaceMouseConfig | None = None,
    ) -> tuple[SpaceMouseBackend, FakeLinuxBoundary]:
        boundary = FakeLinuxBoundary()
        boundary.add_device(
            "/dev/input/by-id/device",
            axes=REQUIRED_AXES,
            name="3Dconnexion SpaceMouse Wireless",
        )
        backend = SpaceMouseBackend(
            config
            or SpaceMouseConfig(
                device_path="/dev/input/by-id/device",
                translation_gain=1.0,
                rotation_gain=1.0,
                deadzone=0.0,
            ),
            boundary=boundary,
        )
        return backend, boundary

    def test_exclusive_access_success_and_cleanup(self) -> None:
        backend, boundary = self.make_backend()
        backend.start()
        acquisition_fd = boundary.grab_calls[0][0]
        self.assertEqual(boundary.grab_calls, [(acquisition_fd, True)])
        backend.close()
        self.assertEqual(
            boundary.grab_calls,
            [(acquisition_fd, True), (acquisition_fd, False)],
        )
        self.assertIn(acquisition_fd, boundary.closed_fds)

    def test_exclusive_access_failure_is_actionable(self) -> None:
        backend, boundary = self.make_backend()
        boundary.fail_grab = True
        with self.assertRaisesRegex(
            SpaceMouseUnavailableError,
            "spacenavd.*sudo fuser -v",
        ):
            backend.start()
        self.assertFalse(backend.connected)

    def test_stale_input_zeros_motion(self) -> None:
        backend, boundary = self.make_backend()
        backend.start()
        boundary.read_results.append(event(EV_ABS, ABS_X, 350))
        active = backend.sample()
        self.assertEqual(active.final_motion[0], 1.0)
        self.assertFalse(active.stale)

        boundary.now += 0.3
        stale = backend.sample()
        np.testing.assert_allclose(stale.final_motion, np.zeros(6))
        self.assertTrue(stale.stale)

    def test_disconnect_zeros_motion_and_buttons(self) -> None:
        backend, boundary = self.make_backend()
        backend.start()
        boundary.read_results.extend(
            [
                event(EV_ABS, ABS_X, 350)
                + event(EV_KEY, 257, 1),
                OSError(errno.ENODEV, "unplugged"),
            ]
        )
        disconnected = backend.sample()
        self.assertFalse(disconnected.connected)
        np.testing.assert_allclose(
            disconnected.final_motion,
            np.zeros(6),
        )
        self.assertFalse(disconnected.close_button_pressed)

    def test_configurable_buttons(self) -> None:
        config = SpaceMouseConfig(
            device_path="/dev/input/by-id/device",
            open_button=300,
            close_button=301,
        )
        backend, boundary = self.make_backend(config=config)
        backend.start()
        boundary.read_results.append(
            event(EV_KEY, 300, 1) + event(EV_KEY, 301, 1)
        )
        sample = backend.sample()
        self.assertTrue(sample.open_button_pressed)
        self.assertTrue(sample.close_button_pressed)


class ControllerNeutralSampleTest(unittest.TestCase):
    def test_spacemouse_motion_becomes_seven_dimensional_action(self) -> None:
        backend = FakeSpaceMouseBackend([state()])
        operator = BrowserOperatorServer(
            input_source="spacemouse",
            spacemouse_backend=backend,
        )
        operator._connected_clients = 1
        operator._armed = True
        operator._physical_device_was_connected = True
        sample_value = operator.sample()
        np.testing.assert_allclose(
            sample_value.action,
            [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
        )
        self.assertEqual(sample_value.action.shape, (7,))
        self.assertEqual(sample_value.input_source, "spacemouse")
        self.assertTrue(sample_value.physical_device_connected)

    def test_disconnect_requires_rearm_and_clears_motion(self) -> None:
        backend = FakeSpaceMouseBackend(
            [state(connected=False, motion=(0.0,) * 6, stale=True)]
        )
        operator = BrowserOperatorServer(
            input_source="spacemouse",
            spacemouse_backend=backend,
        )
        operator._connected_clients = 1
        operator._armed = True
        operator._physical_device_was_connected = True
        sample_value = operator.sample()
        self.assertFalse(sample_value.armed)
        np.testing.assert_allclose(sample_value.action[:6], np.zeros(6))
        self.assertTrue(sample_value.connected)
        self.assertFalse(sample_value.physical_device_connected)

    def test_reconnect_does_not_restore_authority(self) -> None:
        backend = FakeSpaceMouseBackend(
            [
                state(connected=False, motion=(0.0,) * 6, stale=True),
                state(),
            ]
        )
        operator = BrowserOperatorServer(
            input_source="spacemouse",
            spacemouse_backend=backend,
        )
        operator._connected_clients = 1
        operator._armed = True
        operator._physical_device_was_connected = True
        self.assertFalse(operator.sample().armed)
        reconnected = operator.sample()
        self.assertTrue(reconnected.physical_device_connected)
        self.assertFalse(reconnected.armed)
        np.testing.assert_allclose(reconnected.action[:6], np.zeros(6))

    def test_space_button_close_has_deterministic_priority(self) -> None:
        backend = FakeSpaceMouseBackend(
            [state(open_pressed=True, close_pressed=True)]
        )
        operator = BrowserOperatorServer(
            input_source="spacemouse",
            spacemouse_backend=backend,
        )
        operator._connected_clients = 1
        operator._armed = True
        operator._physical_device_was_connected = True
        self.assertEqual(float(operator.sample().action[6]), 1.0)

    def test_keyboard_path_remains_compatible(self) -> None:
        operator = BrowserOperatorServer(input_source="keyboard")
        operator._connected_clients = 1
        operator._armed = True
        operator._pressed_keys = {"w"}
        sample_value = operator.sample()
        self.assertEqual(sample_value.action.shape, (7,))
        self.assertEqual(sample_value.input_source, "keyboard")
        self.assertAlmostEqual(float(sample_value.action[0]), 0.07)


if __name__ == "__main__":
    unittest.main()
