"""Standalone serial owner and diagnostics. Never creates arm interfaces."""
from __future__ import annotations

import argparse
import json
import math
import os
import uuid
from pathlib import Path
import threading
import time
from typing import Any, Callable

from saps.physical.robotiq import RobotiqRtu, calibrated_width


class ActivationError(RuntimeError):
    """Retain the activation trace when a write, feedback or deadline fails."""

    def __init__(self, report: dict[str, Any]) -> None:
        self.report = report
        super().__init__(report['error'])


def reset_activate(
    device: RobotiqRtu, *, lock: Any, timeout: float,
    now: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """One reset/activation attempt, with one shared bounded deadline.

    output() validates each FC16 acknowledgement. Feedback must confirm reset
    before the rising rACT edge; rGTO and rATR remain clear throughout.
    """
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('Activation timeout must be positive and finite')
    started = now()
    deadline = started + timeout
    report: dict[str, Any] = dict(
        mode='reset-activate', success=False, initial_state=None,
        final_state=None, transitions=[], error=None,
    )

    def event(phase: str, **fields: Any) -> None:
        report['transitions'].append(dict(
            phase=phase, elapsed_seconds=now() - started, **fields))

    def read(phase: str) -> Any:
        with lock:
            status = device.read_status()
        report['final_state'] = status.as_dict()
        event(phase, status=status.as_dict())
        return status

    def write(action: int, phase: str) -> None:
        if now() >= deadline:
            raise TimeoutError(f'Robotiq {phase} deadline exceeded')
        event(phase + '_requested', action=action)
        with lock:
            device.output(action=action)
        event(phase + '_acknowledged', action=action)

    def wait(phase: str) -> None:
        while now() < deadline:
            status = read(phase + '_feedback')
            # 0x05/0x07 are activation-related priority notices, not completed
            # activation. All other nonzero faults terminate this attempt.
            if status.fault not in (0, 5, 7):
                raise RuntimeError(f'Robotiq {phase} fault 0x{status.fault:02x}')
            done = (not status.activated and status.activation_state == 0
                    and not status.go_to) if phase == 'reset' else status.ready
            if done and now() < deadline:
                event(phase + '_confirmed')
                return
            sleep(min(.02, max(0., deadline - now())))
        raise TimeoutError(f'Robotiq {phase} feedback timeout')

    try:
        report['initial_state'] = read('initial_status').as_dict()
        write(0, 'reset')
        wait('reset')
        write(1, 'activation')
        wait('activation')
        report['success'] = True
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = f'{type(error).__name__}: {error}'
        event('failed', error=report['error'])
        raise ActivationError(report) from error
    return report


class ObservationError(RuntimeError):
    """Structured read-only settling/conversion failure with last feedback."""

    def __init__(self, report: dict[str, Any]) -> None:
        self.report = report
        super().__init__(report['error'])


def read_observation(
    device: RobotiqRtu, calibration: dict[str, Any], *, timeout: float,
    now: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """Settle feedback using reads only; never retry failed serial operations."""
    if not math.isfinite(timeout) or not 0 < timeout <= 5:
        raise ValueError('Feedback settling timeout must be in (0,5] seconds')
    started = now()
    deadline = started + timeout
    report: dict[str, Any] = dict(
        mode='observation', success=False, last_status=None, attempts=0,
        reason='feedback_not_ready', error=None,
    )
    try:
        while now() < deadline:
            report['attempts'] += 1
            report['reason'] = 'communication_error'
            status = device.read_status()
            report['last_status'] = status.as_dict()
            if now() >= deadline:
                report['reason'] = 'settling_timeout'
                raise TimeoutError('Robotiq feedback settling deadline exceeded')
            if status.ready:
                report['reason'] = 'invalid_calibration'
                width, closure = calibrated_width(status.actual_position, calibration)
                return dict(status.as_dict(), mode='observation', success=True,
                            attempts=report['attempts'], elapsed_seconds=now() - started,
                            estimated_width_m=width, policy_closure=closure,
                            calibration=calibration,
                            width_source='calibrated_motor_position_estimate')
            # Only watchdog 0x09 and normal activation-related notices are
            # allowed to settle. Other faults require operator inspection.
            if status.fault not in (0, 5, 7, 9):
                report['reason'] = 'gripper_fault'
                raise RuntimeError(f'Robotiq fault 0x{status.fault:02x}')
            report['reason'] = 'feedback_not_ready'
            sleep(min(.02, max(0., deadline - now())))
        raise TimeoutError('Robotiq feedback did not become ready while settling')
    except Exception as error:
        report.update(error=f'{type(error).__name__}: {error}',
                      elapsed_seconds=now() - started)
        raise ObservationError(report) from error


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--mode', choices=('status', 'observation', 'serve', 'activate', 'reset-activate', 'open', 'close', 'stop'),
                        default='status')
    parser.add_argument('--feedback-settle-seconds', type=float, default=1.,
                        help='Read-only observation settling bound in (0,5] seconds')
    parser.add_argument('--authorize-gripper', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.feedback_settle_seconds) or not 0 < args.feedback_settle_seconds <= 5:
        parser.error('--feedback-settle-seconds must be in (0,5]')
    config = json.loads(args.config.read_text())['gripper']
    for key in ('speed_register', 'force_register', 'open_command', 'close_command'):
        if type(config[key]) is not int or not 0 <= config[key] <= 255:
            parser.error(f'{key} must be a byte')
    if config['open_command'] >= config['close_command']:
        parser.error('Robotiq open target must be below close target')
    timeout = config['command_timeout_seconds']
    if not isinstance(timeout, (int, float)) or not 0 < timeout <= 30:
        parser.error('command_timeout_seconds must be in (0,30]')
    if args.mode in ('activate', 'reset-activate', 'open', 'close', 'stop') and not args.authorize_gripper:
        parser.error('Explicit --authorize-gripper is required for output writes')
    try:
        device = RobotiqRtu(config['port'], slave=config['slave'],
                            baudrate=config['baudrate'], timeout=config['serial_timeout_seconds'])
    except (OSError, ValueError) as error:
        if args.mode != 'observation':
            raise
        print(json.dumps(dict(
            mode='observation', success=False, last_status=None, attempts=0,
            reason='connection_error', error=f'{type(error).__name__}: {error}'),
            sort_keys=True))
        raise SystemExit(1) from error
    lock = threading.RLock()
    generation = 0
    latched_error = None
    motion_started = False
    busy = False
    failure_stop_attempted = False
    shutting_down = False
    server_instance = str(uuid.uuid4())

    def read() -> Any:
        with lock:
            return device.read_status()

    def command(kind: str) -> dict[str, Any]:
        nonlocal generation, motion_started, latched_error
        command_started = time.monotonic()
        with lock:
            if shutting_down and kind != 'stop':
                raise RuntimeError('Robotiq server shutting down')
            if latched_error and kind != 'stop':
                raise RuntimeError(latched_error)
            generation += 1
            owner = generation
            if kind == 'reset-activate':
                return reset_activate(device, lock=lock, timeout=timeout)
            status = None if kind == 'stop' else device.read_status()
            write_started = time.monotonic()
            if kind == 'stop':
                device.stop()
            elif kind == 'activate':
                if status.activated:
                    if status.ready:
                        return status.as_dict()
                    raise RuntimeError('Activation already in progress or faulted; inspect status')
                # Explicit activation may move fingers as part of device calibration.
                # Activation uses rGTO=0. stop() also writes rACT=1 and
                # cannot stop this routine; do not repeat it on failure/exit.
                device.output(action=1)
            else:
                if not status.ready:
                    raise RuntimeError('Robotiq not activated/ready')
                motion_started = True
                target = config['open_command' if kind == 'open' else 'close_command']
                device.output(action=9, position=target,
                              speed=config['speed_register'], force=config['force_register'])
            write_ack_seconds = time.monotonic() - write_started
        accepted = time.monotonic()
        deadline = accepted + timeout
        while time.monotonic() < deadline:
            with lock:
                if generation != owner:
                    return dict(cancelled=True, success=False)
                status = device.read_status()
            if status.fault:
                raise RuntimeError(f'Robotiq fault 0x{status.fault:02x}')
            done = ((kind == 'activate' and status.ready)
                    or (kind == 'stop' and not status.go_to)
                    or (kind in ('open', 'close') and status.go_to
                        and status.requested_position == target
                        and status.object_status in (1, 2, 3)))
            if done:
                return dict(status.as_dict(), success=True,
                            source_ros_seconds=(node.get_clock().now().nanoseconds / 1e9
                                                if node is not None else time.time()),
                            command_started_monotonic_seconds=command_started,
                            write_acknowledgement_seconds=write_ack_seconds,
                            command_completion_seconds=time.monotonic() - command_started,
                            accepted_monotonic_seconds=accepted,
                            completed_monotonic_seconds=time.monotonic())
            time.sleep(.02)
        raise TimeoutError(f'Robotiq {kind} completion timeout')

    def guarded_command(kind: str) -> dict[str, Any]:
        nonlocal latched_error, busy, failure_stop_attempted
        with lock:
            if kind != 'stop' and busy:
                raise RuntimeError('Robotiq busy; explicit stop and completion required before reversal')
            if kind != 'stop':
                busy = True
        try:
            return command(kind)
        except Exception as error:
            with lock:
                latched_error = f'{type(error).__name__}: {error}'
                # One bounded stop attempt; no releases and no motion retries.
                if motion_started and not failure_stop_attempted:
                    failure_stop_attempted = True
                    try:
                        device.stop()
                    except Exception as stop_error:
                        latched_error += f'; stop unconfirmed: {stop_error}'
            raise
        finally:
            if kind != 'stop':
                with lock:
                    busy = False

    node = executor = None
    ros_initialized = False
    try:
        if args.mode == 'status':
            print(json.dumps(read().as_dict(), sort_keys=True))
        elif args.mode == 'observation':
            try:
                record = read_observation(
                    device, config.get('calibration', {}),
                    timeout=args.feedback_settle_seconds)
            except ObservationError as error:
                print(json.dumps(error.report, sort_keys=True))
                raise SystemExit(1) from error
            print(json.dumps(record, sort_keys=True))
        elif args.mode != 'serve':
            try:
                record = guarded_command(args.mode)
            except ActivationError as error:
                print(json.dumps(error.report, sort_keys=True))
                raise SystemExit(1) from error
            print(json.dumps(record, sort_keys=True))
        else:
            import rclpy
            from rclpy.callback_groups import ReentrantCallbackGroup
            from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
            from std_msgs.msg import String
            from std_srvs.srv import Trigger
            rclpy.init(args=[])
            ros_initialized = True
            node = rclpy.create_node('saps_robotiq')
            group = ReentrantCallbackGroup()
            publisher = node.create_publisher(String, config['state_topic'], 10)

            def publish() -> None:
                nonlocal latched_error
                try:
                    with lock:
                        read_started = time.monotonic()
                        status = read().as_dict()
                        read_seconds = time.monotonic() - read_started
                        status.update(source_ros_seconds=node.get_clock().now().nanoseconds / 1e9,
                                      port=config['port'], slave=config['slave'],
                                      speed_register=config['speed_register'],
                                      force_register=config['force_register'],
                                      open_command=config['open_command'], close_command=config['close_command'],
                                      error=latched_error, serial_exclusive=True,
                                      serial_owner_pid=os.getpid(), server_instance=server_instance,
                                      commands_authorized=args.authorize_gripper,
                                      command_busy=busy, command_generation=generation,
                                      status_read_seconds=read_seconds)
                except Exception as error:
                    with lock:
                        latched_error = str(error)
                    status = dict(error=latched_error)
                publisher.publish(String(data=json.dumps(status)))

            node.create_timer(.05, publish)  # Dedicated mutually exclusive polling callback.
            if args.authorize_gripper:
                def callback(kind: str):
                    def call(request: Any, response: Any) -> Any:
                        try:
                            response.message = json.dumps(guarded_command(kind))
                            response.success = True
                        except Exception as error:
                            response.success, response.message = False, str(error)
                        return response
                    return call
                for kind in ('open', 'close', 'stop'):
                    node.create_service(Trigger, config['namespace'] + '/' + kind,
                                        callback(kind), callback_group=group)
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)
            try:
                executor.spin()
            except ExternalShutdownException:
                pass  # A signal handler or another owner already stopped ROS.
    except KeyboardInterrupt:
        pass  # Intentional Ctrl+C still executes cancellation and serial cleanup.
    finally:
        shutting_down = True
        try:
            # Cancel active command waits before draining the executor. Shutdown
            # never commands release; a read-only owner sends no output writes.
            with lock:
                generation += 1
                if motion_started and not failure_stop_attempted:
                    device.stop()
        finally:
            try:
                if executor is not None:
                    executor.shutdown(timeout_sec=timeout + 2)
            finally:
                try:
                    device.close()
                finally:
                    try:
                        if node is not None:
                            node.destroy_node()
                    finally:
                        if ros_initialized:
                            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
