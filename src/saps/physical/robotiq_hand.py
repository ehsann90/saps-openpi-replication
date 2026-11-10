"""Nonblocking ROS lifecycle for binary Robotiq intent, with bounded reversal."""
from __future__ import annotations

import copy
import json
import math
import threading
import time
from typing import Any, Callable


class RobotiqHand:
    def __init__(self, transport: Any, *, measured: Callable, timeout: float) -> None:
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Positive finite gripper timeout required')
        self.transport, self.measured, self.timeout = transport, measured, timeout
        self.lock = threading.RLock()
        self.requests: list[dict] = []
        self.events: list[dict] = []
        self.desired = self.active = None
        self.future = self.stop_future = None
        self.stopping = False
        self.stop_confirmed = False
        self.stop_attempted = False
        self.deadline = None
        self.error = None

    def ready(self) -> bool:
        return self.transport.ready()

    @property
    def settled(self) -> bool:
        with self.lock:
            return (self.active is None and self.stop_future is None
                    and (self.stopping and self.stop_confirmed or
                         not self.stopping and (self.desired is None or
                         self.requests[self.desired]['disposition'] != 'queued')))

    def check(self) -> None:
        with self.lock:
            if self.error or self.stopping:
                raise RuntimeError(self.error or 'gripper_stopping')
            if not self.ready():
                raise RuntimeError('robotiq_command_endpoints_unavailable')
            self.measured()  # Missing, faulted and stale feedback is a hard failure.

    def _request(self, kind: str) -> dict:
        with self.lock:
            self.check()
            duplicate = self.desired is not None and self.requests[self.desired]['command_type'] == kind
            row = dict(request_id=len(self.requests), command_type=kind,
                       requested_monotonic_seconds=time.monotonic(),
                       disposition='duplicate' if duplicate else 'queued',
                       command_issued=False, issue_monotonic_seconds=None)
            if duplicate:
                row['duplicate_of'] = self.desired
            else:
                if self.desired is not None and self.desired != self.active:
                    prior = self.requests[self.desired]
                    if prior['disposition'] == 'queued':
                        prior['disposition'] = 'superseded_before_issue'
                self.desired = row['request_id']
            self.requests.append(row)
            return dict(row)

    def request_binary(self, closed: bool) -> dict:
        return self._request('close' if closed else 'open')

    @property
    def inference_pending(self) -> bool:
        return not self.settled

    def stop(self) -> None:
        with self.lock:
            self.stopping, self.desired = True, None

    def _event(self, kind: str, **fields: Any) -> None:
        self.events.append(dict(kind=kind, request_id=self.active,
                                monotonic_seconds=time.monotonic(), **fields))

    def _send_stop(self) -> None:
        self.stop_attempted = True
        self.stop_future = self.transport.call('stop')
        self.deadline = time.monotonic() + self.timeout
        self._event('stop_requested')

    def tick(self) -> None:
        with self.lock:
            try:
                self._tick()
            except Exception as error:
                if self.error is None:
                    self.error = f'{type(error).__name__}: {error}'
                    if self.active is not None:
                        self.requests[self.active]['disposition'] = 'failed'
                    self._event('error', error=self.error)
                    self.stopping, self.desired = True, None
                    # Single stop attempt, including on a timeout or lost feedback.
                    if not self.stop_attempted:
                        try:
                            self._send_stop()
                        except Exception as stop_error:
                            self._event('stop_unconfirmed', error=str(stop_error))

    def _tick(self) -> None:
        if self.deadline is not None and time.monotonic() >= self.deadline:
            self.deadline = None
            raise TimeoutError('robotiq_lifecycle_timeout')
        if not self.stopping and (self.active is not None or self.desired is not None):
            self.measured()
        replacing = self.active is not None and self.desired != self.active
        if ((replacing or self.stopping) and self.stop_future is None
                and not self.stop_attempted):
            self._send_stop()
        if self.stop_future is not None and self.stop_future.done():
            response = self.stop_future.result()
            self.stop_future = None
            if not response.success:
                raise RuntimeError('robotiq_stop_failed: ' + response.message)
            self.stop_confirmed = True
            self._event('stop_confirmed', response=json.loads(response.message))
        if self.future is not None and self.future.done():
            response = self.future.result()
            self.future = None
            if not response.success:
                self.requests[self.active]['disposition'] = 'failed'
                self._event('command_failed', error=response.message)
                self.active = None
                raise RuntimeError('robotiq_command_failed: ' + response.message)
            status = json.loads(response.message)
            cancelled = status.get('cancelled', False)
            if cancelled and not (self.stopping or (self.desired != self.active and self.stop_attempted)):
                self.requests[self.active]['disposition'] = 'failed'
                self._event('unexpected_cancellation', response=status)
                self.active = None
                raise RuntimeError('robotiq_command_cancelled_outside_owned_reversal')
            self._event('command_result', response=status,
                        measured_after=(dict(status, source='command_completion_register_read')
                                        if not cancelled else None))
            self.requests[self.active]['disposition'] = 'cancelled' if cancelled else 'completed'
            # Object detection is evidence, never an automatic task/grasp success.
            self.requests[self.active]['result'] = status
            self.requests[self.active]['measured_after'] = (dict(status) if not cancelled else None)
            self.active = None
        if self.active is not None or self.stop_future is not None:
            return
        if self.stopping:
            if self.stop_confirmed:
                self.deadline = None
            return
        if self.desired is None or self.requests[self.desired]['disposition'] != 'queued':
            self.deadline = None
            return
        row = self.requests[self.desired]
        self.active = self.desired
        self.stop_confirmed = False
        self.stop_attempted = False
        self.future = self.transport.call(row['command_type'])
        row.update(disposition='issued', command_issued=True,
                   issue_monotonic_seconds=time.monotonic())
        self.deadline = time.monotonic() + self.timeout
        self._event('command_requested', command_type=row['command_type'])

    def evidence(self) -> dict:
        with self.lock:
            return copy.deepcopy(dict(backend='robotiq_2f85', requests=self.requests,
                                      events=self.events, error=self.error, settled=self.settled))


class RosRobotiqHand(RobotiqHand):
    def __init__(self, node: Any, *, namespace: str, measured: Callable, timeout: float) -> None:
        from std_srvs.srv import Trigger
        clients = {kind: node.create_client(Trigger, namespace + '/' + kind)
                   for kind in ('open', 'close', 'stop')}

        class Transport:
            def ready(self) -> bool:
                return all(client.service_is_ready() for client in clients.values())

            def call(self, kind: str) -> Any:
                if not clients[kind].service_is_ready():
                    raise RuntimeError('robotiq_' + kind + '_unavailable')
                return clients[kind].call_async(Trigger.Request())

        super().__init__(Transport(), measured=measured, timeout=timeout)
        self.timer = node.create_timer(.005, self.tick)
