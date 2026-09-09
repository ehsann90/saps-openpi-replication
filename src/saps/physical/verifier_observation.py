"""Continuously spun, locked snapshots for non-actuating verification."""

from __future__ import annotations

import copy
import threading
import time
from typing import Any

from saps.physical.shadow_ros import SubscriptionBoundary


class ContinuousBoundary(SubscriptionBoundary):
    """Serialize collector mutation and preserve per-source callback evidence."""

    def __init__(self, node: Any) -> None:
        super().__init__(node)
        self.lock = threading.RLock()
        self.events: dict[str, list[tuple[float, float]]] = {}
        self.failure: BaseException | None = None

    def create_subscription(self, msg_type: Any, topic: str,
                            callback: Any, qos: Any) -> Any:
        self.events[topic] = []

        def receive(message: Any) -> None:
            received = time.monotonic()
            source = (message.header.stamp.sec
                      + message.header.stamp.nanosec / 1e9)
            with self.lock:
                self.events[topic].append((received, source))
                callback(message)

        return super().create_subscription(msg_type, topic, receive, qos)

    def snapshot(self, collector: Any) -> Any:
        with self.lock:
            if self.failure is not None:
                raise RuntimeError("ROS subscription thread failed") from self.failure
            snapshot = copy.copy(collector)
            snapshot.errors = dict(collector.errors)
            snapshot.rate_state = copy.deepcopy(collector.rate_state)
            return snapshot

    def continuity(self, start: float, end: float) -> dict[str, Any]:
        """Window counts plus boundary gaps; received callbacks, not raw rates."""
        with self.lock:
            events = {topic: list(values) for topic, values in self.events.items()}
        result = {}
        for topic, history in events.items():
            selected = [(t, s) for t, s in history if start <= t <= end]
            times = [start] + [t for t, _ in selected] + [end]
            source_gaps = [b[1] - a[1] for a, b in zip(selected, selected[1:])]
            result[topic] = {
                "callback_count": len(selected),
                "window_seconds": end - start,
                "callbacks_per_window_second": len(selected) / (end - start)
                if end > start else None,
                "maximum_receive_gap_including_boundaries_seconds":
                    max(b - a for a, b in zip(times, times[1:])),
                "maximum_source_gap_seconds": max(source_gaps) if source_gaps else None,
                "nonadvancing_source_intervals": sum(g <= 0 for g in source_gaps),
                "receive_monotonic_and_source_ros_seconds": selected,
            }
        return result


def spin_continuously(node: Any, boundary: ContinuousBoundary,
                      stop: threading.Event) -> None:
    import rclpy

    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
    except BaseException as error:
        with boundary.lock:
            boundary.failure = error
