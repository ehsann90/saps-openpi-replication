"""Finite policy-only transport using pinned OpenPI's wire encoding."""

from __future__ import annotations

from typing import Any

import numpy as np


class BoundedWebsocketClient:
    """Fail on unavailable servers or delayed responses; never retry a replan."""

    def __init__(self, host: str, port: int, timeout_seconds: float) -> None:
        if not np.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise ValueError("Policy timeout must be finite and positive.")
        from openpi_client import msgpack_numpy
        from websockets.sync.client import connect

        self._codec = msgpack_numpy
        self._timeout = timeout_seconds
        self._socket = connect(
            f"ws://{host}:{port}", compression=None, max_size=None,
            open_timeout=timeout_seconds, close_timeout=1,
        )
        try:
            self._metadata = self._receive()
        except BaseException:
            self.close()
            raise

    def _receive(self) -> dict[str, Any]:
        payload = self._socket.recv(timeout=self._timeout)
        if isinstance(payload, str):
            raise RuntimeError(f"Policy server error: {payload}")
        result = self._codec.unpackb(payload)
        if not isinstance(result, dict):
            raise TypeError("Policy server payload must be a mapping.")
        return result

    def get_server_metadata(self) -> dict[str, Any]:
        return dict(self._metadata)

    def infer(self, request: dict[str, Any]) -> dict[str, Any]:
        self._socket.send(self._codec.Packer().pack(request))
        return self._receive()

    def close(self) -> None:
        self._socket.close()
