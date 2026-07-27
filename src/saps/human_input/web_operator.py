"""Browser-based operator interface for keyboard and future gamepad input."""

from __future__ import annotations

import asyncio
import http.server
import json
import logging
import threading
import time
from typing import Any
from typing import Optional

import cv2
import numpy as np
import websockets

from saps.human_input.keyboard import ALLOWED_KEYS
from saps.human_input.keyboard import HumanInputSample
from saps.human_input.keyboard import KeyboardActionMapper
from saps.human_input.keyboard import SPEED_MODES


LOGGER = logging.getLogger(__name__)


def _build_operator_page(
    *,
    websocket_port: int,
) -> str:
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta
    name="viewport"
    content="width=device-width, initial-scale=1"
  >
  <title>SAPS Operator Console</title>
  <style>
    :root {{
      color-scheme: dark;
      font-family: system-ui, sans-serif;
    }}

    body {{
      margin: 0;
      background: #11151a;
      color: #eef2f6;
    }}

    main {{
      max-width: 1100px;
      margin: 0 auto;
      padding: 22px;
    }}

    h1 {{
      margin-top: 0;
    }}

    .layout {{
      display: grid;
      grid-template-columns: minmax(420px, 2fr) minmax(300px, 1fr);
      gap: 20px;
    }}

    .panel {{
      background: #1c222a;
      border: 1px solid #39434f;
      border-radius: 12px;
      padding: 16px;
    }}

    #camera {{
      width: 100%;
      aspect-ratio: 2 / 1;
      object-fit: contain;
      background: #050607;
      border-radius: 8px;
    }}

    .status {{
      display: grid;
      grid-template-columns: 150px 1fr;
      gap: 7px 12px;
      margin-top: 10px;
    }}

    .label {{
      color: #aeb8c4;
    }}

    .good {{
      color: #67d391;
      font-weight: 700;
    }}

    .warning {{
      color: #ffcc66;
      font-weight: 700;
    }}

    .danger {{
      color: #ff7474;
      font-weight: 700;
    }}

    button {{
      border: 0;
      border-radius: 8px;
      padding: 10px 14px;
      margin: 4px;
      cursor: pointer;
      font-size: 15px;
    }}

    #arm {{
      background: #2f9e60;
      color: white;
    }}

    #disarm {{
      background: #b7791f;
      color: white;
    }}

    #abort {{
      background: #c43d3d;
      color: white;
    }}

    #open-gripper,
    #close-gripper {{
      background: #4169a1;
      color: white;
    }}

    code,
    pre {{
      font-family: ui-monospace, monospace;
    }}

    pre {{
      background: #101419;
      padding: 12px;
      border-radius: 8px;
      overflow-x: auto;
    }}

    table {{
      width: 100%;
      border-collapse: collapse;
    }}

    th,
    td {{
      padding: 5px 8px;
      text-align: left;
      border-bottom: 1px solid #303943;
    }}

    @media (max-width: 850px) {{
      .layout {{
        grid-template-columns: 1fr;
      }}
    }}
  </style>
</head>
<body>
<main>
  <h1>SAPS Operator Console</h1>

  <div class="layout">
    <section class="panel">
      <img id="camera" alt="Robot camera stream">

      <div class="status">
        <div class="label">WebSocket</div>
        <div id="connection" class="warning">Connecting…</div>

        <div class="label">Controls</div>
        <div id="armed" class="warning">Disarmed</div>

        <div class="label">Pressed keys</div>
        <div id="pressed">None</div>

        <div class="label">Motion active</div>
        <div id="motion-active">False</div>

        <div class="label">Gripper</div>
        <div id="gripper">Open (-1)</div>

        <div class="label">Speed mode</div>
        <div id="speed-mode">Fine</div>

        <div class="label">Interface</div>
        <div>Phase 2.2 controls</div>
      </div>

      <h3>Current 7D action</h3>
      <pre id="action">[0, 0, 0, 0, 0, 0, -1]</pre>
    </section>

    <section class="panel">
      <h2>Safety controls</h2>

      <button id="arm">Arm controls</button>
      <button id="disarm">Disarm</button>
      <button id="abort">Abort episode</button>

      <h2>Gripper</h2>

      <button id="open-gripper">Open (Z)</button>
      <button id="close-gripper">Close (X)</button>

      <h2>Speed</h2>

      <button id="speed-fine">Fine (1)</button>
      <button id="speed-normal">Normal (2)</button>
      <button id="speed-fast">Fast (3)</button>

      <h2>Keyboard mapping</h2>

      <table>
        <tr><th>Keys</th><th>Command</th></tr>
        <tr><td>W / S</td><td>screen forward / backward</td></tr>
        <tr><td>A / D</td><td>screen left / right</td></tr>
        <tr><td>Space / Shift</td><td>up / down</td></tr>
        <tr><td>Q / E</td><td>yaw left / right</td></tr>
        <tr><td>Up / Down</td><td>pitch</td></tr>
        <tr><td>Left / Right</td><td>roll</td></tr>
        <tr><td>Z / X</td><td>open / close gripper</td></tr>
        <tr><td>1 / 2 / 3</td><td>fine / normal / fast</td></tr>
        <tr><td>Escape</td><td>abort</td></tr>
      </table>

      <p>
        Click this page before using the keyboard.
        Releasing a key removes its motion command immediately.
        Switching away from the browser clears all pressed keys.
      </p>

      <h3>Runtime status</h3>
      <pre id="runtime-status">{{}}</pre>
    </section>
  </div>
</main>

<script>
"use strict";

const websocketPort = {websocket_port};
const websocketUrl =
  `ws://${{window.location.hostname}}:${{websocketPort}}`;

const allowedKeys = new Set([
  "w", "a", "s", "d",
  "space", "shift",
  "q", "e",
  "arrowup", "arrowdown",
  "arrowleft", "arrowright",
  "z", "x"
]);

const pressedKeys = new Set();
let socket = null;
let reconnectTimer = null;
let currentImageUrl = null;

const connectionElement =
  document.getElementById("connection");
const armedElement =
  document.getElementById("armed");
const pressedElement =
  document.getElementById("pressed");
const motionElement =
  document.getElementById("motion-active");
const gripperElement =
  document.getElementById("gripper");
const speedElement =
  document.getElementById("speed-mode");
const actionElement =
  document.getElementById("action");
const runtimeStatusElement =
  document.getElementById("runtime-status");
const cameraElement =
  document.getElementById("camera");

function send(message) {{
  if (socket && socket.readyState === WebSocket.OPEN) {{
    socket.send(JSON.stringify(message));
  }}
}}

function sendPressedKeys() {{
  send({{
    type: "keys",
    keys: Array.from(pressedKeys).sort(),
    browser_timestamp_ms: performance.now()
  }});
}}

function clearPressedKeys() {{
  if (pressedKeys.size === 0) {{
    return;
  }}

  pressedKeys.clear();
  sendPressedKeys();
}}

function connect() {{
  socket = new WebSocket(websocketUrl);
  socket.binaryType = "blob";

  socket.onopen = () => {{
    connectionElement.textContent = "Connected";
    connectionElement.className = "good";
    clearTimeout(reconnectTimer);
    sendPressedKeys();
  }};

  socket.onclose = () => {{
    connectionElement.textContent = "Disconnected";
    connectionElement.className = "danger";
    clearPressedKeys();

    reconnectTimer = setTimeout(
      connect,
      1000
    );
  }};

  socket.onerror = () => {{
    connectionElement.textContent = "Connection error";
    connectionElement.className = "danger";
  }};

  socket.onmessage = async (event) => {{
    if (typeof event.data === "string") {{
      const message = JSON.parse(event.data);

      if (message.type === "status") {{
        const sample = message.sample;

        armedElement.textContent =
          sample.armed ? "Armed" : "Disarmed";
        armedElement.className =
          sample.armed ? "good" : "warning";

        pressedElement.textContent =
          sample.pressed_keys.length
            ? sample.pressed_keys.join(", ")
            : "None";

        motionElement.textContent =
          String(sample.motion_active);

        gripperElement.textContent =
          sample.gripper_command > 0
            ? "Closed (+1)"
            : "Open (-1)";

        speedElement.textContent =
          `${{sample.speed_mode}} ` +
          `(T=${{sample.translation_gain.toFixed(2)}}, ` +
          `R=${{sample.rotation_gain.toFixed(2)}})`;

        actionElement.textContent =
          JSON.stringify(sample.action, null, 2);

        runtimeStatusElement.textContent =
          JSON.stringify(
            message.runtime_status || {{}},
            null,
            2
          );
      }}

      return;
    }}

    if (currentImageUrl !== null) {{
      URL.revokeObjectURL(currentImageUrl);
    }}

    currentImageUrl = URL.createObjectURL(event.data);
    cameraElement.src = currentImageUrl;
  }};
}}

function normalizeKey(event) {{
  const key = event.key.toLowerCase();

  if (key === " " || key === "spacebar") {{
    return "space";
  }}

  if (key === "shift") {{
    return "shift";
  }}

  return key;
}}

window.addEventListener("keydown", (event) => {{
  const key = normalizeKey(event);

  if (key === "escape") {{
    event.preventDefault();
    send({{type: "abort"}});
    return;
  }}

  const speedModes = {{
    "1": "fine",
    "2": "normal",
    "3": "fast"
  }};

  if (speedModes[key]) {{
    event.preventDefault();
    send({{
      type: "speed",
      value: speedModes[key]
    }});
    return;
  }}

  if (!allowedKeys.has(key)) {{
    return;
  }}

  event.preventDefault();

  if (!pressedKeys.has(key)) {{
    pressedKeys.add(key);
    sendPressedKeys();
  }}
}});

window.addEventListener("keyup", (event) => {{
  const key = normalizeKey(event);

  if (!allowedKeys.has(key)) {{
    return;
  }}

  event.preventDefault();

  if (pressedKeys.delete(key)) {{
    sendPressedKeys();
  }}
}});

window.addEventListener("blur", clearPressedKeys);
document.addEventListener(
  "visibilitychange",
  () => {{
    if (document.hidden) {{
      clearPressedKeys();
    }}
  }}
);

document.getElementById("arm").onclick = () => {{
  send({{type: "arm", value: true}});
}};

document.getElementById("disarm").onclick = () => {{
  clearPressedKeys();
  send({{type: "arm", value: false}});
}};

document.getElementById("abort").onclick = () => {{
  clearPressedKeys();
  send({{type: "abort"}});
}};

document.getElementById("open-gripper").onclick = () => {{
  send({{type: "gripper", value: -1}});
}};

document.getElementById("close-gripper").onclick = () => {{
  send({{type: "gripper", value: 1}});
}};

document.getElementById("speed-fine").onclick = () => {{
  send({{type: "speed", value: "fine"}});
}};

document.getElementById("speed-normal").onclick = () => {{
  send({{type: "speed", value: "normal"}});
}};

document.getElementById("speed-fast").onclick = () => {{
  send({{type: "speed", value: "fast"}});
}};

connect();
</script>
</body>
</html>
"""


class BrowserOperatorServer:
    """Serve an operator webpage and maintain the latest input state."""

    def __init__(
        self,
        *,
        host: str = "0.0.0.0",
        websocket_port: int = 8765,
        http_port: int = 8766,
        fine_translation_gain: float = 0.07,
        normal_translation_gain: float = 0.14,
        fast_translation_gain: float = 0.25,
        fine_rotation_gain: float = 0.10,
        normal_rotation_gain: float = 0.18,
        fast_rotation_gain: float = 0.30,
        default_speed_mode: str = "fine",
        translation_gain: Optional[float] = None,
        rotation_gain: Optional[float] = None,
        jpeg_quality: int = 85,
    ) -> None:
        if not 1 <= jpeg_quality <= 100:
            raise ValueError(
                "jpeg_quality must be within [1, 100]."
            )

        self.host = host
        self.websocket_port = int(websocket_port)
        self.http_port = int(http_port)
        self.jpeg_quality = int(jpeg_quality)

        # Preserve the concise legacy arguments used by the
        # Phase 2.1 operator test. They configure normal speed.
        if translation_gain is not None:
            normal_translation_gain = float(
                translation_gain
            )

        if rotation_gain is not None:
            normal_rotation_gain = float(
                rotation_gain
            )

        self._mapper = KeyboardActionMapper(
            fine_translation_gain=(
                fine_translation_gain
            ),
            normal_translation_gain=(
                normal_translation_gain
            ),
            fast_translation_gain=(
                fast_translation_gain
            ),
            fine_rotation_gain=(
                fine_rotation_gain
            ),
            normal_rotation_gain=(
                normal_rotation_gain
            ),
            fast_rotation_gain=(
                fast_rotation_gain
            ),
            default_speed_mode=(
                default_speed_mode
            ),
        )

        self._lock = threading.Lock()
        self._pressed_keys: set[str] = set()
        self._gripper_command = -1.0
        self._speed_mode = (
            self._mapper.default_speed_mode
        )
        self._armed = False
        self._abort_requested = False
        self._connected_clients = 0
        self._last_event_monotonic_seconds: Optional[
            float
        ] = None

        self._latest_frame_bytes: Optional[bytes] = None
        self._latest_frame_version = 0
        self._latest_runtime_status: dict[str, Any] = {}

        self._clients: set[Any] = set()
        self._client_event = threading.Event()
        self._stopping = threading.Event()

        self._event_loop: Optional[
            asyncio.AbstractEventLoop
        ] = None
        self._websocket_server: Any = None
        self._websocket_thread: Optional[
            threading.Thread
        ] = None

        self._http_server: Optional[
            http.server.ThreadingHTTPServer
        ] = None
        self._http_thread: Optional[
            threading.Thread
        ] = None

    @property
    def operator_url(self) -> str:
        return (
            f"http://127.0.0.1:{self.http_port}"
        )

    def start(self) -> None:
        if self._websocket_thread is not None:
            raise RuntimeError(
                "Operator server has already been started."
            )

        self._stopping.clear()

        self._start_http_server()

        self._websocket_thread = threading.Thread(
            target=self._run_websocket_thread,
            name="saps-operator-websocket",
            daemon=True,
        )
        self._websocket_thread.start()

    def _start_http_server(self) -> None:
        page = _build_operator_page(
            websocket_port=self.websocket_port,
        ).encode("utf-8")

        class OperatorPageHandler(
            http.server.BaseHTTPRequestHandler
        ):
            def do_GET(self) -> None:
                if self.path not in {
                    "/",
                    "/index.html",
                }:
                    self.send_error(404)
                    return

                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "text/html; charset=utf-8",
                )
                self.send_header(
                    "Content-Length",
                    str(len(page)),
                )
                self.send_header(
                    "Cache-Control",
                    "no-store",
                )
                self.end_headers()
                self.wfile.write(page)

            def do_HEAD(self) -> None:
                if self.path not in {
                    "/",
                    "/index.html",
                }:
                    self.send_error(404)
                    return

                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "text/html; charset=utf-8",
                )
                self.send_header(
                    "Content-Length",
                    str(len(page)),
                )
                self.send_header(
                    "Cache-Control",
                    "no-store",
                )
                self.end_headers()

            def log_message(
                self,
                format: str,
                *args: Any,
            ) -> None:
                LOGGER.debug(
                    "Operator HTTP: " + format,
                    *args,
                )

        self._http_server = (
            http.server.ThreadingHTTPServer(
                (self.host, self.http_port),
                OperatorPageHandler,
            )
        )

        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="saps-operator-http",
            daemon=True,
        )
        self._http_thread.start()

    def _run_websocket_thread(self) -> None:
        loop = asyncio.new_event_loop()
        self._event_loop = loop
        asyncio.set_event_loop(loop)

        self._websocket_server = loop.run_until_complete(
            websockets.serve(
                self._handle_client,
                self.host,
                self.websocket_port,
                ping_interval=20,
                ping_timeout=20,
                max_size=2**20,
            )
        )

        loop.create_task(self._broadcast_loop())

        try:
            loop.run_forever()
        finally:
            pending = asyncio.all_tasks(loop)

            for task in pending:
                task.cancel()

            if pending:
                loop.run_until_complete(
                    asyncio.gather(
                        *pending,
                        return_exceptions=True,
                    )
                )

            if self._websocket_server is not None:
                self._websocket_server.close()
                loop.run_until_complete(
                    self._websocket_server.wait_closed()
                )

            loop.close()

    async def _handle_client(
        self,
        websocket: Any,
        path: Any = None,
    ) -> None:
        del path

        if self._clients:
            LOGGER.warning(
                "Rejecting additional operator browser."
            )
            await websocket.close(
                code=4001,
                reason=(
                    "Only one operator browser "
                    "is allowed."
                ),
            )
            return

        self._clients.add(websocket)

        with self._lock:
            self._connected_clients += 1
            self._last_event_monotonic_seconds = (
                time.monotonic()
            )

        self._client_event.set()

        LOGGER.info(
            "Operator browser connected; clients=%d",
            self._connected_clients,
        )

        try:
            async for raw_message in websocket:
                if isinstance(raw_message, bytes):
                    continue

                try:
                    message = json.loads(raw_message)
                except json.JSONDecodeError:
                    LOGGER.warning(
                        "Ignoring invalid operator JSON."
                    )
                    continue

                self._apply_message(message)

        finally:
            self._clients.discard(websocket)

            with self._lock:
                self._connected_clients = max(
                    0,
                    self._connected_clients - 1,
                )

                # Clearing motion and disarming on disconnect is a
                # deliberate safety behavior.
                self._pressed_keys.clear()
                self._armed = False
                self._last_event_monotonic_seconds = (
                    time.monotonic()
                )

                if self._connected_clients == 0:
                    self._client_event.clear()

            LOGGER.info(
                "Operator browser disconnected; clients=%d",
                self._connected_clients,
            )

    def _apply_message(
        self,
        message: dict[str, Any],
    ) -> None:
        message_type = str(
            message.get("type", "")
        )

        with self._lock:
            self._last_event_monotonic_seconds = (
                time.monotonic()
            )

            if message_type == "keys":
                raw_keys = message.get("keys", [])

                self._pressed_keys = {
                    str(key).lower()
                    for key in raw_keys
                    if str(key).lower()
                    in ALLOWED_KEYS
                }

                # Gripper commands are latched. Releasing Z or X
                # does not reverse the requested gripper state.
                if "x" in self._pressed_keys:
                    self._gripper_command = 1.0
                elif "z" in self._pressed_keys:
                    self._gripper_command = -1.0

            elif message_type == "speed":
                requested_mode = str(
                    message.get("value", "")
                ).lower()

                if requested_mode in SPEED_MODES:
                    self._speed_mode = requested_mode

            elif message_type == "arm":
                self._armed = bool(
                    message.get("value", False)
                )

                if not self._armed:
                    self._pressed_keys.clear()

            elif message_type == "abort":
                self._abort_requested = True
                self._armed = False
                self._pressed_keys.clear()

            elif message_type == "clear_abort":
                self._abort_requested = False

            elif message_type == "gripper":
                value = float(
                    message.get("value", -1.0)
                )
                self._gripper_command = (
                    1.0 if value > 0.0 else -1.0
                )

    async def _broadcast_loop(self) -> None:
        last_frame_version = -1

        while not self._stopping.is_set():
            await asyncio.sleep(0.05)

            if not self._clients:
                continue

            sample = self.sample()

            with self._lock:
                runtime_status = dict(
                    self._latest_runtime_status
                )
                frame_version = (
                    self._latest_frame_version
                )
                frame_bytes = (
                    self._latest_frame_bytes
                )

            status_message = json.dumps(
                {
                    "type": "status",
                    "sample": sample.as_dict(),
                    "runtime_status": runtime_status,
                }
            )

            clients = list(self._clients)

            status_results = await asyncio.gather(
                *[
                    client.send(status_message)
                    for client in clients
                ],
                return_exceptions=True,
            )

            for client, result in zip(
                clients,
                status_results,
            ):
                if isinstance(result, Exception):
                    self._clients.discard(client)

            if (
                frame_bytes is None
                or frame_version == last_frame_version
            ):
                continue

            frame_results = await asyncio.gather(
                *[
                    client.send(frame_bytes)
                    for client in list(self._clients)
                ],
                return_exceptions=True,
            )

            for client, result in zip(
                list(self._clients),
                frame_results,
            ):
                if isinstance(result, Exception):
                    self._clients.discard(client)

            last_frame_version = frame_version

    def wait_for_client(
        self,
        timeout_seconds: Optional[float] = None,
    ) -> bool:
        return self._client_event.wait(
            timeout=timeout_seconds
        )

    def sample(self) -> HumanInputSample:
        with self._lock:
            pressed_keys = set(self._pressed_keys)
            gripper_command = self._gripper_command
            speed_mode = self._speed_mode
            connected = self._connected_clients > 0
            armed = self._armed
            abort_requested = self._abort_requested
            last_event = (
                self._last_event_monotonic_seconds
            )

        return self._mapper.sample(
            pressed_keys=pressed_keys,
            gripper_command=gripper_command,
            speed_mode=speed_mode,
            connected=connected,
            armed=armed,
            abort_requested=abort_requested,
            last_event_monotonic_seconds=last_event,
        )

    def publish_frame_rgb(
        self,
        image_rgb: np.ndarray,
        *,
        runtime_status: Optional[
            dict[str, Any]
        ] = None,
    ) -> None:
        image = np.asarray(image_rgb)

        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(
                "Expected an RGB image with shape [H, W, 3]."
            )

        image_uint8 = np.asarray(
            np.clip(image, 0, 255),
            dtype=np.uint8,
        )

        image_bgr = cv2.cvtColor(
            image_uint8,
            cv2.COLOR_RGB2BGR,
        )

        success, encoded = cv2.imencode(
            ".jpg",
            image_bgr,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                self.jpeg_quality,
            ],
        )

        if not success:
            raise RuntimeError(
                "OpenCV failed to encode operator frame."
            )

        with self._lock:
            self._latest_frame_bytes = (
                encoded.tobytes()
            )
            self._latest_frame_version += 1

            if runtime_status is not None:
                self._latest_runtime_status = dict(
                    runtime_status
                )

    def publish_runtime_status(
        self,
        runtime_status: dict[str, Any],
    ) -> None:
        with self._lock:
            self._latest_runtime_status = dict(
                runtime_status
            )

    def close(self) -> None:
        self._stopping.set()

        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()

        if (
            self._event_loop is not None
            and self._event_loop.is_running()
        ):
            self._event_loop.call_soon_threadsafe(
                self._event_loop.stop
            )

        if self._http_thread is not None:
            self._http_thread.join(timeout=2.0)

        if self._websocket_thread is not None:
            self._websocket_thread.join(timeout=2.0)
