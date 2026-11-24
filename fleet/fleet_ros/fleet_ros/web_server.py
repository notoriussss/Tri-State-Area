"""Embedded HTTP web controller and live telemetry dashboard for collector_node."""

import json
import threading
from collections.abc import Callable
from http import HTTPStatus
from http.server import (
    BaseHTTPRequestHandler,
    ThreadingHTTPServer,
)
from typing import Any

DEFAULT_WEB_HOST: str = "0.0.0.0"
DEFAULT_WEB_PORT: int = 8081

INDEX_HTML: str = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Tri-State-Area Collector Controller</title>
<style>
  body { font-family: system-ui, sans-serif; background: #0f172a; color: #e2e8f0;
         margin: 0; padding: 16px; display: flex; flex-direction: column;
         align-items: center; gap: 16px; }
  .card { background: #1e293b; border-radius: 12px; padding: 16px; width: 100%;
          max-width: 440px; box-shadow: 0 4px 16px rgba(0,0,0,0.35); }
  h1 { margin: 0 0 8px 0; font-size: 1.25rem; display: flex;
       justify-content: space-between; align-items: center; }
  .badge { font-size: 0.8rem; padding: 4px 10px; border-radius: 999px;
           background: #334155; font-weight: 700; }
  .badge.auto { background: #16a34a; color: #fff; }
  .badge.line { background: #dc2626; color: #fff; }
  .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-top: 12px; }
  button { padding: 16px 8px; border: none; border-radius: 10px; font-size: 1rem;
           font-weight: 700; cursor: pointer; background: #334155; color: #f8fafc;
           touch-action: none; user-select: none; }
  button:active, button.active { background: #2563eb; }
  button.stop { background: #dc2626; }
  button.mode { width: 100%; margin-top: 10px; padding: 12px; background: #0284c7; }
  button.mode.auto { background: #d97706; }
  .kv { display: grid; grid-template-columns: 1fr 1fr; gap: 6px 12px; font-size: 0.9rem; }
  .k { color: #94a3b8; }
  .v { font-family: monospace; text-align: right; }
</style>
</head>
<body>
<div class="card">
  <h1><span id="robot">collector</span><span id="modeBadge" class="badge">MANUAL</span></h1>
  <div class="kv">
    <div class="k">HAL Mode</div><div class="v" id="halMode">-</div>
    <div class="k">FSM State</div><div class="v" id="fsmState">-</div>
    <div class="k">Line Sensor</div><div class="v" id="lineVal">-</div>
    <div class="k">World Pose</div><div class="v" id="poseVal">-</div>
    <div class="k">Camera Age</div><div class="v" id="camAge">-</div>
    <div class="k">Target Cube</div><div class="v" id="cubeId">-</div>
  </div>
  <button id="toggleAuto" class="mode">Switch to AUTO</button>
</div>
<div class="card">
  <div style="font-size:0.85rem;color:#94a3b8">
    Hold buttons or keyboard: W/S forward/back, A/D strafe, Q/E rotate, Space stop
  </div>
  <div class="grid">
    <button data-cmd="0,0,1">Q Turn L</button>
    <button data-cmd="1,0,0">W Fwd</button>
    <button data-cmd="0,0,-1">E Turn R</button>
    <button data-cmd="0,1,0">A Strafe L</button>
    <button id="stopBtn" class="stop">STOP</button>
    <button data-cmd="0,-1,0">D Strafe R</button>
    <div></div>
    <button data-cmd="-1,0,0">S Back</button>
    <div></div>
  </div>
</div>
<script>
let activeCmd = null;
let autoEnabled = false;
async function postJson(url, body) {
  try {
    await fetch(url, {method:'POST', headers:{'Content-Type':'application/json'},
                      body: JSON.stringify(body)});
  } catch (e) {}
}
function sendStop() {
  activeCmd = null;
  postJson('/api/cmd', {vx: 0, vy: 0, wz: 0});
}
document.querySelectorAll('button[data-cmd]').forEach(btn => {
  const [vx, vy, wz] = btn.dataset.cmd.split(',').map(Number);
  const start = (e) => {
    e.preventDefault(); activeCmd = {vx, vy, wz}; btn.classList.add('active');
  };
  const end = (e) => { e.preventDefault(); btn.classList.remove('active'); sendStop(); };
  btn.addEventListener('pointerdown', start);
  btn.addEventListener('pointerup', end);
  btn.addEventListener('pointerleave', end);
  btn.addEventListener('pointercancel', end);
});
document.getElementById('stopBtn').addEventListener('click', () => {
  if (autoEnabled) postJson('/api/mode', {auto: false});
  sendStop();
});
document.getElementById('toggleAuto').addEventListener('click', () => {
  sendStop();
  postJson('/api/mode', {auto: !autoEnabled});
});
const keys = new Set();
const keyMap = { w:[1,0,0], s:[-1,0,0], a:[0,1,0], d:[0,-1,0], q:[0,0,1], e:[0,0,-1] };
window.addEventListener('keydown', (e) => {
  const k = e.key.toLowerCase();
  if (k === ' ') { sendStop(); return; }
  if (keyMap[k]) keys.add(k);
});
window.addEventListener('keyup', (e) => {
  const k = e.key.toLowerCase();
  if (keyMap[k]) { keys.delete(k); if (!keys.size) sendStop(); }
});
setInterval(() => {
  let cmd = activeCmd;
  if (!cmd && keys.size) {
    let vx = 0, vy = 0, wz = 0;
    keys.forEach(k => { vx += keyMap[k][0]; vy += keyMap[k][1]; wz += keyMap[k][2]; });
    cmd = {vx, vy, wz};
  }
  if (cmd && !autoEnabled) postJson('/api/cmd', cmd);
}, 100);
setInterval(async () => {
  try {
    const r = await fetch('/api/status');
    const s = await r.json();
    autoEnabled = !!s.auto_enabled;
    document.getElementById('robot').textContent = s.robot_name.toUpperCase();
    const badge = document.getElementById('modeBadge');
    badge.textContent = autoEnabled ? 'AUTO' : 'MANUAL';
    badge.className = 'badge ' + (autoEnabled ? 'auto' : '');
    const btn = document.getElementById('toggleAuto');
    btn.textContent = autoEnabled ? 'Pause AUTO (Switch to MANUAL)' : 'Start AUTO';
    btn.className = 'mode ' + (autoEnabled ? 'auto' : '');
    document.getElementById('halMode').textContent = s.hal_mode;
    document.getElementById('fsmState').textContent = s.fsm_state;
    document.getElementById('lineVal').textContent = s.line_detected ? 'BLACK' : 'white';
    document.getElementById('poseVal').textContent = s.pose_text;
    document.getElementById('camAge').textContent = s.camera_age_text;
    document.getElementById('cubeId').textContent = String(s.target_cube_id);
  } catch (e) {}
}, 250);
</script>
</body>
</html>
"""


class CollectorWebServer:
    """Runs a background HTTP server for manual teleop and live status monitoring."""

    def __init__(
        self,
        *,
        host: str = DEFAULT_WEB_HOST,
        port: int = DEFAULT_WEB_PORT,
        get_status: Callable[[], dict[str, Any]],
        on_cmd: Callable[[float, float, float], None],
        on_mode: Callable[[bool], None],
    ) -> None:
        """Binds the HTTP server and starts a daemon thread to serve requests."""
        self._get_status = get_status
        self._on_cmd = on_cmd
        self._on_mode = on_mode
        handler_cls = self._build_handler()
        self._server = ThreadingHTTPServer(
            (host, port),
            handler_cls,
        )
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        """Stops the HTTP server cleanly."""
        self._server.shutdown()
        self._server.server_close()

    def _build_handler(self) -> type[BaseHTTPRequestHandler]:
        """Creates a request handler bound to this server's callbacks."""
        get_status = self._get_status
        on_cmd = self._on_cmd
        on_mode = self._on_mode

        class _Handler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
                return

            def do_GET(self) -> None:  # noqa: N802
                if self.path == "/":
                    payload = INDEX_HTML.encode("utf-8")
                    self._send_bytes(payload, "text/html; charset=utf-8")
                    return

                if self.path == "/api/status":
                    payload = json.dumps(get_status()).encode("utf-8")
                    self._send_bytes(payload, "application/json")
                    return

                self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self) -> None:  # noqa: N802
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length > 0 else b"{}"
                try:
                    data = json.loads(raw.decode("utf-8"))
                except ValueError:
                    self.send_error(HTTPStatus.BAD_REQUEST)
                    return

                if self.path == "/api/cmd":
                    on_cmd(
                        float(data.get("vx", 0.0)),
                        float(data.get("vy", 0.0)),
                        float(data.get("wz", 0.0)),
                    )
                    self._send_bytes(b'{"ok":true}', "application/json")
                    return

                if self.path == "/api/mode":
                    on_mode(bool(data.get("auto", False)))
                    self._send_bytes(b'{"ok":true}', "application/json")
                    return

                self.send_error(HTTPStatus.NOT_FOUND)

            def _send_bytes(
                self,
                payload: bytes,
                content_type: str,
            ) -> None:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

        return _Handler
