"""The one host_server app: a single http.server serving the browser page, the streaming
ingest routes, and the calibration console API, backed by one DeviceSession (the device's
single always-on control channel). See docs/architecture.md "Control channel" and
docs/camera_calibration_and_tuning.md.

    GET  /                    the one browser page (index.html)
    GET  /status.json         device status + streaming stats + recording state
    GET  /stream.mjpg         multipart MJPEG of the latest frames (STREAM_WIFI/STREAM_TCP)
    POST /frame  /imu  /stats device -> host streaming ingest (STREAM_WIFI only; STREAM_TCP
                              uses tcp_ingest.py's separate on-demand ports instead)
    POST /command             {"cmd": ...} forwarded to the device, or one of the host-only
                              pseudo-commands below

    GET  /calib/live.jpg      latest calibration-session capture (LiveView's buffer)
    GET  /calib/output.json   {"lines": [...], "next": N, "busy": bool} -- poll with
                              ?since=<next from the previous poll>
    POST /calib/line          {"line": "..."} -- run one cli.py command (reg/regw/tune/
                              cal/save/... -- anything in cli.py's HELP)
    POST /calib/answer        {"line": "..."} -- answers a pending readline() prompt
                              (e.g. a `tune` step's "press Enter when ready")

Host-only pseudo-commands on POST /command (not sent to the device):
    set_recording {"enabled": true, "bag": "...", "storage": "sqlite3"}
        Starts/stops a BagRecorder (see bag_recorder.RecordingSlot). `bag`/`storage` only
        matter when enabling; a bag name is generated if omitted.

set_state is special-cased (not just forwarded) for two reasons: entering/leaving
stream_tcp opens/closes tcp_ingest.py's on-demand ports around the device call, and
entering/leaving camera_calibration creates/destroys the calibration console.
"""

from __future__ import annotations

import datetime as _dt
import http.server
import json
import socketserver
import threading
import time
from pathlib import Path

from . import wire
from .bag_recorder import RecordingSlot
from .calibration import intrinsics as ix
from .calibration.cli import App
from .calibration.console import CalibrationConsole
from .calibration.liveview import LiveView
from .calibration.session import Session, default_root
from .device_session import DeviceSession, DeviceSessionError
from .hub import Hub
from .tcp_ingest import TcpIngestManager

_INDEX_HTML = (Path(__file__).parent / "index.html").read_bytes()
_BOUNDARY = "slamframe"

_STREAM_STATES = {"stream_wifi", "stream_sdcard", "stream_tcp"}


class CalibrationManager:
    """Owns the one calibration Session/App/CalibrationConsole, live for as long as the
    device is in camera_calibration. Created on entering that state, destroyed on leaving
    it -- see Server._set_state() and Server._poll_loop().

    enter()/leave() are called from at least two threads that can both observe "not active"
    at once: the HTTP handler thread driving a just-issued set_state, and the background
    poller (which independently notices camera_calibration from serial-triggered entry, or
    session teardown from the console's own `exit`). `enter()`'s own body includes
    App.startup_backup()'s ~9 s full register dump, so the naive `if self.active: return`
    guard left a wide check-then-act window open for real: verified on hardware -- both
    threads passed the guard, each built its own Session/App (each doing its own ~9 s
    backup, serialized behind DeviceSession's one-call-at-a-time lock), and console commands
    submitted in between landed on whichever instance happened to currently be
    `self.console` at request time, splitting a single operator session across two orphaned
    ones. `_lock` makes the whole check-and-build atomic instead.
    """

    def __init__(self, device: DeviceSession, out_root: Path | None = None):
        self.device = device
        self.out_root = out_root
        self.session: Session | None = None
        self.console: CalibrationConsole | None = None
        self.view: LiveView | None = None
        self.board = ix.Board(9, 6, 25.0)
        self._lock = threading.Lock()

    @property
    def active(self) -> bool:
        return self.console is not None

    def enter(self) -> None:
        with self._lock:
            self._enter_locked()

    def _enter_locked(self) -> None:
        if self.active:
            return
        self.session = Session(self.out_root or default_root())
        self.session.update_meta(device_hello=self.device.hello)
        self.view = LiveView(None)   # no embedded server -- served via GET /calib/live.jpg

        def factory(out, readline):
            app = App(self.device, self.session, self.view, self.board, out=out, readline=readline)
            try:
                app.startup_backup()
            except DeviceSessionError as exc:
                out(f"startup backup failed: {exc}")
            return app

        self.console = CalibrationConsole(factory)

    def leave(self) -> None:
        with self._lock:
            if self.console is not None:
                self.console.close()
            self.console = None
            self.session = None
            self.view = None


class Server:
    """All the state one host_server process owns. `_Handler` below is a thin HTTP
    adapter over this."""

    def __init__(self, device_port: int, calib_root: Path | None,
                tcp_host: str, tcp_ports: tuple[int, int, int]):
        self.hub = Hub()
        self.recorder = RecordingSlot()
        self.device = DeviceSession("0.0.0.0", device_port)
        self.calib = CalibrationManager(self.device, calib_root)
        self.tcp = TcpIngestManager(tcp_host, *tcp_ports, self.hub, self.recorder)
        self._last_state: str | None = None
        self._stop_poll = False
        # Sole authoritative source for "did the device leave/enter camera_calibration" --
        # /command's set_state path (below) also reacts immediately for a responsive UI when
        # the browser itself is what triggered the change, but this catches every other way
        # it can happen: the calibration console's own `exit`/`quit` command (leaves
        # calibration on the device without going through this class's set_state path at
        # all), or the state changing via serial instead of the API.
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True, name="state-poll")
        self._poll_thread.start()

    def _poll_loop(self) -> None:
        # Calibration-console lifecycle only -- NOT tcp.stop()/tcp.start(): a state read here
        # can legitimately lag the host's own just-issued request for up to
        # CONFIG_STATE_MACHINE_POLL_MS (the device hasn't drained its queued transition yet).
        # Tried tearing down stream_tcp's ports here too on "st != stream_tcp" and it raced
        # exactly that lag on real hardware -- _set_state() had *just* opened them, this loop
        # read the device's still-stale get_status one tick later, and closed them again
        # before the device's own tcp_client.c ever got a chance to connect. set_state's own
        # handler (below) already opens/closes tcp on the *request* it issued, which is the
        # one thing that's actually authoritative for tcp's lifecycle -- there's no "left
        # stream_tcp via some other side channel" case the way there is for camera_calibration
        # (its console's own `exit` command, unrelated to this class's set_state path at all).
        while not self._stop_poll:
            try:
                st = self.device.call("get_status", timeout=2.0).data.get("state")
            except DeviceSessionError:
                st = None
            if st is not None and st != self._last_state:
                if st == "camera_calibration" and not self.calib.active:
                    self.calib.enter()
                elif st != "camera_calibration" and self.calib.active:
                    self.calib.leave()
                self._last_state = st
            time.sleep(1.0)

    # -- /command --------------------------------------------------------------
    def command(self, body: dict) -> dict:
        cmd = body.pop("cmd", None)
        if not cmd:
            return {"ok": False, "err": "missing cmd"}
        try:
            if cmd == "set_recording":
                return self._set_recording(body)
            if cmd == "set_state":
                return self._set_state(body)
            data = self.device.call(cmd, **body).data
            return data
        except DeviceSessionError as exc:
            return {"ok": False, "err": str(exc)}

    def _set_recording(self, body: dict) -> dict:
        enabled = bool(body.get("enabled"))
        if enabled:
            bag = body.get("bag") or _default_bag_uri()
            storage = body.get("storage", "sqlite3")
            started = self.recorder.start(bag, storage)
            if not started:
                return {"ok": False, "err": "ROS 2 not available -- source /opt/ros/<distro>/setup.bash"}
            return {"ok": True, "recording": True, "bag_uri": self.recorder.bag_uri}
        self.recorder.stop()
        return {"ok": True, "recording": False}

    def _set_state(self, body: dict) -> dict:
        state = body.get("state", "")
        was_calibrating = self._last_state == "camera_calibration"
        try:
            data = self.device.call("set_state", **body).data
        except DeviceSessionError as exc:
            return {"ok": False, "err": str(exc)}

        # tcp_ingest's ports are only meaningful while stream_tcp is the active mode.
        if state == "stream_tcp":
            self.tcp.start()
        elif self.tcp.running:
            self.tcp.stop()

        # The calibration console is only meaningful while camera_calibration is active.
        # set_state's own transition is asynchronous (see control_link.h) -- the queued
        # command lands on the device within CONFIG_STATE_MACHINE_POLL_MS, not instantly --
        # so wait for get_status to actually agree before running the console's startup
        # register backup, or its first reg_dump would race the firmware's own transition
        # and come back "not in camera_calibration".
        if state == "camera_calibration":
            self._wait_for_device_state("camera_calibration", timeout=2.0)
            # Set *before* calib.enter() (its startup register backup takes ~9s), not after:
            # otherwise the poll loop's own "st != self._last_state" keeps reading True for
            # that whole window and repeatedly tries to enter too -- harmless now that
            # CalibrationManager.enter() is lock-protected (only one build ever actually
            # happens), but pointless work avoided by updating this first.
            self._last_state = state
            self.calib.enter()
        else:
            if was_calibrating:
                self.calib.leave()
            self._last_state = state
        return data

    def _wait_for_device_state(self, want: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                got = self.device.call("get_status", timeout=1.0).data.get("state")
            except DeviceSessionError:
                got = None
            if got == want:
                return
            time.sleep(0.1)

    def status(self) -> dict:
        snap = self.hub.snapshot()
        snap["recording"] = self.recorder.enabled
        snap["bag_uri"] = self.recorder.bag_uri
        snap["device"] = self.device.status()
        try:
            snap["device"]["get_status"] = self.device.call("get_status", timeout=3.0).data
        except DeviceSessionError as exc:
            snap["device"]["get_status_error"] = str(exc)
        snap["calibration_active"] = self.calib.active
        return snap

    def close(self) -> None:
        self._stop_poll = True
        self.tcp.stop()
        self.recorder.stop()
        self.calib.leave()
        self.device.close()


def _default_bag_uri() -> str:
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"bags/slam_{stamp}"


# --------------------------------------------------------------------------
# HTTP layer
# --------------------------------------------------------------------------

class _Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    server_state: Server   # injected by make_server()

    def log_message(self, fmt, *args):
        pass

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length) if length else b""
        if not body:
            return {}
        try:
            return json.loads(body)
        except ValueError:
            return {}

    def _send_json(self, obj, status: int = 200) -> None:
        payload = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _send_bytes(self, body: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self) -> bytes:
        length = int(self.headers.get("Content-Length", 0))
        return self.rfile.read(length) if length else b""

    # -- GET -----------------------------------------------------------------
    def do_GET(self):
        s = self.server_state
        path = self.path.split("?", 1)[0]
        if path == "/" or path.startswith("/index"):
            self._send_bytes(_INDEX_HTML, "text/html; charset=utf-8")
        elif path.startswith("/status.json"):
            self._send_json(s.status())
        elif path.startswith("/stream.mjpg"):
            self._stream_mjpeg(s)
        elif path.startswith("/calib/live.jpg"):
            view = s.calib.view
            jpeg = view.jpeg if view else b""
            self._send_bytes(jpeg, "image/jpeg")
        elif path.startswith("/calib/output.json"):
            self._calib_output(s)
        else:
            self.send_error(404)

    def _stream_mjpeg(self, s: Server) -> None:
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={_BOUNDARY}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        version = -1
        try:
            while True:
                version, jpeg = s.hub.wait_frame(version, timeout=5.0)
                if jpeg is None:
                    continue
                head = (f"--{_BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                       f"Content-Length: {len(jpeg)}\r\n\r\n").encode()
                self.wfile.write(head)
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _calib_output(self, s: Server) -> None:
        if s.calib.console is None:
            self._send_json({"lines": [], "next": 0, "busy": False, "active": False,
                             "live_version": 0, "live_text": ""})
            return
        qs = self.path.split("?", 1)
        since = 0
        if len(qs) > 1:
            for kv in qs[1].split("&"):
                if kv.startswith("since="):
                    try:
                        since = int(kv[len("since="):])
                    except ValueError:
                        since = 0
        lines, nxt = s.calib.console.output(since)
        view = s.calib.view
        self._send_json({"lines": lines, "next": nxt, "busy": s.calib.console.busy,
                         "waiting_for_answer": s.calib.console.waiting_for_answer, "active": True,
                         "live_version": view.version if view else 0,
                         "live_text": view.text if view else ""})

    # -- POST ----------------------------------------------------------------
    def do_POST(self):
        s = self.server_state
        path = self.path.split("?", 1)[0]
        if path.startswith("/frame"):
            body = self._read_body()
            ts = int(self.headers.get("X-Timestamp-Us", "0"))
            if body:
                s.hub.put_frame(ts, body)
                s.recorder.write_frame(ts, body)
            self.send_response(204); self.send_header("Content-Length", "0"); self.end_headers()
        elif path.startswith("/imu"):
            body = self._read_body()
            try:
                samples = wire.parse_imu_payload(body)
            except ValueError as exc:
                self.send_error(400, str(exc)); return
            s.hub.put_imu(samples)
            for sample in samples:
                s.recorder.write_imu(sample)
            self.send_response(204); self.send_header("Content-Length", "0"); self.end_headers()
        elif path.startswith("/stats"):
            body = self._read_body()
            try:
                stats = wire.parse_stats_payload(body)
            except ValueError as exc:
                self.send_error(400, str(exc)); return
            s.hub.put_stats(stats)
            s.recorder.write_stats(stats)
            self.send_response(204); self.send_header("Content-Length", "0"); self.end_headers()
        elif path.startswith("/command"):
            req = self._read_json()
            self._send_json(s.command(req))
        elif path.startswith("/calib/line"):
            req = self._read_json()
            line = req.get("line", "")
            if s.calib.console is None:
                self._send_json({"ok": False, "err": "not in camera_calibration"}, status=409)
                return
            s.calib.console.submit(line)
            self._send_json({"ok": True})
        elif path.startswith("/calib/answer"):
            req = self._read_json()
            line = req.get("line", "")
            ok = s.calib.console.answer(line) if s.calib.console else False
            self._send_json({"ok": ok})
        else:
            self.send_error(404)


class _HTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def make_server(http_host: str, http_port: int, state: Server) -> _HTTPServer:
    handler = type("_BoundHandler", (_Handler,), {"server_state": state})
    return _HTTPServer((http_host, http_port), handler)
