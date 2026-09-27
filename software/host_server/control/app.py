"""Threaded HTTP app serving the browser control UI, backed by the device's
always-on control channel (device_channel.ControlLink -> control_link.c).

    GET  /             control_ui.html
    GET  /status.json  connection state + last device get_status + last event
    POST /command      {"cmd": ..., ...} forwarded to the device, JSON reply back

Built on the standard library only (http.server), matching server.py/liveview.py.
"""

from __future__ import annotations

import http.server
import json
import socketserver
from pathlib import Path

from .device_channel import ControlLink, ControlLinkError

_UI_HTML = (Path(__file__).parent / "control_ui.html").read_bytes()


class _Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    link: ControlLink  # injected by make_server()

    def log_message(self, fmt, *args):  # quieter default logging
        pass

    def _send_json(self, obj, status: int = 200) -> None:
        payload = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(_UI_HTML)))
            self.end_headers()
            self.wfile.write(_UI_HTML)
        elif self.path.startswith("/status.json"):
            snap = self.link.status()
            if self.link.connected:
                try:
                    snap["device"] = self.link.call("get_status", timeout=3.0)
                except ControlLinkError as exc:
                    snap["device_error"] = str(exc)
            self._send_json(snap)
        else:
            self.send_error(404)

    def do_POST(self):
        if not self.path.startswith("/command"):
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length) if length else b""
        try:
            req = json.loads(body) if body else {}
        except ValueError:
            self._send_json({"ok": False, "err": "bad json"}, status=400)
            return
        cmd = req.pop("cmd", None)
        if not cmd:
            self._send_json({"ok": False, "err": "missing cmd"}, status=400)
            return
        try:
            data = self.link.call(cmd, **req)
            self._send_json(data)
        except ControlLinkError as exc:
            self._send_json({"ok": False, "err": str(exc)})


class _Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def make_server(http_host: str, http_port: int, link: ControlLink) -> _Server:
    handler = type("_BoundHandler", (_Handler,), {"link": link})
    return _Server((http_host, http_port), handler)
