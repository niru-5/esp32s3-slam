"""Threaded HTTP server that ingests the ESP32-S3 stream and serves a live view.

Ingest (what the firmware POSTs when built with CONFIG_ENABLE_REMOTE_STREAM):
    POST /frame   body = JPEG,  header X-Timestamp-Us = esp_timer us
    POST /imu     body = binary IMU drain (see wire.py)

Browser:
    GET  /             viewer page (viewer.html)
    GET  /stream.mjpg  multipart MJPEG of the latest frames
    GET  /imu.json     latest IMU sample + running stats (polled by the page)

Built on the standard library only (http.server) so it runs under a sourced
ROS 2 environment with no extra pip installs.
"""

from __future__ import annotations

import http.server
import socketserver
from pathlib import Path

from .bag_recorder import BagRecorder
from .hub import Hub
from .wire import parse_imu_payload, parse_stats_payload

_VIEWER_HTML = (Path(__file__).parent / "viewer.html").read_bytes()
_BOUNDARY = "slamframe"


class _Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    # injected by make_server()
    hub: Hub
    recorder: BagRecorder

    def log_message(self, fmt, *args):  # quieter default logging
        pass

    # -- helpers -----------------------------------------------------------
    def _read_body(self) -> bytes:
        length = int(self.headers.get("Content-Length", 0))
        return self.rfile.read(length) if length else b""

    def _send_json(self, obj) -> None:
        import json
        payload = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    # -- POST ingest -------------------------------------------------------
    def do_POST(self):
        if self.path.startswith("/frame"):
            body = self._read_body()
            ts = int(self.headers.get("X-Timestamp-Us", "0"))
            if body:
                self.hub.put_frame(ts, body)
                self.recorder.write_frame(ts, body)
            self.send_response(204)
            self.send_header("Content-Length", "0")
            self.end_headers()
        elif self.path.startswith("/imu"):
            body = self._read_body()
            try:
                samples = parse_imu_payload(body)
            except ValueError as exc:
                self.send_error(400, str(exc))
                return
            self.hub.put_imu(samples)
            for s in samples:
                self.recorder.write_imu(s)
            self.send_response(204)
            self.send_header("Content-Length", "0")
            self.end_headers()
        elif self.path.startswith("/stats"):
            body = self._read_body()
            try:
                stats = parse_stats_payload(body)
            except ValueError as exc:
                self.send_error(400, str(exc))
                return
            self.hub.put_stats(stats)
            self.recorder.write_stats(stats)
            self.send_response(204)
            self.send_header("Content-Length", "0")
            self.end_headers()
        else:
            self.send_error(404)

    # -- GET browser -------------------------------------------------------
    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(_VIEWER_HTML)))
            self.end_headers()
            self.wfile.write(_VIEWER_HTML)
        elif self.path.startswith("/stream.mjpg"):
            self._stream_mjpeg()
        elif self.path.startswith("/imu.json"):
            snap = self.hub.snapshot()
            snap["recording"] = self.recorder.enabled
            snap["bag_uri"] = self.recorder.bag_uri if self.recorder.enabled else ""
            self._send_json(snap)
        else:
            self.send_error(404)

    def _stream_mjpeg(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={_BOUNDARY}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        version = -1
        try:
            while True:
                version, jpeg = self.hub.wait_frame(version, timeout=5.0)
                if jpeg is None:
                    continue
                head = (
                    f"--{_BOUNDARY}\r\n"
                    f"Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(jpeg)}\r\n\r\n"
                ).encode()
                self.wfile.write(head)
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass  # browser closed the stream


class _Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def make_server(host: str, port: int, hub: Hub, recorder: BagRecorder) -> _Server:
    handler = type("_BoundHandler", (_Handler,), {"hub": hub, "recorder": recorder})
    return _Server((host, port), handler)
