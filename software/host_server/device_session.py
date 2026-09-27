"""Persistent client for the device's single always-on control channel
(firmware/data_capture/main/control_link.c).

Evolves what was control/device_channel.py's ControlLink: the firmware side merged
cam_calib.c's whole calibration protocol (including MSG_IMAGE framing) into this one
channel, so this client now reconstructs what host_server/calibration/link.py's DeviceLink
used to do ("gather images + reg_chunk events until the id's final reply arrives") *on top
of* the original persistent, event-aware design (a background thread that keeps reading so
unprompted events like imu_cal_preview are captured even when nothing is mid-call).

Wire framing: uint32 len (LE) | uint8 type | body.
    type 0x01 JSON   both directions
    type 0x02 IMAGE  device -> host: uint32 meta_len | meta JSON | frame bytes

Exactly one call() may be in flight at a time -- the device answers one command fully
before reading the next, so concurrent callers (the calibration console's worker thread,
the browser's mode-toggle endpoint) are serialized by an internal lock.
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time
from dataclasses import dataclass, field

_LEN = struct.Struct("<I")
MSG_JSON = 0x01
MSG_IMAGE = 0x02


class DeviceSessionError(RuntimeError):
    pass


@dataclass
class Image:
    meta: dict
    data: bytes

    @property
    def fmt(self) -> str:
        return self.meta.get("fmt", "")


@dataclass
class Reply:
    data: dict
    images: list[Image] = field(default_factory=list)
    chunks: list[dict] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return bool(self.data.get("ok"))


class DeviceSession:
    def __init__(self, host: str = "0.0.0.0", port: int = 8085):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, port))
        self._srv.listen(1)
        self.port = self._srv.getsockname()[1]   # resolves port=0 to the OS-assigned port

        self._call_lock = threading.Lock()   # serializes call() end-to-end
        self._conn: socket.socket | None = None
        self._next_id = 1
        self._pending: dict | None = None    # {"id", "ev": Event, "reply": Reply} or None
        self._stop = False

        self.connected = False
        self.hello: dict = {}
        self.last_event: dict | None = None
        self.last_event_at: float = 0.0

        threading.Thread(target=self._accept_loop, daemon=True, name="device-accept").start()

    # -- accept / read loop --------------------------------------------------
    def _accept_loop(self) -> None:
        while not self._stop:
            self._srv.settimeout(1.0)
            try:
                conn, _peer = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            if self._conn is not None:
                self._conn.close()
            self._conn = conn
            self.connected = True
            self._read_loop(conn)
            if self._conn is conn:
                self._conn = None
                self.connected = False
            self._fail_pending("device disconnected")

    def _fail_pending(self, msg: str) -> None:
        p = self._pending
        if p is not None:
            self._pending = None
            p["reply"].data = {"ok": False, "err": msg}
            p["ev"].set()

    def _read_loop(self, conn: socket.socket) -> None:
        while not self._stop:
            head = self._recv_exact(conn, 4)
            if head is None:
                return
            (length,) = _LEN.unpack(head)
            body = self._recv_exact(conn, length)
            if body is None or not body:
                return
            kind = body[0]
            if kind == MSG_IMAGE:
                if len(body) < 5:
                    continue
                (meta_len,) = struct.unpack_from("<I", body, 1)
                try:
                    meta = json.loads(body[5:5 + meta_len])
                except ValueError:
                    continue
                p = self._pending
                if p is not None:
                    p["reply"].images.append(Image(meta, body[5 + meta_len:]))
                continue
            if kind != MSG_JSON:
                continue
            try:
                obj = json.loads(body[1:])
            except ValueError:
                continue
            if obj.get("evt") == "hello":
                self.hello = obj
                continue
            p = self._pending
            if obj.get("evt") == "reg_chunk" and p is not None and obj.get("id") == p["id"]:
                p["reply"].chunks.append(obj)
                continue
            if "evt" in obj:
                self.last_event = obj
                self.last_event_at = time.time()
                continue
            rid = obj.get("id")
            if p is not None and rid == p["id"]:
                p["reply"].data = obj
                self._pending = None
                p["ev"].set()
            # else: reply for an id nobody's waiting on (e.g. a just-timed-out call) -- drop.

    @staticmethod
    def _recv_exact(conn: socket.socket, n: int) -> bytes | None:
        buf = bytearray()
        while len(buf) < n:
            try:
                chunk = conn.recv(n - len(buf))
            except OSError:
                return None
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    # -- requests -------------------------------------------------------------
    def call(self, cmd: str, timeout: float = 15.0, **kwargs) -> Reply:
        with self._call_lock:
            conn = self._conn
            if conn is None:
                raise DeviceSessionError("device not connected")
            rid = self._next_id
            self._next_id += 1
            ev = threading.Event()
            reply = Reply(data={})
            self._pending = {"id": rid, "ev": ev, "reply": reply}
            payload = json.dumps({"cmd": cmd, "id": rid, **kwargs}, separators=(",", ":")).encode()
            frame = _LEN.pack(len(payload) + 1) + bytes([MSG_JSON]) + payload
            try:
                conn.sendall(frame)
            except OSError as exc:
                self._pending = None
                raise DeviceSessionError(f"send failed: {exc}") from exc

            if not ev.wait(timeout):
                self._pending = None
                raise DeviceSessionError(f"timed out waiting for a reply to {cmd!r}")
            if not reply.data.get("ok"):
                raise DeviceSessionError(reply.data.get("err", f"{cmd} failed"))
            return reply

    def wait_for_device(self, timeout: float = 60.0) -> None:
        """Block until connected or timeout. Unlike calibration/link.py's DeviceLink (a
        one-shot listener), this session is always reconnecting in the background already --
        this just polls that instead of accepting a connection itself. Compatibility shim
        for cli.py's App.run_line()'s "device link lost -- waiting to reconnect" path."""
        deadline = time.monotonic() + timeout
        while not self.connected and time.monotonic() < deadline:
            time.sleep(0.2)

    def status(self) -> dict:
        return {
            "connected": self.connected,
            "hello": self.hello,
            "last_event": self.last_event,
            "last_event_age_s": (time.time() - self.last_event_at) if self.last_event else None,
        }

    def close(self) -> None:
        self._stop = True
        if self._conn is not None:
            self._conn.close()
        self._srv.close()
