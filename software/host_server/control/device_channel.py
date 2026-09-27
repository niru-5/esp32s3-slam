"""Persistent client for the device's always-on control channel
(firmware/data_capture/main/control_link.c).

Same wire framing as software/host_server/calibration/link.py's DeviceLink
(uint32 len LE | uint8 type | body; MSG_JSON only -- this channel never
carries images), but a different shape on purpose: DeviceLink is built for a
one-shot interactive session (cli.py) where exactly one call() is ever in
flight and nothing needs to happen between calls. This channel is driven by
an always-running host service that must also notice *unprompted* events
(imu_cal_preview, imu_cal_report, ...) the device can send at any time, not
just while something is waiting on a reply -- so a background thread reads
continuously and routes each incoming message to whichever call() is
waiting on its "id", or into `last_event` if it's an event instead of a
reply.
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time

_LEN = struct.Struct("<I")
_MSG_JSON = 0x01


class ControlLinkError(RuntimeError):
    pass


class ControlLink:
    """Listens once, keeps re-accepting the device's reconnects for its own lifetime."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8085):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, port))
        self._srv.listen(1)
        self.port = self._srv.getsockname()[1]   # resolves port=0 to the OS-assigned port

        self._lock = threading.Lock()
        self._conn: socket.socket | None = None
        self._next_id = 1
        self._pending: dict[int, tuple[threading.Event, dict]] = {}
        self._stop = False

        self.connected = False
        self.hello: dict = {}
        self.last_event: dict | None = None
        self.last_event_at: float = 0.0

        threading.Thread(target=self._accept_loop, daemon=True, name="control-accept").start()

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
            with self._lock:
                if self._conn is not None:
                    self._conn.close()
                self._conn = conn
                self.connected = True
            self._read_loop(conn)
            with self._lock:
                if self._conn is conn:
                    self._conn = None
                    self.connected = False
            # Fail every call left waiting on the connection that just dropped.
            with self._lock:
                pending, self._pending = self._pending, {}
            for ev, box in pending.values():
                box["error"] = "device disconnected"
                ev.set()

    def _read_loop(self, conn: socket.socket) -> None:
        while not self._stop:
            head = self._recv_exact(conn, 4)
            if head is None:
                return
            (length,) = _LEN.unpack(head)
            body = self._recv_exact(conn, length)
            if body is None:
                return
            if body[0] != _MSG_JSON:
                continue
            try:
                obj = json.loads(body[1:])
            except ValueError:
                continue
            if obj.get("evt") == "hello":
                self.hello = obj
                continue
            if "evt" in obj:
                self.last_event = obj
                self.last_event_at = time.time()
                continue
            rid = obj.get("id")
            with self._lock:
                waiter = self._pending.pop(rid, None) if rid is not None else None
            if waiter:
                ev, box = waiter
                box["data"] = obj
                ev.set()

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
    def call(self, cmd: str, timeout: float = 10.0, **kwargs) -> dict:
        with self._lock:
            conn = self._conn
            if conn is None:
                raise ControlLinkError("device not connected")
            rid = self._next_id
            self._next_id += 1
            ev = threading.Event()
            box: dict = {}
            self._pending[rid] = (ev, box)
        payload = json.dumps({"cmd": cmd, "id": rid, **kwargs}, separators=(",", ":")).encode()
        frame = _LEN.pack(len(payload) + 1) + bytes([_MSG_JSON]) + payload
        try:
            conn.sendall(frame)
        except OSError as exc:
            with self._lock:
                self._pending.pop(rid, None)
            raise ControlLinkError(f"send failed: {exc}") from exc

        if not ev.wait(timeout):
            with self._lock:
                self._pending.pop(rid, None)
            raise ControlLinkError(f"timed out waiting for a reply to {cmd!r}")
        if "error" in box:
            raise ControlLinkError(box["error"])
        data = box["data"]
        if not data.get("ok"):
            raise ControlLinkError(data.get("err", f"{cmd} failed"))
        return data

    def status(self) -> dict:
        return {
            "connected": self.connected,
            "hello": self.hello,
            "last_event": self.last_event,
            "last_event_age_s": (time.time() - self.last_event_at) if self.last_event else None,
        }

    def close(self) -> None:
        self._stop = True
        with self._lock:
            if self._conn is not None:
                self._conn.close()
        self._srv.close()
