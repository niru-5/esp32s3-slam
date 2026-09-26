"""Control link to the device in CAMERA_CALIBRATION mode (firmware/data_capture/main/cam_calib.c).

The device dials us (we listen). One persistent socket, framed as

    uint32 len (LE) | uint8 type | body[len-1]

    type 0x01  JSON   both directions
    type 0x02  IMAGE  device -> host:  uint32 meta_len | meta JSON | frame bytes

Requests carry an ``id``; the device answers with ``{"id":..,"ok":..}``. While a
request is in flight the device may also emit IMAGE messages (capture) and
``reg_chunk`` events (reg_dump) -- ``call()`` gathers them into the returned Reply.
"""

from __future__ import annotations

import json
import socket
import struct
import time
from dataclasses import dataclass, field

MSG_JSON = 0x01
MSG_IMAGE = 0x02

_LEN = struct.Struct("<I")


class LinkError(RuntimeError):
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


class DeviceLink:
    """Host side: we listen, the device (cam_calib.c) dials in and re-dials if the link drops."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8084):
        self.host, self.port = host, port
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, port))
        self._srv.listen(1)
        self._conn: socket.socket | None = None
        self._next_id = 1
        self.hello: dict = {}
        self.peer = None

    # -- connection --------------------------------------------------------
    def wait_for_device(self, timeout: float | None = None, on_wait=None) -> dict:
        """Block until the device connects and sends its hello event."""
        self.close_conn()
        deadline = None if timeout is None else time.monotonic() + timeout
        last_note = 0.0
        while True:
            self._srv.settimeout(1.0)
            try:
                conn, self.peer = self._srv.accept()
                break
            except socket.timeout:
                if deadline is not None and time.monotonic() > deadline:
                    raise LinkError("timed out waiting for the device to connect -- is it in camera "
                                    "calibration mode (serial command 5) and is port "
                                    f"{self.port} open on this host?") from None
                if on_wait and time.monotonic() - last_note > 5:
                    on_wait()
                    last_note = time.monotonic()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._conn = conn
        kind, body = self._read_msg(10.0)
        if kind != MSG_JSON:
            raise LinkError("expected hello JSON from device")
        self.hello = json.loads(body)
        return self.hello

    @property
    def connected(self) -> bool:
        return self._conn is not None

    def close_conn(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            finally:
                self._conn = None

    def close(self) -> None:
        self.close_conn()
        self._srv.close()

    # -- framing -----------------------------------------------------------
    def _recv_exact(self, n: int, deadline: float) -> bytes:
        assert self._conn is not None
        buf = bytearray()
        while len(buf) < n:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise LinkError("timed out waiting for device")
            self._conn.settimeout(remaining)
            try:
                chunk = self._conn.recv(min(n - len(buf), 1 << 16))
            except socket.timeout as exc:
                raise LinkError("timed out waiting for device") from exc
            except OSError as exc:
                self.close_conn()
                raise LinkError(f"link error: {exc}") from exc
            if not chunk:
                self.close_conn()
                raise LinkError("device disconnected")
            buf.extend(chunk)
        return bytes(buf)

    def _read_msg(self, timeout: float) -> tuple[int, bytes]:
        deadline = time.monotonic() + timeout
        (length,) = _LEN.unpack(self._recv_exact(4, deadline))
        body = self._recv_exact(length, deadline)
        return body[0], body[1:]

    def _send_json(self, obj: dict) -> None:
        if self._conn is None:
            raise LinkError("no device connected")
        payload = json.dumps(obj, separators=(",", ":")).encode()
        try:
            self._conn.sendall(_LEN.pack(len(payload) + 1) + bytes([MSG_JSON]) + payload)
        except OSError as exc:
            self.close_conn()
            raise LinkError(f"link error: {exc}") from exc

    # -- request/response --------------------------------------------------
    def call(self, cmd: str, timeout: float = 15.0, **args) -> Reply:
        """Send ``{"cmd": cmd, ...}`` and gather everything up to its reply."""
        rid = self._next_id
        self._next_id += 1
        self._send_json({"cmd": cmd, "id": rid, **args})
        reply = Reply(data={})
        deadline = time.monotonic() + timeout
        while True:
            kind, body = self._read_msg(max(deadline - time.monotonic(), 0.01))
            if kind == MSG_IMAGE:
                (meta_len,) = struct.unpack_from("<I", body, 0)
                meta = json.loads(body[4:4 + meta_len])
                reply.images.append(Image(meta, body[4 + meta_len:]))
                deadline = max(deadline, time.monotonic() + 5.0)   # progress -> extend
            elif kind == MSG_JSON:
                obj = json.loads(body)
                if obj.get("evt") == "reg_chunk" and obj.get("id") == rid:
                    reply.chunks.append(obj)
                    deadline = max(deadline, time.monotonic() + 5.0)
                elif obj.get("id") == rid:
                    reply.data = obj
                    return reply
                # anything else (stale/other events) is ignored
