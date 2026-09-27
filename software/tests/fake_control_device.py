"""In-process stand-in for firmware/data_capture control_link.c, for host-side tests.

Speaks the same framed protocol as host_server/control/device_channel.py (uint32 len LE |
uint8 type | body, MSG_JSON only): sends a hello event on connect, answers get_status/
set_state/imu_cal_axis/imu_cal_abort against an in-memory `state`, and lets a test push an
arbitrary event (e.g. imu_cal_preview) on demand via send_event().
"""

from __future__ import annotations

import json
import socket
import struct
import threading

_LEN = struct.Struct("<I")
_MSG_JSON = 0x01


class FakeControlDevice:
    def __init__(self, port: int, state: str = "idle"):
        self.state = state
        self.received: list[dict] = []
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect(("127.0.0.1", port))
        self._lock = threading.Lock()
        self._stop = False
        self._send({"evt": "hello", "mode": "control", "state": self.state})
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _send(self, obj: dict) -> None:
        payload = json.dumps(obj).encode()
        with self._lock:
            self._sock.sendall(_LEN.pack(len(payload) + 1) + bytes([_MSG_JSON]) + payload)

    def send_event(self, obj: dict) -> None:
        self._send(obj)

    def _recv_exact(self, n: int) -> bytes | None:
        buf = bytearray()
        while len(buf) < n:
            try:
                chunk = self._sock.recv(n - len(buf))
            except OSError:
                return None
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _serve(self) -> None:
        while not self._stop:
            head = self._recv_exact(4)
            if head is None:
                return
            (length,) = _LEN.unpack(head)
            body = self._recv_exact(length)
            if body is None:
                return
            if body[0] != _MSG_JSON:
                continue
            req = json.loads(body[1:])
            self.received.append(req)
            self._handle(req)

    def _handle(self, req: dict) -> None:
        rid = req.get("id")
        cmd = req.get("cmd")
        if cmd == "get_status":
            self._send({"id": rid, "ok": True, "state": self.state,
                       "uptime_us": 42, "heap_free": 111, "psram_free": 222})
        elif cmd == "set_state":
            self.state = req["state"]
            self._send({"id": rid, "ok": True, "requested": self.state})
        elif cmd == "imu_cal_axis":
            self._send({"id": rid, "ok": True})
        elif cmd == "imu_cal_abort":
            self._send({"id": rid, "ok": True})
        else:
            self._send({"id": rid, "ok": False, "err": f"unhandled in fake: {cmd}"})

    def close(self) -> None:
        self._stop = True
        try:
            self._sock.close()
        except OSError:
            pass
        self._thread.join(timeout=2.0)
