"""In-process stand-in for firmware/data_capture control_link.c's merged protocol
(state + camera calibration + IMU calibration, see control_link.h), for host-side tests.

Speaks the same framed protocol as host_server/device_session.py (uint32 len LE | uint8
type | body; MSG_JSON=0x01, MSG_IMAGE=0x02): sends a hello event on connect, answers
get_status/set_state/imu_cal_axis/imu_cal_abort and a minimal-but-complete camera
calibration command set (info/reg_read/reg_write/reg_dump/capture/save_camera_regs/
clear_camera_regs/get_camera_overrides/exit) against in-memory state, and lets a test push
an arbitrary event (e.g. imu_cal_preview) on demand via send_event().
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time

_LEN = struct.Struct("<I")
_MSG_JSON = 0x01
_MSG_IMAGE = 0x02


class FakeControlDevice:
    def __init__(self, port: int, state: str = "idle", imu_available: bool = False,
                reg_dump_delay: float = 0.0):
        self.state = state
        self.imu_available = imu_available
        # Artificial per-call delay before answering reg_dump -- widens the window for
        # tests that need to reliably provoke a race around App.startup_backup()'s ~9s
        # real-hardware reg_dump (see test_control.py's concurrent-enter() regression test).
        self.reg_dump_delay = reg_dump_delay
        self.received: list[dict] = []
        self.saved_overrides: dict[int, int] = {}
        self._seq = 0
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

    def _send_image(self, rid, data: bytes = b"\xff\xd8\xff\xd9") -> None:
        meta = json.dumps({"seq": self._seq, "ts_us": 1, "frame_ts_us": 1, "fmt": "jpeg",
                           "w": 800, "h": 600, "len": len(data), "size": "svga", "tag": ""}).encode()
        self._seq += 1
        body = bytes([_MSG_IMAGE]) + struct.pack("<I", len(meta)) + meta + data
        with self._lock:
            self._sock.sendall(_LEN.pack(len(body)) + body)

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
            self._send({"id": rid, "ok": True, "state": self.state, "uptime_us": 42,
                       "heap_free": 111, "psram_free": 222, "imu_available": self.imu_available})
        elif cmd == "set_state":
            self.state = req["state"]
            self._send({"id": rid, "ok": True, "requested": self.state})
        elif cmd == "imu_cal_axis":
            self._send({"id": rid, "ok": True})
        elif cmd == "imu_cal_abort":
            self._send({"id": rid, "ok": True})
        elif self.state != "camera_calibration" and cmd in (
                "info", "set_mode", "reg_read", "reg_write", "reg_dump", "set_orientation",
                "fps_probe", "capture", "save_camera_regs", "clear_camera_regs",
                "get_camera_overrides", "exit"):
            self._send({"id": rid, "ok": False, "err": "not in camera_calibration"})
        elif cmd == "info":
            self._send({"id": rid, "ok": True, "sensor_pid": 22080, "fmt": "jpeg", "size": "svga",
                       "quality": 6, "uptime_us": 1, "heap_free": 1, "psram_free": 1, "fw_build": "x"})
        elif cmd == "reg_read":
            addr, count = req.get("addr", 0), req.get("count", 1)
            self._send({"id": rid, "ok": True, "addr": addr, "hex": "a3" * count})
        elif cmd == "reg_write":
            results = [{"a": w["a"], "old": 0, "new": w["v"], "rc": 0} for w in req.get("writes", [])]
            self._send({"id": rid, "ok": True, "results": results})
        elif cmd == "reg_dump":
            if self.reg_dump_delay:
                time.sleep(self.reg_dump_delay)
            total = 0
            for a, b in req.get("ranges", []):
                for start in range(a, b + 1, 256):
                    n = min(256, b - start + 1)
                    self._send({"evt": "reg_chunk", "id": rid, "start": start, "hex": "00" * n})
                    total += n
            self._send({"id": rid, "ok": True, "regs": total, "ms": 1})
        elif cmd == "set_orientation":
            self._send({"id": rid, "ok": True, "old_3820": 0, "old_3821": 0,
                       "new_3820": 0, "new_3821": 0, "new_4514": 0})
        elif cmd == "fps_probe":
            self._send({"id": rid, "ok": True, "ts": [0, 40000, 80000]})
        elif cmd == "capture":
            n = req.get("n", 1)
            for _ in range(n):
                self._send_image(rid)
            self._send({"id": rid, "ok": True, "captured": n})
        elif cmd == "save_camera_regs":
            for w in req.get("writes", []):
                self.saved_overrides[w["a"]] = w["v"]
            self._send({"id": rid, "ok": True, "saved_this_call": len(req.get("writes", [])),
                       "total_saved": len(self.saved_overrides)})
        elif cmd == "clear_camera_regs":
            self.saved_overrides.clear()
            self._send({"id": rid, "ok": True})
        elif cmd == "get_camera_overrides":
            regs = [{"a": a, "v": v} for a, v in self.saved_overrides.items()]
            self._send({"id": rid, "ok": True, "regs": regs})
        elif cmd == "exit":
            self.state = "idle"
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
