"""In-process stand-in for firmware/data_capture cam_calib.c, for host-side tests.

Speaks the same framed protocol (see host_server/calibration/link.py), keeps a fake register
file, and renders a checkerboard with KNOWN intrinsics from a sequence of poses so the
calibration flow can be checked against ground truth without hardware.
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time

import cv2
import numpy as np

W, H = 800, 600
K_TRUE = np.array([[700.0, 0, 402.0], [0, 695.0, 297.0], [0, 0, 1]])
DIST_TRUE = np.array([-0.12, 0.03, 0.001, -0.0008, 0.0])


def render_board(rvec, tvec, cols=9, rows=6, sq=0.025) -> np.ndarray:
    """Grey image of a (cols+1)x(rows+1)-square checkerboard seen from pose (rvec, tvec), with lens distortion."""
    u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    pix = np.stack([u.ravel(), v.ravel()], 1).reshape(-1, 1, 2)
    norm = cv2.undistortPoints(pix, K_TRUE, DIST_TRUE).reshape(-1, 2)          # normalised rays z=1
    R, _ = cv2.Rodrigues(rvec)
    n = R[:, 2]                                                                 # plane normal (cam frame)
    o = tvec.reshape(3)                                                         # plane origin (cam frame)
    rays = np.hstack([norm, np.ones((len(norm), 1))])
    denom = rays @ n
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (o @ n) / denom
    pts = rays * s[:, None]
    local = (pts - o) @ R                                                       # board coords (x,y,~0)
    bx, by = local[:, 0] / sq + 1, local[:, 1] / sq + 1                          # squares; border of one square
    inside = (bx >= 0) & (bx < cols + 1) & (by >= 0) & (by < rows + 1) & (s > 0)
    img = np.full(H * W, 235.0)
    chk = ((np.floor(bx) + np.floor(by)) % 2 == 0)
    img[inside] = np.where(chk[inside], 25.0, 225.0)
    img = img.reshape(H, W).astype(np.uint8)
    return cv2.GaussianBlur(img, (0, 0), 0.8)


def pose_sequence(n: int, seed: int = 1):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        rvec = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-0.3, 0.3)])
        # put the board centre roughly in view at 0.45..0.8 m
        z = rng.uniform(0.45, 0.8)
        tvec = np.array([rng.uniform(-0.12, 0.02) - 0.11, rng.uniform(-0.08, 0.02) - 0.075, z])
        yield rvec, tvec


class FakeDevice:
    def __init__(self, port: int, n_poses: int = 40):
        self.port = port
        self.regs = {a: 0 for r in ((0x3000, 0x3FFF), (0x4000, 0x4FFF), (0x5000, 0x5FFF)) for a in range(r[0], r[1] + 1)}
        self.regs[0x3820], self.regs[0x3821] = 0x01, 0x21
        self.mode = {"fmt": "jpeg", "size": "svga"}
        self.poses = list(pose_sequence(n_poses))
        self.pose_i = 0
        self.stop = threading.Event()
        self.exit_requested = False
        self.captured = 0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    # -- wire ----------------------------------------------------------------
    @staticmethod
    def _frame(kind: int, body: bytes) -> bytes:
        return struct.pack("<I", len(body) + 1) + bytes([kind]) + body

    def _send_json(self, conn, obj) -> None:
        conn.sendall(self._frame(1, json.dumps(obj).encode()))

    def _recv(self, conn, n):
        buf = b""
        while len(buf) < n:
            c = conn.recv(n - len(buf))
            if not c:
                raise ConnectionError
            buf += c
        return buf

    def _run(self) -> None:
        """Like cam_calib.c: dial the host (retrying) and serve until told to exit."""
        while not self.stop.is_set() and not self.exit_requested:
            try:
                conn = socket.create_connection(("127.0.0.1", self.port), timeout=1)
            except OSError:
                time.sleep(0.1)
                continue
            with conn:
                conn.settimeout(5)
                self._send_json(conn, {"evt": "hello", "mode": "camera_calibration", "sensor_pid": 0x5640, **self.mode})
                try:
                    while not self.stop.is_set():
                        (ln,) = struct.unpack("<I", self._recv(conn, 4))
                        body = self._recv(conn, ln)
                        if self._handle(conn, json.loads(body[1:])):
                            break
                except (ConnectionError, socket.timeout, OSError):
                    pass

    # -- commands --------------------------------------------------------------
    def _handle(self, conn, req) -> bool:
        cmd, rid = req["cmd"], req["id"]
        ok = lambda **kv: self._send_json(conn, {"id": rid, "ok": True, **kv})
        if cmd == "ping":
            ok()
        elif cmd in ("info", "set_mode"):
            if cmd == "set_mode":
                self.mode = {"fmt": req.get("format", self.mode["fmt"]), "size": req.get("framesize", self.mode["size"])}
            ok(sensor_pid=0x5640, quality=6, uptime_us=int(time.monotonic() * 1e6), **self.mode)
        elif cmd == "reg_read":
            ok(addr=req["addr"], hex="".join(f"{self.regs.get(req['addr'] + i, 0):02x}" for i in range(req.get("count", 1))))
        elif cmd == "reg_write":
            res = []
            for w in req["writes"]:
                m = w.get("m", 255)
                old = self.regs[w["a"]]
                self.regs[w["a"]] = (old & ~m) | (w["v"] & m)
                res.append({"a": w["a"], "old": old, "new": self.regs[w["a"]], "rc": 0})
            ok(results=res)
        elif cmd == "reg_dump":
            tot = 0
            for a, b in req["ranges"]:
                for s in range(a, b + 1, 256):
                    n = min(256, b - s + 1)
                    self._send_json(conn, {"evt": "reg_chunk", "id": rid, "start": s,
                                           "hex": "".join(f"{self.regs.get(s + i, 0):02x}" for i in range(n))})
                    tot += n
            ok(regs=tot, ms=1)
        elif cmd == "set_orientation":
            o20, o21 = self.regs[0x3820], self.regs[0x3821]
            self.regs[0x3820] = (o20 & ~6) | (6 if req["flip"] else 0)
            self.regs[0x3821] = (o21 & ~6) | (6 if req["mirror"] else 0)
            ok(old_3820=o20, old_3821=o21, new_3820=self.regs[0x3820], new_3821=self.regs[0x3821], new_4514=0)
        elif cmd == "fps_probe":
            ok(ts=[int(i * 45000) for i in range(req.get("n", 30))])
        elif cmd == "capture":
            for i in range(req.get("n", 1)):
                rvec, tvec = self.poses[self.pose_i % len(self.poses)]
                self.pose_i += 1
                img = render_board(rvec, tvec)
                if self.mode["fmt"] == "jpeg":
                    data = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
                else:
                    data = img.tobytes()
                meta = {"seq": self.captured, "ts_us": int(time.monotonic() * 1e6), "frame_ts_us": 0,
                        "fmt": self.mode["fmt"], "w": W, "h": H, "len": len(data), "size": "svga", "tag": req.get("tag", "")}
                if req.get("regs", "key") != "none":
                    meta["regs"] = {"3500": 0}
                mj = json.dumps(meta).encode()
                conn.sendall(self._frame(2, struct.pack("<I", len(mj)) + mj + data))
                self.captured += 1
            ok(captured=req.get("n", 1))
        elif cmd == "exit":
            ok()
            self.exit_requested = True
            return True
        else:
            self._send_json(conn, {"id": rid, "ok": False, "err": "unknown cmd"})
        return False

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=2)
