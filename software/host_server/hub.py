"""Shared in-memory state between the ingest handlers and the browser view.

A single ``Hub`` instance holds the most recent JPEG frame and IMU sample plus
running counters. Ingest handlers (``POST /frame``, ``POST /imu``) publish into
it; the browser endpoints (MJPEG stream, ``/imu.json``) read from it. Access is
guarded so the threaded HTTP server can serve concurrent clients safely.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from .wire import ImuSample, SysStats


@dataclass
class Hub:
    _lock: threading.Lock = field(default_factory=threading.Lock)

    latest_jpeg: bytes | None = None
    latest_frame_us: int = 0
    frame_version: int = 0          # bumped on every new frame
    frame_count: int = 0

    latest_imu: ImuSample | None = None
    imu_count: int = 0

    latest_stats: SysStats | None = None
    stats_count: int = 0

    started_at: float = field(default_factory=time.time)

    def __post_init__(self) -> None:
        self._frame_cond = threading.Condition(self._lock)

    # -- producers ---------------------------------------------------------
    def put_frame(self, esp_us: int, jpeg: bytes) -> None:
        with self._frame_cond:
            self.latest_jpeg = jpeg
            self.latest_frame_us = esp_us
            self.frame_version += 1
            self.frame_count += 1
            self._frame_cond.notify_all()

    def put_imu(self, samples: list[ImuSample]) -> None:
        if not samples:
            return
        with self._lock:
            self.latest_imu = samples[-1]
            self.imu_count += len(samples)

    def put_stats(self, stats: SysStats) -> None:
        with self._lock:
            self.latest_stats = stats
            self.stats_count += 1

    # -- consumers ---------------------------------------------------------
    def wait_frame(self, last_version: int, timeout: float) -> tuple[int, bytes | None]:
        """Block until a frame newer than ``last_version`` arrives (or timeout)."""
        with self._frame_cond:
            if self.frame_version == last_version:
                self._frame_cond.wait(timeout)
            return self.frame_version, self.latest_jpeg

    def snapshot(self) -> dict:
        with self._lock:
            imu = self.latest_imu
            st = self.latest_stats
            uptime = max(time.time() - self.started_at, 1e-6)
            return {
                "frame_count": self.frame_count,
                "frame_fps": round(self.frame_count / uptime, 2),
                "latest_frame_us": self.latest_frame_us,
                "imu_count": self.imu_count,
                "imu_hz": round(self.imu_count / uptime, 1),
                "imu": None if imu is None else {
                    "esp_us": imu.esp_us,
                    "accel_g": {"x": imu.ax, "y": imu.ay, "z": imu.az},
                    "gyro_dps": {"x": imu.gx, "y": imu.gy, "z": imu.gz},
                },
                "stats_count": self.stats_count,
                "stats": None if st is None else {
                    "esp_us": st.esp_us,
                    "cpu0_load": round(st.cpu0_load, 1),
                    "cpu1_load": round(st.cpu1_load, 1),
                    "chip_temp_c": round(st.chip_temp_c, 1),
                    "wifi_rssi": st.wifi_rssi,
                    "device_uptime_s": st.uptime_s,
                    "heap_free": st.heap_free,
                    "heap_min_free": st.heap_min_free,
                    "internal_free": st.int_free,
                    "internal_largest": st.int_largest,
                    "internal_total": st.int_total,
                    "psram_free": st.psram_free,
                    "psram_min_free": st.psram_min_free,
                    "psram_largest": st.psram_largest,
                    "psram_total": st.psram_total,
                },
                "uptime_s": round(uptime, 1),
            }
