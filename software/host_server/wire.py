"""Decode the binary wire formats the ESP32-S3 firmware sends.

The firmware (``firmware/data_capture``) pushes two payloads to this server
when built with ``CONFIG_ENABLE_REMOTE_STREAM``:

* ``POST /frame`` — a JPEG body plus an ``X-Timestamp-Us`` header (esp_timer
  microseconds-since-boot captured at frame grab).
* ``POST /imu`` — the binary IMU ring drain described below.

IMU payload (little-endian, no padding — the firmware builds it with
sequential ``memcpy`` in ``imu.c::imu_serialize``):

    header (16 bytes):  uint32 num_samples
                        int64  ref_esp_us         esp_timer us at drain time
                        uint32 ref_sensor_ticks   BMI270 tick at drain time
    sample (28 bytes):  uint32 sens_time          BMI270 tick when sampled
                        float  ax, ay, az         accelerometer (g)
                        float  gx, gy, gz          gyroscope (deg/s)

Each sample's esp_timer timestamp is reconstructed on this side as:

    sample_esp_us = ref_esp_us + wrap24(sens_time - ref_sensor_ticks) * 39.0625

where ``wrap24`` interprets the tick delta as a signed 24-bit value (the BMI270
sensor clock is 24 bits and wraps at 2**24 ticks). This is the mechanism that
keeps camera and IMU samples on one shared clock — keep it in sync with the
firmware's ``imu.h``.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

# BMI270 sensor-time resolution — must match IMU_SENSORTIME_US in imu.h.
SENSORTIME_US = 39.0625

_HEADER = struct.Struct("<IqI")   # num_samples, ref_esp_us, ref_sensor_ticks
_SAMPLE = struct.Struct("<Iffffff")

HEADER_LEN = _HEADER.size          # 16
SAMPLE_LEN = _SAMPLE.size          # 28

_TICK_MASK = 0xFFFFFF              # 24-bit sensor clock
_TICK_HALF = 0x800000


def _wrap24(delta: int) -> int:
    """Interpret a raw tick delta as a signed 24-bit value."""
    delta &= _TICK_MASK
    if delta >= _TICK_HALF:
        delta -= 0x1000000
    return delta


@dataclass
class ImuSample:
    esp_us: int          # reconstructed esp_timer microseconds
    ax: float            # accelerometer (g)
    ay: float
    az: float
    gx: float            # gyroscope (deg/s)
    gy: float
    gz: float


# System-stats payload (little-endian, no padding — built field-by-field in
# firmware/data_capture/main/sysstats.c::sysstats_serialize). Keep in sync with
# SYSSTATS_WIRE_LEN / sysstats_snapshot_t in sysstats.h.
_STATS = struct.Struct("<qfffiIIIIIIIIII")
STATS_LEN = _STATS.size            # 64


@dataclass
class SysStats:
    esp_us: int              # esp_timer µs at snapshot (shared clock)
    cpu0_load: float         # core 0 load over last interval (%)
    cpu1_load: float         # core 1 load (%)
    chip_temp_c: float       # internal temperature sensor (°C)
    wifi_rssi: int           # associated AP RSSI (dBm); 0 if unknown
    uptime_s: int            # seconds since boot
    heap_free: int           # total free heap (bytes, all caps)
    heap_min_free: int       # minimum-ever free heap (bytes)
    int_free: int            # internal DRAM free (bytes)
    int_largest: int         # largest free internal block (bytes)
    int_total: int           # internal DRAM total (bytes)
    psram_free: int          # SPIRAM free (bytes; 0 if no PSRAM)
    psram_min_free: int      # SPIRAM minimum-ever free (bytes)
    psram_largest: int       # largest free SPIRAM block (bytes)
    psram_total: int         # SPIRAM total (bytes)


def parse_stats_payload(body: bytes) -> SysStats:
    """Decode a ``/stats`` payload into a :class:`SysStats` snapshot.

    Raises ``ValueError`` if the payload is the wrong length.
    """
    if len(body) < STATS_LEN:
        raise ValueError(f"stats payload too short: {len(body)} < {STATS_LEN}")
    return SysStats(*_STATS.unpack_from(body, 0))


def parse_imu_payload(body: bytes) -> list[ImuSample]:
    """Decode an ``/imu`` payload into timestamped samples (oldest first).

    Returns an empty list for a well-formed drain that carried no samples, and
    raises ``ValueError`` if the payload is truncated or malformed.
    """
    if len(body) < HEADER_LEN:
        raise ValueError(f"IMU payload too short: {len(body)} < {HEADER_LEN}")

    num, ref_esp_us, ref_ticks = _HEADER.unpack_from(body, 0)
    expected = HEADER_LEN + num * SAMPLE_LEN
    if len(body) < expected:
        raise ValueError(
            f"IMU payload truncated: got {len(body)} bytes, "
            f"expected {expected} for {num} samples"
        )

    samples: list[ImuSample] = []
    off = HEADER_LEN
    for _ in range(num):
        sens_time, ax, ay, az, gx, gy, gz = _SAMPLE.unpack_from(body, off)
        off += SAMPLE_LEN
        esp_us = ref_esp_us + round(_wrap24(sens_time - ref_ticks) * SENSORTIME_US)
        samples.append(ImuSample(esp_us, ax, ay, az, gx, gy, gz))
    return samples
