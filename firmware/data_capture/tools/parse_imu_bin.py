#!/usr/bin/env python3
"""Decode raw IMU .bin files written by STREAM_SDCARD to <session>/imu/.

Standalone (no dependency on software/host_server) so it can be run directly
against files copied off the SD card:

    python3 parse_imu_bin.py 20260708-153045/imu/imu_000012_0045210.bin
    python3 parse_imu_bin.py 20260708-153045/imu/ -o imu.csv

Each file is one batch drained from imu_queue by sdcard.c's
imu_sdcard_consumer_task, written byte-for-byte in the same wire format
STREAM_WIFI POSTs to /imu (see firmware/data_capture/main/imu.h and
software/host_server/wire.py, which this mirrors):

    header (16 bytes, little-endian): uint32 num_samples
                                       int64  ref_esp_us       esp_timer us at drain time
                                       uint32 ref_sensor_ticks BMI270 tick at drain time
    sample (28 bytes, little-endian): uint32 sens_time         BMI270 tick when sampled
                                       float  ax, ay, az        accelerometer (g)
                                       float  gx, gy, gz        gyroscope (deg/s)

Each sample's esp_timer timestamp is reconstructed as:

    sample_esp_us = ref_esp_us + wrap24(sens_time - ref_sensor_ticks) * 39.0625

where wrap24 interprets the tick delta as a signed 24-bit value (the BMI270
sensor clock is 24 bits and wraps at 2**24 ticks).
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from pathlib import Path

SENSORTIME_US = 39.0625  # BMI270 tick resolution (us) -- must match IMU_SENSORTIME_US in imu.h

_HEADER = struct.Struct("<IqI")   # num_samples, ref_esp_us, ref_sensor_ticks
_SAMPLE = struct.Struct("<Iffffff")

HEADER_LEN = _HEADER.size  # 16
SAMPLE_LEN = _SAMPLE.size  # 28

_TICK_MASK = 0xFFFFFF  # 24-bit sensor clock
_TICK_HALF = 0x800000


def _wrap24(delta: int) -> int:
    """Interpret a raw tick delta as a signed 24-bit value."""
    delta &= _TICK_MASK
    if delta >= _TICK_HALF:
        delta -= 0x1000000
    return delta


def parse_imu_bin(data: bytes) -> list[tuple[int, float, float, float, float, float, float]]:
    """Decode one imu_*.bin file's bytes into (esp_us, ax, ay, az, gx, gy, gz) tuples."""
    if len(data) < HEADER_LEN:
        raise ValueError(f"file too short: {len(data)} < {HEADER_LEN}")

    num, ref_esp_us, ref_ticks = _HEADER.unpack_from(data, 0)
    expected = HEADER_LEN + num * SAMPLE_LEN
    if len(data) < expected:
        raise ValueError(
            f"file truncated: got {len(data)} bytes, expected {expected} for {num} samples"
        )

    rows = []
    off = HEADER_LEN
    for _ in range(num):
        sens_time, ax, ay, az, gx, gy, gz = _SAMPLE.unpack_from(data, off)
        off += SAMPLE_LEN
        esp_us = ref_esp_us + round(_wrap24(sens_time - ref_ticks) * SENSORTIME_US)
        rows.append((esp_us, ax, ay, az, gx, gy, gz))
    return rows


def _iter_bin_files(path: Path):
    if path.is_dir():
        yield from sorted(path.glob("imu_*.bin"))
    else:
        yield path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("path", type=Path, help="a single imu_*.bin file, or a directory containing them")
    ap.add_argument("-o", "--output", type=Path, default=None,
                     help="write CSV here instead of stdout")
    args = ap.parse_args(argv)

    files = list(_iter_bin_files(args.path))
    if not files:
        print(f"no imu_*.bin files found under {args.path}", file=sys.stderr)
        return 1

    out_fh = args.output.open("w", newline="") if args.output else sys.stdout
    try:
        writer = csv.writer(out_fh)
        writer.writerow(["esp_us", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"])
        total = 0
        for f in files:
            for row in parse_imu_bin(f.read_bytes()):
                writer.writerow(row)
                total += 1
        print(f"decoded {total} samples from {len(files)} file(s)", file=sys.stderr)
    finally:
        if args.output:
            out_fh.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
