"""Entry point: ``python3 -m host_server [options]`` (run from ``software/``).

One command, one URL: this is the whole host side of the rig -- mode/streaming control,
calibration (register console, ISP tuning steps, checkerboard intrinsics), ROS bag
recording, all from the one browser page this serves. See docs/architecture.md "Control
channel" and docs/camera_calibration_and_tuning.md.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
from pathlib import Path

from .app import Server, make_server
from .bag_recorder import ros_available, ros_import_error


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="host_server",
        description="ESP32-S3 SLAM rig: the one host-side command -- streaming, calibration, "
                    "and ROS bag recording, all from one browser page.",
    )
    ap.add_argument("--http-host", default="0.0.0.0", help="browser page bind address")
    ap.add_argument("--http-port", type=int, default=8080,
                    help="browser page + streaming-ingest port (must match CONFIG_REMOTE_PORT "
                         "for STREAM_WIFI; default 8080)")
    ap.add_argument("--device-port", type=int, default=8085,
                    help="device control channel port (must match CONFIG_CONTROL_PORT; default 8085)")
    ap.add_argument("--tcp-host", default="0.0.0.0", help="bind address for the on-demand STREAM_TCP ports")
    ap.add_argument("--tcp-frame-port", type=int, default=8081,
                    help="STREAM_TCP frame ingest port (must match CONFIG_REMOTE_TCP_FRAME_PORT)")
    ap.add_argument("--tcp-imu-port", type=int, default=8082,
                    help="STREAM_TCP IMU ingest port (must match CONFIG_REMOTE_TCP_IMU_PORT)")
    ap.add_argument("--tcp-stats-port", type=int, default=8083,
                    help="STREAM_TCP stats ingest port (must match CONFIG_REMOTE_TCP_STATS_PORT)")
    ap.add_argument("--calib-root", help="calibration session root (default software/calib_data)")
    args = ap.parse_args(argv)

    state = Server(
        device_port=args.device_port,
        calib_root=Path(args.calib_root) if args.calib_root else None,
        tcp_host=args.tcp_host,
        tcp_ports=(args.tcp_frame_port, args.tcp_imu_port, args.tcp_stats_port),
    )
    server = make_server(args.http_host, args.http_port, state)

    print(f"[host_server] open http://localhost:{args.http_port}/ in a browser")
    print(f"[host_server] waiting for the device on control port :{args.device_port}")
    print(f"[host_server] STREAM_WIFI ingest on :{args.http_port} (always listening); "
          f"STREAM_TCP ingest on :{args.tcp_frame_port}-{args.tcp_stats_port} (opened on demand)")
    print(f"[host_server] ROS 2 available: {ros_available()}" +
          ("" if ros_available() else f"  ({ros_import_error()})"))

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        stop.wait()
    finally:
        print("\n[host_server] shutting down…")
        server.shutdown()
        server.server_close()
        state.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
