"""Entry point: ``python3 -m host_server.control [options]`` (run from ``software/``)."""

from __future__ import annotations

import argparse
import signal
import sys
import threading

from .app import make_server
from .device_channel import ControlLink


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="host_server.control",
        description="Always-on browser control page for the ESP32-S3 SLAM rig: "
                    "streaming/calibration mode toggle + host-driven IMU calibration.",
    )
    ap.add_argument("--device-port", type=int, default=8085,
                    help="control channel port the device dials (must match "
                         "CONFIG_CONTROL_PORT; default 8085)")
    ap.add_argument("--http-host", default="0.0.0.0", help="browser page bind address")
    ap.add_argument("--http-port", type=int, default=8091, help="browser page port")
    args = ap.parse_args(argv)

    link = ControlLink("0.0.0.0", args.device_port)
    server = make_server(args.http_host, args.http_port, link)
    print(f"[host_server.control] waiting for the device on :{args.device_port}")
    print(f"[host_server.control] open http://localhost:{args.http_port}/ in a browser")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        stop.wait()
    finally:
        print("\n[host_server.control] shutting down…")
        server.shutdown()
        server.server_close()
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
