"""Entry point: ``python3 -m host_server [options]`` (run from ``software/``)."""

from __future__ import annotations

import argparse
import datetime as _dt
import signal
import sys
import threading

from .bag_recorder import BagRecorder, ros_available, ros_import_error
from .hub import Hub
from .server import make_server
from .tcp_ingest import serve_tcp_ingest


def _default_bag_uri() -> str:
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"bags/slam_{stamp}"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="host_server",
        description="Receive ESP32-S3 camera + IMU stream, record to a ROS 2 "
                    "bag, and show a live browser view.",
    )
    ap.add_argument("--host", default="0.0.0.0",
                    help="bind address (default: all interfaces)")
    ap.add_argument("--port", type=int, default=8080,
                    help="listen port (must match CONFIG_REMOTE_PORT; default 8080)")
    ap.add_argument("--tcp-frame-port", type=int, default=8081,
                    help="raw-TCP STREAM_TCP frame ingest port (must match "
                         "CONFIG_REMOTE_TCP_FRAME_PORT; default 8081)")
    ap.add_argument("--tcp-imu-port", type=int, default=8082,
                    help="raw-TCP STREAM_TCP IMU ingest port (must match "
                         "CONFIG_REMOTE_TCP_IMU_PORT; default 8082)")
    ap.add_argument("--tcp-stats-port", type=int, default=8083,
                    help="raw-TCP STREAM_TCP stats ingest port (must match "
                         "CONFIG_REMOTE_TCP_STATS_PORT; default 8083)")
    ap.add_argument("--bag", default=None,
                    help="rosbag2 output directory (default: bags/slam_<timestamp>)")
    ap.add_argument("--storage", default="sqlite3", choices=["sqlite3", "mcap"],
                    help="rosbag2 storage backend (default: sqlite3)")
    ap.add_argument("--no-record", action="store_true",
                    help="live view only; do not write a bag")
    args = ap.parse_args(argv)

    hub = Hub()

    if args.no_record:
        recorder = BagRecorder.__new__(BagRecorder)
        recorder.enabled = False
        recorder.bag_uri = ""
        print("[host_server] recording disabled (--no-record)")
    else:
        bag_uri = args.bag or _default_bag_uri()
        recorder = BagRecorder(bag_uri, storage_id=args.storage)
        if recorder.enabled:
            print(f"[host_server] recording to rosbag2: {bag_uri} ({args.storage})")
        else:
            print("[host_server] ROS 2 not available — recording disabled, "
                  "serving live view only")
            print(f"[host_server]   import error: {ros_import_error()}")
            print("[host_server]   (source /opt/ros/<distro>/setup.bash to enable)")

    server = make_server(args.host, args.port, hub, recorder)
    print(f"[host_server] listening on http://{args.host}:{args.port}  "
          f"(open http://localhost:{args.port}/ in a browser)")
    print(f"[host_server] ROS 2 available: {ros_available()}")

    # Run the blocking HTTP server and the raw-TCP ingest listener (STREAM_TCP
    # mode) each on their own worker thread, sharing `hub`/`recorder`, so the
    # main thread is free to call their shutdown methods from the signal
    # handler -- calling shutdown() from within a server's own serving thread
    # deadlocks.
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    http_worker = threading.Thread(target=server.serve_forever, daemon=True)
    http_worker.start()
    tcp_worker = threading.Thread(target=serve_tcp_ingest,
                                  args=(args.host, args.tcp_frame_port, args.tcp_imu_port,
                                        args.tcp_stats_port, hub, recorder, stop),
                                  daemon=True)
    tcp_worker.start()
    try:
        stop.wait()
    finally:
        print("\n[host_server] shutting down…")
        server.shutdown()
        server.server_close()
        tcp_worker.join(timeout=2.0)
        recorder.close()
        print("[host_server] bag finalized, sockets closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
