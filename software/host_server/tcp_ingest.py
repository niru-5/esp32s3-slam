"""Raw-TCP ingest listeners for STREAM_TCP mode (firmware/data_capture/main/tcp_client.c).

The firmware normally streams over HTTP POST (server.py). STREAM_TCP is a
lower-overhead alternative: three independent persistent connections, one per
stream (camera frames, IMU batches, sysstats snapshots), each on its own port
-- no per-message HTTP request/response, and (since each connection carries
only one message type) no per-message type tag either, just a length prefix.

Wire framing per connection (little-endian, no padding -- keep in sync with
tcp_client.h):

    uint32_t len;      payload length that follows
    uint8_t  payload[len]

Payloads:
    frame connection  -- int64 ts_us + JPEG bytes
    imu connection    -- same layout wire.py.parse_imu_payload already decodes
    stats connection  -- same layout wire.py.parse_stats_payload already decodes

This module feeds the same in-memory Hub and BagRecorder the HTTP ingest path
(server.py) uses, so frames/samples arriving over either transport show up
identically in the browser view and the recorded bag.
"""

from __future__ import annotations

import queue
import socket
import struct
import threading
from typing import Callable, Iterator

from .bag_recorder import BagRecorder
from .hub import Hub
from .wire import parse_imu_payload, parse_stats_payload

_LEN_PREFIX = struct.Struct("<I")   # payload length, little-endian

# Depth of the async bag-write queue -- generous vs. the firmware's few-fps
# camera rate, so a brief slow write doesn't immediately start dropping data.
_WRITE_QUEUE_LEN = 128


def _recv_exact(conn: socket.socket, n: int) -> bytes | None:
    """Read exactly `n` bytes, or None if the connection closed first."""
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def _read_messages(conn: socket.socket) -> Iterator[bytes]:
    """Yield successive length-prefixed payloads from `conn` until it closes."""
    while True:
        hdr = _recv_exact(conn, _LEN_PREFIX.size)
        if hdr is None:
            return
        (length,) = _LEN_PREFIX.unpack(hdr)
        payload = _recv_exact(conn, length)
        if payload is None:
            return
        yield payload


def _bag_writer_loop(write_queue: "queue.Queue") -> None:
    """Drain queued recorder writes off the network threads.

    rosbag2's SequentialWriter (sqlite3 backend) does a synchronous disk write
    per message -- fast most of the time, but occasionally stalls for 100+ ms
    (fsync/disk contention). Calling it straight from a connection's recv loop
    meant that stall left the socket unread; the ESP32 sender then blocks in
    send() once the kernel's TCP send buffer fills up from the backpressure,
    which showed up firmware-side as wildly variable "camera sending took
    Xms" times. Running the write here, off the recv loops, means a slow
    write only delays the bag, never a network read.
    """
    while True:
        item = write_queue.get()
        if item is None:  # sentinel -> shut down
            return
        write_fn, args = item
        try:
            write_fn(*args)
        except Exception as exc:  # keep the writer thread alive across a bad record
            print(f"[host_server] tcp ingest: bag write failed: {exc}")


def _enqueue_write(write_queue: "queue.Queue", write_fn, *args) -> None:
    try:
        write_queue.put_nowait((write_fn, args))
    except queue.Full:
        # Recording can't keep up -- drop the oldest queued write to make
        # room, mirroring the firmware queues' own drop-oldest overflow
        # policy, rather than blocking (which would reintroduce the same
        # backpressure onto the recv loop this queue exists to avoid).
        try:
            write_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            write_queue.put_nowait((write_fn, args))
        except queue.Full:
            pass


def _frame_handler(hub: Hub, recorder: BagRecorder, write_queue: "queue.Queue"):
    def handle(conn: socket.socket, addr) -> None:
        for payload in _read_messages(conn):
            if len(payload) < 8:
                continue
            ts_us = struct.unpack_from("<q", payload, 0)[0]
            jpeg = payload[8:]
            hub.put_frame(ts_us, jpeg)
            _enqueue_write(write_queue, recorder.write_frame, ts_us, jpeg)
    return handle


def _imu_handler(hub: Hub, recorder: BagRecorder, write_queue: "queue.Queue"):
    def handle(conn: socket.socket, addr) -> None:
        for payload in _read_messages(conn):
            try:
                samples = parse_imu_payload(payload)
            except ValueError:
                continue
            hub.put_imu(samples)
            for s in samples:
                _enqueue_write(write_queue, recorder.write_imu, s)
    return handle


def _stats_handler(hub: Hub, recorder: BagRecorder, write_queue: "queue.Queue"):
    def handle(conn: socket.socket, addr) -> None:
        for payload in _read_messages(conn):
            try:
                stats = parse_stats_payload(payload)
            except ValueError:
                continue
            hub.put_stats(stats)
            _enqueue_write(write_queue, recorder.write_stats, stats)
    return handle


def _serve_stream(host: str, port: int, label: str, stop: threading.Event,
                  on_connection: Callable[[socket.socket, object], None]) -> None:
    """Accept connections on `port` until `stop` is set, running `on_connection`
    (already wrapped with TCP_NODELAY + connect/disconnect logging) on its own
    thread per connection."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    srv.settimeout(1.0)
    print(f"[host_server] tcp {label} ingest listening on {host}:{port}")

    def run_connection(conn: socket.socket, addr) -> None:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"[host_server] tcp {label}: {addr} connected")
        try:
            with conn:
                on_connection(conn, addr)
        finally:
            print(f"[host_server] tcp {label}: {addr} disconnected")

    try:
        while not stop.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            threading.Thread(target=run_connection, args=(conn, addr), daemon=True).start()
    finally:
        srv.close()


def serve_tcp_ingest(host: str, frame_port: int, imu_port: int, stats_port: int,
                     hub: Hub, recorder: BagRecorder, stop: threading.Event) -> None:
    """Run the three STREAM_TCP listeners (frame/imu/stats) until `stop` is set.

    The rig only ever runs one device at a time, but each stream gets its own
    listener/port -- matching tcp_client.c's one-connection-per-stream design
    (see its module docstring for why: no cross-stream mutex, no head-of-line
    blocking between a large frame send and a small time-critical IMU/stats
    send). One background thread (shared across all three streams and any
    reconnects) drains `write_queue` into `recorder` -- see _bag_writer_loop.
    """
    write_queue: "queue.Queue" = queue.Queue(maxsize=_WRITE_QUEUE_LEN)
    writer = threading.Thread(target=_bag_writer_loop, args=(write_queue,), daemon=True)
    writer.start()

    listeners = [
        threading.Thread(target=_serve_stream,
                         args=(host, frame_port, "frame", stop, _frame_handler(hub, recorder, write_queue)),
                         daemon=True),
        threading.Thread(target=_serve_stream,
                         args=(host, imu_port, "imu", stop, _imu_handler(hub, recorder, write_queue)),
                         daemon=True),
        threading.Thread(target=_serve_stream,
                         args=(host, stats_port, "stats", stop, _stats_handler(hub, recorder, write_queue)),
                         daemon=True),
    ]
    for t in listeners:
        t.start()

    stop.wait()
    for t in listeners:
        t.join(timeout=2.0)
    write_queue.put(None)
    writer.join(timeout=2.0)
