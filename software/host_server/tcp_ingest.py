"""Raw-TCP ingest listener for STREAM_TCP mode (firmware/data_capture/main/tcp_client.c).

The firmware normally streams over HTTP POST (server.py). STREAM_TCP is a
lower-overhead alternative: one persistent TCP connection carrying all three
streams (camera frames, IMU batches, sysstats snapshots), multiplexed by a
small header per message -- no HTTP request/response per frame.

Wire framing (little-endian, no padding -- keep in sync with tcp_client.h):

    uint8_t  type;     MSG_FRAME=1 / MSG_IMU=2 / MSG_STATS=3
    uint32_t len;      payload length that follows
    uint8_t  payload[len]

Payloads:
    FRAME  -- int64 ts_us + JPEG bytes
    IMU    -- same layout wire.py.parse_imu_payload already decodes
    STATS  -- same layout wire.py.parse_stats_payload already decodes

This module feeds the same in-memory Hub and BagRecorder the HTTP ingest path
(server.py) uses, so frames/samples arriving over either transport show up
identically in the browser view and the recorded bag.
"""

from __future__ import annotations

import queue
import socket
import struct
import threading

from .bag_recorder import BagRecorder
from .hub import Hub
from .wire import parse_imu_payload, parse_stats_payload

_HEADER = struct.Struct("<BI")   # type, payload len
MSG_FRAME = 1
MSG_IMU = 2
MSG_STATS = 3

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


def _bag_writer_loop(write_queue: "queue.Queue") -> None:
    """Drain queued recorder writes off the network thread.

    rosbag2's SequentialWriter (sqlite3 backend) does a synchronous disk write
    per message -- fast most of the time, but occasionally stalls for 100+ ms
    (fsync/disk contention). Calling it straight from _handle_connection's recv
    loop meant that stall left the socket unread; the ESP32 sender then blocks
    in send() once the kernel's TCP send buffer fills up from the backpressure,
    which is what showed up firmware-side as wildly variable "camera sending
    took Xms" times. Running the write here, off the recv loop, means a slow
    write only delays the bag, never the network read.
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


def _handle_connection(conn: socket.socket, addr, hub: Hub, recorder: BagRecorder,
                       write_queue: "queue.Queue") -> None:
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"[host_server] tcp ingest: {addr} connected")
    try:
        with conn:
            while True:
                hdr = _recv_exact(conn, _HEADER.size)
                if hdr is None:
                    return
                msg_type, length = _HEADER.unpack(hdr)
                payload = _recv_exact(conn, length)
                if payload is None:
                    return

                if msg_type == MSG_FRAME:
                    if len(payload) < 8:
                        continue
                    ts_us = struct.unpack_from("<q", payload, 0)[0]
                    jpeg = payload[8:]
                    hub.put_frame(ts_us, jpeg)
                    _enqueue_write(write_queue, recorder.write_frame, ts_us, jpeg)
                elif msg_type == MSG_IMU:
                    try:
                        samples = parse_imu_payload(payload)
                    except ValueError:
                        continue
                    hub.put_imu(samples)
                    for s in samples:
                        _enqueue_write(write_queue, recorder.write_imu, s)
                elif msg_type == MSG_STATS:
                    try:
                        stats = parse_stats_payload(payload)
                    except ValueError:
                        continue
                    hub.put_stats(stats)
                    _enqueue_write(write_queue, recorder.write_stats, stats)
                # unknown type: length-prefix already let us skip the payload above
    finally:
        print(f"[host_server] tcp ingest: {addr} disconnected")


def serve_tcp_ingest(host: str, port: int, hub: Hub, recorder: BagRecorder,
                     stop: threading.Event) -> None:
    """Accept STREAM_TCP connections and feed `hub`/`recorder` until `stop` is set.

    The rig only ever runs one device at a time, but connections are handled
    on their own thread so a reconnect (e.g. after a mode switch on the
    firmware) doesn't need any special-casing here. One background thread
    (shared across reconnects) drains `write_queue` into `recorder` -- see
    _bag_writer_loop for why that's not done inline in the recv loop.
    """
    write_queue: "queue.Queue" = queue.Queue(maxsize=_WRITE_QUEUE_LEN)
    writer = threading.Thread(target=_bag_writer_loop, args=(write_queue,), daemon=True)
    writer.start()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    srv.settimeout(1.0)
    print(f"[host_server] tcp ingest listening on {host}:{port}")
    try:
        while not stop.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            threading.Thread(target=_handle_connection,
                             args=(conn, addr, hub, recorder, write_queue),
                             daemon=True).start()
    finally:
        srv.close()
        write_queue.put(None)
        writer.join(timeout=2.0)
