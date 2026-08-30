# Host server (`host_server`)

Receives the camera + IMU stream the ESP32-S3 firmware pushes over WiFi,
records it to a **ROS 2 bag**, and serves a **live browser view**.

This is the receiving side of the firmware's remote-stream mode
(`firmware/data_capture` built with `CONFIG_ENABLE_REMOTE_STREAM`). The board
POSTs JPEG frames and binary IMU drains to this server; both are stamped on the
ESP32's shared `esp_timer` clock so camera and IMU stay time-aligned in the bag.

## Requirements

- Python 3.10+ (standard library only — no pip installs, no venv, for the web
  layer: HTTP/TCP ingest, the live viewer, and IMU/stats decoding all use only
  `http.server`, `socketserver`, and friends from the stdlib).
- ROS 2 (developed against **Jazzy**) for bag recording. Source it in the
  *same shell* you'll launch the server from, before running `python3 -m
  host_server`, so `rosbag2_py` + `sensor_msgs` + `diagnostic_msgs` are
  importable:

  ```bash
  source /opt/ros/jazzy/setup.bash
  ```

  Without a ROS environment (or if `source` was skipped, or run from a fresh
  shell that never sourced it) the import at the top of `bag_recorder.py`
  fails, `ros_available()` returns `False`, and the server logs:

  ```
  [host_server] ROS 2 not available — recording disabled, serving live view only
  [host_server]   import error: No module named 'rosbag2_py'
  [host_server]   (source /opt/ros/<distro>/setup.bash to enable)
  ```

  It then still runs as a **live viewer** with no bag written — the same
  state as passing `--no-record` explicitly. This is a soft fallback, not an
  error: if you only want the browser view, there's nothing to fix.
- `--storage mcap` additionally needs the `rosbag2_storage_mcap` ROS package
  installed (`ros-jazzy-rosbag2-storage-mcap` via apt) — `sqlite3` (the
  default) needs nothing beyond a base ROS 2 install.

## Run

From the `software/` directory:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m host_server --port 8080
```

Then open <http://localhost:8080/> in a browser. Point the firmware at this
machine by setting, in `firmware/data_capture/main/config.h`:

```c
#define CONFIG_ENABLE_REMOTE_STREAM 1
#define CONFIG_REMOTE_HOST "<this-machine-ip>"
#define CONFIG_REMOTE_PORT 8080
```

### Options

| Flag | Default | Meaning |
|------|---------|---------|
| `--host` | `0.0.0.0` | Bind address. |
| `--port` | `8080` | HTTP listen port (match `CONFIG_REMOTE_PORT`). |
| `--tcp-frame-port` | `8081` | Raw-TCP `STREAM_TCP` frame ingest port (match `CONFIG_REMOTE_TCP_FRAME_PORT`). |
| `--tcp-imu-port` | `8082` | Raw-TCP `STREAM_TCP` IMU ingest port (match `CONFIG_REMOTE_TCP_IMU_PORT`). |
| `--tcp-stats-port` | `8083` | Raw-TCP `STREAM_TCP` stats ingest port (match `CONFIG_REMOTE_TCP_STATS_PORT`). |
| `--bag`  | `bags/slam_<timestamp>` | rosbag2 output directory. |
| `--storage` | `sqlite3` | rosbag2 backend (`sqlite3` or `mcap`). |
| `--no-record` | off | Live view only, no bag. |

### Network / firewall

The board and the host must be on the same network (the firmware's WiFi is
**STA mode**, joining your existing network rather than hosting its own) — a
board on an isolated guest/IoT VLAN can't reach a host on the main LAN even if
both have internet access.

- Find the host's LAN IP with `ip addr show` or `hostname -I` and put that
  (not `127.0.0.1` / `localhost`) in `CONFIG_REMOTE_HOST` — the ESP32 needs an
  address it can route to, not the host's view of itself.
- `--host 0.0.0.0` (the default) binds all interfaces, but a host firewall can
  still drop the inbound connection silently — the firmware will just fail to
  connect/POST with no server-side log line, since the packets never reach
  the process. On Ubuntu with `ufw` enabled, allow the ports in use:
  ```bash
  sudo ufw allow 8080:8083/tcp
  ```
  (only `8080` if you're not using `STREAM_TCP`).
- If the server is on a laptop, check it isn't asleep/suspended when idle —
  suspend drops the WiFi interface and the board's connection with it.

## STREAM_TCP (raw socket ingest)

`STREAM_TCP` (serial command `6` on the firmware) is a lower-overhead
alternative to `STREAM_WIFI`'s HTTP POSTs: three independent persistent TCP
connections, one per stream (camera frames, IMU batches, stats snapshots),
each on its own port -- no HTTP request/response per message, and (since each
connection only ever carries one message type) no type tag either, just a
4-byte length prefix per message. One connection per stream also means a
large/slow frame send can never head-of-line-block a small time-critical
IMU/stats send behind it, and there's no cross-stream lock on the firmware
side. This server always listens for all three (`--tcp-frame-port`/
`--tcp-imu-port`/`--tcp-stats-port`, `8081`/`8082`/`8083` by default)
alongside the HTTP server — see
[`host_server/tcp_ingest.py`](host_server/tcp_ingest.py) for the wire framing.
Frames/samples arriving this way feed the same `Hub` and bag recorder as the
HTTP path, so the live viewer and recorded bag look identical regardless of
which mode the firmware is in.

## HTTP endpoints

Ingest (called by the firmware):

| Method / path | Body | Notes |
|---------------|------|-------|
| `POST /frame` | JPEG | `X-Timestamp-Us` header = esp_timer µs at grab. |
| `POST /imu`   | binary IMU drain | Format in [`host_server/wire.py`](host_server/wire.py), matches firmware `imu.h`. |
| `POST /stats` | binary system snapshot | Per-core CPU load, heap/PSRAM, chip temp, RSSI. Format in [`host_server/wire.py`](host_server/wire.py), matches firmware `sysstats.h`. |

Browser:

| Method / path | Purpose |
|---------------|---------|
| `GET /`            | Live viewer page. |
| `GET /stream.mjpg` | MJPEG stream of the latest frames. |
| `GET /imu.json`    | Latest IMU sample, ESP32-S3 system stats + running counters (polled by the page). |

## Recorded topics

| Topic | Type | Units |
|-------|------|-------|
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG (`format: "jpeg"`) |
| `/imu/data` | `sensor_msgs/msg/Imu` | accel m/s², gyro rad/s (converted from g / deg-s; no orientation) |
| `/esp32/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | per-core CPU load %, heap/PSRAM bytes, chip temp °C, RSSI dBm (as `KeyValue`s) |

Inspect / replay with the usual tooling:

```bash
ros2 bag info bags/slam_<timestamp>
ros2 bag play bags/slam_<timestamp>
```

## Layout

```
host_server/
  __main__.py     CLI entry point (python3 -m host_server)
  server.py       threaded http.server: ingest + live view
  tcp_ingest.py   raw-TCP STREAM_TCP ingest listener (frame/imu/stats framing)
  wire.py         decode the firmware's binary IMU + system-stats payloads
  bag_recorder.py rosbag2 writer (CompressedImage + Imu + DiagnosticArray)
  hub.py          shared latest-frame / latest-IMU / latest-stats state + counters
  viewer.html     browser page
```

## Timestamps

Every frame and IMU sample carries an esp_timer microsecond stamp on one shared
clock. The recorder keeps that relative timing and anchors it to wall time using
the first stamp seen on either stream:

```
ros_ns = t0_wall_ns + (esp_us - t0_esp_us) * 1000
```

so the two streams remain mutually synchronised in the bag.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `[host_server] ROS 2 not available` on startup | ROS 2 wasn't sourced in this shell before `python3 -m host_server` | `source /opt/ros/jazzy/setup.bash` first — must be the *same* shell/session that runs the server. Harmless if you only wanted the live view. |
| `OSError: [Errno 98] Address already in use` | Another process (often a previous, still-running server) already bound that port | `pkill -f host_server`, or pick a different `--port`/`--tcp-*-port` and update `config.h`/re-flash to match. `allow_reuse_address` is already set, so this means a *live* listener, not just a lingering TIME_WAIT socket. |
| Browser at `http://<host>:8080/` never loads / times out | Host firewall dropping the inbound connection, or board and host aren't on the same network | See [Network / firewall](#network--firewall) above. |
| Page loads but never shows a frame, `/imu.json` stays empty | Firmware not built with `CONFIG_ENABLE_REMOTE_STREAM 1`, or `CONFIG_REMOTE_HOST`/`CONFIG_REMOTE_PORT` don't match this server's bind address/port | Check `firmware/data_capture/main/config.h`, rebuild + reflash. |
| `ModuleNotFoundError: No module named 'rosbag2_storage_mcap'` with `--storage mcap` | mcap storage plugin not installed | `sudo apt install ros-jazzy-rosbag2-storage-mcap`, or drop back to the default `--storage sqlite3`. |
| Bag directory already exists / writer fails to open | A previous run used the same `--bag` path (or ran twice in the same second with the default timestamp) | Pick a different `--bag <path>`, or delete/move the old bag directory first. |
| Server runs but bags land somewhere unexpected | `--bag`'s default (`bags/slam_<timestamp>`) is relative to the **current working directory**, not this file's location | Run `python3 -m host_server` from `software/`, as shown above, or pass an absolute `--bag` path. |
