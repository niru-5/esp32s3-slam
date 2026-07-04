# Host server (`host_server`)

Receives the camera + IMU stream the ESP32-S3 firmware pushes over WiFi,
records it to a **ROS 2 bag**, and serves a **live browser view**.

This is the receiving side of the firmware's remote-stream mode
(`firmware/data_capture` built with `CONFIG_ENABLE_REMOTE_STREAM`). The board
POSTs JPEG frames and binary IMU drains to this server; both are stamped on the
ESP32's shared `esp_timer` clock so camera and IMU stay time-aligned in the bag.

## Requirements

- Python 3.10+ (standard library only — no pip installs for the web layer).
- ROS 2 (developed against **Jazzy**) for bag recording. Source it before
  running so `rosbag2_py` + `sensor_msgs` are importable:

  ```bash
  source /opt/ros/jazzy/setup.bash
  ```

  Without a ROS environment the server still runs as a **live viewer** and just
  skips recording (or pass `--no-record` explicitly).

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
| `--port` | `8080` | Listen port (match `CONFIG_REMOTE_PORT`). |
| `--bag`  | `bags/slam_<timestamp>` | rosbag2 output directory. |
| `--storage` | `sqlite3` | rosbag2 backend (`sqlite3` or `mcap`). |
| `--no-record` | off | Live view only, no bag. |

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
