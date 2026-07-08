# `data_capture` architecture

A single-file-per-module ESP-IDF app (`firmware/data_capture/main/`) that turns the
ESP32-S3 into a **networked capture rig**: it streams OV5640 camera frames and BMI270
IMU samples to a host over WiFi (or logs them to a local SD card), with timestamps on a
**shared clock** so the two can be aligned offline for visual-inertial SLAM.

This document describes the **runtime state-machine / task-queue architecture** — agreed
here before implementation — that replaces the previous boot-time, `config.h`-flag-selected
"capture is always on, sinks are compile-time" model. The state machine now decides *at
runtime*, from serial commands, whether the rig is idle, streaming to WiFi, logging to SD,
or calibrating.

> **Status: implemented** (`firmware/data_capture/main/{state_machine,imu,camera,sysstats,
> net_client,sdcard}.c`), builds clean. Not yet flashed/tested on hardware. The old
> pull-based local HTTP server (`server_local.c`) has been removed — see
> [Open questions / assumptions](#open-questions--assumptions) for why, and the wire-format
> compatibility constraint that shaped several of the decisions below.

---

## Hardware

```
                 ESP32-S3
   ┌───────────────────────────────────┐
   │  Core 0 (PRO)     Core 1 (APP)    │
   └───────────────────────────────────┘
        │  SCCB (I2C_NUM_1)      │ I2C_NUM_0 @400kHz
        │  GPIO 4/5              │ SDA=41 SCL=42
   ┌────┴─────┐            ┌─────┴──────┐
   │  OV5640  │            │  BMI270    │
   │  camera  │            │  IMU       │
   └──────────┘            └────────────┘
                            CSB=GPIO47 → high (I2C mode)
                            SA0=GPIO48 → low  (addr 0x68)
                            VDD → 3.3V
```

- **Camera** and **IMU** are on **separate I2C ports**. The esp32-camera SCCB claims
  `I2C_NUM_1` (default), so the IMU must use `I2C_NUM_0`. See `docs/learnings.md §3`.
- CSB must be high from power-on or the BMI270 latches into SPI mode. See
  `docs/learnings.md §2`.
- Core assignment is no longer a simple "core 0 = wifi+camera, core 1 = IMU" split — see
  the task table below for the full per-task core/priority layout.

---

## Runtime state machine

A single task, `main_state_machine_task`, owns the rig's operating mode. It is the only
thing that creates/tears down the producer/consumer/stats task-and-queue sets described
in the next section.

| Field | Value |
|---|---|
| Core | 0 |
| Priority | 10 |
| Trigger | polls `stdin` (console = USB-Serial-JTAG, driver explicitly installed for RX — see `docs/learnings.md §12`) every `CONFIG_STATE_MACHINE_POLL_MS` (500 ms), non-blocking (`O_NONBLOCK` fd, `fgetc` returns EOF when idle) |

It is deliberately cheap: sleep 500 ms, check for a pending byte, act on it if present, go
back to sleep. This is a human-in-the-loop control channel (bring-up / test tool), not a
real-time path, so the extra ~0-500 ms latency to react to a command is acceptable. The
same dispatch function can later be driven by WiFi or BLE commands in addition to (or
instead of) serial.

### Commands / states

| Serial input | State | Meaning |
|---|---|---|
| `1` | `STREAM_WIFI` | start camera+IMU capture, stream to the host over WiFi |
| `2` | `STREAM_SDCARD` | start camera+IMU capture, log to the SD card |
| `3` | `IDLE` | stop streaming (either sink) — tear everything down |
| `4` | `IMU_CALIBRATION` | run IMU calibration routine (provisioned only, see below) |
| `5` | `CAMERA_CALIBRATION` | run camera calibration routine (provisioned only, see below) |

### Transition rules

- **`1` ↔ `2` switches directly**, no need to send `3` first. Entering a new state
  implicitly tears down whatever pipeline is currently running (see below) before
  standing up the new one.
- **`4`/`5` while in `1` or `2`** also force an implicit teardown of the active streaming
  pipeline before entering the calibration state.
- Every teardown (`3`, or the implicit teardown on `1`↔`2`/`4`/`5`) **deletes** the
  producer/consumer/stats tasks it started and **flushes/resets their queues** — nothing
  is left suspended or holding stale data across a transition.
- Tasks are **created fresh on entering `STREAM_WIFI`/`STREAM_SDCARD`** and **deleted on
  exit**, rather than pre-created and suspended. Transitions are operator-driven and rare,
  so the task-create/delete churn is an acceptable trade for zero idle CPU/RAM cost.

### Calibration states (provisioning only)

`IMU_CALIBRATION` and `CAMERA_CALIBRATION` are stubbed for now — entering the state calls
a placeholder (`imu_calibration_run()` / `camera_calibration_run()`) that logs
"not implemented" and returns; the exact calibration procedure (how long it runs, what it
records, whether it needs its own task) is deferred until that work starts.

---

## Task & queue architecture

Three independent producer→queue→consumer pipelines (IMU, camera, stats), all created and
destroyed together by the state machine when entering/leaving `STREAM_WIFI`/`STREAM_SDCARD`.
Only one of the two sink-specific consumer tasks per pipeline runs at a time — the other
sink's consumer simply isn't created — so, unlike the old dual-ring IMU hack, there's no
need to worry about two sinks draining the same queue concurrently.

Each pipeline's consumer is **sink-specific** (a `..._wifi_consumer_task` and a
`..._sdcard_consumer_task`, not one task branching on state), because these are expected to
diverge and be reused independently later — except the stats pipeline, which uses one
branching consumer (see table).

### Task table

| Task | Core | Priority | Trigger / period | Config knobs |
|---|---|---|---|---|
| `main_state_machine_task` | 0 | 10 | poll every 500 ms | `CONFIG_STATE_MACHINE_TASK_PRIORITY`, `CONFIG_STATE_MACHINE_POLL_MS` |
| `imu_capture_task` | 1 | 22 | `esp_timer` @ 1 ms → task-notify | `CONFIG_IMU_CAPTURE_PRIORITY`, `CONFIG_IMU_CAPTURE_CORE`, `CONFIG_IMU_CAPTURE_PERIOD_MS` |
| `imu_wifi_consumer_task` / `imu_sdcard_consumer_task` | 0 | 6 | every 100 ms, drain ≤100 items | `CONFIG_IMU_CONSUMER_PRIORITY`, `CONFIG_IMU_CONSUMER_CORE`, `CONFIG_IMU_CONSUMER_PERIOD_MS`, `CONFIG_IMU_CONSUMER_BATCH` |
| `camera_capture_task` | 1 | 20 | `esp_timer` @ configurable period → task-notify | `CONFIG_CAMERA_CAPTURE_PRIORITY`, `CONFIG_CAMERA_CAPTURE_CORE`, `CONFIG_CAMERA_CAPTURE_FPS` |
| `camera_wifi_consumer_task` / `camera_sdcard_consumer_task` | 0 | 6 | every 100 ms, drain available frames | `CONFIG_CAMERA_CONSUMER_PRIORITY`, `CONFIG_CAMERA_CONSUMER_CORE`, `CONFIG_CAMERA_CONSUMER_PERIOD_MS` |
| `stats_producer_task` | 1 | 12 | every 500 ms | `CONFIG_STATS_PRODUCER_PRIORITY`, `CONFIG_STATS_PRODUCER_CORE`, `CONFIG_STATS_PRODUCER_PERIOD_MS` |
| `stats_writer_task` (branches on active sink) | 0 | 7 | every 500 ms | `CONFIG_STATS_CONSUMER_PRIORITY`, `CONFIG_STATS_CONSUMER_CORE`, `CONFIG_STATS_CONSUMER_PERIOD_MS` |

All of the above priorities/cores/periods are `#define`s in `config.h` — nothing hardcoded
in the task bodies.

### Why `esp_timer` + task-notify, not a FreeRTOS software timer

`sdkconfig` has `CONFIG_FREERTOS_HZ=100` (10 ms tick) and
`CONFIG_FREERTOS_TIMER_TASK_PRIORITY=1`. Two consequences that rule out a literal
`xTimerCreate`-based "1 ms software timer at priority 22":

1. A FreeRTOS software timer's period is quantized to the tick (10 ms) — it cannot fire
   every 1 ms without raising `CONFIG_FREERTOS_HZ`, which we are **not** doing (it would
   change tick granularity for every `vTaskDelay` in the app).
2. All FreeRTOS software-timer callbacks run on the one Timer Service task, fixed at
   priority 1 — a timer callback can never itself run "at priority 22".

Instead: `imu_capture_task` is a real task created at priority 22 / core 1, blocked on
`ulTaskNotifyTake()`. An `esp_timer` (µs-resolution, independent of the FreeRTOS tick) fires
every 1 ms and does nothing but `xTaskNotifyGive()` the task — the lightweight timer
callback just wakes the real high-priority task, which does the actual BMI270 read +
queue push. `camera_capture_task` uses the same notify pattern (its period is ≥50 ms, so a
real `xTimerCreate` timer would technically work there too, but using the same
timer+notify pattern for both keeps the two capture tasks symmetric).

### Queues

| Queue | Depth | Element | Overflow policy |
|---|---|---|---|
| `imu_queue` | 200 (`CONFIG_IMU_QUEUE_LEN`) | `{sens_time, ax, ay, az, gx, gy, gz}` (`imu_sample_t`, see `imu.h`) | non-blocking send; if full, drop oldest + increment overflow counter |
| `camera_queue` | 10 (`CONFIG_CAMERA_QUEUE_LEN`) | pointer: `{camera_fb_t *fb, int64_t ts_us}` | non-blocking send; if full, **`camera_release()` the dropped entry's `fb`** before dropping (must not leak/starve the frame-buffer pool), + increment overflow counter |
| `stats_queue` | 8 (`CONFIG_STATS_QUEUE_LEN`) | `sysstats_snapshot_t` (see `sysstats.h`) | non-blocking send; drop oldest + increment overflow counter |

Producers never block on a full queue — a slow SD-card write or WiFi send must not stall
the priority-22/20/12 capture/stats tasks. Overflow counters are exposed back out through
the stats snapshot (new fields — see below) so drops are visible, not silent.

### Camera frame buffer count

The camera currently has a single DRAM frame buffer (`CAMERA_FB_IN_DRAM`, 1 fb), which
means `camera_grab()` blocks until the previous frame is released — no real pipelining is
possible with a 10-deep pointer queue. `CONFIG_CAMERA_FB_COUNT` will be raised (PSRAM-backed)
to **equal `CONFIG_CAMERA_QUEUE_LEN` (10)** — every frame sitting in `camera_queue` holds one
buffer checked out, so a smaller `fb_count` would make `camera_grab()` block once that many
frames are in flight, well before the queue's own drop-oldest backpressure ever triggers,
silently capping real throughput below the queue's depth.

### IMU pipeline data flow

```
imu_capture_task (core 1, prio 22)          imu_{wifi,sdcard}_consumer_task (core 0, prio 6)
   esp_timer fires every 1 ms                  every 100 ms:
     └─ notify task                              take up to 100 samples from imu_queue
   task wakes:                                   capture a fresh (ref_esp_us, ref_sensor_ticks)
     read BMI270 → sample                        pair for this batch (shared-clock reference)
     xQueueSend(imu_queue, drop-oldest-if-full)   bulk-send (wifi) or bulk-write one file (sdcard)
```

### Camera pipeline data flow

```
camera_capture_task (core 1, prio 20)       camera_{wifi,sdcard}_consumer_task (core 0, prio 6)
   esp_timer fires every 1000/fps ms           every 100 ms:
     └─ notify task                              drain available {fb, ts_us} entries
   task wakes:                                   stream (wifi) or write-to-file-named-by-ts_us (sdcard)
     camera_grab() → {fb, ts_us}                 camera_release(fb) once done
     xQueueSend(camera_queue, drop-oldest-if-full,
                camera_release()'ing any dropped fb)
```

### Stats pipeline data flow

```
stats_producer_task (core 1, prio 12)       stats_writer_task (core 0, prio 7)
   every 500 ms:                               every 500 ms:
     build sysstats_snapshot_t                    drain stats_queue
     xQueueSend(stats_queue, drop-oldest-if-full)  write/send to whichever sink is active
```

`sysstats_snapshot_t` already includes `chip_temp_c` (ESP32-S3 internal temperature
sensor) — no new work needed there. New fields to add for queue visibility:
`imu_queue_depth`/`imu_queue_overflows`, `camera_queue_depth`/`camera_queue_overflows`,
`stats_queue_overflows`.

---

## The shared-clock mechanism (the important part)

Camera and IMU timestamps must live on one timeline. Two different clocks are involved:

- **ESP32 `esp_timer`** — µs since boot. Used to stamp camera frames (`ts_us` captured at
  `camera_grab()` time, carried through the queue with the frame pointer).
- **BMI270 sensor-time** — the chip's own tick counter, 39.0625 µs/tick, wraps at 2²⁴
  (~655 s). Each IMU sample carries its own `sens_time` tick.

Each time an IMU consumer task drains a batch (every 100 ms), it captures a **reference
pair** — the current `esp_timer` µs and the current BMI270 tick at the same instant — and
ships it alongside the batch. The host then maps each sample's tick onto the ESP32 timeline:

```
sample_esp_us = ref_esp_us + (sens_time − ref_sensor_ticks) × 39.0625
```

Because every queued camera frame already carries its own `esp_timer` `ts_us`, IMU samples
and camera frames end up on the **same clock** — that's what makes offline sync possible.
**Preserve this if you touch the IMU or capture paths.**

---

## Config / constants worth knowing

- WiFi SSID/PASS are `#define`s at the top of `config.h`.
- PSRAM is **Octal mode @80 MHz** for this board (wrong mode = boot crash).
- The IMU uses the **legacy** `driver/i2c.h` (hence the migration warning at boot);
  candidate for future modernization to `driver/i2c_master.h`.
- New `config.h` knobs introduced by this design (all task priorities/cores/periods/queue
  depths from the tables above), plus:
  - `CONFIG_CAMERA_CAPTURE_FPS` — clamped 1–20 fps (i.e. capture period clamped
    1000 ms–50 ms).
  - `CONFIG_CAMERA_FB_COUNT` — frame buffer count backing the camera queue's real depth.
- The previous boot-time sink flags (`CONFIG_ENABLE_LOCAL_SERVER`,
  `CONFIG_ENABLE_REMOTE_STREAM`, `CONFIG_USE_SDCARD`) are superseded by the runtime state
  machine — see open questions below on how the existing pull-based local HTTP server
  fits in going forward.

---

## Open questions / assumptions

Decisions made during implementation, recorded here since they weren't pinned down in the
original discussion:

1. **`STREAM_WIFI` pushes, it doesn't serve.** `server_local.c` (the pull-based local HTTP
   server — host does `GET /capture`/`/imu`) has been **deleted**. Two things forced this:
   it structurally conflicts with the new pipeline (frames/IMU samples are now owned by
   `camera_queue`/`imu_queue`, drained once by their consumer task, not grabbed on-demand by
   an HTTP handler), and `software/host_server/` — a working Python receiver
   (`wire.py`/`server.py`/`bag_recorder.py`) with real recorded bags under
   `software/bags/` — already expects the **push** model (`POST /frame`, `/imu`, `/stats`).
   So `imu_wifi_consumer_task`/`camera_wifi_consumer_task` push to
   `CONFIG_REMOTE_HOST:CONFIG_REMOTE_PORT` via `net_client.c`, matching what that host code
   already speaks.
2. **Wire formats are byte-identical to before.** Because `software/host_server` hard-codes
   the `/frame`, `/imu`, and `/stats` payload layouts (`wire.py`'s `struct.Struct` formats),
   none of the three were changed. In particular the new queue-depth/overflow visibility
   (`imu_queue_depth/overflows`, `camera_queue_depth/overflows`) is **not** in the binary
   `/stats` payload — it only appears in the SD-card `stats.jsonl` (`sysstats_snapshot_to_json`)
   and in `ESP_LOGW` at the moment of a drop. Extending the wire format would need `wire.py`
   updated in lockstep, which was out of scope here.
3. **BMI270 ODR raised to 1600Hz** (from 100Hz) — the native rate closest to and above the
   1ms capture cadence. ODR steps are powers of two, so "1kHz" isn't selectable directly;
   1600Hz means the capture task very rarely re-reads a stale sample (worst case ~0.625ms
   staleness), whereas 800Hz would have meant roughly half of all reads repeating the
   previous value.
4. **`esp_timer` + task-notify is used only for the two capture tasks** (`imu_capture_task`,
   `camera_capture_task`) — the ones with a hard tick-rate constraint. Everything else
   (`imu_wifi_consumer_task`, `camera_wifi_consumer_task`, `imu_sdcard_consumer_task`,
   `camera_sdcard_consumer_task`, `stats_producer_task`, `stats_writer_task`) uses a plain
   `vTaskDelayUntil` loop — their periods (50ms-1000ms) are comfortably above the 10ms
   FreeRTOS tick, so the extra indirection wouldn't buy anything.
5. **`CONFIG_CAMERA_FB_COUNT` frame buffers moved to PSRAM** (`CAMERA_FB_IN_PSRAM`, from
   `CAMERA_FB_IN_DRAM`) to actually hold `CONFIG_CAMERA_QUEUE_LEN` (10) buffers — VGA JPEG
   frames don't fit 10-deep in internal DRAM alongside everything else.

---

## Where to look
- `firmware/data_capture/main/data_capture.c` — boot sequence, WiFi bring-up.
- `firmware/data_capture/main/imu.c` / `imu.h` — BMI270 driver + (currently ring-buffer
  based, to be replaced by `imu_queue`) sample capture.
- `firmware/data_capture/main/camera.c` / `camera.h` — OV5640 capture.
- `firmware/data_capture/main/sysstats.c` / `sysstats.h` — telemetry snapshot.
- `firmware/data_capture/main/config.h` — all compile-time knobs, including the new
  priority/core/period/queue-depth `#define`s from this design.
- `docs/learnings.md` — the bring-up traps (delay callback, SPI latch, I2C port, stale builds).
- `SparkFun_BMI270_Arduino_Library/src/bmi270_api/{bmi2.c,bmi270.c}` — vendored Bosch
  driver, compiled directly into `main` (see `main/CMakeLists.txt`).
