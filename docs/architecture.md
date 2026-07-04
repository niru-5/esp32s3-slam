# `data_capture` architecture

A single-file ESP-IDF app (`firmware/data_capture/main/data_capture.c`) that turns the
ESP32-S3 into a **networked capture rig**: it streams OV5640 camera frames and BMI270
IMU samples to a host over WiFi/HTTP, with timestamps on a **shared clock** so the two
can be aligned offline for visual-inertial SLAM.

It is deliberately one file — this is bring-up firmware, not a layered system.

---

## Hardware

```
                 ESP32-S3
   ┌───────────────────────────────────┐
   │  Core 0 (PRO)     Core 1 (APP)    │
   │  WiFi + camera    IMU task        │
   │  + HTTP server    (100 Hz)        │
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

---

## Concurrency model

Two producers/consumers, split across cores so the real-time IMU sampling never
contends with WiFi/camera work:

| Context | Core | Role |
|---------|------|------|
| `app_main` → HTTP server callbacks | 0 | serve `/capture`, `/imu`, `/imu.json` on request |
| WiFi / camera driver | 0 | managed by IDF |
| `imu_task` | 1 | poll BMI270 at 100 Hz, push into ring buffer |

Shared state is the **IMU ring buffer** (`s_imu_ring`, 200 samples ≈ 2 s), guarded by
`s_imu_mutex`:

```
imu_task (core 1)                         imu_handler (core 0, on HTTP GET)
   every 10 ms:                              on request:
     read BMI270 ─► sample                     take mutex
     take mutex                                snapshot all unread samples
     ring[write++] = sample                    reset ring (count=0)
     count = min(count+1, CAP)                 give mutex
     give mutex                                build binary payload ─► client
```

- Writer overwrites oldest when full (circular).
- `/imu` is a **destructive drain** — each read returns samples-since-last-read and
  clears the buffer. `/imu.json` is a **non-destructive peek** at the newest sample.

---

## The shared-clock mechanism (the important part)

Camera and IMU timestamps must live on one timeline. Two different clocks are involved:

- **ESP32 `esp_timer`** — µs since boot. Used to stamp camera frames.
- **BMI270 sensor-time** — the chip's own tick counter, 39.0625 µs/tick, wraps at 2²⁴
  (~655 s). Each IMU sample carries its own `sens_time` tick.

On every `/imu` response the firmware captures a **reference pair** — the current
`esp_timer` µs and the current BMI270 tick at the same instant. The host then maps each
sample's tick onto the ESP32 timeline:

```
sample_esp_us = ref_esp_us + (sens_time − ref_sensor_ticks) × 39.0625
```

Because `/capture` returns the frame-grab `esp_timer` µs in the `X-Timestamp-Us` header,
IMU samples and camera frames end up on the **same clock** — that's what makes offline
sync possible. **Preserve this if you touch the IMU or capture paths.**

---

## HTTP API

| Route | Type | Notes |
|-------|------|-------|
| `GET /` | HTML | viewer: polls `/capture` for a live image + live `/imu.json` readout |
| `GET /capture` | `image/jpeg` | one VGA JPEG; `X-Timestamp-Us` header = grab time (µs) |
| `GET /imu` | binary, **destructive** | drains the ring; see wire format below |
| `GET /imu.json` | JSON, **non-destructive** | latest sample + `buffered` count; safe to poll |

Discoverable as `http://slam-cam.local` (mDNS) once connected.

### `/imu` wire format (little-endian)

```
Header (16 bytes):
  uint32  count               number of samples following
  int64   ref_esp_us          ESP32 µs at response time
  uint32  ref_sensor_ticks    BMI270 tick at response time
Then count × 28-byte samples:
  uint32  sens_time           BMI270 tick for this sample
  float32 ax, ay, az          accel, g
  float32 gx, gy, gz          gyro, deg/s
```

---

## Boot sequence (`app_main`)

```
1. nvs_flash_init / netif / event loop
2. WiFi STA start + connect        ── on GOT_IP: start mDNS (slam-cam.local)
3. esp_camera_init  (OV5640, JPEG, VGA, 1 fb in DRAM)   ← installs SCCB on I2C_NUM_1
4. s_imu_mutex = create; imu_init()                      ← I2C_NUM_0 + BMI270 bring-up
5. httpd_start + register 4 routes
6. xTaskCreatePinnedToCore(imu_task, core 1)
```

If camera or IMU init fails, `app_main` logs and returns (WiFi/HTTP may still come up
via the event handler, but there's no capture).

### `imu_init()` (BMI270 bring-up), in order
1. Preset CSB=high / SA0=low latches, **then** set pins to output (no CSB low-glitch),
   pull-up on CSB. → `docs/learnings.md §2`
2. `i2c_param_config` + `i2c_driver_install` on `I2C_NUM_0`.
3. I2C bus scan (diagnostic — expect ACK from `0x68`).
4. Wire BMI2 callbacks: `bmi2_i2c_read/write`, `bmi2_delay_us_cb` (busy-wait — critical,
   `docs/learnings.md §1`).
5. `bmi270_init` → enable accel+gyro → read config → override to ±4 G / ±500 dps / 100 Hz.
6. Derive raw→g and raw→dps scale factors **from the ranges read back**, not requested.

---

## Config / constants worth knowing

- WiFi SSID/PASS are `#define`s at the top of the file.
- Frame: JPEG, VGA (`FRAMESIZE_VGA`), quality 12, single frame buffer in **DRAM**
  (`CAMERA_FB_IN_DRAM`), `GRAB_WHEN_EMPTY`.
- PSRAM is **Octal mode @80 MHz** for this board (wrong mode = boot crash).
- IMU ring: `IMU_RING_CAP = 200` (~2 s at 100 Hz).
- The IMU uses the **legacy** `driver/i2c.h` (hence the migration warning at boot);
  candidate for future modernization to `driver/i2c_master.h`.

---

## Where to look
- `firmware/data_capture/main/data_capture.c` — everything above.
- `docs/learnings.md` — the bring-up traps (delay callback, SPI latch, I2C port, stale builds).
- `SparkFun_BMI270_Arduino_Library/src/bmi270_api/{bmi2.c,bmi270.c}` — vendored Bosch
  driver, compiled directly into `main` (see `main/CMakeLists.txt`).
