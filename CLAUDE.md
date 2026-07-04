# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Firmware + tooling for an ESP32-S3-based visual-inertial SLAM rig (see `TODO.md`):

1. **Goal 1** — capture camera frames and IMU samples with reliable timestamps, calibrate camera/IMU, record to a ROS bag, run SLAM offline with ROS tooling.
2. **Goal 2** — eventually run a sparse/fast SLAM pipeline on the ESP32-S3 itself.

The project is currently at the firmware bring-up stage: two standalone ESP-IDF apps under `firmware/`. `hardware/` and `software/` are empty placeholders for future PCB/host-side work.

## Repo layout

- `firmware/data_capture/` — main app: OV5640 camera + BMI270 IMU + WiFi + HTTP server, streams both to a host over the network.
- `firmware/imu_testing/` — standalone BMI270 bring-up app (I2C bus scan + raw sample printout). Use this when debugging IMU wiring/config in isolation from the camera/WiFi stack.
- `firmware/camera_calibration/` — calibration target PDFs (ChArUco/circles/Kalibr boards), no code.
- `firmware/esp-idf/` — full ESP-IDF SDK checkout (v5.3.5, target esp32s3). Gitignored — treat as a local toolchain install, not project source.
- `SparkFun_BMI270_Arduino_Library/` — vendored BMI270 driver. Only `src/bmi270_api/bmi2.c` and `bmi270.c` are used; both firmware apps compile these two files directly into their `main` component (see each `main/CMakeLists.txt`) rather than consuming it as an ESP-IDF component or Arduino library.
- Each firmware project (`data_capture`, `imu_testing`) is an independent ESP-IDF project with its own `CMakeLists.txt`, `main/`, and `sdkconfig` — there is no shared top-level build.

## Build / flash / monitor

Each app is built from its own project directory, not from the repo root. There are currently two ESP32-S3 projects: `firmware/data_capture` and `firmware/imu_testing`.

```bash
source ~/.espressif/tools/activate_idf_v5.3.5.sh   # put idf.py and toolchain on PATH (once per shell)
cd firmware/data_capture       # or firmware/imu_testing
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
idf.py menuconfig              # sdkconfig changes
```

Target is `esp32s3` (already set in the committed `sdkconfig`). Serial port in this dev environment is `/dev/ttyACM0`.

There is no unit test suite — this is embedded firmware, verified by flashing to real hardware and exercising it over serial monitor / the HTTP endpoints below.

### PSRAM config

This board uses **Octal mode** PSRAM, not the ESP-IDF default (Quad). If PSRAM/menuconfig is touched: Component config → ESP PSRAM → enable external SPI RAM, SPI RAM config → Mode → `Octal Mode PSRAM`, clock speed → `80MHz`. Wrong mode causes an immediate boot crash.

## `data_capture` architecture

Single-file app (`firmware/data_capture/main/data_capture.c`) combining:

- **Camera**: OV5640 via the `espressif/esp32-camera` managed component (JPEG, VGA, single frame buffer in DRAM).
- **IMU**: BMI270 accel+gyro over I2C (`IMU_I2C_PORT` = `I2C_NUM_0`, SDA=GPIO38, SCL=GPIO39, CSB=GPIO40 driven high to force I2C mode), polled at 100 Hz by a dedicated FreeRTOS task **pinned to core 1** so it doesn't contend with WiFi/camera/HTTP on core 0. Samples land in a 200-entry ring buffer (`s_imu_ring`) guarded by `s_imu_mutex`.
- **WiFi**: STA mode, connects to the SSID/password constants at the top of the file; on IP acquisition starts mDNS as `slam-cam.local`.
- **HTTP server** (3 routes):
  - `GET /` — MJPEG-style viewer page (polls `/capture` in a loop).
  - `GET /capture` — one JPEG frame, with an `X-Timestamp-Us` header (esp_timer µs since boot at frame-grab time).
  - `GET /imu` — binary snapshot of the ring buffer since last read: 16-byte header (`num_samples`, `ref_esp_us`, `ref_sensor_ticks`) + 28 bytes/sample (`sens_time`, ax/ay/az, gx/gy/gz). The BMI270's own sensor-time ticks (39.0625 µs/tick, wraps at 2^24) are stored per sample; the host reconstructs each sample's ESP32 µs timestamp as `ref_esp_us + (sens_time - ref_sensor_ticks) * 39.0625`. This is the mechanism that keeps camera and IMU timestamps on a common clock — preserve it if touching the IMU or capture paths.

`imu_testing/main/imu_testing.c` is the same BMI270 init/read pattern minus camera/WiFi/HTTP, plus an I2C address scan on boot — useful as a reference or for isolating IMU-only hardware issues.

Both apps share identical BMI270 init code (init → enable accel+gyro → read default config → override ODR/range → derive raw→g / raw→dps scale factors from the ranges actually applied, not the requested ones).
