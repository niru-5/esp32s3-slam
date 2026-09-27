# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Firmware + tooling for an ESP32-S3-based visual-inertial SLAM rig (see `TODO.md`):

1. **Goal 1** — capture camera frames and IMU samples with reliable timestamps, calibrate camera/IMU, record to a ROS bag, run SLAM offline with ROS tooling.
2. **Goal 2** — eventually run a sparse/fast SLAM pipeline on the ESP32-S3 itself.

The project is currently at the firmware bring-up stage: three standalone ESP-IDF apps under `firmware/`. `hardware/` and `software/` are empty placeholders for future PCB/host-side work.

## Repo layout

- `firmware/data_capture/` — main app: OV5640 camera + BMI270 IMU + WiFi, streams both to a host over the network (HTTP, raw TCP, or SD card) and dials a host-driven control channel for mode switching + calibration (`control_link.c`, `cam_calib.c`; see `docs/architecture.md`).
- `firmware/imu_testing/` — standalone BMI270 bring-up app (I2C bus scan + raw sample printout). Use this when debugging IMU wiring/config in isolation from the camera/WiFi stack.
- `firmware/inference_on_esp32s3/` — on-device esp-dl inference: RGB565 camera frames → `espressif/hand_detect` → temporal wave detector (`main/wave_detector.hpp`, host-unit-tested in `test/`). Logs camera FPS / inference FPS over serial. Model is baked into flash by default (no SD card); SD mode optional. Doesn't use the BMI270 lib, so no worktree symlink needed. See `docs/inference_on_esp32s3.md`.
- `firmware/data_capture/main/cam_calib.c` — `CAMERA_CALIBRATION` mode (serial command `5`, or `set_state camera_calibration` over the control channel / browser UI): device-side of the host-driven camera calibration / ISP tuning workflow, plus persisting tuned registers to NVS (`camera_overrides.c`). Host side is `software/host_server/calibration/` (needs numpy+OpenCV: `software/.venv`, `requirements-calib.txt`; tests in `software/tests/`) and `software/host_server/control/` (the always-on browser control app, stdlib only). See `docs/camera_calibration_and_tuning.md`.
- `firmware/camera_calibration/` — calibration target PDFs (ChArUco/circles/Kalibr boards), no code.
- `firmware/esp-idf/` — full ESP-IDF SDK checkout (v5.3.5, target esp32s3). Gitignored — treat as a local toolchain install, not project source.
- `SparkFun_BMI270_Arduino_Library/` — vendored BMI270 driver. Only `src/bmi270_api/bmi2.c` and `bmi270.c` are used; both firmware apps compile these two files directly into their `main` component (see each `main/CMakeLists.txt`) rather than consuming it as an ESP-IDF component or Arduino library.
- Each firmware project (`data_capture`, `imu_testing`) is an independent ESP-IDF project with its own `CMakeLists.txt`, `main/`, and `sdkconfig` — there is no shared top-level build.

## Build / flash / monitor

Each app is built from its own project directory, not from the repo root. There are currently three ESP32-S3 projects: `firmware/data_capture`, `firmware/imu_testing` and `firmware/inference_on_esp32s3`.

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

Modular app under `firmware/data_capture/main/` (not a single file — `data_capture.c` is just
boot/WiFi bring-up + handing off to `state_machine.c`), see `docs/architecture.md` for the
full design:

- **Camera**: OV5640 via the `espressif/esp32-camera` managed component (`camera.c`), JPEG/SVGA
  by default, adaptive JPEG quality based on queue depth.
- **IMU**: BMI270 accel+gyro over I2C (`imu.c`; `IMU_I2C_PORT` = `I2C_NUM_0`, SDA=GPIO38,
  SCL=GPIO39, CSB=GPIO40 driven high to force I2C mode), a dedicated FreeRTOS task **pinned to
  core 1**, esp_timer-notified rather than polled. Currently compiled out by default
  (`CONFIG_ENABLE_IMU=0` in `config.h`).
- **WiFi**: STA mode (`data_capture.c`), connects to the SSID/password constants at the top of
  the file; on IP acquisition syncs time over SNTP.
- **No HTTP server on the device.** The device is a client, never a server: `net_client.c` POSTs
  frames/IMU/stats to a host over HTTP (`STREAM_WIFI`), `tcp_client.c` does the raw-TCP
  equivalent (`STREAM_TCP`), `sdcard.c` logs locally (`STREAM_SDCARD`). Both `cam_calib.c` (the
  camera-calibration protocol, port `CONFIG_CAM_CALIB_PORT`) and `control_link.c` (see below,
  `CONFIG_CONTROL_PORT`) work the same way in the other direction — the device **dials the
  host**, which listens.
- **Runtime mode**: `state_machine.c` owns IDLE/STREAM_WIFI/STREAM_SDCARD/STREAM_TCP/
  IMU_CALIBRATION/CAMERA_CALIBRATION, driven by two command sources that funnel into the same
  transition functions — the serial console (digits 1-6, the original bring-up/recovery path,
  still fully supported) and `control_link.c`, an always-on channel to a host control app
  (`software/host_server/control`, browser UI) that is now the primary way to drive the rig:
  toggle streaming vs. calibration, pick a calibration target (camera or IMU), and run IMU
  calibration's gravity-axis prompt over the network instead of the serial console.
- **Persisted camera tuning**: `camera_overrides.c` stores a set of OV5640 register overrides in
  NVS (the sensor itself has no persistent registers) and re-applies them after every
  `camera_init()`, so registers tuned via `cam_calib.c`'s calibration protocol (commands
  `save_camera_regs`/`clear_camera_regs`, exposed as `cli.py`'s `save`/`save clear`) survive a
  reboot without any host involvement.

`imu_testing/main/imu_testing.c` is the same BMI270 init/read pattern minus camera/WiFi/
streaming, plus an I2C address scan on boot — useful as a reference or for isolating IMU-only
hardware issues.

Both apps share identical BMI270 init code (init → enable accel+gyro → read default config →
override ODR/range → derive raw→g / raw→dps scale factors from the ranges actually applied, not
the requested ones).
