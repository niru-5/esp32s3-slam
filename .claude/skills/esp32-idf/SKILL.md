---
name: esp32-idf
description: Build, flash, and monitor the ESP32-S3 firmware projects in firmware/ (data_capture, imu_testing). Use whenever the user asks to build, flash, or read serial logs from the ESP32-S3 boards.
---

# ESP32-S3 IDF workflow

This repo has two independent ESP-IDF projects under `firmware/`:

- `data_capture` — camera + IMU + WiFi + HTTP server (main app).
- `imu_testing` — standalone BMI270 bring-up (I2C scan + raw sample printout).

Each is built from its own project directory; there is no shared top-level
build. The ESP-IDF toolchain is not on PATH by default — it must be sourced
first. Serial port in this dev environment is `/dev/ttyACM0`, target is
`esp32s3`.

Helper scripts live in `scripts/` next to this file and do the env-sourcing
for you. Prefer them over hand-rolling `idf.py` invocations.

## Build

```bash
.claude/skills/esp32-idf/scripts/build.sh <data_capture|imu_testing>
```

Sources `~/.espressif/tools/activate_idf_v5.3.5.sh`, cds into the project,
runs `idf.py build`. Fails loudly if the project directory doesn't exist.

## Flash + monitor (time-boxed, logs to a file)

`idf.py monitor` runs forever and is interactive — not something to run
directly in an agent session. Use the wrapper instead: it flashes, captures
serial output through a pty (so `idf.py`'s ANSI/progress output behaves as if
run in a terminal), stops after a fixed duration via `SIGINT` (same as
Ctrl+C, so monitor exits cleanly), and writes both the raw and an
ANSI-stripped log file.

```bash
.claude/skills/esp32-idf/scripts/flash_monitor.sh <data_capture|imu_testing> [port] [duration_seconds] [log_path]
# defaults: port=/dev/ttyACM0  duration_seconds=30  log_path=/tmp/esp32-idf/<project>.log
```

The script prints the path to the cleaned log (`<log_path minus .log>_clean.log`)
on stdout — read that file after the script returns to see the boot/serial
output. Example:

```bash
.claude/skills/esp32-idf/scripts/flash_monitor.sh imu_testing /dev/ttyACM0 20
# -> /tmp/esp32-idf/imu_testing_clean.log
```

Then read the printed path with the Read tool.

## menuconfig / sdkconfig changes

Interactive, so don't script it — tell the user to run it themselves:

```bash
source ~/.espressif/tools/activate_idf_v5.3.5.sh
cd firmware/<project>
idf.py menuconfig
```

Note the PSRAM gotcha: this board needs **Octal mode** PSRAM at 80MHz, not
the ESP-IDF default (Quad). Wrong mode = boot crash.

## Extending this skill

Add new project-specific commands (e.g. `erase_flash`, `size`, a specific
`idf.py monitor` filter) as new scripts in `scripts/`, each taking the
project name as `$1` for consistency, and list them in this file so future
invocations know they exist.
