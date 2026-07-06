#pragma once

#include "esp_err.h"

// --------------------------------------------------------------------------
// SD card logging — camera frames + IMU samples to local storage, independent
// of the network sinks (net_client.c / server_local.c).
//
// This board's built-in microSD slot is wired for SDMMC 1-bit mode:
//   CLK=GPIO39  CMD=GPIO38  D0=GPIO40
//
// Mounts FAT at /sdcard, creates a fresh session directory per boot (frame/
// IMU filenames are esp_timer-relative and reset to 0 every boot, so a new
// directory avoids one session's files colliding with the last), and starts
// a logging task that:
//   - saves each camera frame as <session>/frame_<capture_time_ms>.jpg
//   - dumps the IMU ring as <session>/imu_<dump_time_ms>.json once a second
//
// Gated by config.h: CONFIG_USE_SDCARD / CONFIG_ENABLE_SDCARD_FORMAT.
// --------------------------------------------------------------------------

// Mount the card and start the logging task. Best-effort: on failure (no
// card, wiring issue, unformatted card with CONFIG_ENABLE_SDCARD_FORMAT off)
// this logs the reason and returns an error — callers should treat SD
// logging as optional and keep running without it.
esp_err_t sdcard_start(void);
