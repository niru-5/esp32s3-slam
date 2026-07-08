#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// SD card sink — logs captured data to local storage, independent of the
// network sink (net_client.c).
//
// This board's built-in microSD slot is wired for SDMMC 1-bit mode:
//   CLK=GPIO39  CMD=GPIO38  D0=GPIO40
//
// sdcard_init() mounts FAT at /sdcard and creates a fresh session directory
// (frame/IMU/stats filenames are esp_timer-relative and reset to 0 every
// boot, so a new directory avoids one session's files colliding with the
// last). sdcard_pipeline_start()/sdcard_pipeline_stop() create/tear down the
// two consumer tasks that actually write:
//   - imu_sdcard_consumer_task    -- <session>/imu_<drain_time_ms>.json, one
//                                    file per CONFIG_IMU_CONSUMER_PERIOD_MS
//                                    batch drained from imu_queue
//   - camera_sdcard_consumer_task -- <session>/frame_<capture_time_ms>.jpg,
//                                    one file per frame drained from
//                                    camera_queue
// sdcard_write_stats() appends one JSON line per call to
// <session>/stats.jsonl (opened once for the whole session, not per call).
//
// Gated by config.h: CONFIG_USE_SDCARD / CONFIG_ENABLE_SDCARD_FORMAT.
// --------------------------------------------------------------------------

// Mount the card and create a session directory. Boot-time, once.
// Best-effort: on failure (no card, wiring issue, unformatted card with
// CONFIG_ENABLE_SDCARD_FORMAT off) this logs the reason and returns an
// error -- callers should treat SD logging as optional and keep running
// without it (and must not call sdcard_pipeline_start() afterward).
esp_err_t sdcard_init(void);

// Open stats.jsonl for the session and create imu_sdcard_consumer_task +
// camera_sdcard_consumer_task (priorities/cores/periods from config.h).
// Returns ESP_OK once both are running. Only call if sdcard_init() succeeded.
esp_err_t sdcard_pipeline_start(void);

// Delete the sdcard consumer tasks and close stats.jsonl.
void sdcard_pipeline_stop(void);

// Append one system-stats snapshot (as JSON, see
// sysstats_snapshot_to_json()) as a line to the session's stats.jsonl.
// Called by stats_writer_task when the sdcard sink is active.
esp_err_t sdcard_write_stats(const uint8_t *json, size_t len);
