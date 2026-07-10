#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// Lightweight ESP32-S3 system telemetry — per-core CPU load, heap/PSRAM usage,
// chip temperature, WiFi RSSI and uptime.
//
// Per-core load is derived from the FreeRTOS per-core idle-task runtime
// counters (needs CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS + USE_TRACE_FACILITY,
// both enabled in sdkconfig). Each snapshot reports the load over the interval
// since the previous snapshot, so callers should sample at a steady cadence.
//
// The wire payload (sysstats_serialize_snapshot) is a fixed 64-byte
// little-endian encoding of sysstats_snapshot_t, built field-by-field to
// avoid struct padding, and matches software/host_server/wire.py exactly --
// do not change this layout without updating wire.py too.
//
// Stats pipeline (see docs/architecture.md "Stats pipeline data flow"):
// stats_producer_task samples this module every CONFIG_STATS_PRODUCER_PERIOD_MS
// and pushes the raw snapshot to stats_queue; stats_writer_task drains it and
// either serializes to the wire format (wifi sink) or formats it as JSON
// (sdcard sink) -- see sysstats_pipeline_start().
// --------------------------------------------------------------------------

// On-wire payload: i64 + 3*f32 + i32 + 10*u32 = 8 + 12 + 4 + 40 = 64 bytes.
#define SYSSTATS_WIRE_LEN 64

typedef struct {
    int64_t  esp_us;            // esp_timer µs at snapshot (shared clock)
    float    cpu0_load;         // core 0 load over last interval (%)
    float    cpu1_load;         // core 1 load over last interval (%)
    float    chip_temp_c;       // internal temperature sensor (°C)
    int32_t  wifi_rssi;         // associated AP RSSI (dBm); 0 if unknown
    uint32_t uptime_s;          // seconds since boot
    uint32_t heap_free;         // esp_get_free_heap_size (bytes, all caps)
    uint32_t heap_min_free;     // esp_get_minimum_free_heap_size (bytes)
    uint32_t int_free;          // internal DRAM free (bytes)
    uint32_t int_largest;       // largest free internal block (bytes)
    uint32_t int_total;         // internal DRAM total (bytes)
    uint32_t psram_free;        // SPIRAM free (bytes; 0 if no PSRAM)
    uint32_t psram_min_free;    // SPIRAM minimum-ever free (bytes)
    uint32_t psram_largest;     // largest free SPIRAM block (bytes)
    uint32_t psram_total;       // SPIRAM total (bytes)
} sysstats_snapshot_t;

typedef enum {
    SYSSTATS_SINK_WIFI,
    SYSSTATS_SINK_SDCARD,
    SYSSTATS_SINK_TCP,
} sysstats_sink_t;

// Initialise the chip temperature sensor and prime the CPU-load baseline.
// Boot-time, once. Safe to skip — sysstats_fill() still works, just without
// temperature and with the first CPU-load reading reported as 0.
esp_err_t sysstats_start(void);

// Fill a fresh snapshot (current CPU load, heap, temperature, ...).
void sysstats_fill(sysstats_snapshot_t *out);

// Serialize an already-filled snapshot into `out` (>= SYSSTATS_WIRE_LEN
// bytes). Pure serialize -- does not sample anything. Returns
// SYSSTATS_WIRE_LEN.
size_t sysstats_serialize_snapshot(const sysstats_snapshot_t *s, uint8_t *out);

// Format an already-filled snapshot as one JSON object (no trailing
// newline), for the SD card stats log. Returns the number of bytes written
// (excluding the NUL terminator).
size_t sysstats_snapshot_to_json(const sysstats_snapshot_t *s, char *out, size_t out_sz);

// Create stats_producer_task (prio CONFIG_STATS_PRODUCER_PRIORITY, core
// CONFIG_STATS_PRODUCER_CORE, period CONFIG_STATS_PRODUCER_PERIOD_MS) +
// stats_queue (CONFIG_STATS_QUEUE_LEN deep), and stats_writer_task (prio
// CONFIG_STATS_CONSUMER_PRIORITY, core CONFIG_STATS_CONSUMER_CORE, period
// CONFIG_STATS_CONSUMER_PERIOD_MS) wired to `sink` for its whole lifetime
// (tasks are recreated on every state transition, so there's no need for
// runtime sink branching -- see docs/architecture.md "Task lifecycle").
// Returns ESP_OK once both are running.
esp_err_t sysstats_pipeline_start(sysstats_sink_t sink);

// Delete stats_producer_task + stats_writer_task and stats_queue.
void sysstats_pipeline_stop(void);
