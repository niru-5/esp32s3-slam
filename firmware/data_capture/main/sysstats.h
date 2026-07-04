#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// Lightweight ESP32-S3 system telemetry — per-core CPU load, heap/PSRAM usage,
// chip temperature, WiFi RSSI and uptime. Collected cheaply (a ~1 Hz snapshot,
// a handful of O(1) heap queries plus one FreeRTOS task-state enumeration) so
// it never competes with the camera/IMU/WiFi work.
//
// Per-core load is derived from the FreeRTOS per-core idle-task runtime
// counters (needs CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS + USE_TRACE_FACILITY,
// both enabled in sdkconfig). Each snapshot reports the load over the interval
// since the previous snapshot, so callers should sample at a steady cadence.
//
// The snapshot is serialized into a fixed 64-byte little-endian payload (mirror
// of sysstats_snapshot_t below, built field-by-field to avoid struct padding)
// and pushed to the host as `POST /stats`. Keep this layout in sync with the
// host decoder in software/host_server/wire.py::parse_stats_payload.
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

// Initialise the chip temperature sensor and prime the CPU-load baseline.
// Safe to skip — sysstats_serialize() still works, just without temperature and
// with the first CPU-load reading reported as 0.
esp_err_t sysstats_start(void);

// Fill a fresh snapshot and serialize it into `out` (>= SYSSTATS_WIRE_LEN
// bytes). Returns the number of bytes written (SYSSTATS_WIRE_LEN).
size_t sysstats_serialize(uint8_t *out);
