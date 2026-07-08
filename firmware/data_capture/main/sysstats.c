#include "sysstats.h"

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/temperature_sensor.h"

#include "config.h"
#include "imu.h"
#include "camera.h"
#include "net_client.h"
#include "sdcard.h"

static const char *TAG = "STATS";

// --------------------------------------------------------------------------
// CPU load via per-core idle-task runtime counters.
//
// With CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER (the default when run
// time stats are on) each task's ulRunTimeCounter is accumulated esp_timer
// microseconds, directly comparable to esp_timer_get_time(). ESP-IDF names the
// per-core idle tasks "IDLE0" and "IDLE1"; the fraction of an interval each one
// ran is that core's idle fraction, so load = 100 * (1 - idle_fraction).
// --------------------------------------------------------------------------
#define MAX_TASKS 40

static int64_t  s_prev_us       = 0;    // esp_timer at previous snapshot
static uint32_t s_prev_idle0_us = 0;    // IDLE0 cumulative runtime at prev snap
static uint32_t s_prev_idle1_us = 0;    // IDLE1 cumulative runtime at prev snap
static bool     s_have_baseline = false;

static temperature_sensor_handle_t s_temp = NULL;

static QueueHandle_t s_stats_queue     = NULL;
static TaskHandle_t  s_producer_handle = NULL;
static TaskHandle_t  s_writer_handle   = NULL;
static sysstats_sink_t s_sink;

// Read the cumulative runtime of the two idle tasks. Returns false if the
// FreeRTOS trace facility is unavailable or the tasks were not found.
static bool read_idle_runtimes(uint32_t *idle0, uint32_t *idle1) {
    static TaskStatus_t tasks[MAX_TASKS];   // static: keep it off the task stack
    uint32_t total_runtime = 0;
    UBaseType_t n = uxTaskGetSystemState(tasks, MAX_TASKS, &total_runtime);
    if (n == 0) return false;

    bool got0 = false, got1 = false;
    for (UBaseType_t i = 0; i < n; i++) {
        const char *name = tasks[i].pcTaskName;
        if (!got0 && strcmp(name, "IDLE0") == 0) {
            *idle0 = tasks[i].ulRunTimeCounter;
            got0 = true;
        } else if (!got1 && strcmp(name, "IDLE1") == 0) {
            *idle1 = tasks[i].ulRunTimeCounter;
            got1 = true;
        }
    }
    return got0 && got1;
}

// Load (%) over [prev, now] given the idle-runtime delta on that core.
static float core_load(uint32_t idle_now, uint32_t idle_prev, int64_t span_us) {
    if (span_us <= 0) return 0.0f;
    uint32_t idle_delta = idle_now - idle_prev;   // wraps naturally (uint32)
    float busy = (float)(span_us - (int64_t)idle_delta);
    float load = 100.0f * busy / (float)span_us;
    if (load < 0.0f)   load = 0.0f;
    if (load > 100.0f) load = 100.0f;
    return load;
}

static void fill_cpu_load(sysstats_snapshot_t *s, int64_t now_us) {
    uint32_t idle0 = 0, idle1 = 0;
    if (!read_idle_runtimes(&idle0, &idle1)) {
        s->cpu0_load = s->cpu1_load = 0.0f;
        return;
    }
    if (s_have_baseline) {
        int64_t span = now_us - s_prev_us;
        s->cpu0_load = core_load(idle0, s_prev_idle0_us, span);
        s->cpu1_load = core_load(idle1, s_prev_idle1_us, span);
    } else {
        s->cpu0_load = s->cpu1_load = 0.0f;
        s_have_baseline = true;
    }
    s_prev_us       = now_us;
    s_prev_idle0_us = idle0;
    s_prev_idle1_us = idle1;
}

void sysstats_fill(sysstats_snapshot_t *s) {
    memset(s, 0, sizeof(*s));
    int64_t now_us = esp_timer_get_time();
    s->esp_us   = now_us;
    s->uptime_s = (uint32_t)(now_us / 1000000);

    fill_cpu_load(s, now_us);

    if (s_temp) {
        float t = 0.0f;
        if (temperature_sensor_get_celsius(s_temp, &t) == ESP_OK)
            s->chip_temp_c = t;
    }

    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
        s->wifi_rssi = ap.rssi;

    s->heap_free     = (uint32_t)esp_get_free_heap_size();
    s->heap_min_free = (uint32_t)esp_get_minimum_free_heap_size();

    s->int_free    = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    s->int_largest = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    s->int_total   = (uint32_t)heap_caps_get_total_size(MALLOC_CAP_INTERNAL);

    s->psram_free     = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    s->psram_min_free = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
    s->psram_largest  = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    s->psram_total    = (uint32_t)heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
}

esp_err_t sysstats_start(void) {
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    esp_err_t err = temperature_sensor_install(&cfg, &s_temp);
    if (err == ESP_OK) err = temperature_sensor_enable(s_temp);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "temperature sensor unavailable: %s", esp_err_to_name(err));
        s_temp = NULL;   // carry on without temperature
    }

    // Prime the CPU-load baseline so the first real snapshot has a reference.
    uint32_t idle0 = 0, idle1 = 0;
    if (read_idle_runtimes(&idle0, &idle1)) {
        s_prev_us       = esp_timer_get_time();
        s_prev_idle0_us = idle0;
        s_prev_idle1_us = idle1;
        s_have_baseline = true;
    } else {
        ESP_LOGW(TAG, "run-time stats unavailable — CPU load will read 0 "
                      "(enable CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS)");
    }
    ESP_LOGI(TAG, "system stats ready");
    return ESP_OK;
}

size_t sysstats_serialize_snapshot(const sysstats_snapshot_t *s, uint8_t *out) {
    // Field-by-field, little-endian, no padding — mirror in wire.py.
    uint8_t *p = out;
    memcpy(p, &s->esp_us,         8); p += 8;
    memcpy(p, &s->cpu0_load,      4); p += 4;
    memcpy(p, &s->cpu1_load,      4); p += 4;
    memcpy(p, &s->chip_temp_c,    4); p += 4;
    memcpy(p, &s->wifi_rssi,      4); p += 4;
    memcpy(p, &s->uptime_s,       4); p += 4;
    memcpy(p, &s->heap_free,      4); p += 4;
    memcpy(p, &s->heap_min_free,  4); p += 4;
    memcpy(p, &s->int_free,       4); p += 4;
    memcpy(p, &s->int_largest,    4); p += 4;
    memcpy(p, &s->int_total,      4); p += 4;
    memcpy(p, &s->psram_free,     4); p += 4;
    memcpy(p, &s->psram_min_free, 4); p += 4;
    memcpy(p, &s->psram_largest,  4); p += 4;
    memcpy(p, &s->psram_total,    4); p += 4;
    return (size_t)(p - out);
}

size_t sysstats_snapshot_to_json(const sysstats_snapshot_t *s, char *out, size_t out_sz) {
    // Queue depth/overflow counters are only surfaced here (SD JSON), not in
    // the wire payload above -- that layout is fixed by software/host_server.
    int n = snprintf(out, out_sz,
        "{\"esp_us\":%lld,\"cpu0_load\":%.2f,\"cpu1_load\":%.2f,"
        "\"chip_temp_c\":%.2f,\"wifi_rssi\":%d,\"uptime_s\":%u,"
        "\"heap_free\":%u,\"heap_min_free\":%u,"
        "\"int_free\":%u,\"int_largest\":%u,\"int_total\":%u,"
        "\"psram_free\":%u,\"psram_min_free\":%u,\"psram_largest\":%u,\"psram_total\":%u,"
        "\"imu_queue_depth\":%u,\"imu_queue_overflows\":%u,"
        "\"camera_queue_depth\":%u,\"camera_queue_overflows\":%u}",
        (long long)s->esp_us, s->cpu0_load, s->cpu1_load,
        s->chip_temp_c, (int)s->wifi_rssi, (unsigned)s->uptime_s,
        (unsigned)s->heap_free, (unsigned)s->heap_min_free,
        (unsigned)s->int_free, (unsigned)s->int_largest, (unsigned)s->int_total,
        (unsigned)s->psram_free, (unsigned)s->psram_min_free,
        (unsigned)s->psram_largest, (unsigned)s->psram_total,
        (unsigned)imu_queue_depth(), (unsigned)imu_queue_overflow_count(),
        (unsigned)camera_queue_depth(), (unsigned)camera_queue_overflow_count());
    return n > 0 ? (size_t)n : 0;
}

// --------------------------------------------------------------------------
// Stats pipeline
// --------------------------------------------------------------------------

static void stats_producer_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_STATS_PRODUCER_PERIOD_MS));

        sysstats_snapshot_t snap;
        sysstats_fill(&snap);

        if (xQueueSend(s_stats_queue, &snap, 0) != pdTRUE) {
            sysstats_snapshot_t discard;
            xQueueReceive(s_stats_queue, &discard, 0);
            xQueueSend(s_stats_queue, &snap, 0);
            ESP_LOGW(TAG, "stats_queue full, dropped oldest snapshot");
        }
    }
}

static void stats_writer_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_STATS_CONSUMER_PERIOD_MS));

        sysstats_snapshot_t snap;
        if (xQueueReceive(s_stats_queue, &snap, 0) != pdTRUE) continue;

        if (s_sink == SYSSTATS_SINK_WIFI) {
            uint8_t wire[SYSSTATS_WIRE_LEN];
            sysstats_serialize_snapshot(&snap, wire);
            net_client_send_stats(wire, SYSSTATS_WIRE_LEN);
        } else {
            char json[384];
            size_t n = sysstats_snapshot_to_json(&snap, json, sizeof(json));
            sdcard_write_stats((const uint8_t *)json, n);
        }
    }
}

esp_err_t sysstats_pipeline_start(sysstats_sink_t sink) {
    s_sink = sink;
    s_stats_queue = xQueueCreate(CONFIG_STATS_QUEUE_LEN, sizeof(sysstats_snapshot_t));
    if (!s_stats_queue) return ESP_ERR_NO_MEM;

    if (xTaskCreatePinnedToCore(stats_producer_task, "stats_prod", 4096, NULL,
                                CONFIG_STATS_PRODUCER_PRIORITY, &s_producer_handle,
                                CONFIG_STATS_PRODUCER_CORE) != pdPASS) {
        vQueueDelete(s_stats_queue);
        s_stats_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreatePinnedToCore(stats_writer_task, "stats_wr", 4096, NULL,
                                CONFIG_STATS_CONSUMER_PRIORITY, &s_writer_handle,
                                CONFIG_STATS_CONSUMER_CORE) != pdPASS) {
        vTaskDelete(s_producer_handle);
        s_producer_handle = NULL;
        vQueueDelete(s_stats_queue);
        s_stats_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "stats pipeline started (sink=%s)",
             sink == SYSSTATS_SINK_WIFI ? "wifi" : "sdcard");
    return ESP_OK;
}

void sysstats_pipeline_stop(void) {
    if (s_producer_handle) { vTaskDelete(s_producer_handle); s_producer_handle = NULL; }
    if (s_writer_handle)   { vTaskDelete(s_writer_handle);   s_writer_handle   = NULL; }
    if (s_stats_queue)     { vQueueDelete(s_stats_queue);    s_stats_queue     = NULL; }
}
