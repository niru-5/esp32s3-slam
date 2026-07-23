#include "sdcard.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <time.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "camera.h"
#include "imu.h"
#include "debug_time.h"

static const char *TAG = "SDCARD";

#define SD_MOUNT_POINT "/sdcard"

// Batches physical flash writes instead of one syscall per small fwrite() —
// see the CONFIG_DEBUG_TIME session notes on why tiny unbatched SD writes
// are ~80x slower than large sequential ones.
#define SD_IO_BUFSZ 8192

static char s_session_dir[48];
static char s_imu_dir[56];
static char s_camera_dir[56];
static char s_stats_dir[56];

static TaskHandle_t s_imu_consumer_handle    = NULL;
static TaskHandle_t s_camera_consumer_handle = NULL;

// Wall-clock timestamp tag (data_capture.c syncs it over SNTP before
// sdcard_init runs), used for both the session directory name and one-off
// file names so they sort chronologically and are identifiable without
// cross-referencing boot logs. If SNTP never synced (no WiFi / no internet
// that boot), time(NULL) is still epoch-relative — fall back to a
// boot-relative tag instead of writing "19700101-000000".
static void format_timestamp_tag(char *out, size_t out_sz) {
    time_t now = time(NULL);
    if (now > 1577836800) {  // > 2020-01-01 => a real sync happened
        struct tm tm_info;
        localtime_r(&now, &tm_info);
        strftime(out, out_sz, "%Y%m%d-%H%M%S", &tm_info);
    } else {
        ESP_LOGW(TAG, "no wall-clock time available — using a boot-relative name");
        snprintf(out, out_sz, "boot_%010" PRId64, esp_timer_get_time() / 1000);
    }
}

// Find (and create) a fresh session directory.
static esp_err_t make_session_dir(void) {
    DEBUG_TIME_START(t0);

    char base[32];
    format_timestamp_tag(base, sizeof(base));

    // Suffix with an index on collision (e.g. two boots within the same
    // second, or SNTP never synced so every boot hits the same "boot_..."
    // name only if esp_timer also happened to match, which it won't across
    // reboots — kept anyway since it's cheap and makes the loop uniform).
    for (int i = 0; i < 100; i++) {
        if (i == 0)
            snprintf(s_session_dir, sizeof(s_session_dir), SD_MOUNT_POINT "/%s", base);
        else
            snprintf(s_session_dir, sizeof(s_session_dir), SD_MOUNT_POINT "/%s_%02d", base, i);
        if (mkdir(s_session_dir, 0775) == 0) {
            DEBUG_TIME_END(t0, TAG, "make_session_dir");
            return ESP_OK;
        }
        if (errno != EEXIST) {
            ESP_LOGE(TAG, "mkdir %s failed: %s", s_session_dir, strerror(errno));
            return ESP_FAIL;
        }
    }
    ESP_LOGE(TAG, "no free session slot under " SD_MOUNT_POINT " for base %s", base);
    return ESP_FAIL;
}

// Create one <session_dir>/<name> subdirectory, storing its path in *out.
static esp_err_t make_subdir(const char *name, char *out, size_t out_sz) {
    snprintf(out, out_sz, "%s/%s", s_session_dir, name);
    if (mkdir(out, 0775) == 0 || errno == EEXIST) return ESP_OK;
    ESP_LOGE(TAG, "mkdir %s failed: %s", out, strerror(errno));
    return ESP_FAIL;
}

// Save one JPEG frame into <session_dir>/camera/ as frame_<index>_<ts_ms>.jpg:
// the index is a monotonic per-session counter so files sort in capture order
// regardless of filesystem listing order, and ts_ms (esp_timer microseconds
// since boot / 1000) is kept alongside it for precise SLAM-side timing.
static void save_frame(int64_t ts_us, const uint8_t *buf, size_t len) {
    static uint32_t s_frame_index = 0;
    uint32_t idx = s_frame_index++;

    char path[112];
    snprintf(path, sizeof(path), "%s/frame_%06" PRIu32 "_%010" PRId64 ".jpg",
             s_camera_dir, idx, ts_us / 1000);

    DEBUG_TIME_START(t0);
    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGW(TAG, "open %s failed", path);
        return;
    }
    setvbuf(f, NULL, _IOFBF, SD_IO_BUFSZ);
    size_t written = fwrite(buf, 1, len, f);
    fclose(f);
    DEBUG_TIME_END(t0, TAG, "save_frame");
    if (written != len)
        ESP_LOGW(TAG, "short write on %s (%u/%u bytes)",
                 path, (unsigned)written, (unsigned)len);
}

// Write one batch drained from imu_queue into <session_dir>/imu/ as
// imu_<index>_<ts_ms>.bin: raw bytes in the same wire format used by
// STREAM_WIFI (imu_serialize_wire() in imu.h/imu.c) rather than JSON text.
// Per-sample float formatting (fprintf's dtoa path) measurably competed with
// camera capture for CPU time at 1kHz; a single fwrite() of pre-packed bytes
// avoids that entirely. software/host_server/wire.py's parse_imu_payload()
// already decodes this exact layout -- see also tools/parse_imu_bin.py, a
// standalone copy for reading these files straight off the card.
static void save_imu_bin(const uint8_t *wire, size_t wire_len) {
    static uint32_t s_batch_index = 0;
    uint32_t idx = s_batch_index++;

    char path[112];
    snprintf(path, sizeof(path), "%s/imu_%06" PRIu32 "_%010" PRId64 ".bin",
             s_imu_dir, idx, esp_timer_get_time() / 1000);

    DEBUG_TIME_START(t0);
    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGW(TAG, "open %s failed", path);
        return;
    }
    setvbuf(f, NULL, _IOFBF, SD_IO_BUFSZ);
    size_t written = fwrite(wire, 1, wire_len, f);
    fclose(f);
    DEBUG_TIME_END(t0, TAG, "save_imu_bin");
    if (written != wire_len)
        ESP_LOGW(TAG, "short write on %s (%u/%u bytes)",
                 path, (unsigned)written, (unsigned)wire_len);
}

// --------------------------------------------------------------------------
// Consumer tasks
// --------------------------------------------------------------------------

static void imu_sdcard_consumer_task(void *arg) {
    const size_t max_len = IMU_WIRE_HEADER_LEN + CONFIG_IMU_CONSUMER_BATCH * IMU_WIRE_SAMPLE_LEN;
    imu_sample_t *samples = malloc(CONFIG_IMU_CONSUMER_BATCH * sizeof(imu_sample_t));
    uint8_t      *wire    = malloc(max_len);
    if (!samples || !wire) {
        ESP_LOGE(TAG, "out of memory for IMU consumer buffers");
        free(samples); free(wire);
        vTaskDelete(NULL);
        return;
    }

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_IMU_CONSUMER_PERIOD_MS));

        int64_t  ref_esp_us;
        uint32_t ref_ticks;
        DEBUG_TIME_START(t_drain);
        uint32_t n = imu_queue_drain(samples, CONFIG_IMU_CONSUMER_BATCH, &ref_esp_us, &ref_ticks);
        DEBUG_TIME_END(t_drain, TAG, "imu draining");
        if (n == 0) continue;

        size_t wire_len = imu_serialize_wire(samples, n, ref_esp_us, ref_ticks, wire);
        save_imu_bin(wire, wire_len);
    }
}

static void camera_sdcard_consumer_task(void *arg) {
    camera_frame_t *frames = malloc(CONFIG_CAMERA_QUEUE_LEN * sizeof(camera_frame_t));
    if (!frames) {
        ESP_LOGE(TAG, "out of memory for camera consumer buffer");
        vTaskDelete(NULL);
        return;
    }

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_CAMERA_CONSUMER_PERIOD_MS));

        DEBUG_TIME_START(t_drain);
        uint32_t n = camera_queue_drain(frames, CONFIG_CAMERA_QUEUE_LEN);
        DEBUG_TIME_END(t_drain, TAG, "camera draining");
        for (uint32_t i = 0; i < n; i++) {
            save_frame(frames[i].ts_us, frames[i].fb->buf, frames[i].fb->len);
            camera_release(frames[i].fb);
        }
    }
}

// --------------------------------------------------------------------------
// Public API
// --------------------------------------------------------------------------

esp_err_t sdcard_write_imu_calibration_report(const char *text, size_t len) {
    char tag[32];
    format_timestamp_tag(tag, sizeof(tag));

    // Written at the SD root, not inside a session directory -- calibration
    // is a standalone operator action, not a capture session.
    char path[80];
    snprintf(path, sizeof(path), SD_MOUNT_POINT "/imu_calibration_%s.txt", tag);

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "open %s failed: %s", path, strerror(errno));
        return ESP_FAIL;
    }
    size_t written = fwrite(text, 1, len, f);
    fclose(f);
    if (written != len) {
        ESP_LOGW(TAG, "short write on %s (%u/%u bytes)", path, (unsigned)written, (unsigned)len);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "wrote calibration report to %s", path);
    return ESP_OK;
}

esp_err_t sdcard_init(void) {
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags        = SDMMC_HOST_FLAG_1BIT;   // only CLK/CMD/D0 are wired
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;     // 20 MHz; try _HIGHSPEED once verified stable

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk   = GPIO_NUM_39;
    slot_config.cmd   = GPIO_NUM_38;
    slot_config.d0    = GPIO_NUM_40;
    slot_config.d1    = GPIO_NUM_NC;
    slot_config.d2    = GPIO_NUM_NC;
    slot_config.d3    = GPIO_NUM_NC;
    slot_config.width = 1;
    // Fallback only — this board should have its own pull-ups on the slot;
    // internal ones are weak and may not hold at higher clock speeds.
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = CONFIG_ENABLE_SDCARD_FORMAT,
        .max_files              = 4,
        .allocation_unit_size   = 32 * 1024,
    };

    sdmmc_card_t *card;
    DEBUG_TIME_START(t_mount);
    esp_err_t err = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config,
                                             &mount_config, &card);
    DEBUG_TIME_END(t_mount, TAG, "sd mount");
    if (err != ESP_OK) {
        if (err == ESP_FAIL)
            ESP_LOGE(TAG, "mount failed — card isn't FAT-formatted and "
                          "CONFIG_ENABLE_SDCARD_FORMAT is off");
        else
            ESP_LOGE(TAG, "mount failed: %s (check wiring CLK=39 CMD=38 D0=40)",
                     esp_err_to_name(err));
        return err;
    }
    sdmmc_card_print_info(stdout, card);

    if (make_session_dir() != ESP_OK) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
        return ESP_FAIL;
    }

    if (make_subdir("imu",    s_imu_dir,    sizeof(s_imu_dir))    != ESP_OK ||
        make_subdir("camera", s_camera_dir, sizeof(s_camera_dir)) != ESP_OK ||
        make_subdir("stats",  s_stats_dir,  sizeof(s_stats_dir))  != ESP_OK) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "logging to %s", s_session_dir);
    return ESP_OK;
}

esp_err_t sdcard_pipeline_start(void) {
    if (xTaskCreatePinnedToCore(imu_sdcard_consumer_task, "imu_sd", 8192, NULL,
                                CONFIG_IMU_CONSUMER_PRIORITY, &s_imu_consumer_handle,
                                CONFIG_IMU_CONSUMER_CORE) != pdPASS)
        return ESP_ERR_NO_MEM;

    if (xTaskCreatePinnedToCore(camera_sdcard_consumer_task, "cam_sd", 8192, NULL,
                                CONFIG_CAMERA_CONSUMER_PRIORITY, &s_camera_consumer_handle,
                                CONFIG_CAMERA_CONSUMER_CORE) != pdPASS) {
        vTaskDelete(s_imu_consumer_handle);
        s_imu_consumer_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "sdcard consumers started");
    return ESP_OK;
}

void sdcard_pipeline_stop(void) {
    if (s_imu_consumer_handle)    { vTaskDelete(s_imu_consumer_handle);    s_imu_consumer_handle    = NULL; }
    if (s_camera_consumer_handle) { vTaskDelete(s_camera_consumer_handle); s_camera_consumer_handle = NULL; }
}

// Write one stats snapshot into <session_dir>/stats/ as
// stats_<index>_<ts_ms>.json — same per-file naming convention as
// save_frame/save_imu_json, one file per sample (CONFIG_STATS_*_PERIOD_MS
// apart) rather than a single appended .jsonl.
esp_err_t sdcard_write_stats(const uint8_t *json, size_t len) {
    static uint32_t s_stats_index = 0;
    uint32_t idx = s_stats_index++;

    char path[112];
    snprintf(path, sizeof(path), "%s/stats_%06" PRIu32 "_%010" PRId64 ".json",
             s_stats_dir, idx, esp_timer_get_time() / 1000);

    DEBUG_TIME_START(t0);
    FILE *f = fopen(path, "w");
    if (!f) {
        ESP_LOGW(TAG, "open %s failed", path);
        return ESP_FAIL;
    }
    setvbuf(f, NULL, _IOFBF, SD_IO_BUFSZ);
    size_t written = fwrite(json, 1, len, f);
    fclose(f);
    DEBUG_TIME_END(t0, TAG, "sdcard_write_stats");
    if (written != len)
        ESP_LOGW(TAG, "short write on %s (%u/%u bytes)",
                 path, (unsigned)written, (unsigned)len);
    return ESP_OK;
}
