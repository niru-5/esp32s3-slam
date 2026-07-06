#include "sdcard.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <sys/stat.h>

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

static const char *TAG = "SDCARD";

#define SD_MOUNT_POINT "/sdcard"

// Batches physical flash writes instead of one syscall per small fwrite() —
// see the CONFIG_DEBUG_TIME session notes on why tiny unbatched SD writes
// are ~80x slower than large sequential ones.
#define SD_IO_BUFSZ            8192
#define SD_IMU_DUMP_PERIOD_US  (1000 * 1000)  // one JSON dump per second

static char s_session_dir[48];

// Find (and create) a fresh session directory. Filenames below are
// esp_timer-relative (reset to 0 every boot), so without this a second power
// cycle would silently overwrite the first session's files.
static esp_err_t make_session_dir(void) {
    for (int i = 0; i < 10000; i++) {
        snprintf(s_session_dir, sizeof(s_session_dir), SD_MOUNT_POINT "/sess_%04d", i);
        if (mkdir(s_session_dir, 0775) == 0) return ESP_OK;
        if (errno != EEXIST) {
            ESP_LOGE(TAG, "mkdir %s failed: %s", s_session_dir, strerror(errno));
            return ESP_FAIL;
        }
    }
    ESP_LOGE(TAG, "no free session slot under " SD_MOUNT_POINT " (sess_0000..sess_9999 all exist)");
    return ESP_FAIL;
}

// Save one JPEG frame, named by its capture timestamp to millisecond
// accuracy (esp_timer microseconds since boot / 1000).
static void save_frame(int64_t ts_us, const uint8_t *buf, size_t len) {
    char path[96];
    snprintf(path, sizeof(path), "%s/frame_%010" PRId64 ".jpg",
             s_session_dir, ts_us / 1000);

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGW(TAG, "open %s failed", path);
        return;
    }
    setvbuf(f, NULL, _IOFBF, SD_IO_BUFSZ);
    size_t written = fwrite(buf, 1, len, f);
    fclose(f);
    if (written != len)
        ESP_LOGW(TAG, "short write on %s (%u/%u bytes)",
                 path, (unsigned)written, (unsigned)len);
}

// Drain the SD-log IMU ring and write it as a JSON array of
// {t_ms, ax, ay, az, gx, gy, gz}, one entry per sample. Each sample's
// timestamp is reconstructed from the ring's reference pair the same way
// imu.h documents for the network wire format, so files are self-contained.
static void save_imu_json(void) {
    imu_sample_t *samples = malloc(IMU_RING_CAP * sizeof(imu_sample_t));
    if (!samples) {
        ESP_LOGW(TAG, "out of memory for IMU dump");
        return;
    }

    int64_t  ref_esp_us = 0;
    uint32_t ref_ticks  = 0;
    uint32_t count = imu_sdlog_drain(samples, &ref_esp_us, &ref_ticks);
    if (count == 0) {
        free(samples);
        return;
    }

    char path[96];
    snprintf(path, sizeof(path), "%s/imu_%010" PRId64 ".json",
             s_session_dir, esp_timer_get_time() / 1000);

    FILE *f = fopen(path, "w");
    if (!f) {
        ESP_LOGW(TAG, "open %s failed", path);
        free(samples);
        return;
    }
    setvbuf(f, NULL, _IOFBF, SD_IO_BUFSZ);

    fputc('[', f);
    for (uint32_t i = 0; i < count; i++) {
        int32_t delta_ticks = (int32_t)(samples[i].sens_time - ref_ticks);
        double  t_ms = (double)(ref_esp_us + (int64_t)(delta_ticks * IMU_SENSORTIME_US)) / 1000.0;
        fprintf(f, "%s{\"t_ms\":%.3f,\"ax\":%.5f,\"ay\":%.5f,\"az\":%.5f,"
                   "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f}",
                i == 0 ? "" : ",", t_ms,
                samples[i].ax, samples[i].ay, samples[i].az,
                samples[i].gx, samples[i].gy, samples[i].gz);
    }
    fputc(']', f);
    fclose(f);
    free(samples);
}

static void sdlog_task(void *arg) {
    const TickType_t frame_period =
        pdMS_TO_TICKS(1000 / (CONFIG_SDCARD_FPS > 0 ? CONFIG_SDCARD_FPS : 1));
    int64_t last_imu_dump_us = 0;

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last_wake, frame_period);

        int64_t ts;
        camera_fb_t *fb = camera_grab(&ts);
        if (fb) {
            save_frame(ts, fb->buf, fb->len);
            camera_release(fb);
        }

        int64_t now = esp_timer_get_time();
        if (now - last_imu_dump_us >= SD_IMU_DUMP_PERIOD_US) {
            last_imu_dump_us = now;
            save_imu_json();
        }
    }
}

esp_err_t sdcard_start(void) {
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
    esp_err_t err = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config,
                                             &mount_config, &card);
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
    ESP_LOGI(TAG, "logging to %s", s_session_dir);

    // Shares core 0 with camera/WiFi; kept off core 1 so an occasional SD
    // write stall (flash wear-leveling) can't jitter the 100 Hz IMU task.
    if (xTaskCreatePinnedToCore(sdlog_task, "sdlog", 8192, NULL, 5, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}
