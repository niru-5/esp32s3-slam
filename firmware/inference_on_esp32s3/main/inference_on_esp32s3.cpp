// inference_on_esp32s3 -- on-device hand detection + wave recognition.
//
//   OV5640 (RGB565 QVGA) --> capture task (core 1) --> [latest-frame slot]
//                                                          |
//                          inference task (core 0): esp-dl HandDetect (ESPDet-Pico)
//                                                          |
//                                       WaveDetector (temporal: box-x reversals)
//                                                          |
//                                       serial log: camera FPS, inference FPS, WAVING
//
// Full explanation: docs/inference_on_esp32s3.md

#include <atomic>
#include <cstdio>
#include <cstring>
#include <list>

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "hand_detect.hpp"   // esp-dl model component (espressif/hand_detect)

#include "config.h"
#include "wave_detector.hpp"

#if CONFIG_HAND_DETECT_MODEL_IN_SDCARD
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#if SD_BUS_SDMMC_1BIT
#include "driver/sdmmc_host.h"
#else
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#endif
#endif

static const char *TAG = "INFER";

// --------------------------------------------------------------------------
// Latest-frame slot shared between the capture task (producer) and the
// inference task (consumer). Holds at most one frame; a new frame replaces
// (and returns to the driver) an older one nobody consumed.
// --------------------------------------------------------------------------
static SemaphoreHandle_t s_slot_mutex;
static camera_fb_t      *s_slot_fb    = nullptr;
static int64_t           s_slot_ts_us = 0;
static TaskHandle_t      s_infer_task = nullptr;

static std::atomic<uint32_t> s_cam_frames = 0;   // every frame the sensor delivered
static std::atomic<uint32_t> s_infer_frames = 0;   // frames that went through the model
static std::atomic<uint32_t> s_infer_us_sum = 0;   // sum of model run time, for the mean
static std::atomic<uint32_t> s_dropped     = 0;   // frames replaced before inference got them

// Latest inference result, read by the stats log.
static volatile bool  s_hand_present = false;
static volatile float s_hand_score   = 0;
static volatile int   s_hand_box[4]  = {0, 0, 0, 0};
static volatile bool  s_waving       = false;
static volatile int   s_reversals    = 0;

// --------------------------------------------------------------------------
// SD card (only when the model location is "sdcard" in menuconfig)
// --------------------------------------------------------------------------
#if CONFIG_HAND_DETECT_MODEL_IN_SDCARD
static esp_err_t sdcard_mount(void) {
    esp_vfs_fat_mount_config_t mount_cfg = {};
    mount_cfg.format_if_mount_failed = false;   // never wipe a card holding the models
    mount_cfg.max_files = 4;
    mount_cfg.allocation_unit_size = 16 * 1024;
    sdmmc_card_t *card = nullptr;
    esp_err_t err;

#if SD_BUS_SDMMC_1BIT
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk = (gpio_num_t)SD_PIN_CLK;
    slot.cmd = (gpio_num_t)SD_PIN_CMD;
    slot.d0  = (gpio_num_t)SD_PIN_D0;
    slot.d1 = slot.d2 = slot.d3 = GPIO_NUM_NC;
    slot.width = 1;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot, &mount_cfg, &card);
#else
    spi_bus_config_t bus = {};
    bus.mosi_io_num = SD_PIN_MOSI;
    bus.miso_io_num = SD_PIN_MISO;
    bus.sclk_io_num = SD_PIN_SCK;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 4000;
    err = spi_bus_initialize(SPI2_HOST, &bus, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) return err;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = (gpio_num_t)SD_PIN_CS;
    slot.host_id = SPI2_HOST;
    err = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot, &mount_cfg, &card);
#endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s (card inserted? FAT32? wiring?)", esp_err_to_name(err));
        return err;
    }
    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}
#endif

// --------------------------------------------------------------------------
// Camera
// --------------------------------------------------------------------------
static esp_err_t camera_setup(void) {
    camera_config_t cfg = {};
    cfg.pin_pwdn     = CONFIG_CAM_PWDN_GPIO;
    cfg.pin_reset    = CONFIG_CAM_RESET_GPIO;
    cfg.pin_xclk     = CONFIG_CAM_XCLK_GPIO;
    cfg.pin_sccb_sda = CONFIG_CAM_SIOD_GPIO;
    cfg.pin_sccb_scl = CONFIG_CAM_SIOC_GPIO;
    cfg.pin_d7 = CONFIG_CAM_Y9_GPIO; cfg.pin_d6 = CONFIG_CAM_Y8_GPIO;
    cfg.pin_d5 = CONFIG_CAM_Y7_GPIO; cfg.pin_d4 = CONFIG_CAM_Y6_GPIO;
    cfg.pin_d3 = CONFIG_CAM_Y5_GPIO; cfg.pin_d2 = CONFIG_CAM_Y4_GPIO;
    cfg.pin_d1 = CONFIG_CAM_Y3_GPIO; cfg.pin_d0 = CONFIG_CAM_Y2_GPIO;
    cfg.pin_vsync = CONFIG_CAM_VSYNC_GPIO;
    cfg.pin_href  = CONFIG_CAM_HREF_GPIO;
    cfg.pin_pclk  = CONFIG_CAM_PCLK_GPIO;
    cfg.xclk_freq_hz = INFER_CAM_XCLK_HZ;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.pixel_format = PIXFORMAT_RGB565;
    cfg.frame_size   = INFER_FRAME_SIZE;
    cfg.fb_count     = INFER_CAM_FB_COUNT;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;   // capture task blocks for each new frame

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "camera ready: RGB565 %dx%d, %d PSRAM frame buffers",
             INFER_FRAME_W, INFER_FRAME_H, INFER_CAM_FB_COUNT);
    return ESP_OK;
}

// Producer: grab every frame at sensor rate, publish only the newest.
static void capture_task(void *) {
    for (;;) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
        int64_t ts = esp_timer_get_time();
        s_cam_frames++;

        xSemaphoreTake(s_slot_mutex, portMAX_DELAY);
        camera_fb_t *stale = s_slot_fb;
        s_slot_fb    = fb;
        s_slot_ts_us = ts;
        xSemaphoreGive(s_slot_mutex);

        if (stale) { esp_camera_fb_return(stale); s_dropped++; }
        xTaskNotifyGive(s_infer_task);
    }
}

// --------------------------------------------------------------------------
// Inference
// --------------------------------------------------------------------------
static void inference_task(void *) {
    HandDetect detect;                                   // lazy: model loads on first use
    detect.set_score_thr(INFER_HAND_SCORE_THR, 0);

    WaveConfig wcfg;
    wcfg.window_us     = (int64_t)WAVE_WINDOW_MS * 1000;
    wcfg.min_reversals = WAVE_MIN_REVERSALS;
    wcfg.amp_frac      = WAVE_AMP_FRAC;
    wcfg.min_amp_px    = WAVE_MIN_AMP_PX;
    wcfg.lost_us       = (int64_t)WAVE_LOST_MS * 1000;
    wcfg.hold_us       = (int64_t)WAVE_HOLD_MS * 1000;
    WaveDetector wave(wcfg);

    const dl::image::pix_type_t pix =
#if INFER_RGB565_IS_BIG_ENDIAN
        dl::image::DL_IMAGE_PIX_TYPE_RGB565BE;
#else
        dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;
#endif

    bool was_waving = false;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        xSemaphoreTake(s_slot_mutex, portMAX_DELAY);
        camera_fb_t *fb = s_slot_fb;
        int64_t ts      = s_slot_ts_us;
        s_slot_fb = nullptr;                       // we own it now
        xSemaphoreGive(s_slot_mutex);
        if (!fb) continue;                         // a newer notify already consumed it

        dl::image::img_t img;
        img.data     = fb->buf;
        img.width    = fb->width;
        img.height   = fb->height;
        img.pix_type = pix;

        int64_t t0 = esp_timer_get_time();
        std::list<dl::detect::result_t> &results = detect.run(img);   // preprocess + model + NMS
        s_infer_us_sum.fetch_add((uint32_t)(esp_timer_get_time() - t0));
        s_infer_frames++;

        // Track the single most confident hand.
        const dl::detect::result_t *best = nullptr;
        for (const auto &r : results)
            if (!best || r.score > best->score) best = &r;

        bool present = best != nullptr;
        if (present) {
            float cx = 0.5f * (best->box[0] + best->box[2]);
            float w  = (float)(best->box[2] - best->box[0]);
            s_hand_score = best->score;
            for (int i = 0; i < 4; i++) s_hand_box[i] = best->box[i];
            s_waving = wave.update(ts, true, cx, w);
        } else {
            s_waving = wave.update(ts, false, 0, 0);
        }
        s_hand_present = present;
        s_reversals    = wave.reversals();

        esp_camera_fb_return(fb);

        bool waving = s_waving;
        if (waving && !was_waving) ESP_LOGW(TAG, ">>> WAVE DETECTED <<<");
        if (!waving && was_waving) ESP_LOGI(TAG, "wave ended");
        was_waving = waving;
    }
}

// --------------------------------------------------------------------------
// app_main: bring-up + 1 Hz stats
// --------------------------------------------------------------------------
extern "C" void app_main(void) {
#if CONFIG_HAND_DETECT_MODEL_IN_SDCARD
    ESP_LOGI(TAG, "model location: SD card (/sdcard/%s/)", CONFIG_HAND_DETECT_MODEL_SDCARD_DIR);
    if (sdcard_mount() != ESP_OK) return;
#elif CONFIG_HAND_DETECT_MODEL_IN_FLASH_PARTITION
    ESP_LOGI(TAG, "model location: flash partition 'hand_det'");
#else
    ESP_LOGI(TAG, "model location: flash rodata (baked into the app image)");
#endif

    if (camera_setup() != ESP_OK) return;

    s_slot_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(inference_task, "infer", INFER_INFERENCE_STACK, nullptr,
                            INFER_INFERENCE_PRIORITY, &s_infer_task, INFER_INFERENCE_CORE);
    xTaskCreatePinnedToCore(capture_task, "cam_cap", INFER_CAPTURE_STACK, nullptr,
                            INFER_CAPTURE_PRIORITY, nullptr, INFER_CAPTURE_CORE);

    uint32_t last_cam = 0, last_inf = 0, last_us = 0, last_drop = 0;
    int64_t  last_t = esp_timer_get_time();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(INFER_STATS_PERIOD_MS));
        int64_t now = esp_timer_get_time();
        double  dt  = (now - last_t) / 1e6;
        last_t = now;

        uint32_t cam = s_cam_frames, inf = s_infer_frames, us = s_infer_us_sum, drop = s_dropped;
        uint32_t d_inf = inf - last_inf;
        double cam_fps = (cam - last_cam) / dt;
        double inf_fps = d_inf / dt;
        double inf_ms  = d_inf ? (double)(us - last_us) / d_inf / 1000.0 : 0.0;
        uint32_t d_drop = drop - last_drop;
        last_cam = cam; last_inf = inf; last_us = us; last_drop = drop;

        if (s_hand_present) {
            ESP_LOGI(TAG, "cam %.1f fps | infer %.1f fps (%.0f ms/frame, %lu dropped) | hand %.2f box[%d,%d,%d,%d] rev=%d | %s",
                     cam_fps, inf_fps, inf_ms, (unsigned long)d_drop, (double)s_hand_score,
                     s_hand_box[0], s_hand_box[1], s_hand_box[2], s_hand_box[3],
                     s_reversals, s_waving ? "WAVING" : "-");
        } else {
            ESP_LOGI(TAG, "cam %.1f fps | infer %.1f fps (%.0f ms/frame, %lu dropped) | no hand",
                     cam_fps, inf_fps, inf_ms, (unsigned long)d_drop);
        }
    }
}
