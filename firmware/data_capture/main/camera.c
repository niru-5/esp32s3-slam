#include "camera.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "config.h"
#include "camera_overrides.h"

static const char *TAG = "CAM";

static QueueHandle_t      s_camera_queue        = NULL;
static esp_timer_handle_t s_capture_timer       = NULL;
static TaskHandle_t       s_capture_task_handle = NULL;
static uint32_t           s_overflow_count       = 0;

static int                s_current_jpeg_quality = CONFIG_CAMERA_JPEG_QUALITY_INITIAL;
static uint32_t           s_quality_last_overflow_count = 0;

static bool s_camera_up = false;

esp_err_t camera_init_ex(pixformat_t fmt, framesize_t size, int jpeg_quality,
                         int fb_count, bool raw8, bool grab_latest) {
    if (s_camera_up) {
        esp_camera_deinit();
        s_camera_up = false;
    }
    // RAW8: the ESP32-S3 DVP driver has no PIXFORMAT_RAW path (ll_cam.c rejects
    // it), so bring the sensor up as GRAYSCALE -- 1 byte/pixel, no conversion --
    // and afterwards flip the OV5640 ISP to its RAW (Bayer) output. The DMA path
    // can't tell the difference: it just copies width*height bytes.
    pixformat_t driver_fmt = raw8 ? PIXFORMAT_GRAYSCALE : fmt;

    camera_config_t cam_cfg = {
        .pin_pwdn     = CONFIG_CAM_PWDN_GPIO,
        .pin_reset    = CONFIG_CAM_RESET_GPIO,
        .pin_xclk     = CONFIG_CAM_XCLK_GPIO,
        .pin_sccb_sda = CONFIG_CAM_SIOD_GPIO,
        .pin_sccb_scl = CONFIG_CAM_SIOC_GPIO,
        .pin_d7 = CONFIG_CAM_Y9_GPIO, .pin_d6 = CONFIG_CAM_Y8_GPIO,
        .pin_d5 = CONFIG_CAM_Y7_GPIO, .pin_d4 = CONFIG_CAM_Y6_GPIO,
        .pin_d3 = CONFIG_CAM_Y5_GPIO, .pin_d2 = CONFIG_CAM_Y4_GPIO,
        .pin_d1 = CONFIG_CAM_Y3_GPIO, .pin_d0 = CONFIG_CAM_Y2_GPIO,
        .pin_vsync    = CONFIG_CAM_VSYNC_GPIO,
        .pin_href     = CONFIG_CAM_HREF_GPIO,
        .pin_pclk     = CONFIG_CAM_PCLK_GPIO,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = driver_fmt,
        .frame_size   = size,
        .jpeg_quality = jpeg_quality,
        .fb_count     = fb_count,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = grab_latest ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY,
    };
    esp_err_t err = esp_camera_init(&cam_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed (%s)", esp_err_to_name(err));
        return err;
    }
    s_camera_up = true;

    if (raw8) {
        sensor_t *sensor = esp_camera_sensor_get();
        // Same two writes as esp32-camera's own sensor_fmt_raw table
        // (0x501F ISP format = RAW, 0x4300 format control = RAW).
        if (sensor) {
            sensor->set_reg(sensor, 0x501F, 0xFF, 0x03);
            sensor->set_reg(sensor, 0x4300, 0xFF, 0x00);
        }
    }
    ESP_LOGI(TAG, "Camera ready — fmt=%d%s size=%d, %d PSRAM frame buffers",
             (int)fmt, raw8 ? "(raw8)" : "", (int)size, fb_count);
    return ESP_OK;
}

esp_err_t camera_init(void) {
    esp_err_t err = camera_init_ex(PIXFORMAT_JPEG, FRAMESIZE_SVGA, CONFIG_CAMERA_JPEG_QUALITY_INITIAL,
                                   CONFIG_CAMERA_FB_COUNT, false, false);
    if (err != ESP_OK) return err;
    // Re-apply any NVS-persisted register overrides from a previous calibration
    // session (see camera_overrides.h) -- deliberately only here, not in
    // camera_init_ex(), so a live calibration/tuning session (which reinits via
    // camera_init_ex directly) never gets its own half-tuned experiment
    // clobbered by a previously-saved set.
    camera_overrides_apply();
    return ESP_OK;
}

void camera_release(camera_fb_t *fb) {
    if (fb) esp_camera_fb_return(fb);
}

// --------------------------------------------------------------------------
// camera_capture_task — woken by esp_timer every
// CONFIG_CAMERA_CAPTURE_PERIOD_MS via task-notify (see camera.h / imu.h for
// why not a FreeRTOS software timer).
// --------------------------------------------------------------------------

static void capture_timer_cb(void *arg) {
    xTaskNotifyGive(s_capture_task_handle);
}

// 3-zone hysteresis on camera_queue_depth(), plus an overflow fast-path (see
// docs/architecture.md "Adaptive JPEG quality control"). Only touches the
// sensor -- an SCCB/I2C write -- when the target quality actually changes.
static void adjust_jpeg_quality(uint32_t queue_depth_after_enqueue) {
    uint32_t overflow_now = s_overflow_count;
    bool overflowed = overflow_now != s_quality_last_overflow_count;
    s_quality_last_overflow_count = overflow_now;

    int quality = s_current_jpeg_quality;
    if (overflowed) {
        quality += CONFIG_CAMERA_JPEG_QUALITY_OVERFLOW_STEP;
    } else if (queue_depth_after_enqueue >= CONFIG_CAMERA_JPEG_QUALITY_QUEUE_HIGH) {
        quality += CONFIG_CAMERA_JPEG_QUALITY_STEP;
    } else if (queue_depth_after_enqueue == 0) {
        quality -= CONFIG_CAMERA_JPEG_QUALITY_STEP;
    }

    if (quality < CONFIG_CAMERA_JPEG_QUALITY_MIN) quality = CONFIG_CAMERA_JPEG_QUALITY_MIN;
    if (quality > CONFIG_CAMERA_JPEG_QUALITY_MAX) quality = CONFIG_CAMERA_JPEG_QUALITY_MAX;

    if (quality != s_current_jpeg_quality) {
        sensor_t *sensor = esp_camera_sensor_get();
        if (sensor) sensor->set_quality(sensor, quality);
        s_current_jpeg_quality = quality;
        ESP_LOGI(TAG, "jpeg quality -> %d (queue depth %lu%s)", quality,
                 (unsigned long)queue_depth_after_enqueue, overflowed ? ", overflow" : "");
    }
}

static void camera_capture_task(void *arg) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        camera_frame_t frame;
        frame.ts_us = esp_timer_get_time();
        frame.fb    = esp_camera_fb_get();
        if (!frame.fb) continue;

        // Producer never blocks on a full queue -- drop the oldest frame
        // (releasing its buffer, or the PSRAM pool starves) to make room.
        if (xQueueSend(s_camera_queue, &frame, 0) != pdTRUE) {
            camera_frame_t discard;
            if (xQueueReceive(s_camera_queue, &discard, 0) == pdTRUE)
                camera_release(discard.fb);
            xQueueSend(s_camera_queue, &frame, 0);
            s_overflow_count++;
            ESP_LOGW(TAG, "camera_queue full, dropped oldest frame (overflow #%lu)",
                     (unsigned long)s_overflow_count);
        }

        adjust_jpeg_quality(camera_queue_depth());
    }
}

// --------------------------------------------------------------------------
// Public API
// --------------------------------------------------------------------------

esp_err_t camera_pipeline_start(void) {
    s_overflow_count = 0;

    // Reset adaptive quality state fresh each session, mirroring the queues/
    // overflow counters -- nothing carries over across a state transition.
    s_current_jpeg_quality = CONFIG_CAMERA_JPEG_QUALITY_INITIAL;
    s_quality_last_overflow_count = 0;
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) sensor->set_quality(sensor, s_current_jpeg_quality);

    s_camera_queue = xQueueCreate(CONFIG_CAMERA_QUEUE_LEN, sizeof(camera_frame_t));
    if (!s_camera_queue) return ESP_ERR_NO_MEM;

    if (xTaskCreatePinnedToCore(camera_capture_task, "cam_cap", 4096, NULL,
                                CONFIG_CAMERA_CAPTURE_PRIORITY, &s_capture_task_handle,
                                CONFIG_CAMERA_CAPTURE_CORE) != pdPASS) {
        vQueueDelete(s_camera_queue);
        s_camera_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = capture_timer_cb,
        .name     = "cam_cap_timer",
    };
    if (esp_timer_create(&timer_args, &s_capture_timer) != ESP_OK ||
        esp_timer_start_periodic(s_capture_timer, CONFIG_CAMERA_CAPTURE_PERIOD_MS * 1000ULL) != ESP_OK) {
        vTaskDelete(s_capture_task_handle);
        s_capture_task_handle = NULL;
        vQueueDelete(s_camera_queue);
        s_camera_queue = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "capture pipeline started (%d ms period, queue depth %d)",
             CONFIG_CAMERA_CAPTURE_PERIOD_MS, CONFIG_CAMERA_QUEUE_LEN);
    return ESP_OK;
}

void camera_pipeline_stop(void) {
    if (s_capture_timer) {
        esp_timer_stop(s_capture_timer);
        esp_timer_delete(s_capture_timer);
        s_capture_timer = NULL;
    }
    if (s_capture_task_handle) {
        vTaskDelete(s_capture_task_handle);
        s_capture_task_handle = NULL;
    }
    if (s_camera_queue) {
        camera_frame_t frame;
        while (xQueueReceive(s_camera_queue, &frame, 0) == pdTRUE)
            camera_release(frame.fb);
        vQueueDelete(s_camera_queue);
        s_camera_queue = NULL;
    }
}

uint32_t camera_queue_drain(camera_frame_t *out, uint32_t max_count) {
    uint32_t n = 0;
    if (s_camera_queue) {
        while (n < max_count && xQueueReceive(s_camera_queue, &out[n], 0) == pdTRUE) n++;
    }
    return n;
}

uint32_t camera_queue_depth(void) {
    return s_camera_queue ? (uint32_t)uxQueueMessagesWaiting(s_camera_queue) : 0;
}

uint32_t camera_queue_overflow_count(void) {
    return s_overflow_count;
}
