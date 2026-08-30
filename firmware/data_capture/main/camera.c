#include "camera.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "config.h"

// --------------------------------------------------------------------------
// Camera pins (OV5640)
// --------------------------------------------------------------------------
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM     8
#define Y3_GPIO_NUM     9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM  13

static const char *TAG = "CAM";

static QueueHandle_t      s_camera_queue        = NULL;
static esp_timer_handle_t s_capture_timer       = NULL;
static TaskHandle_t       s_capture_task_handle = NULL;
static uint32_t           s_overflow_count       = 0;

esp_err_t camera_init(void) {
    camera_config_t cam_cfg = {
        .pin_pwdn     = PWDN_GPIO_NUM,
        .pin_reset    = RESET_GPIO_NUM,
        .pin_xclk     = XCLK_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM, .pin_d6 = Y8_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM, .pin_d4 = Y6_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM, .pin_d2 = Y4_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM, .pin_d0 = Y2_GPIO_NUM,
        .pin_vsync    = VSYNC_GPIO_NUM,
        .pin_href     = HREF_GPIO_NUM,
        .pin_pclk     = PCLK_GPIO_NUM,
        .xclk_freq_hz = 10000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_VGA, // FRAMESIZE_SVGA, // FRAMESIZE_VGA,
        .jpeg_quality = 12,
        .fb_count     = CONFIG_CAMERA_FB_COUNT,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
    };
    esp_err_t err = esp_camera_init(&cam_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed");
        return err;
    }
    ESP_LOGI(TAG, "Camera ready — JPEG VGA, %d PSRAM frame buffers", CONFIG_CAMERA_FB_COUNT);
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
    }
}

// --------------------------------------------------------------------------
// Public API
// --------------------------------------------------------------------------

esp_err_t camera_pipeline_start(void) {
    s_overflow_count = 0;
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
