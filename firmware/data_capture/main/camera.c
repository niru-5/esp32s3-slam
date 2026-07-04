#include "camera.h"

#include "esp_log.h"
#include "esp_timer.h"

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

esp_err_t camera_start(void) {
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
        .frame_size   = FRAMESIZE_VGA,
        .jpeg_quality = 12,
        .fb_count     = 1,
        .fb_location  = CAMERA_FB_IN_DRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
    };
    esp_err_t err = esp_camera_init(&cam_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed");
        return err;
    }
    ESP_LOGI(TAG, "Camera ready — JPEG VGA");
    return ESP_OK;
}

camera_fb_t *camera_grab(int64_t *ts_us) {
    // Timestamp at frame grab — consistent reference for the IMU clock offset.
    if (ts_us) *ts_us = esp_timer_get_time();
    return esp_camera_fb_get();
}

void camera_release(camera_fb_t *fb) {
    if (fb) esp_camera_fb_return(fb);
}
