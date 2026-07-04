#include "net_client.h"

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "camera.h"
#include "imu.h"

static const char *TAG = "NET";

typedef struct {
    char     host[64];
    uint16_t port;
    int      fps;
} stream_cfg_t;

// POST a body to http://host:port<path>. `ts_us` (>=0) is sent as the
// X-Timestamp-Us header. Returns ESP_OK on a completed request.
static esp_err_t post_bytes(const stream_cfg_t *cfg, const char *path,
                            const char *content_type,
                            const uint8_t *body, size_t len, int64_t ts_us) {
    char url[128];
    snprintf(url, sizeof(url), "http://%s:%u%s", cfg->host, (unsigned)cfg->port, path);

    esp_http_client_config_t hc = {
        .url        = url,
        .method     = HTTP_METHOD_POST,
        .timeout_ms = 2000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&hc);
    if (!client) return ESP_FAIL;

    esp_http_client_set_header(client, "Content-Type", content_type);
    if (ts_us >= 0) {
        char ts_str[24];
        snprintf(ts_str, sizeof(ts_str), "%lld", ts_us);
        esp_http_client_set_header(client, "X-Timestamp-Us", ts_str);
    }
    esp_http_client_set_post_field(client, (const char *)body, len);

    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "POST %s failed: %s", path, esp_err_to_name(err));
    esp_http_client_cleanup(client);
    return err;
}

static void stream_task(void *arg) {
    stream_cfg_t *cfg = (stream_cfg_t *)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / (cfg->fps > 0 ? cfg->fps : 1));

    uint8_t *imu_buf = malloc(IMU_WIRE_MAXLEN);
    if (!imu_buf) {
        ESP_LOGE(TAG, "out of memory for IMU buffer");
        free(cfg);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "streaming to http://%s:%u at %d fps",
             cfg->host, (unsigned)cfg->port, cfg->fps);

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last_wake, period);

        // Frame with its grab timestamp.
        int64_t ts;
        camera_fb_t *fb = camera_grab(&ts);
        if (fb) {
            post_bytes(cfg, "/frame", "image/jpeg", fb->buf, fb->len, ts);
            camera_release(fb);
        }

        // IMU drain since last cycle.
        size_t imu_len = imu_serialize(imu_buf);
        post_bytes(cfg, "/imu", "application/octet-stream", imu_buf, imu_len, -1);
    }
}

esp_err_t net_client_start(const char *host, uint16_t port, int fps) {
    stream_cfg_t *cfg = calloc(1, sizeof(stream_cfg_t));
    if (!cfg) return ESP_ERR_NO_MEM;
    strncpy(cfg->host, host, sizeof(cfg->host) - 1);
    cfg->port = port;
    cfg->fps  = fps;

    // Runs on core 0 with WiFi; the IMU sampling task owns core 1.
    if (xTaskCreatePinnedToCore(stream_task, "net_stream", 8192, cfg, 5, NULL, 0) != pdPASS) {
        free(cfg);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
