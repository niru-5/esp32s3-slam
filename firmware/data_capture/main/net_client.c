#include "net_client.h"

#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "camera.h"
#include "imu.h"
#include "debug_time.h"

static const char *TAG = "NET";

// One persistent esp_http_client per producer -- imu/camera consumers run
// concurrently on core 0, and a single esp_http_client handle isn't safe to
// drive from two tasks at once (set_url/set_header/perform on a shared
// handle from two callers would interleave). A fresh client per POST was the
// original design and left closed sockets in TIME_WAIT; with
// LWIP_MAX_ACTIVE_TCP=16 and TCP_MSL=60s that pool exhausted within seconds
// at this request rate, so each of these keeps one HTTP/1.1 keep-alive
// connection open instead.
typedef struct {
    esp_http_client_handle_t client;
} net_conn_t;

static net_conn_t s_frame_conn = {0};
static net_conn_t s_imu_conn   = {0};
static net_conn_t s_stats_conn = {0};

// Cooperative shutdown: consumer tasks are never vTaskDelete()d from outside (a task killed while
// blocked inside lwIP/esp_http_client leaves lwIP signalling a dead task's semaphore -> panic in
// lwIP's timer thread). Every perform() is bounded by timeout_ms, so they notice s_stop promptly.
static volatile bool s_stop  = false;
static volatile int  s_alive = 0;

// POST a body to http://CONFIG_REMOTE_HOST:CONFIG_REMOTE_PORT<path>, reusing
// `conn`'s connection across calls. `ts_us` (>=0) is sent as the
// X-Timestamp-Us header. Returns ESP_OK on a completed request.
static esp_err_t post_bytes(net_conn_t *conn, const char *path,
                            const char *content_type,
                            const uint8_t *body, size_t len, int64_t ts_us) {
    char url[96];
    snprintf(url, sizeof(url), "http://%s:%u%s",
             CONFIG_REMOTE_HOST, (unsigned)CONFIG_REMOTE_PORT, path);

    if (!conn->client) {
        esp_http_client_config_t hc = {
            .url        = url,
            .method     = HTTP_METHOD_POST,
            .timeout_ms = 2000,
        };
        conn->client = esp_http_client_init(&hc);
        if (!conn->client) return ESP_FAIL;
    } else {
        esp_http_client_set_url(conn->client, url);
        esp_http_client_set_method(conn->client, HTTP_METHOD_POST);
    }

    esp_http_client_set_header(conn->client, "Content-Type", content_type);
    if (ts_us >= 0) {
        char ts_str[24];
        snprintf(ts_str, sizeof(ts_str), "%lld", ts_us);
        esp_http_client_set_header(conn->client, "X-Timestamp-Us", ts_str);
    } else {
        esp_http_client_delete_header(conn->client, "X-Timestamp-Us");
    }
    esp_http_client_set_post_field(conn->client, (const char *)body, len);

    esp_err_t err = esp_http_client_perform(conn->client);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "POST %s failed: %s", path, esp_err_to_name(err));
        // Connection may be wedged -- drop it and reconnect fresh next call.
        esp_http_client_cleanup(conn->client);
        conn->client = NULL;
    }
    return err;
}

static void close_conn(net_conn_t *conn) {
    if (conn->client) {
        esp_http_client_cleanup(conn->client);
        conn->client = NULL;
    }
}

// --------------------------------------------------------------------------
// imu_wifi_consumer_task — drains imu_queue every CONFIG_IMU_CONSUMER_PERIOD_MS
// and POSTs the batch in the same wire format the ring-buffer drain used to
// produce (see imu.h).
// --------------------------------------------------------------------------

#if CONFIG_ENABLE_IMU
static void imu_wifi_consumer_task(void *arg) {
    const size_t max_len = IMU_WIRE_HEADER_LEN + CONFIG_IMU_CONSUMER_BATCH * IMU_WIRE_SAMPLE_LEN;
    imu_sample_t *samples = malloc(CONFIG_IMU_CONSUMER_BATCH * sizeof(imu_sample_t));
    uint8_t      *wire    = malloc(max_len);
    if (!samples || !wire) {
        ESP_LOGE(TAG, "out of memory for IMU consumer buffers");
        free(samples); free(wire);
        s_alive--;
        vTaskDelete(NULL);
        return;
    }

    TickType_t last_wake = xTaskGetTickCount();
    while (!s_stop) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_IMU_CONSUMER_PERIOD_MS));
        if (s_stop) break;

        int64_t  ref_esp_us;
        uint32_t ref_ticks;
        DEBUG_TIME_START(t_cap);
        uint32_t n = imu_queue_drain(samples, CONFIG_IMU_CONSUMER_BATCH, &ref_esp_us, &ref_ticks);
        DEBUG_TIME_END(t_cap, TAG, "imu draining");
        if (n == 0) continue;

        size_t wire_len = imu_serialize_wire(samples, n, ref_esp_us, ref_ticks, wire);

        DEBUG_TIME_START(t_send);
        post_bytes(&s_imu_conn, "/imu", "application/octet-stream", wire, wire_len, -1);
        DEBUG_TIME_END(t_send, TAG, "imu sending");
    }
    free(samples);
    free(wire);
    s_alive--;
    vTaskDelete(NULL);
}
#endif // CONFIG_ENABLE_IMU

// --------------------------------------------------------------------------
// camera_wifi_consumer_task — drains camera_queue every
// CONFIG_CAMERA_CONSUMER_PERIOD_MS and POSTs each frame.
// --------------------------------------------------------------------------

#if CONFIG_ENABLE_CAMERA
static void camera_wifi_consumer_task(void *arg) {
    camera_frame_t *frames = malloc(CONFIG_CAMERA_QUEUE_LEN * sizeof(camera_frame_t));
    if (!frames) {
        ESP_LOGE(TAG, "out of memory for camera consumer buffer");
        s_alive--;
        vTaskDelete(NULL);
        return;
    }

    TickType_t last_wake = xTaskGetTickCount();
    while (!s_stop) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_CAMERA_CONSUMER_PERIOD_MS));

        uint32_t n = camera_queue_drain(frames, CONFIG_CAMERA_QUEUE_LEN);
        for (uint32_t i = 0; i < n; i++) {
            if (!s_stop) {
                DEBUG_TIME_START(t_send);
                post_bytes(&s_frame_conn, "/frame", "image/jpeg",
                          frames[i].fb->buf, frames[i].fb->len, frames[i].ts_us);
                DEBUG_TIME_END(t_send, TAG, "camera sending");
            }
            camera_release(frames[i].fb);   // always -- also for frames drained but not sent on stop
        }
    }
    free(frames);
    s_alive--;
    vTaskDelete(NULL);
}
#endif // CONFIG_ENABLE_CAMERA

// --------------------------------------------------------------------------
// Public API
// --------------------------------------------------------------------------

esp_err_t net_client_pipeline_start(void) {
    s_stop = false;
    s_alive = 0;
#if CONFIG_ENABLE_IMU
    s_alive++;
    if (xTaskCreatePinnedToCore(imu_wifi_consumer_task, "imu_wifi", 8192, NULL,
                                CONFIG_IMU_CONSUMER_PRIORITY, NULL,
                                CONFIG_IMU_CONSUMER_CORE) != pdPASS) {
        s_alive--;
        return ESP_ERR_NO_MEM;
    }
#else
    ESP_LOGW(TAG, "IMU wifi streaming disabled (CONFIG_ENABLE_IMU=0)");
#endif

#if CONFIG_ENABLE_CAMERA
    s_alive++;
    if (xTaskCreatePinnedToCore(camera_wifi_consumer_task, "cam_wifi", 8192, NULL,
                                CONFIG_CAMERA_CONSUMER_PRIORITY, NULL,
                                CONFIG_CAMERA_CONSUMER_CORE) != pdPASS) {
        s_alive--;
        net_client_pipeline_stop();
        return ESP_ERR_NO_MEM;
    }
#else
    ESP_LOGW(TAG, "camera wifi streaming disabled (CONFIG_ENABLE_CAMERA=0)");
#endif

    ESP_LOGI(TAG, "wifi consumers started -> http://%s:%u",
             CONFIG_REMOTE_HOST, (unsigned)CONFIG_REMOTE_PORT);
    return ESP_OK;
}

void net_client_pipeline_request_stop(void) {
    s_stop = true;
}

void net_client_pipeline_stop(void) {
    s_stop = true;
    for (int i = 0; i < 500 && s_alive > 0; i++) vTaskDelay(pdMS_TO_TICKS(20));
    if (s_alive > 0) {
        ESP_LOGE(TAG, "%d wifi consumer task(s) still busy after 10 s; leaving them to finish", (int)s_alive);
        return;
    }
    close_conn(&s_frame_conn);
    close_conn(&s_imu_conn);
    close_conn(&s_stats_conn);
}

esp_err_t net_client_send_stats(const uint8_t *payload, size_t len) {
    if (s_stop) return ESP_FAIL;
    return post_bytes(&s_stats_conn, "/stats", "application/octet-stream", payload, len, -1);
}
