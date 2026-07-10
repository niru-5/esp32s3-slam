#include "tcp_client.h"

#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "camera.h"
#include "imu.h"
#include "debug_time.h"

static const char *TAG = "TCPNET";

// One socket per stream, each written to by exactly one task -- no mutex
// needed (see tcp_client.h for why this replaced a single shared socket).
typedef struct {
    int         sock;
    uint16_t    port;
    const char *label;   // for logging only
} tcp_conn_t;

static tcp_conn_t s_frame_conn = { .sock = -1, .port = CONFIG_REMOTE_TCP_FRAME_PORT, .label = "frame" };
static tcp_conn_t s_imu_conn   = { .sock = -1, .port = CONFIG_REMOTE_TCP_IMU_PORT,   .label = "imu"   };
static tcp_conn_t s_stats_conn = { .sock = -1, .port = CONFIG_REMOTE_TCP_STATS_PORT, .label = "stats" };

static TaskHandle_t s_imu_consumer_handle    = NULL;
static TaskHandle_t s_camera_consumer_handle = NULL;

// --------------------------------------------------------------------------
// Socket plumbing
// --------------------------------------------------------------------------

static esp_err_t tcp_connect(tcp_conn_t *conn) {
    if (conn->sock >= 0) return ESP_OK;

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGW(TAG, "[%s] socket() failed: errno %d", conn->label, errno);
        return ESP_FAIL;
    }

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(conn->port);
    if (inet_pton(AF_INET, CONFIG_REMOTE_HOST, &addr.sin_addr) != 1) {
        ESP_LOGE(TAG, "[%s] inet_pton failed for host '%s'", conn->label, CONFIG_REMOTE_HOST);
        close(sock);
        return ESP_FAIL;
    }

    struct timeval snd_timeout = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &snd_timeout, sizeof(snd_timeout));

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGW(TAG, "[%s] connect to %s:%d failed: errno %d",
                 conn->label, CONFIG_REMOTE_HOST, conn->port, errno);
        close(sock);
        return ESP_FAIL;
    }

    int one = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    conn->sock = sock;
    ESP_LOGI(TAG, "[%s] connected to %s:%d", conn->label, CONFIG_REMOTE_HOST, conn->port);
    return ESP_OK;
}

static void tcp_close(tcp_conn_t *conn) {
    if (conn->sock >= 0) {
        close(conn->sock);
        conn->sock = -1;
    }
}

static esp_err_t send_all(int sock, const uint8_t *buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        int n = send(sock, buf + sent, len - sent, 0);
        if (n <= 0) return ESP_FAIL;
        sent += (size_t)n;
    }
    return ESP_OK;
}

// Send one length-prefixed message on `conn`: a 4-byte little-endian length
// then up to two payload chunks back-to-back (e.g. a timestamp followed by
// the JPEG bytes it belongs to) -- avoids an extra copy just to concatenate
// them first. `conn` has exactly one caller task, so no locking needed.
static esp_err_t send_msg(tcp_conn_t *conn,
                          const uint8_t *chunk1, size_t len1,
                          const uint8_t *chunk2, size_t len2) {
    esp_err_t err = tcp_connect(conn);
    if (err == ESP_OK) {
        uint8_t hdr[4];
        uint32_t total_len = (uint32_t)(len1 + len2);
        memcpy(hdr, &total_len, 4);
        err = send_all(conn->sock, hdr, sizeof(hdr));
        if (err == ESP_OK && len1) err = send_all(conn->sock, chunk1, len1);
        if (err == ESP_OK && len2) err = send_all(conn->sock, chunk2, len2);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "[%s] send failed, reconnecting next call", conn->label);
        tcp_close(conn);  // reconnect fresh next call
    }
    return err;
}

// --------------------------------------------------------------------------
// imu_tcp_consumer_task — same drain/serialize as net_client.c's
// imu_wifi_consumer_task, sent on its own connection instead of POSTed.
// --------------------------------------------------------------------------

#if CONFIG_ENABLE_IMU
static void imu_tcp_consumer_task(void *arg) {
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
        DEBUG_TIME_START(t_cap);
        uint32_t n = imu_queue_drain(samples, CONFIG_IMU_CONSUMER_BATCH, &ref_esp_us, &ref_ticks);
        DEBUG_TIME_END(t_cap, TAG, "imu draining");
        if (n == 0) continue;

        size_t wire_len = imu_serialize_wire(samples, n, ref_esp_us, ref_ticks, wire);

        DEBUG_TIME_START(t_send);
        send_msg(&s_imu_conn, wire, wire_len, NULL, 0);
        DEBUG_TIME_END(t_send, TAG, "imu sending");
    }
}
#endif // CONFIG_ENABLE_IMU

// --------------------------------------------------------------------------
// camera_tcp_consumer_task — same drain as net_client.c's
// camera_wifi_consumer_task; payload = ts_us (8 bytes) + JPEG bytes.
// --------------------------------------------------------------------------

#if CONFIG_ENABLE_CAMERA
static void camera_tcp_consumer_task(void *arg) {
    camera_frame_t *frames = malloc(CONFIG_CAMERA_QUEUE_LEN * sizeof(camera_frame_t));
    if (!frames) {
        ESP_LOGE(TAG, "out of memory for camera consumer buffer");
        vTaskDelete(NULL);
        return;
    }

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_CAMERA_CONSUMER_PERIOD_MS));

        uint32_t n = camera_queue_drain(frames, CONFIG_CAMERA_QUEUE_LEN);
        for (uint32_t i = 0; i < n; i++) {
            DEBUG_TIME_START(t_send);
            send_msg(&s_frame_conn,
                     (const uint8_t *)&frames[i].ts_us, sizeof(frames[i].ts_us),
                     frames[i].fb->buf, frames[i].fb->len);
            DEBUG_TIME_END(t_send, TAG, "camera sending");
            camera_release(frames[i].fb);
        }
    }
}
#endif // CONFIG_ENABLE_CAMERA

// --------------------------------------------------------------------------
// Public API
// --------------------------------------------------------------------------

esp_err_t tcp_client_pipeline_start(void) {
#if CONFIG_ENABLE_IMU
    if (xTaskCreatePinnedToCore(imu_tcp_consumer_task, "imu_tcp", 8192, NULL,
                                CONFIG_IMU_CONSUMER_PRIORITY, &s_imu_consumer_handle,
                                CONFIG_IMU_CONSUMER_CORE) != pdPASS)
        return ESP_ERR_NO_MEM;
#else
    ESP_LOGW(TAG, "IMU tcp streaming disabled (CONFIG_ENABLE_IMU=0)");
#endif

#if CONFIG_ENABLE_CAMERA
    if (xTaskCreatePinnedToCore(camera_tcp_consumer_task, "cam_tcp", 8192, NULL,
                                CONFIG_CAMERA_CONSUMER_PRIORITY, &s_camera_consumer_handle,
                                CONFIG_CAMERA_CONSUMER_CORE) != pdPASS) {
        if (s_imu_consumer_handle) { vTaskDelete(s_imu_consumer_handle); s_imu_consumer_handle = NULL; }
        return ESP_ERR_NO_MEM;
    }
#else
    ESP_LOGW(TAG, "camera tcp streaming disabled (CONFIG_ENABLE_CAMERA=0)");
#endif

    ESP_LOGI(TAG, "tcp consumers started -> %s frame:%u imu:%u stats:%u",
             CONFIG_REMOTE_HOST, (unsigned)CONFIG_REMOTE_TCP_FRAME_PORT,
             (unsigned)CONFIG_REMOTE_TCP_IMU_PORT, (unsigned)CONFIG_REMOTE_TCP_STATS_PORT);
    return ESP_OK;
}

void tcp_client_pipeline_stop(void) {
    if (s_imu_consumer_handle)    { vTaskDelete(s_imu_consumer_handle);    s_imu_consumer_handle    = NULL; }
    if (s_camera_consumer_handle) { vTaskDelete(s_camera_consumer_handle); s_camera_consumer_handle = NULL; }

    tcp_close(&s_frame_conn);
    tcp_close(&s_imu_conn);
    tcp_close(&s_stats_conn);
}

esp_err_t tcp_client_send_stats(const uint8_t *payload, size_t len) {
    return send_msg(&s_stats_conn, payload, len, NULL, 0);
}
