#include "tcp_client.h"

#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
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

// Cooperative shutdown (see tcp_client_pipeline_stop): consumer tasks are never vTaskDelete()d from
// outside. Killing a task while it is blocked inside lwIP (connect()/send()) leaves lwIP holding a
// semaphore that belonged to the dead task, and lwIP's own timer thread later panics signalling it.
static volatile bool s_stop  = false;
static volatile int  s_alive = 0;      // consumer tasks still running

// --------------------------------------------------------------------------
// Socket plumbing
// --------------------------------------------------------------------------

static esp_err_t tcp_connect(tcp_conn_t *conn) {
    if (conn->sock >= 0) return ESP_OK;
    if (s_stop) return ESP_FAIL;

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

    // Non-blocking connect polled in 200 ms slices (3 s overall) so a stop request is honoured
    // promptly even when the host is unreachable and the SYN is being dropped.
    int fl = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, fl | O_NONBLOCK);
    int rc = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
    int err = errno;
    if (rc != 0 && err == EINPROGRESS) {
        rc = -1;
        err = ETIMEDOUT;
        for (int i = 0; i < 15 && !s_stop; i++) {
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(sock, &wfds);
            struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
            if (select(sock + 1, NULL, &wfds, NULL, &tv) > 0) {
                int soerr = 0;
                socklen_t sl = sizeof(soerr);
                getsockopt(sock, SOL_SOCKET, SO_ERROR, &soerr, &sl);
                rc = soerr ? -1 : 0;
                err = soerr;
                break;
            }
        }
    }
    if (rc != 0) {
        if (!s_stop)
            ESP_LOGW(TAG, "[%s] connect to %s:%d failed: errno %d",
                     conn->label, CONFIG_REMOTE_HOST, conn->port, err);
        close(sock);
        return ESP_FAIL;
    }
    fcntl(sock, F_SETFL, fl);

    struct timeval snd_timeout = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &snd_timeout, sizeof(snd_timeout));
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
        if (s_stop) return ESP_FAIL;
        int n = send(sock, buf + sent, len - sent, 0);
        if (n <= 0) return ESP_FAIL;
        sent += (size_t)n;
    }
    return ESP_OK;
}

// Send one length-prefixed message on `conn`: a 4-byte little-endian length
// then up to two payload chunks (e.g. a timestamp followed by the JPEG bytes
// it belongs to). When chunk1 is small (e.g. the frame path's 8-byte
// timestamp) it's copied into the same stack buffer as the header and sent
// in one call, instead of a separate few-byte send() -- that used to show up
// on the wire as its own tiny TCP segment. chunk2 (the bulk payload, e.g. the
// JPEG) is still sent on its own to avoid copying it. `conn` has exactly one
// caller task, so no locking needed.
static esp_err_t send_msg(tcp_conn_t *conn,
                          const uint8_t *chunk1, size_t len1,
                          const uint8_t *chunk2, size_t len2) {
    esp_err_t err = tcp_connect(conn);
    if (err == ESP_OK) {
        uint32_t total_len = (uint32_t)(len1 + len2);
        uint8_t hdr_buf[4 + 16];
        if (len1 <= sizeof(hdr_buf) - 4) {
            memcpy(hdr_buf, &total_len, 4);
            if (len1) memcpy(hdr_buf + 4, chunk1, len1);
            err = send_all(conn->sock, hdr_buf, 4 + len1);
        } else {
            uint8_t hdr[4];
            memcpy(hdr, &total_len, 4);
            err = send_all(conn->sock, hdr, sizeof(hdr));
            if (err == ESP_OK) err = send_all(conn->sock, chunk1, len1);
        }
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
        send_msg(&s_imu_conn, wire, wire_len, NULL, 0);
        DEBUG_TIME_END(t_send, TAG, "imu sending");
    }
    free(samples);
    free(wire);
    s_alive--;
    vTaskDelete(NULL);
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
                send_msg(&s_frame_conn,
                         (const uint8_t *)&frames[i].ts_us, sizeof(frames[i].ts_us),
                         frames[i].fb->buf, frames[i].fb->len);
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

esp_err_t tcp_client_pipeline_start(void) {
    s_stop = false;
    s_alive = 0;
#if CONFIG_ENABLE_IMU
    s_alive++;
    if (xTaskCreatePinnedToCore(imu_tcp_consumer_task, "imu_tcp", 8192, NULL,
                                CONFIG_IMU_CONSUMER_PRIORITY, NULL,
                                CONFIG_IMU_CONSUMER_CORE) != pdPASS) {
        s_alive--;
        return ESP_ERR_NO_MEM;
    }
#else
    ESP_LOGW(TAG, "IMU tcp streaming disabled (CONFIG_ENABLE_IMU=0)");
#endif

#if CONFIG_ENABLE_CAMERA
    s_alive++;
    if (xTaskCreatePinnedToCore(camera_tcp_consumer_task, "cam_tcp", 8192, NULL,
                                CONFIG_CAMERA_CONSUMER_PRIORITY, NULL,
                                CONFIG_CAMERA_CONSUMER_CORE) != pdPASS) {
        s_alive--;
        tcp_client_pipeline_stop();
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

void tcp_client_pipeline_request_stop(void) {
    s_stop = true;
}

void tcp_client_pipeline_stop(void) {
    // Ask the consumers to finish and wait for them: every blocking call above is bounded (connect
    // <= 3 s, send <= 2 s per call) and checks s_stop, so this normally takes well under a second.
    s_stop = true;
    for (int i = 0; i < 300 && s_alive > 0; i++) vTaskDelay(pdMS_TO_TICKS(20));
    if (s_alive > 0) {
        // Never vTaskDelete a task that may be inside lwIP -- that is exactly the crash this avoids.
        // Leave the sockets alone too; the tasks will notice s_stop and exit on their own.
        ESP_LOGE(TAG, "%d tcp consumer task(s) still busy after 6 s; leaving them to finish", (int)s_alive);
        return;
    }
    tcp_close(&s_frame_conn);
    tcp_close(&s_imu_conn);
    tcp_close(&s_stats_conn);
}

esp_err_t tcp_client_send_stats(const uint8_t *payload, size_t len) {
    return send_msg(&s_stats_conn, payload, len, NULL, 0);
}
