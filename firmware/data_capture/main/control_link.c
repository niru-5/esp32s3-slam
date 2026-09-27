#include "control_link.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include "lwip/sockets.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "cJSON.h"

#include "config.h"
#include "state_machine.h"

static const char *TAG = "CTRL";

#define MSG_JSON 0x01

#define MAX_CMD_LEN 2048
#define TX_BUF_LEN  2048

static int               s_sock      = -1;
static SemaphoreHandle_t s_tx_mutex  = NULL;
static uint8_t           s_tx[TX_BUF_LEN];   // guarded by s_tx_mutex

// --------------------------------------------------------------------------
// Framing -- identical shape to cam_calib.c's, duplicated rather than shared
// because the two sockets/tasks have independent lifetimes and this channel
// never carries MSG_IMAGE.
// --------------------------------------------------------------------------

static esp_err_t send_all(const uint8_t *buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        int n = send(s_sock, buf + sent, len - sent, 0);
        if (n <= 0) return ESP_FAIL;
        sent += (size_t)n;
    }
    return ESP_OK;
}

// Shared by request replies and control_link_send_event() -- both format into
// s_tx under s_tx_mutex so a state_machine-task event can't interleave with a
// reply this task is mid-way through sending.
static esp_err_t send_jsonf_locked(const char *fmt, va_list ap) {
    if (s_sock < 0) return ESP_FAIL;
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    int n = vsnprintf((char *)s_tx + 5, TX_BUF_LEN - 5, fmt, ap);
    esp_err_t err;
    if (n < 0 || n >= TX_BUF_LEN - 5) {
        err = ESP_ERR_INVALID_SIZE;
    } else {
        uint32_t total = (uint32_t)(n + 1);
        memcpy(s_tx, &total, 4);
        s_tx[4] = MSG_JSON;
        err = send_all(s_tx, 5 + (size_t)n);
    }
    xSemaphoreGive(s_tx_mutex);
    return err;
}

static esp_err_t send_jsonf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static esp_err_t send_jsonf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    esp_err_t err = send_jsonf_locked(fmt, ap);
    va_end(ap);
    return err;
}

bool control_link_send_event(const char *json_fmt, ...) {
    if (s_sock < 0) return false;
    va_list ap;
    va_start(ap, json_fmt);
    esp_err_t err = send_jsonf_locked(json_fmt, ap);
    va_end(ap);
    return err == ESP_OK;
}

static esp_err_t recv_exact(uint8_t *buf, size_t len) {
    size_t got = 0;
    while (got < len) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(s_sock, &rfds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
        int r = select(s_sock + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) return ESP_FAIL;
        if (r == 0) continue;
        int n = recv(s_sock, buf + got, len - got, 0);
        if (n <= 0) return ESP_FAIL;
        got += (size_t)n;
    }
    return ESP_OK;
}

static bool json_int(const cJSON *obj, const char *key, int *out) {
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (!cJSON_IsNumber(it)) return false;
    *out = (int)it->valuedouble;
    return true;
}

static const char *json_str(const cJSON *obj, const char *key, const char *dflt) {
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    return cJSON_IsString(it) ? it->valuestring : dflt;
}

static esp_err_t reply_err(int id, const char *msg) {
    return send_jsonf("{\"id\":%d,\"ok\":false,\"err\":\"%s\"}", id, msg);
}

// --------------------------------------------------------------------------
// Commands
// --------------------------------------------------------------------------

static esp_err_t cmd_get_status(int id) {
    return send_jsonf("{\"id\":%d,\"ok\":true,\"state\":\"%s\",\"uptime_us\":%lld,"
                      "\"heap_free\":%u,\"psram_free\":%u}",
                      id, state_machine_state_name(state_machine_get_state()),
                      (long long)esp_timer_get_time(),
                      (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                      (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

// "state" -> the same digit the serial console would send for it.
static bool state_name_to_digit(const char *name, char *out) {
    static const struct { const char *name; char digit; } MAP[] = {
        {"stream_wifi", '1'}, {"stream_sdcard", '2'}, {"idle", '3'},
        {"imu_calibration", '4'}, {"camera_calibration", '5'}, {"stream_tcp", '6'},
    };
    for (size_t i = 0; i < sizeof(MAP) / sizeof(MAP[0]); i++)
        if (!strcmp(MAP[i].name, name)) { *out = MAP[i].digit; return true; }
    return false;
}

static esp_err_t cmd_set_state(int id, const cJSON *req) {
    const char *name = json_str(req, "state", "");
    char digit;
    if (!state_name_to_digit(name, &digit))
        return reply_err(id, "unknown state (idle|stream_wifi|stream_sdcard|stream_tcp|"
                             "camera_calibration|imu_calibration)");
    if (state_machine_post_command(digit) != ESP_OK)
        return reply_err(id, "command queue full -- try again");
    return send_jsonf("{\"id\":%d,\"ok\":true,\"requested\":\"%s\"}", id, name);
}

// {"axis": 1-6}: answers the imu_cal_preview prompt. Anything outside 1-6
// (including this command's own validation failure) aborts, same as serial.
static esp_err_t cmd_imu_cal_axis(int id, const cJSON *req) {
    int axis;
    if (!json_int(req, "axis", &axis)) return reply_err(id, "missing axis");
    char digit = (char)('0' + axis);
    if (!state_machine_post_imu_axis(digit))
        return reply_err(id, "not waiting for an axis answer (not in imu_calibration, or no prompt pending)");
    return send_jsonf("{\"id\":%d,\"ok\":true}", id);
}

static esp_err_t cmd_imu_cal_abort(int id) {
    state_machine_post_imu_axis('0');   // '0' is outside 1-6 -> select_gravity_axis() treats it as abort
    return send_jsonf("{\"id\":%d,\"ok\":true}", id);
}

static esp_err_t handle_command(const uint8_t *body, size_t len) {
    cJSON *req = cJSON_ParseWithLength((const char *)body, len);
    if (!req) return send_jsonf("{\"id\":-1,\"ok\":false,\"err\":\"bad json\"}");
    int id = -1;
    json_int(req, "id", &id);
    const char *cmd = json_str(req, "cmd", "");
    esp_err_t err;
    if      (!strcmp(cmd, "ping"))          err = send_jsonf("{\"id\":%d,\"ok\":true}", id);
    else if (!strcmp(cmd, "get_status"))    err = cmd_get_status(id);
    else if (!strcmp(cmd, "set_state"))     err = cmd_set_state(id, req);
    else if (!strcmp(cmd, "imu_cal_axis"))  err = cmd_imu_cal_axis(id, req);
    else if (!strcmp(cmd, "imu_cal_abort")) err = cmd_imu_cal_abort(id);
    else                                     err = reply_err(id, "unknown cmd");
    cJSON_Delete(req);
    return err;
}

// --------------------------------------------------------------------------
// Connection + task -- dial/retry/reconnect, same shape as cam_calib.c's
// connect_host()/serve_connection(), minus the stop flag (this channel is
// meant to run for the device's whole lifetime, not be torn down on a state
// transition).
// --------------------------------------------------------------------------

static void close_sock(void) {
    if (s_sock >= 0) { close(s_sock); s_sock = -1; }
}

static esp_err_t connect_host(void) {
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) return ESP_FAIL;
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(CONFIG_CONTROL_PORT);
    if (inet_pton(AF_INET, CONFIG_REMOTE_HOST, &addr.sin_addr) != 1) { close(sock); return ESP_FAIL; }

    int fl = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, fl | O_NONBLOCK);
    int rc = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
    if (rc != 0 && errno == EINPROGRESS) {
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(sock, &wfds);
        struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };
        if (select(sock + 1, NULL, &wfds, NULL, &tv) > 0) {
            int soerr = 0;
            socklen_t sl = sizeof(soerr);
            getsockopt(sock, SOL_SOCKET, SO_ERROR, &soerr, &sl);
            rc = soerr ? -1 : 0;
            errno = soerr;
        } else {
            rc = -1;
            errno = ETIMEDOUT;
        }
    }
    if (rc != 0) {
        ESP_LOGW(TAG, "connect to %s:%d failed: errno %d (retrying)", CONFIG_REMOTE_HOST, CONFIG_CONTROL_PORT, errno);
        close(sock);
        return ESP_FAIL;
    }
    fcntl(sock, F_SETFL, fl);
    struct timeval tmo = { .tv_sec = 5, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tmo, sizeof(tmo));
    int one = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    s_sock = sock;
    return ESP_OK;
}

static void serve_connection(void) {
    send_jsonf("{\"evt\":\"hello\",\"mode\":\"control\",\"state\":\"%s\",\"uptime_us\":%lld}",
              state_machine_state_name(state_machine_get_state()), (long long)esp_timer_get_time());

    uint8_t *cmd = malloc(MAX_CMD_LEN);
    if (!cmd) return;
    while (1) {
        uint32_t len;
        if (recv_exact((uint8_t *)&len, 4) != ESP_OK) break;
        if (len < 1 || len > MAX_CMD_LEN) { ESP_LOGW(TAG, "bad command length %u", (unsigned)len); break; }
        if (recv_exact(cmd, len) != ESP_OK) break;
        if (cmd[0] != MSG_JSON) continue;
        if (handle_command(cmd + 1, len - 1) != ESP_OK) break;
    }
    free(cmd);
}

static void control_link_task(void *arg) {
    ESP_LOGI(TAG, "control channel: connecting to host %s:%d", CONFIG_REMOTE_HOST, CONFIG_CONTROL_PORT);
    while (1) {
        esp_err_t err = connect_host();
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "connect_host() -> %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "control host connected");
        serve_connection();
        close_sock();
        ESP_LOGI(TAG, "control host disconnected (reconnecting)");
    }
}

esp_err_t control_link_start(void) {
    s_tx_mutex = xSemaphoreCreateMutex();
    if (!s_tx_mutex) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(control_link_task, "control_link", 6144, NULL, 4, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}
