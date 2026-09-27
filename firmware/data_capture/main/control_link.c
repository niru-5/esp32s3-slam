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
#include "cam_calib.h"
#include "camera.h"
#include "camera_overrides.h"
#include "imu.h"

static const char *TAG = "CTRL";

#define MSG_JSON  0x01
#define MSG_IMAGE 0x02

// Sized for calibration-mode commands (image metadata, register dumps), not just the small
// status/state JSON this channel started with -- see the "merge cam_calib.c" note below.
#define MAX_CMD_LEN 4096
#define TX_BUF_LEN  4096

static int               s_sock      = -1;
static SemaphoreHandle_t s_tx_mutex  = NULL;
// PSRAM, not internal DRAM: cmd_set_mode()/camera_init_ex() needs a ~32 KB contiguous
// internal-DRAM block for the camera DMA buffer when switching to a non-JPEG mode, and every
// small internal allocation this module keeps makes that fail (see camera_init_ex / RAW8).
// Allocated once in control_link_start(), for the task's whole lifetime.
static uint8_t           *s_tx       = NULL;
static uint32_t           s_img_seq  = 0;

// --------------------------------------------------------------------------
// Framing. Single always-on socket (device dials CONFIG_REMOTE_HOST:
// CONFIG_CONTROL_PORT) carrying everything: state control, camera
// calibration (merged in from what used to be cam_calib.c's own dedicated
// socket -- see cam_calib.h for why that split existed and no longer needs
// to), and the IMU calibration flow. s_tx_mutex guards every outgoing
// message (a reply mid-build must never interleave with an unprompted event
// from state_machine.c's IMU calibration flow, which runs on a different
// task) -- each function below takes/gives it around exactly one message.
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

// s_tx[5 .. 5+len) already holds the JSON text; prefix the header and send it
// in one call. Caller must hold s_tx_mutex.
static esp_err_t send_built_locked(size_t len) {
    uint32_t total = (uint32_t)(len + 1);
    memcpy(s_tx, &total, 4);
    s_tx[4] = MSG_JSON;
    return send_all(s_tx, 5 + len);
}

static esp_err_t send_jsonf_locked(const char *fmt, va_list ap) {
    if (s_sock < 0) return ESP_FAIL;
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    int n = vsnprintf((char *)s_tx + 5, TX_BUF_LEN - 5, fmt, ap);
    esp_err_t err;
    if (n < 0 || n >= TX_BUF_LEN - 5) {
        err = ESP_ERR_INVALID_SIZE;
    } else {
        err = send_built_locked((size_t)n);
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
// State / status commands -- always available, any app_state_t.
// --------------------------------------------------------------------------

static esp_err_t cmd_get_status(int id) {
    return send_jsonf("{\"id\":%d,\"ok\":true,\"state\":\"%s\",\"uptime_us\":%lld,"
                      "\"heap_free\":%u,\"psram_free\":%u,\"imu_available\":%s}",
                      id, state_machine_state_name(state_machine_get_state()),
                      (long long)esp_timer_get_time(),
                      (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                      (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                      imu_available() ? "true" : "false");
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

// {"state": "...", "fps": n, "framesize": "...", "include_imu": true|false}. fps/framesize/
// include_imu only matter for the stream_* states (see state_machine.h enter_stream_*());
// harmlessly ignored for other states. digit + params are posted as one queue item (not two
// separate calls) so a second set_state arriving before main_state_machine_task drains the
// first can't overwrite its params out from under it.
static esp_err_t cmd_set_state(int id, const cJSON *req) {
    const char *name = json_str(req, "state", "");
    char digit;
    if (!state_name_to_digit(name, &digit))
        return reply_err(id, "unknown state (idle|stream_wifi|stream_sdcard|stream_tcp|"
                             "camera_calibration|imu_calibration)");
    int fps = 0, include_imu = 0;
    json_int(req, "fps", &fps);
    json_int(req, "include_imu", &include_imu);
    const char *framesize = json_str(req, "framesize", "");
    esp_err_t err = state_machine_post_stream_command(digit, (uint32_t)(fps < 0 ? 0 : fps),
                                                       framesize, include_imu != 0);
    if (err == ESP_ERR_INVALID_ARG) return reply_err(id, "unknown framesize");
    if (err != ESP_OK) return reply_err(id, "command queue full -- try again");
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

// --------------------------------------------------------------------------
// Camera calibration commands -- merged in from what used to be cam_calib.c's
// own dedicated socket/task (CONFIG_CAM_CALIB_PORT). That split only existed
// so two separate host processes wouldn't contend for one port; with a
// single host process (software/host_server) there's no more reason for a
// second socket, so this channel now carries it too, valid only while
// APP_STATE_CAMERA_CALIBRATION is active (checked once in handle_command()).
// --------------------------------------------------------------------------

// Registers snapshotted alongside every captured frame ("regs":"key"): exposure/
// gain/AWB live values, orientation, timing, BLC, ISP enables. See
// docs/camera_calibration_and_tuning.md for what each one is.
static const uint16_t KEY_REGS[] = {
    0x3500, 0x3501, 0x3502,             // exposure (E*16)
    0x3503,                             // AEC/AGC manual bits
    0x350A, 0x350B,                     // gain
    0x3400, 0x3401, 0x3402, 0x3403, 0x3404, 0x3405, 0x3406,   // AWB gains / manual
    0x3808, 0x3809, 0x380A, 0x380B,     // output size
    0x380C, 0x380D, 0x380E, 0x380F,     // HTS, VTS
    0x3820, 0x3821,                     // flip / mirror
    0x3A00, 0x3A08, 0x3A09, 0x3A0A, 0x3A0B, // AEC ctrl + B50/B60 steps
    0x3C00, 0x3C01,                     // banding auto/manual
    0x4000, 0x4003, 0x4005, 0x4009,     // BLC
    0x4300, 0x501F,                     // output format / ISP format
    0x5000, 0x5001,                     // ISP enables (LENC/gamma/DPC, AWB/CMX)
    0x5308, 0x5302, 0x5303,             // CIP sharpen/denoise
    0x519F, 0x51A0, 0x51A1, 0x51A2, 0x51A3, 0x51A4,   // AWB gain readback
};

static bool parse_fmt(const char *name, pixformat_t *out, bool *raw8) {
    *raw8 = false;
    if (!strcmp(name, "jpeg"))   { *out = PIXFORMAT_JPEG;      return true; }
    if (!strcmp(name, "raw8"))   { *out = PIXFORMAT_GRAYSCALE; *raw8 = true; return true; }
    if (!strcmp(name, "gray"))   { *out = PIXFORMAT_GRAYSCALE; return true; }
    if (!strcmp(name, "rgb565")) { *out = PIXFORMAT_RGB565;    return true; }
    if (!strcmp(name, "yuv422")) { *out = PIXFORMAT_YUV422;    return true; }
    return false;
}

static int reg_get(uint16_t addr) {
    sensor_t *s = esp_camera_sensor_get();
    return s ? s->get_reg(s, addr, 0xFF) : -1;
}

static esp_err_t cmd_info(int id) {
    sensor_t *s = esp_camera_sensor_get();
    return send_jsonf(
        "{\"id\":%d,\"ok\":true,\"sensor_pid\":%d,\"fmt\":\"%s\",\"size\":\"%s\","
        "\"quality\":%d,\"uptime_us\":%lld,\"heap_free\":%u,\"psram_free\":%u,\"fw_build\":\"%s %s\"}",
        id, s ? s->id.PID : -1, cam_calib_fmt(), cam_calib_size(), s ? s->status.quality : -1,
        (long long)esp_timer_get_time(),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM), __DATE__, __TIME__);
}

static esp_err_t cmd_set_mode(int id, const cJSON *req) {
    const char *fmt_s  = json_str(req, "format", cam_calib_fmt());
    const char *size_s = json_str(req, "framesize", cam_calib_size());
    int quality = 6, fb_count = 2;
    json_int(req, "quality", &quality);
    json_int(req, "fb_count", &fb_count);
    if (fb_count < 1) fb_count = 1;
    if (fb_count > 5) fb_count = 5;

    pixformat_t fmt; bool raw8; framesize_t size;
    if (!parse_fmt(fmt_s, &fmt, &raw8))          return reply_err(id, "unknown format (jpeg|raw8|gray|rgb565|yuv422)");
    if (!camera_parse_framesize(size_s, &size))  return reply_err(id, "unknown framesize");

    char old_fmt[8], old_size[8];
    strlcpy(old_fmt, cam_calib_fmt(), sizeof(old_fmt));
    strlcpy(old_size, cam_calib_size(), sizeof(old_size));

    esp_err_t err = camera_init_ex(fmt, size, quality, fb_count, raw8, true);
    if (err != ESP_OK) {
        // Roll back so the session stays usable.
        pixformat_t ofmt; bool oraw; framesize_t osize;
        parse_fmt(old_fmt, &ofmt, &oraw); camera_parse_framesize(old_size, &osize);
        camera_init_ex(ofmt, osize, 6, 2, oraw, true);
        return reply_err(id, "camera_init_ex failed (out of PSRAM? try fb_count 1)");
    }
    cam_calib_set_mode(fmt_s, size_s);

    // Let AEC/AWB settle and flush the first frames after a reinit.
    for (int i = 0; i < 3; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
    }
    return cmd_info(id);
}

static esp_err_t cmd_reg_read(int id, const cJSON *req) {
    int addr, count = 1;
    if (!json_int(req, "addr", &addr)) return reply_err(id, "missing addr");
    json_int(req, "count", &count);
    if (count < 1 || count > 1024) return reply_err(id, "count 1..1024");
    char *hex = malloc((size_t)count * 2 + 1);
    if (!hex) return reply_err(id, "oom");
    for (int i = 0; i < count; i++) {
        int v = reg_get((uint16_t)(addr + i));
        sprintf(hex + i * 2, "%02x", v < 0 ? 0 : v);
    }
    esp_err_t err = send_jsonf("{\"id\":%d,\"ok\":true,\"addr\":%d,\"hex\":\"%s\"}", id, addr, hex);
    free(hex);
    return err;
}

// {"writes":[{"a":addr,"v":val,"m":mask(default 255)}, ...], "delay_ms": n}
// Replies with per-write old/new bytes (full register byte, not masked) so the
// host can build an undo log.
static esp_err_t cmd_reg_write(int id, const cJSON *req) {
    const cJSON *writes = cJSON_GetObjectItemCaseSensitive(req, "writes");
    if (!cJSON_IsArray(writes)) return reply_err(id, "missing writes[]");
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return reply_err(id, "no sensor");

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    int off = snprintf((char *)s_tx + 5, TX_BUF_LEN - 5, "{\"id\":%d,\"ok\":true,\"results\":[", id);
    bool first = true;
    bool too_many = false;
    const cJSON *w;
    cJSON_ArrayForEach(w, writes) {
        int a, v, m = 0xFF;
        if (!json_int(w, "a", &a) || !json_int(w, "v", &v)) {
            xSemaphoreGive(s_tx_mutex);
            return reply_err(id, "bad write entry");
        }
        json_int(w, "m", &m);
        int old = reg_get((uint16_t)a);
        int rc = s->set_reg(s, a, m & 0xFF, v);
        int now = reg_get((uint16_t)a);
        if (off > TX_BUF_LEN - 80) { too_many = true; break; }
        off += snprintf((char *)s_tx + 5 + off, TX_BUF_LEN - 5 - off,
                        "%s{\"a\":%d,\"old\":%d,\"new\":%d,\"rc\":%d}", first ? "" : ",", a, old, now, rc);
        first = false;
    }
    if (too_many) { xSemaphoreGive(s_tx_mutex); return reply_err(id, "too many writes in one message"); }
    int dly = 0;
    json_int(req, "delay_ms", &dly);
    if (dly > 0) vTaskDelay(pdMS_TO_TICKS(dly));
    off += snprintf((char *)s_tx + 5 + off, TX_BUF_LEN - 5 - off, "]}");
    esp_err_t err = send_built_locked((size_t)off);
    xSemaphoreGive(s_tx_mutex);
    return err;
}

// {"ranges":[[start,end],...]}: streams "reg_chunk" events (256 regs each, hex),
// then the final reply. The OV5640 has ~12k meaningful registers; at SCCB speed
// a full dump takes several seconds. Each chunk is its own mutex-guarded message
// (not the whole dump under one lock), so a long dump doesn't starve other events.
static esp_err_t cmd_reg_dump(int id, const cJSON *req) {
    const cJSON *ranges = cJSON_GetObjectItemCaseSensitive(req, "ranges");
    if (!cJSON_IsArray(ranges)) return reply_err(id, "missing ranges[]");
    int64_t t0 = esp_timer_get_time();
    int total = 0;
    const cJSON *r;
    cJSON_ArrayForEach(r, ranges) {
        const cJSON *a = cJSON_GetArrayItem(r, 0), *b = cJSON_GetArrayItem(r, 1);
        if (!cJSON_IsNumber(a) || !cJSON_IsNumber(b)) return reply_err(id, "bad range");
        for (int start = (int)a->valuedouble; start <= (int)b->valuedouble; start += 256) {
            int n = (int)b->valuedouble - start + 1;
            if (n > 256) n = 256;
            char hex[513];
            for (int i = 0; i < n; i++) {
                int v = reg_get((uint16_t)(start + i));
                sprintf(hex + i * 2, "%02x", v < 0 ? 0 : v);
            }
            esp_err_t e = send_jsonf("{\"evt\":\"reg_chunk\",\"id\":%d,\"start\":%d,\"hex\":\"%s\"}", id, start, hex);
            if (e != ESP_OK) return e;
            total += n;
        }
    }
    return send_jsonf("{\"id\":%d,\"ok\":true,\"regs\":%d,\"ms\":%d}", id, total,
                      (int)((esp_timer_get_time() - t0) / 1000));
}

// {"mirror":0|1,"flip":0|1}: goes through the driver (not raw 0x3820/0x3821
// writes) because set_hmirror/set_vflip also fix up the OV5640's 0x4514/0x4520
// black-level-line registers for the new readout direction.
static esp_err_t cmd_set_orientation(int id, const cJSON *req) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return reply_err(id, "no sensor");
    int mirror, flip;
    if (!json_int(req, "mirror", &mirror) || !json_int(req, "flip", &flip)) return reply_err(id, "need mirror and flip");
    int old20 = reg_get(0x3820), old21 = reg_get(0x3821);
    s->set_hmirror(s, mirror ? 1 : 0);
    s->set_vflip(s, flip ? 1 : 0);
    return send_jsonf("{\"id\":%d,\"ok\":true,\"old_3820\":%d,\"old_3821\":%d,\"new_3820\":%d,\"new_3821\":%d,\"new_4514\":%d}",
                      id, old20, old21, reg_get(0x3820), reg_get(0x3821), reg_get(0x4514));
}

// {"n":40}: grab n frames back to back WITHOUT sending them and return the
// driver's per-frame completion timestamps -- the sensor's true frame cadence,
// unaffected by how fast we can push image data over WiFi.
static esp_err_t cmd_fps_probe(int id, const cJSON *req) {
    int n = 30;
    json_int(req, "n", &n);
    if (n < 2 || n > 64) return reply_err(id, "n must be 2..64");
    int64_t ts[64];
    int got = 0;
    for (int i = 0; i < n; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) break;
        ts[got++] = (int64_t)fb->timestamp.tv_sec * 1000000LL + fb->timestamp.tv_usec;
        esp_camera_fb_return(fb);
    }
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    int off = snprintf((char *)s_tx + 5, TX_BUF_LEN - 5, "{\"id\":%d,\"ok\":true,\"ts\":[", id);
    for (int i = 0; i < got; i++)
        off += snprintf((char *)s_tx + 5 + off, TX_BUF_LEN - 5 - off, "%s%lld", i ? "," : "", (long long)ts[i]);
    off += snprintf((char *)s_tx + 5 + off, TX_BUF_LEN - 5 - off, "]}");
    esp_err_t err = send_built_locked((size_t)off);
    xSemaphoreGive(s_tx_mutex);
    return err;
}

static const char *pix_name(pixformat_t f) {
    switch (f) {
    case PIXFORMAT_JPEG: return "jpeg";
    case PIXFORMAT_GRAYSCALE: return "gray";
    case PIXFORMAT_RGB565: return "rgb565";
    case PIXFORMAT_YUV422: return "yuv422";
    default: return "other";
    }
}

// Sends one IMAGE message: header + meta JSON in s_tx, then the frame bytes.
static esp_err_t send_image(camera_fb_t *fb, int64_t ts_us, uint32_t seq, const char *tag, bool with_regs) {
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    char *meta = (char *)s_tx + 5 + 4;
    size_t cap = TX_BUF_LEN - 5 - 4;
    const char *fmt_name = strcmp(cam_calib_fmt(), "raw8") == 0 ? "raw8" : pix_name(fb->format);
    int n = snprintf(meta, cap,
        "{\"seq\":%u,\"ts_us\":%lld,\"frame_ts_us\":%lld,\"fmt\":\"%s\",\"w\":%u,\"h\":%u,\"len\":%u,\"size\":\"%s\",\"tag\":\"%s\"",
        (unsigned)seq, (long long)ts_us,
        (long long)fb->timestamp.tv_sec * 1000000LL + fb->timestamp.tv_usec, fmt_name, (unsigned)fb->width, (unsigned)fb->height,
        (unsigned)fb->len, cam_calib_size(), tag);
    if (with_regs) {
        n += snprintf(meta + n, cap - n, ",\"regs\":{");
        for (size_t i = 0; i < sizeof(KEY_REGS) / sizeof(KEY_REGS[0]); i++) {
            n += snprintf(meta + n, cap - n, "%s\"%04x\":%d", i ? "," : "", KEY_REGS[i], reg_get(KEY_REGS[i]));
        }
        n += snprintf(meta + n, cap - n, "}");
    }
    n += snprintf(meta + n, cap - n, "}");
    if (n <= 0 || (size_t)n >= cap) { xSemaphoreGive(s_tx_mutex); return ESP_ERR_INVALID_SIZE; }

    uint32_t meta_len = (uint32_t)n;
    uint32_t total = 1 + 4 + meta_len + (uint32_t)fb->len;
    memcpy(s_tx, &total, 4);
    s_tx[4] = MSG_IMAGE;
    memcpy(s_tx + 5, &meta_len, 4);
    esp_err_t err = send_all(s_tx, 5 + 4 + meta_len);
    if (err == ESP_OK) err = send_all(fb->buf, fb->len);
    xSemaphoreGive(s_tx_mutex);
    return err;
}

// {"n":5,"interval_ms":300,"flush":2,"regs":"key"|"none","tag":"..."}
static esp_err_t cmd_capture(int id, const cJSON *req) {
    int n = 1, interval_ms = 0, flush = 2;
    json_int(req, "n", &n);
    json_int(req, "interval_ms", &interval_ms);
    json_int(req, "flush", &flush);
    if (n < 1 || n > 200) return reply_err(id, "n must be 1..200");
    bool with_regs = strcmp(json_str(req, "regs", "key"), "none") != 0;
    char tag[32];
    strlcpy(tag, json_str(req, "tag", ""), sizeof(tag));
    for (char *p = tag; *p; p++) if (*p == '"' || *p == '\\') *p = '_';

    // A register write may not show up until a couple of frames later, and with
    // GRAB_LATEST the first frame out can still predate it.
    for (int i = 0; i < flush; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
    }

    int captured = 0;
    for (int i = 0; i < n; i++) {
        int64_t ts = esp_timer_get_time();
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) return reply_err(id, "fb_get failed");
        esp_err_t err = send_image(fb, ts, s_img_seq++, tag, with_regs);
        esp_camera_fb_return(fb);
        if (err != ESP_OK) return err;
        captured++;
        if (interval_ms > 0 && i + 1 < n) vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
    return send_jsonf("{\"id\":%d,\"ok\":true,\"captured\":%d}", id, captured);
}

// --------------------------------------------------------------------------
// Persisted register overrides (camera_overrides.h) -- "save tuned registers"
// half of the workflow. The host computes *what* is safe to persist (see
// software/host_server/calibration/registers.py RegisterBank.save_candidates,
// which reuses the same UNSAFE_RESTORE/volatile() filtering `restore` already
// applies) and sends the final list here; this module just stores/applies it.
// --------------------------------------------------------------------------

// {"writes":[{"a":addr,"v":val}, ...]}
static esp_err_t cmd_save_camera_regs(int id, const cJSON *req) {
    const cJSON *writes = cJSON_GetObjectItemCaseSensitive(req, "writes");
    if (!cJSON_IsArray(writes)) return reply_err(id, "missing writes[]");
    int n = cJSON_GetArraySize(writes);
    if (n < 1) return reply_err(id, "writes[] must be non-empty");

    camera_override_write_t *buf = malloc((size_t)n * sizeof(camera_override_write_t));
    if (!buf) return reply_err(id, "oom");
    int i = 0;
    const cJSON *w;
    cJSON_ArrayForEach(w, writes) {
        int a, v;
        if (!json_int(w, "a", &a) || !json_int(w, "v", &v)) { free(buf); return reply_err(id, "bad write entry"); }
        buf[i].addr = (uint16_t)a;
        buf[i].val  = (uint8_t)v;
        i++;
    }
    esp_err_t err = camera_overrides_save(buf, (size_t)i);
    free(buf);
    if (err == ESP_ERR_NO_MEM)
        return send_jsonf("{\"id\":%d,\"ok\":false,\"err\":\"too many saved registers (limit %d total)\"}",
                          id, CAMERA_OVERRIDES_MAX_REGS);
    if (err != ESP_OK) return reply_err(id, "save failed");

    camera_override_write_t saved[CAMERA_OVERRIDES_MAX_REGS];
    size_t total = camera_overrides_get(saved, CAMERA_OVERRIDES_MAX_REGS);
    return send_jsonf("{\"id\":%d,\"ok\":true,\"saved_this_call\":%d,\"total_saved\":%u}", id, i, (unsigned)total);
}

static esp_err_t cmd_clear_camera_regs(int id) {
    esp_err_t err = camera_overrides_clear();
    if (err != ESP_OK) return reply_err(id, "clear failed");
    return send_jsonf("{\"id\":%d,\"ok\":true}", id);
}

static esp_err_t cmd_get_camera_overrides(int id) {
    camera_override_write_t regs[CAMERA_OVERRIDES_MAX_REGS];
    size_t count = camera_overrides_get(regs, CAMERA_OVERRIDES_MAX_REGS);
    if (count > CAMERA_OVERRIDES_MAX_REGS) count = CAMERA_OVERRIDES_MAX_REGS;  // get() may report more than it copied
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    int off = snprintf((char *)s_tx + 5, TX_BUF_LEN - 5, "{\"id\":%d,\"ok\":true,\"regs\":[", id);
    for (size_t i = 0; i < count; i++)
        off += snprintf((char *)s_tx + 5 + off, TX_BUF_LEN - 5 - off, "%s{\"a\":%d,\"v\":%d}",
                        i ? "," : "", regs[i].addr, regs[i].val);
    off += snprintf((char *)s_tx + 5 + off, TX_BUF_LEN - 5 - off, "]}");
    esp_err_t err = send_built_locked((size_t)off);
    xSemaphoreGive(s_tx_mutex);
    return err;
}

// exit calibration mode: restore streaming camera config and ask
// state_machine.c to drop back to IDLE (same queue set_state uses).
static esp_err_t cmd_exit_calibration(int id) {
    cam_calib_exit_mode();
    state_machine_post_command('3');
    return send_jsonf("{\"id\":%d,\"ok\":true}", id);
}

// --------------------------------------------------------------------------
// Dispatch
// --------------------------------------------------------------------------

static esp_err_t handle_command(const uint8_t *body, size_t len) {
    cJSON *req = cJSON_ParseWithLength((const char *)body, len);
    if (!req) return send_jsonf("{\"id\":-1,\"ok\":false,\"err\":\"bad json\"}");
    int id = -1;
    json_int(req, "id", &id);
    const char *cmd = json_str(req, "cmd", "");
    bool in_calibration = state_machine_get_state() == APP_STATE_CAMERA_CALIBRATION;
    esp_err_t err;
    if      (!strcmp(cmd, "ping"))          err = send_jsonf("{\"id\":%d,\"ok\":true}", id);
    else if (!strcmp(cmd, "get_status"))    err = cmd_get_status(id);
    else if (!strcmp(cmd, "set_state"))     err = cmd_set_state(id, req);
    else if (!strcmp(cmd, "imu_cal_axis"))  err = cmd_imu_cal_axis(id, req);
    else if (!strcmp(cmd, "imu_cal_abort")) err = cmd_imu_cal_abort(id);
    else if (!in_calibration && (!strcmp(cmd, "info") || !strcmp(cmd, "set_mode") ||
             !strcmp(cmd, "reg_read") || !strcmp(cmd, "reg_write") || !strcmp(cmd, "reg_dump") ||
             !strcmp(cmd, "set_orientation") || !strcmp(cmd, "fps_probe") || !strcmp(cmd, "capture") ||
             !strcmp(cmd, "save_camera_regs") || !strcmp(cmd, "clear_camera_regs") ||
             !strcmp(cmd, "get_camera_overrides") || !strcmp(cmd, "exit")))
        err = reply_err(id, "not in camera_calibration");
    else if (!strcmp(cmd, "info"))      err = cmd_info(id);
    else if (!strcmp(cmd, "set_mode"))  err = cmd_set_mode(id, req);
    else if (!strcmp(cmd, "reg_read"))  err = cmd_reg_read(id, req);
    else if (!strcmp(cmd, "reg_write")) err = cmd_reg_write(id, req);
    else if (!strcmp(cmd, "reg_dump"))  err = cmd_reg_dump(id, req);
    else if (!strcmp(cmd, "set_orientation")) err = cmd_set_orientation(id, req);
    else if (!strcmp(cmd, "fps_probe")) err = cmd_fps_probe(id, req);
    else if (!strcmp(cmd, "capture"))   err = cmd_capture(id, req);
    else if (!strcmp(cmd, "save_camera_regs"))     err = cmd_save_camera_regs(id, req);
    else if (!strcmp(cmd, "clear_camera_regs"))    err = cmd_clear_camera_regs(id);
    else if (!strcmp(cmd, "get_camera_overrides")) err = cmd_get_camera_overrides(id);
    else if (!strcmp(cmd, "exit"))      err = cmd_exit_calibration(id);
    else                                 err = reply_err(id, "unknown cmd");
    cJSON_Delete(req);
    return err;
}

// --------------------------------------------------------------------------
// Connection + task -- dial/retry/reconnect. Runs for the device's whole
// lifetime; never torn down on a state transition (unlike the old cam_calib
// task this absorbed).
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

    uint8_t *cmd = heap_caps_malloc(MAX_CMD_LEN, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
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
    s_tx = heap_caps_malloc(TX_BUF_LEN, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_tx) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(control_link_task, "control_link", 8192, NULL, 5, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}
