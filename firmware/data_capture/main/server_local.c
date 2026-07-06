#include "server_local.h"

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_http_server.h"

#include "camera.h"
#include "imu.h"
#include "debug_time.h"

static const char *TAG = "SRV";

// --------------------------------------------------------------------------
// GET /capture — one JPEG frame with its grab timestamp.
// --------------------------------------------------------------------------
static esp_err_t capture_handler(httpd_req_t *req) {
    DEBUG_TIME_START(t_cap);
    int64_t ts;
    camera_fb_t *fb = camera_grab(&ts);
    DEBUG_TIME_END(t_cap, TAG, "camera capturing");
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    char ts_str[24];
    snprintf(ts_str, sizeof(ts_str), "%lld", ts);
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "X-Timestamp-Us", ts_str);
    DEBUG_TIME_START(t_send);
    esp_err_t res = httpd_resp_send(req, (char *)fb->buf, fb->len);
    DEBUG_TIME_END(t_send, TAG, "camera sending");
    camera_release(fb);
    return res;
}

// --------------------------------------------------------------------------
// GET /imu — binary drain of the IMU ring (wire format documented in imu.h).
// --------------------------------------------------------------------------
static esp_err_t imu_handler(httpd_req_t *req) {
    uint8_t *body = malloc(IMU_WIRE_MAXLEN);
    if (!body) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    DEBUG_TIME_START(t_cap);
    size_t body_len = imu_serialize(body);
    DEBUG_TIME_END(t_cap, TAG, "imu capturing");

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    DEBUG_TIME_START(t_send);
    esp_err_t res = httpd_resp_send(req, (char *)body, (ssize_t)body_len);
    DEBUG_TIME_END(t_send, TAG, "imu sending");
    free(body);
    return res;
}

// --------------------------------------------------------------------------
// GET /imu.json — non-destructive peek at the latest IMU sample (safe to poll
// from a browser alongside the binary /imu drain).
// --------------------------------------------------------------------------
static esp_err_t imu_json_handler(httpd_req_t *req) {
    imu_sample_t latest;
    uint32_t count = imu_peek_latest(&latest);

    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{\"buffered\":%u,\"sens_time\":%u,"
        "\"accel_g\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f},"
        "\"gyro_dps\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
        (unsigned)count, (unsigned)latest.sens_time,
        latest.ax, latest.ay, latest.az,
        latest.gx, latest.gy, latest.gz);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, n);
}

// --------------------------------------------------------------------------
// GET / — viewer page.
// --------------------------------------------------------------------------
static esp_err_t index_handler(httpd_req_t *req) {
    const char *html =
    "<html><body style='background:#000;color:#fff;font-family:sans-serif'>"
    "<h2>ESP32-S3 SLAM Camera</h2>"
    "<img id='img' style='width:640px'>"
    "<p>IMU binary stream: <code>GET /imu</code> &nbsp;|&nbsp; readable: <code>GET /imu.json</code></p>"
    "<pre id='imu'></pre>"
    "<script>"
    "setInterval(function(){"
    "  fetch('/imu.json').then(r=>r.json()).then(j=>{"
    "    document.getElementById('imu').textContent=JSON.stringify(j,null,2);"
    "  }).catch(e=>{});"
    "},200);"
    "</script>"
    "<script>"
    "var img=document.getElementById('img');"
    "function next(){"
    "  var i=new Image();"
    "  i.onload=function(){img.src=this.src;setTimeout(next,100);};"
    "  i.onerror=function(){setTimeout(next,500);};"
    "  i.src='/capture?t='+Date.now();"
    "}"
    "next();"
    "</script>"
    "</body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

esp_err_t server_local_start(void) {
    httpd_handle_t server = NULL;
    httpd_config_t srv_cfg = HTTPD_DEFAULT_CONFIG();
    srv_cfg.max_uri_handlers = 8;
    esp_err_t err = httpd_start(&server, &srv_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %d", err);
        return err;
    }

    httpd_uri_t uris[] = {
        { .uri = "/",         .method = HTTP_GET, .handler = index_handler    },
        { .uri = "/capture",  .method = HTTP_GET, .handler = capture_handler  },
        { .uri = "/imu",      .method = HTTP_GET, .handler = imu_handler      },
        { .uri = "/imu.json", .method = HTTP_GET, .handler = imu_json_handler },
    };
    for (int i = 0; i < 4; i++)
        httpd_register_uri_handler(server, &uris[i]);

    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}
