#include "esp_camera.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "driver/i2c.h"
#include "mdns.h"

#define WIFI_SSID "Jarvis_slow"
#define WIFI_PASS "Someoneisusingmydata"

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

// Convert grayscale buffer to BMP and serve it
// static esp_err_t capture_handler(httpd_req_t *req) {
//     camera_fb_t *fb = esp_camera_fb_get();
//     if (!fb) {
//         ESP_LOGE(TAG, "Frame capture failed");
//         httpd_resp_send_500(req);
//         return ESP_FAIL;
//     }

//     // BMP header for 320x240 grayscale
//     int w = fb->width, h = fb->height;
//     int row_size = (w + 3) & ~3;  // padded to 4 bytes
//     int pixel_data_size = row_size * h;
//     int file_size = 54 + 1024 + pixel_data_size;  // header + palette + pixels

//     uint8_t *bmp = malloc(file_size);
//     if (!bmp) {
//         esp_camera_fb_return(fb);
//         httpd_resp_send_500(req);
//         return ESP_FAIL;
//     }
//     memset(bmp, 0, file_size);

//     // BMP file header
//     bmp[0]='B'; bmp[1]='M';
//     *(uint32_t*)(bmp+2)  = file_size;
//     *(uint32_t*)(bmp+10) = 54 + 1024;  // pixel data offset

//     // DIB header
//     *(uint32_t*)(bmp+14) = 40;   // header size
//     *(int32_t*) (bmp+18) = w;
//     *(int32_t*) (bmp+22) = -h;   // negative = top-down
//     *(uint16_t*)(bmp+26) = 1;    // color planes
//     *(uint16_t*)(bmp+28) = 8;    // bits per pixel
//     *(uint32_t*)(bmp+34) = pixel_data_size;

//     // Grayscale palette (256 entries)
//     for (int i = 0; i < 256; i++) {
//         bmp[54 + i*4 + 0] = i;
//         bmp[54 + i*4 + 1] = i;
//         bmp[54 + i*4 + 2] = i;
//         bmp[54 + i*4 + 3] = 0;
//     }

//     // Pixel data (copy rows with padding)
//     uint8_t *pixels = bmp + 54 + 1024;
//     for (int y = 0; y < h; y++) {
//         memcpy(pixels + y * row_size, fb->buf + y * w, w);
//     }

//     esp_camera_fb_return(fb);

//     httpd_resp_set_type(req, "image/bmp");
//     httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
//     esp_err_t res = httpd_resp_send(req, (char*)bmp, file_size);
//     free(bmp);
//     return res;
// }

static esp_err_t capture_handler(httpd_req_t *req) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    esp_err_t res = httpd_resp_send(req, (char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

// Mjpeg-style streaming page
static esp_err_t index_handler(httpd_req_t *req) {
    const char *html =
    "<html><body style='background:#000;text-align:center'>"
    "<h2 style='color:#fff'>ESP32-S3 Camera</h2>"
    "<img id='img' style='width:640px'>"
    "<script>"
    "var img = document.getElementById('img');"
    "function next() {"
    "  var i = new Image();"
    "  i.onload = function(){ img.src=this.src; setTimeout(next,100); };"
    "  i.onerror = function(){ setTimeout(next,500); };"  // back off on error
    "  i.src = '/capture?t=' + Date.now();"
    "}"
    "next();"
    "</script>"
    "</body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "Open http://" IPSTR " or http://turtle-camera.local in browser", IP2STR(&e->ip_info.ip));
        mdns_init();
        mdns_hostname_set("turtle-camera");
        mdns_instance_name_set("ESP32-S3 Camera");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    }
}

void app_main(void) {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();
    esp_wifi_connect();

    // Camera init
    camera_config_t config = {
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
        .pixel_format = PIXFORMAT_JPEG, //  PIXFORMAT_GRAYSCALE PIXFORMAT_JPEG
        .frame_size   = FRAMESIZE_VGA, // FRAMESIZE_VGA  FRAMESIZE_QVGA
        .jpeg_quality = 12,
        .fb_count     = 1,
        .fb_location  = CAMERA_FB_IN_DRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_pixformat(s, PIXFORMAT_JPEG); // PIXFORMAT_JPEG
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_quality(s, 12);

    // HTTP server
    httpd_handle_t server = NULL;
    httpd_config_t server_cfg = HTTPD_DEFAULT_CONFIG();
    httpd_start(&server, &server_cfg);

    httpd_uri_t index_uri = { .uri="/",        .method=HTTP_GET, .handler=index_handler };
    httpd_uri_t cap_uri   = { .uri="/capture", .method=HTTP_GET, .handler=capture_handler };
    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &cap_uri);

    ESP_LOGI(TAG, "HTTP server started");
}