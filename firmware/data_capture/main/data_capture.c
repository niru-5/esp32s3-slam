#include "esp_camera.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "mdns.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

#include "bmi270.h"

#define WIFI_SSID "Jarvis_slow"
#define WIFI_PASS "Someoneisusingmydata"

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

// --------------------------------------------------------------------------
// BMI270 over I2C (IMU_I2C_PORT). All mode/address pins are driven by the
// ESP32, so the module needs only direct wiring — no breadboard pull resistors:
//   SDA→GPIO41  SCL→GPIO42  CSB→GPIO47 (high→I2C)  SA0→GPIO48 (low→0x68)
// --------------------------------------------------------------------------
#define IMU_I2C_PORT   I2C_NUM_0
#define IMU_SDA_GPIO   41
#define IMU_SCL_GPIO   42
#define IMU_I2C_FREQ   400000
#define BMI270_ADDR    BMI2_I2C_PRIM_ADDR  // 0x68 when SA0=low
#define IMU_CSB_GPIO   47                   // driven HIGH → I2C mode on BMI270
#define IMU_ADDR_GPIO  48                   // driven LOW  → I2C address 0x68 (SA0)

// BMI2_SENSORTIME_RESOLUTION = 0.0000390625 s/tick = 39.0625 µs/tick
// Host converts: sample_esp_us = ref_esp_us + (sens_time - ref_ticks) * 39.0625

// --------------------------------------------------------------------------
// IMU ring buffer — 2 s at 100 Hz
// --------------------------------------------------------------------------
#define IMU_RING_CAP  200

typedef struct {
    uint32_t sens_time;        // raw BMI270 24-bit sensor clock ticks
    float    ax, ay, az;       // accelerometer (g)
    float    gx, gy, gz;       // gyroscope (deg/s)
} imu_sample_t;

static imu_sample_t      s_imu_ring[IMU_RING_CAP];
static uint32_t          s_imu_write = 0;   // index of next write slot
static uint32_t          s_imu_count = 0;   // valid samples in ring
static SemaphoreHandle_t s_imu_mutex;

static struct bmi2_dev   s_bmi2;
static float             s_raw_to_gs;       // int16 → g
static float             s_raw_to_dps;      // int16 → deg/s

static const char *TAG = "SLAM";

// --------------------------------------------------------------------------
// BMI2 I2C read/write callbacks
// --------------------------------------------------------------------------

static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *data,
                                             uint32_t len, void *intf_ptr) {
    if (len == 0) return BMI2_E_COM_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMI270_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMI270_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(IMU_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data,
                                              uint32_t len, void *intf_ptr) {
    if (len == 0) return BMI2_E_COM_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMI270_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(IMU_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static void bmi2_delay_us_cb(uint32_t period, void *intf_ptr) {
    // Busy-wait the full period. BMI270 init needs an accurate 2 ms settle
    // after soft-reset; routing sub-10 ms delays through
    // vTaskDelay(pdMS_TO_TICKS(...)) truncates to 0 ticks at the default
    // 100 Hz FreeRTOS tick and skips the wait (-> config upload COM_FAIL).
    esp_rom_delay_us(period);
}

static void i2c_bus_scan(void) {
    ESP_LOGI(TAG, "Scanning I2C%d on SDA=GPIO%d SCL=GPIO%d ...",
             IMU_I2C_PORT, IMU_SDA_GPIO, IMU_SCL_GPIO);
    int found = 0;
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(IMU_I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  -> ACK from address 0x%02X", (unsigned)addr);
            found++;
        }
    }
    if (found == 0)
        ESP_LOGW(TAG, "  no devices found (check SDA/SCL wiring, power, pull-ups)");
}

// --------------------------------------------------------------------------
// IMU init
// --------------------------------------------------------------------------

static esp_err_t imu_init(void) {
    // Drive the BMI270 mode/address pins before any communication so the
    // module needs no external pull resistors:
    //   CSB high → I2C mode,  SA0 low → address 0x68
    // A falling edge on CSB latches the BMI270 into SPI mode until the next
    // power-on, so CSB must never glitch low. Preset the output latches BEFORE
    // switching the pins to output (otherwise output mode drives 0 first), and
    // enable a pull-up on CSB to bias it high through the boot window.
    gpio_set_level(IMU_CSB_GPIO, 1);   // preset CSB latch high
    gpio_set_level(IMU_ADDR_GPIO, 0);  // preset SA0 latch low
    gpio_config_t imu_pins = {
        .pin_bit_mask = (1ULL << IMU_CSB_GPIO) | (1ULL << IMU_ADDR_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&imu_pins);
    gpio_set_level(IMU_CSB_GPIO, 1);   // I2C mode
    gpio_set_level(IMU_ADDR_GPIO, 0);  // address 0x68
    vTaskDelay(pdMS_TO_TICKS(5)); // let BMI270 settle into I2C mode

    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = IMU_SDA_GPIO,
        .scl_io_num       = IMU_SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = IMU_I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(IMU_I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(IMU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    i2c_bus_scan();

    s_bmi2.read           = bmi2_i2c_read;
    s_bmi2.write          = bmi2_i2c_write;
    s_bmi2.delay_us       = bmi2_delay_us_cb;
    s_bmi2.intf_ptr       = NULL;
    s_bmi2.intf           = BMI2_I2C_INTF;
    s_bmi2.read_write_len = 32;

    int8_t err = bmi270_init(&s_bmi2);
    if (err != BMI2_OK) {
        ESP_LOGE(TAG, "bmi270_init failed: %d", err);
        return ESP_FAIL;
    }

    uint8_t sens_list[] = {BMI2_ACCEL, BMI2_GYRO};
    err = bmi270_sensor_enable(sens_list, 2, &s_bmi2);
    if (err != BMI2_OK) {
        ESP_LOGE(TAG, "sensor_enable failed: %d", err);
        return ESP_FAIL;
    }

    // Read defaults then override non-default values only
    struct bmi2_sens_config config[2];
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;
    err = bmi270_get_sensor_config(config, 2, &s_bmi2);
    if (err != BMI2_OK) {
        ESP_LOGE(TAG, "get_sensor_config failed: %d", err);
        return ESP_FAIL;
    }

    config[0].cfg.acc.odr   = BMI2_ACC_ODR_100HZ;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;

    config[1].cfg.gyr.odr   = BMI2_GYR_ODR_100HZ;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_500;

    err = bmi270_set_sensor_config(config, 2, &s_bmi2);
    if (err != BMI2_OK) {
        ESP_LOGE(TAG, "set_sensor_config failed: %d", err);
        return ESP_FAIL;
    }

    // Conversion scalars — must be computed from the ranges we just set
    // accel: raw int16 * scalar → g
    s_raw_to_gs  = (float)(2 << config[0].cfg.acc.range) / 32768.0f;
    // gyro: raw int16 * scalar → deg/s  (BMI2_GYR_RANGE_125 = 4)
    s_raw_to_dps = (125.0f * (float)(1 << (BMI2_GYR_RANGE_125 - config[1].cfg.gyr.range))) / 32768.0f;

    ESP_LOGI(TAG, "BMI270 ready — accel ±%dG  gyro ±%ddps  ODR 100Hz",
             (int)(2 << config[0].cfg.acc.range),
             (int)(125 * (1 << (BMI2_GYR_RANGE_125 - config[1].cfg.gyr.range))));
    return ESP_OK;
}

// --------------------------------------------------------------------------
// IMU task — 100 Hz
// Stores raw sensor timestamp per sample; host applies clock offset at read.
// NOTE: for reliable 10 ms tick, set CONFIG_FREERTOS_HZ=1000 in sdkconfig.
// --------------------------------------------------------------------------

static void imu_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        struct bmi2_sens_data raw;
        memset(&raw, 0, sizeof(raw));
        if (bmi2_get_sensor_data(&raw, &s_bmi2) != BMI2_OK) continue;

        imu_sample_t s = {
            .sens_time = raw.sens_time,
            .ax = (float)raw.acc.x * s_raw_to_gs,
            .ay = (float)raw.acc.y * s_raw_to_gs,
            .az = (float)raw.acc.z * s_raw_to_gs,
            .gx = (float)raw.gyr.x * s_raw_to_dps,
            .gy = (float)raw.gyr.y * s_raw_to_dps,
            .gz = (float)raw.gyr.z * s_raw_to_dps,
        };

        xSemaphoreTake(s_imu_mutex, portMAX_DELAY);
        s_imu_ring[s_imu_write] = s;
        s_imu_write = (s_imu_write + 1) % IMU_RING_CAP;
        if (s_imu_count < IMU_RING_CAP) s_imu_count++;
        xSemaphoreGive(s_imu_mutex);
    }
}

// --------------------------------------------------------------------------
// HTTP handlers
// --------------------------------------------------------------------------

static esp_err_t capture_handler(httpd_req_t *req) {
    // Timestamp at frame grab — consistent reference for the IMU clock offset
    int64_t ts = esp_timer_get_time();
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    char ts_str[24];
    snprintf(ts_str, sizeof(ts_str), "%lld", ts);
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "X-Timestamp-Us", ts_str);
    esp_err_t res = httpd_resp_send(req, (char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

// /imu — binary response
//
// Header (16 bytes, little-endian):
//   uint32  num_samples
//   int64   ref_esp_us        — ESP32 µs-since-boot at response time
//   uint32  ref_sensor_ticks  — BMI270 tick at response time
//
// Per sample (28 bytes):
//   uint32  sens_time          — BMI270 tick when sample was polled
//   float   ax, ay, az         — g
//   float   gx, gy, gz         — deg/s
//
// Host converts to ESP32 time:
//   sample_esp_us = ref_esp_us + (int64)(sens_time - ref_sensor_ticks) * 39.0625
//
// Sensor clock wraps at 2^24 ticks ≈ 655 s; safe for a 2 s ring buffer.
static esp_err_t imu_handler(httpd_req_t *req) {
    // Snapshot ring and clear it
    xSemaphoreTake(s_imu_mutex, portMAX_DELAY);
    uint32_t count  = s_imu_count;
    uint32_t oldest = (s_imu_write + IMU_RING_CAP - count) % IMU_RING_CAP;

    imu_sample_t *snap = NULL;
    if (count > 0) {
        snap = malloc(count * sizeof(imu_sample_t));
        if (!snap) {
            s_imu_count = 0;
            s_imu_write = 0;
            xSemaphoreGive(s_imu_mutex);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        for (uint32_t i = 0; i < count; i++)
            snap[i] = s_imu_ring[(oldest + i) % IMU_RING_CAP];
    }
    s_imu_count = 0;
    s_imu_write = 0;
    xSemaphoreGive(s_imu_mutex);

    // Reference pair — taken right after clearing so offset applies to snapshotted samples
    struct bmi2_sens_data ref;
    memset(&ref, 0, sizeof(ref));
    bmi2_get_sensor_data(&ref, &s_bmi2);
    int64_t  ref_esp_us        = esp_timer_get_time();
    uint32_t ref_sensor_ticks  = ref.sens_time;

    // Build binary payload
    size_t body_len = 16 + (size_t)count * 28;
    uint8_t *body = malloc(body_len);
    if (!body) {
        free(snap);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    uint8_t *p = body;
    memcpy(p, &count,            4); p += 4;
    memcpy(p, &ref_esp_us,       8); p += 8;
    memcpy(p, &ref_sensor_ticks, 4); p += 4;
    for (uint32_t i = 0; i < count; i++) {
        memcpy(p, &snap[i].sens_time, 4); p += 4;
        memcpy(p, &snap[i].ax,        4); p += 4;
        memcpy(p, &snap[i].ay,        4); p += 4;
        memcpy(p, &snap[i].az,        4); p += 4;
        memcpy(p, &snap[i].gx,        4); p += 4;
        memcpy(p, &snap[i].gy,        4); p += 4;
        memcpy(p, &snap[i].gz,        4); p += 4;
    }
    free(snap);

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    esp_err_t res = httpd_resp_send(req, (char *)body, (ssize_t)body_len);
    free(body);
    return res;
}

// Human-readable, non-destructive peek at the latest IMU sample (does NOT
// drain the ring — safe to poll from a browser alongside the binary /imu).
static esp_err_t imu_json_handler(httpd_req_t *req) {
    xSemaphoreTake(s_imu_mutex, portMAX_DELAY);
    uint32_t count = s_imu_count;
    imu_sample_t latest = {0};
    if (count > 0)
        latest = s_imu_ring[(s_imu_write + IMU_RING_CAP - 1) % IMU_RING_CAP];
    xSemaphoreGive(s_imu_mutex);

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

// --------------------------------------------------------------------------
// WiFi
// --------------------------------------------------------------------------

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "http://" IPSTR "  or  http://slam-cam.local", IP2STR(&e->ip_info.ip));
        mdns_init();
        mdns_hostname_set("slam-cam");
        mdns_instance_name_set("ESP32-S3 SLAM Camera");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    }
}

// --------------------------------------------------------------------------
// Entry point
// --------------------------------------------------------------------------

void app_main(void) {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,  wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_cfg = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();
    esp_wifi_connect();

    // Camera
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
    if (esp_camera_init(&cam_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed");
        return;
    }

    // IMU
    s_imu_mutex = xSemaphoreCreateMutex();
    if (imu_init() != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed — check wiring (SDA=GPIO%d SCL=GPIO%d addr=0x%02X)",
                 IMU_SDA_GPIO, IMU_SCL_GPIO, BMI270_ADDR);
        return;
    }

    // HTTP server
    httpd_handle_t server = NULL;
    httpd_config_t srv_cfg = HTTPD_DEFAULT_CONFIG();
    srv_cfg.max_uri_handlers = 8;
    httpd_start(&server, &srv_cfg);

    httpd_uri_t uris[] = {
        { .uri = "/",         .method = HTTP_GET, .handler = index_handler    },
        { .uri = "/capture",  .method = HTTP_GET, .handler = capture_handler  },
        { .uri = "/imu",      .method = HTTP_GET, .handler = imu_handler      },
        { .uri = "/imu.json", .method = HTTP_GET, .handler = imu_json_handler },
    };
    for (int i = 0; i < 4; i++)
        httpd_register_uri_handler(server, &uris[i]);

    ESP_LOGI(TAG, "HTTP server started");

    // IMU task pinned to core 1 so camera/WiFi (core 0) stays uncontended
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, NULL, 5, NULL, 1);
}
