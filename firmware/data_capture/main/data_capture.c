#include <stdbool.h>
#include <time.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "config.h"
#include "camera.h"
#include "imu.h"
#include "sysstats.h"
#include "sdcard.h"
#include "state_machine.h"

static const char *TAG = "SLAM";

// --------------------------------------------------------------------------
// WiFi (STA)
// --------------------------------------------------------------------------

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_event_group;

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_start(void) {
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,    wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_cfg = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();
    // Default modem-sleep power save duty-cycles the radio around the AP's
    // DTIM interval, adding 100s of ms of variable latency to every
    // request/response round trip. This app is latency-sensitive (streaming
    // frames + IMU every cycle), not power-constrained, so keep the radio
    // fully awake.
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_connect();
}

// --------------------------------------------------------------------------
// Wall-clock time (SNTP) — the ESP32-S3 has no battery-backed RTC, so on
// every boot the clock starts at the epoch until synced over the network.
// sdcard.c uses time(NULL) to name each session folder "YYYYMMDD-HHMMSS";
// without this, that would just be 19700101-000000 every boot.
// --------------------------------------------------------------------------

static void sync_time(void) {
    ESP_LOGI(TAG, "syncing time via SNTP...");
    esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&sntp_cfg);
    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK) {
        ESP_LOGW(TAG, "SNTP sync timed out — session folder will fall back to a boot-relative name");
        return;
    }
    time_t now = time(NULL);
    struct tm tm_info;
    localtime_r(&now, &tm_info);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_info);
    ESP_LOGI(TAG, "time synced (UTC): %s", buf);
}

// --------------------------------------------------------------------------
// Entry point — brings up WiFi + the camera/IMU/SD hardware once, then hands
// off to main_state_machine_task, which creates/tears down the actual
// capture pipelines at runtime based on serial commands (see
// docs/architecture.md "Runtime state machine").
// --------------------------------------------------------------------------

void app_main(void) {
    nvs_flash_init();
    wifi_start();

    if (xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
                             pdMS_TO_TICKS(10000)) & WIFI_CONNECTED_BIT) {
        sync_time();
    } else {
        ESP_LOGW(TAG, "WiFi not connected within 10s — skipping time sync");
    }

    if (camera_init() != ESP_OK) return;
    if (imu_init() != ESP_OK) return;
    sysstats_start();  // best-effort telemetry; failure is non-fatal

    bool sdcard_available = false;
#if CONFIG_USE_SDCARD
    sdcard_available = (sdcard_init() == ESP_OK);
    if (!sdcard_available)
        ESP_LOGW(TAG, "SD card logging unavailable — continuing without it");
#endif

    state_machine_start(sdcard_available);
}
