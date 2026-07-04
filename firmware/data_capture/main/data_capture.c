#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mdns.h"

#include "config.h"
#include "camera.h"
#include "imu.h"
#include "sysstats.h"
#include "server_local.h"
#include "net_client.h"

static const char *TAG = "SLAM";

// --------------------------------------------------------------------------
// WiFi (STA)
// --------------------------------------------------------------------------

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
#if CONFIG_ENABLE_LOCAL_SERVER
        ESP_LOGI(TAG, "http://" IPSTR "  or  http://slam-cam.local", IP2STR(&e->ip_info.ip));
        mdns_init();
        mdns_hostname_set("slam-cam");
        mdns_instance_name_set("ESP32-S3 SLAM Camera");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
#endif
    }
}

static void wifi_start(void) {
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
    esp_wifi_connect();
}

// --------------------------------------------------------------------------
// Entry point — capture (camera + IMU) is always on; the data sink(s) are
// selected in config.h.
// --------------------------------------------------------------------------

void app_main(void) {
    nvs_flash_init();
    wifi_start();

    if (camera_start() != ESP_OK) return;
    if (imu_start() != ESP_OK) return;
    sysstats_start();  // best-effort telemetry; failure is non-fatal

#if CONFIG_ENABLE_LOCAL_SERVER
    server_local_start();
#endif

#if CONFIG_ENABLE_REMOTE_STREAM
    net_client_start(CONFIG_REMOTE_HOST, CONFIG_REMOTE_PORT, CONFIG_REMOTE_STREAM_FPS);
#endif

#if !CONFIG_ENABLE_LOCAL_SERVER && !CONFIG_ENABLE_REMOTE_STREAM
    ESP_LOGW(TAG, "no data sink enabled — capturing but not transmitting");
#endif
}
