#include "state_machine.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"

#include "config.h"
#include "imu.h"
#include "camera.h"
#include "sysstats.h"
#include "net_client.h"
#include "tcp_client.h"
#include "sdcard.h"

static const char *TAG = "STATE";

static app_state_t s_state             = APP_STATE_IDLE;
static bool        s_sdcard_available  = false;

// --------------------------------------------------------------------------
// Calibration stubs — provisioning only. The exact procedure (duration,
// what's recorded, whether it needs its own task) is deferred until that
// work starts; for now entering the state just logs and returns to IDLE.
// --------------------------------------------------------------------------

static void imu_calibration_run(void) {
    ESP_LOGW(TAG, "IMU calibration not implemented yet");
}

static void camera_calibration_run(void) {
    ESP_LOGW(TAG, "camera calibration not implemented yet");
}

// --------------------------------------------------------------------------
// Transitions — every streaming pipeline is created fresh on entry and
// deleted (queues flushed, not suspended) on exit, per docs/architecture.md
// "Task lifecycle".
// --------------------------------------------------------------------------

static void teardown_active_pipelines(void) {
    switch (s_state) {
    case APP_STATE_STREAM_WIFI:
        net_client_pipeline_stop();
        sysstats_pipeline_stop();
        camera_pipeline_stop();
        imu_pipeline_stop();
        break;
    case APP_STATE_STREAM_SDCARD:
        sdcard_pipeline_stop();
        sysstats_pipeline_stop();
        camera_pipeline_stop();
        imu_pipeline_stop();
        break;
    case APP_STATE_STREAM_TCP:
        tcp_client_pipeline_stop();
        sysstats_pipeline_stop();
        camera_pipeline_stop();
        imu_pipeline_stop();
        break;
    default:
        break;
    }
}

static void enter_stream_wifi(void) {
    teardown_active_pipelines();
    s_state = APP_STATE_IDLE;

    if (imu_pipeline_start() != ESP_OK ||
        camera_pipeline_start() != ESP_OK ||
        net_client_pipeline_start() != ESP_OK ||
        sysstats_pipeline_start(SYSSTATS_SINK_WIFI) != ESP_OK) {
        ESP_LOGE(TAG, "failed to start wifi streaming pipeline — rolling back");
        net_client_pipeline_stop();
        sysstats_pipeline_stop();
        camera_pipeline_stop();
        imu_pipeline_stop();
        return;
    }
    s_state = APP_STATE_STREAM_WIFI;
    ESP_LOGI(TAG, "-> STREAM_WIFI");
}

static void enter_stream_sdcard(void) {
    if (!s_sdcard_available) {
        ESP_LOGW(TAG, "SD card unavailable — ignoring command 2");
        return;
    }
    teardown_active_pipelines();
    s_state = APP_STATE_IDLE;

    if (imu_pipeline_start() != ESP_OK ||
        camera_pipeline_start() != ESP_OK ||
        sdcard_pipeline_start() != ESP_OK ||
        sysstats_pipeline_start(SYSSTATS_SINK_SDCARD) != ESP_OK) {
        ESP_LOGE(TAG, "failed to start sdcard streaming pipeline — rolling back");
        sdcard_pipeline_stop();
        sysstats_pipeline_stop();
        camera_pipeline_stop();
        imu_pipeline_stop();
        return;
    }
    s_state = APP_STATE_STREAM_SDCARD;
    ESP_LOGI(TAG, "-> STREAM_SDCARD");
}

static void enter_stream_tcp(void) {
    teardown_active_pipelines();
    s_state = APP_STATE_IDLE;

    if (imu_pipeline_start() != ESP_OK ||
        camera_pipeline_start() != ESP_OK ||
        tcp_client_pipeline_start() != ESP_OK ||
        sysstats_pipeline_start(SYSSTATS_SINK_TCP) != ESP_OK) {
        ESP_LOGE(TAG, "failed to start tcp streaming pipeline — rolling back");
        tcp_client_pipeline_stop();
        sysstats_pipeline_stop();
        camera_pipeline_stop();
        imu_pipeline_stop();
        return;
    }
    s_state = APP_STATE_STREAM_TCP;
    ESP_LOGI(TAG, "-> STREAM_TCP");
}

static void enter_idle(void) {
    teardown_active_pipelines();
    s_state = APP_STATE_IDLE;
    ESP_LOGI(TAG, "-> IDLE");
}

static void enter_calibration(app_state_t which) {
    teardown_active_pipelines();
    s_state = which;
    if (which == APP_STATE_IMU_CALIBRATION) imu_calibration_run();
    else                                    camera_calibration_run();
    // Stub runs synchronously and returns immediately -- nothing to stay
    // "in progress" for yet, so drop straight back to idle.
    s_state = APP_STATE_IDLE;
}

static void handle_command(char c) {
    switch (c) {
    case '1': enter_stream_wifi();                              break;
    case '2': enter_stream_sdcard();                             break;
    case '3': enter_idle();                                      break;
    case '4': enter_calibration(APP_STATE_IMU_CALIBRATION);       break;
    case '5': enter_calibration(APP_STATE_CAMERA_CALIBRATION);    break;
    case '6': enter_stream_tcp();                                 break;
    default: break;  // ignore newlines / anything else
    }
}

// --------------------------------------------------------------------------
// main_state_machine_task
// --------------------------------------------------------------------------

static void main_state_machine_task(void *arg) {
    // The console (CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG) only wires up a simple
    // boot-time TX-capable path by default -- log output works out of the
    // box, but stdin reads silently return EOF forever unless the full
    // interrupt-driven USB-Serial-JTAG driver is installed explicitly (see
    // docs/learnings.md). Without this, every byte sent over the port is
    // dropped at the USB level before it ever reaches stdin.
    usb_serial_jtag_driver_config_t usj_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&usj_cfg);
    usb_serial_jtag_vfs_use_driver();

    // usb_serial_jtag_vfs_use_driver() makes reads blocking -- reapply
    // O_NONBLOCK so fgetc() returns EOF immediately when nothing is waiting,
    // matching this task's poll-then-drain design instead of blocking it.
    int flags = fcntl(fileno(stdin), F_GETFL, 0);
    fcntl(fileno(stdin), F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "ready — send 1=wifi 2=sdcard 3=stop 4=imu_cal 5=cam_cal 6=tcp");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_STATE_MACHINE_POLL_MS));
        int c;
        while ((c = fgetc(stdin)) != EOF)
            handle_command((char)c);
    }
}

esp_err_t state_machine_start(bool sdcard_available) {
    s_sdcard_available = sdcard_available;
    if (!s_sdcard_available)
        ESP_LOGW(TAG, "SD card unavailable — command 2 (STREAM_SDCARD) will be rejected");

    if (xTaskCreatePinnedToCore(main_state_machine_task, "state_machine", 4096, NULL,
                                CONFIG_STATE_MACHINE_TASK_PRIORITY, NULL,
                                CONFIG_STATE_MACHINE_TASK_CORE) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}
