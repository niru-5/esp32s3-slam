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
// IMU calibration — hardware bias/offset only (BMI270 FOC + NVM commit, see
// imu.h imu_run_hw_foc_calibration). Runs synchronously on this task, same as
// every other transition here. Accel FOC needs to know which axis gravity is
// aligned with (bmi2_perform_accel_foc has no way to detect orientation
// itself — see docs/calibration.md) so this walks the operator through it:
// show a handful of live raw samples, explain the axis/sign encoding, then
// block waiting for one digit. That block is a deliberate use of the console
// as "a human-in-the-loop bring-up/test control channel, not a real-time
// path" (see state_machine.h) — nothing else needs this task to keep
// running promptly while it waits.
//
// Noise-density/random-walk/Allan-variance work is not implemented -- see
// docs/calibration.md. Camera calibration is still a provisioning stub.
// --------------------------------------------------------------------------

// Block until the operator sends one non-blank byte, reusing the same
// non-blocking stdin main_state_machine_task already configured. '\n'/'\r'
// are skipped rather than treated as an answer -- the command that got us
// here (e.g. "4\n" from a line-buffered terminal) can leave one buffered,
// and without this it would be misread as the operator's answer before they
// ever see the prompt.
static int wait_for_console_digit(void) {
    int c;
    while (1) {
        c = fgetc(stdin);
        if (c == EOF) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
        if (c == '\n' || c == '\r') continue;
        return c;
    }
}

// Show 10 raw samples over ~100ms and let the operator pick which axis is
// reading ~1g. Returns false (calibration aborted, caller should bail
// without touching the BMI270) if the operator sends anything but '1'-'6'.
static bool select_gravity_axis(imu_accel_foc_axis_t *out) {
    ESP_LOGI(TAG, "Hold the rig stationary. Sampling accel/gyro to help pick the gravity axis...");

    imu_raw_preview_sample_t preview[10];
    if (imu_preview_raw_samples(preview, 10, 10) != ESP_OK) {
        ESP_LOGE(TAG, "preview capture failed -- aborting calibration");
        return false;
    }
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    for (int i = 0; i < 10; i++) {
        ESP_LOGI(TAG, "  sample %2d: accel g = (% .4f, % .4f, % .4f)  gyro dps = (% .4f, % .4f, % .4f)",
                 i, (double)preview[i].ax, (double)preview[i].ay, (double)preview[i].az,
                 (double)preview[i].gx, (double)preview[i].gy, (double)preview[i].gz);
        ax_sum += preview[i].ax; ay_sum += preview[i].ay; az_sum += preview[i].az;
    }
    ESP_LOGI(TAG, "  mean accel g = (% .4f, % .4f, % .4f) — whichever axis reads closest to +-1.0 "
                  "is the one gravity is pulling along; the sign tells you which direction",
             (double)(ax_sum / 10), (double)(ay_sum / 10), (double)(az_sum / 10));

    ESP_LOGI(TAG, "Select that axis+sign: 1=+X 2=-X 3=+Y 4=-Y 5=+Z 6=-Z  (anything else aborts)");
    ESP_LOGI(TAG, "Send one digit now:");

    int c = wait_for_console_digit();
    if (c < '1' || c > '6') {
        ESP_LOGW(TAG, "calibration aborted (got 0x%02x, expected 1-6)", (unsigned)c);
        return false;
    }
    *out = (imu_accel_foc_axis_t)(c - '0');
    ESP_LOGI(TAG, "Using %s as the gravity axis for accel FOC", imu_accel_foc_axis_label(*out));
    return true;
}

static void imu_calibration_run(void) {
#if !CONFIG_ENABLE_IMU
    ESP_LOGW(TAG, "IMU calibration unavailable — IMU disabled (CONFIG_ENABLE_IMU=0) — ignoring command 4");
    return;
#endif

    ESP_LOGI(TAG, "=== IMU hardware FOC calibration ===");

    imu_accel_foc_axis_t gravity_axis;
    if (!select_gravity_axis(&gravity_axis)) return;

    imu_stationary_stats_t before, after;
    uint32_t foc_run_count = 0;

    esp_err_t err = imu_run_hw_foc_calibration(CONFIG_IMU_CALIBRATION_WINDOW_MS, gravity_axis,
                                                &before, &after, &foc_run_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IMU calibration failed or NVM commit didn't stick (err=%s) — see log above",
                 esp_err_to_name(err));
        return;
    }

    // static, not a local: main_state_machine_task's stack is sized for a
    // lightweight console poller (see CONFIG_STATE_MACHINE_TASK_STACK_SIZE) --
    // a 1KB buffer sitting live on that stack for the whole synchronous
    // calibration flow, on top of imu_capture_stationary_stats/bmi2_perform_
    // *_foc's own nested locals, is most of what caused a stack-overflow ->
    // heap-corruption crash (tlsf.c block_trim_free assert) the first time
    // this ran on real hardware.
    static char report[1024];
    int n = snprintf(report, sizeof(report),
        "IMU hardware FOC calibration report\n"
        "foc_run_count=%lu  gravity_axis=%s  local_gravity_mps2=%.4f (CONFIG_LOCAL_GRAVITY_MPS2, "
        "Belgium estimate — annotation only, not fed into the BMI270 FOC call)\n"
        "\n"
        "BEFORE (n=%lu samples):\n"
        "  accel g   x=%.5f y=%.5f z=%.5f   std x=%.5f y=%.5f z=%.5f\n"
        "  gyro  dps x=%.5f y=%.5f z=%.5f   std x=%.5f y=%.5f z=%.5f\n"
        "\n"
        "AFTER (n=%lu samples, hardware FOC + offset comp applied, committed to NVM):\n"
        "  accel g   x=%.5f y=%.5f z=%.5f   std x=%.5f y=%.5f z=%.5f\n"
        "  gyro  dps x=%.5f y=%.5f z=%.5f   std x=%.5f y=%.5f z=%.5f\n"
        "\n"
        "NOTE: bias/offset only. White-noise density, bias random walk, and\n"
        "Allan-variance characterization are not yet implemented — see\n"
        "docs/calibration.md.\n",
        (unsigned long)foc_run_count, imu_accel_foc_axis_label(gravity_axis), (double)CONFIG_LOCAL_GRAVITY_MPS2,
        (unsigned long)before.sample_count,
        (double)before.ax_mean, (double)before.ay_mean, (double)before.az_mean,
        (double)before.ax_std,  (double)before.ay_std,  (double)before.az_std,
        (double)before.gx_mean, (double)before.gy_mean, (double)before.gz_mean,
        (double)before.gx_std,  (double)before.gy_std,  (double)before.gz_std,
        (unsigned long)after.sample_count,
        (double)after.ax_mean, (double)after.ay_mean, (double)after.az_mean,
        (double)after.ax_std,  (double)after.ay_std,  (double)after.az_std,
        (double)after.gx_mean, (double)after.gy_mean, (double)after.gz_mean,
        (double)after.gx_std,  (double)after.gy_std,  (double)after.gz_std);

    ESP_LOGI(TAG, "\n%s", report);
    if (s_sdcard_available) {
        sdcard_write_imu_calibration_report(report, (size_t)n);
    } else {
        ESP_LOGW(TAG, "SD card unavailable — calibration report only in the serial log above");
    }
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

    if (xTaskCreatePinnedToCore(main_state_machine_task, "state_machine",
                                CONFIG_STATE_MACHINE_TASK_STACK_SIZE, NULL,
                                CONFIG_STATE_MACHINE_TASK_PRIORITY, NULL,
                                CONFIG_STATE_MACHINE_TASK_CORE) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}
