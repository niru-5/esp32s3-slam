#include "cam_calib.h"

#include <string.h>
#include "esp_log.h"

#include "camera.h"
#include "camera_overrides.h"

static const char *TAG = "CAMCAL";

static char s_fmt[8]  = "jpeg";
static char s_size[8] = "svga";

esp_err_t cam_calib_enter_mode(void) {
    esp_err_t err = camera_init_ex(PIXFORMAT_JPEG, FRAMESIZE_SVGA, 6, 2, false, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera bring-up for calibration failed (%s)", esp_err_to_name(err));
        return err;
    }
    // Re-apply any NVS-persisted register overrides here too, same as
    // camera_init_streaming() -- otherwise a calibration session starts from the
    // driver's un-tuned defaults while streaming reflects the saved overrides
    // (e.g. orientation), so the "current" picture in the calibration panel looks
    // normal while a live stream looks flipped, and any register-diffing the
    // console does starts from the wrong baseline.
    camera_overrides_apply();
    strlcpy(s_fmt, "jpeg", sizeof(s_fmt));
    strlcpy(s_size, "svga", sizeof(s_size));
    ESP_LOGI(TAG, "-> calibration camera config (jpeg svga)");
    return ESP_OK;
}

void cam_calib_exit_mode(void) {
    camera_init();   // restores the normal streaming camera config
    ESP_LOGI(TAG, "-> streaming camera config restored");
}

const char *cam_calib_fmt(void)  { return s_fmt; }
const char *cam_calib_size(void) { return s_size; }

void cam_calib_set_mode(const char *fmt, const char *size) {
    strlcpy(s_fmt, fmt, sizeof(s_fmt));
    strlcpy(s_size, size, sizeof(s_size));
}
