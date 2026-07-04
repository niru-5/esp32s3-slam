#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "esp_camera.h"

// --------------------------------------------------------------------------
// OV5640 camera capture (JPEG, VGA, single DRAM frame buffer).
// --------------------------------------------------------------------------

// Initialize the camera. Returns ESP_OK when ready to grab frames.
esp_err_t camera_start(void);

// Grab one JPEG frame. `*ts_us` (if non-NULL) is set to esp_timer µs captured
// just before the grab — the reference used to align frames to the IMU clock.
// Returns NULL on failure. The caller must release the frame with
// camera_release().
camera_fb_t *camera_grab(int64_t *ts_us);

// Return a frame buffer obtained from camera_grab().
void camera_release(camera_fb_t *fb);
