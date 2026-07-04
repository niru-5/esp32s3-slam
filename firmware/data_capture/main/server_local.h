#pragma once

#include "esp_err.h"

// --------------------------------------------------------------------------
// On-board HTTP server the host pulls from:
//   GET /          — MJPEG-style viewer page
//   GET /capture   — one JPEG frame, X-Timestamp-Us header
//   GET /imu       — binary drain of the IMU ring (see imu.h wire format)
//   GET /imu.json  — human-readable peek at the latest IMU sample
// --------------------------------------------------------------------------

// Start the HTTP server and register the routes. Returns ESP_OK on success.
esp_err_t server_local_start(void);
