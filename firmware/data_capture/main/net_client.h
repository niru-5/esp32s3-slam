#pragma once

#include <stdint.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// Remote streaming client — pushes captured data to a computer over the
// network instead of (or alongside) the on-board server. It POSTs to:
//   POST http://<host>:<port>/frame  — body = JPEG, X-Timestamp-Us header
//   POST http://<host>:<port>/imu    — body = IMU wire payload (see imu.h)
// --------------------------------------------------------------------------

// Start the streaming task, pushing frames at `fps` and draining the IMU each
// cycle to `host:port`. Returns ESP_OK once the task is running.
esp_err_t net_client_start(const char *host, uint16_t port, int fps);
