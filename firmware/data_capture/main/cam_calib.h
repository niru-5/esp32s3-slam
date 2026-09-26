#pragma once

#include <stdbool.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// CAMERA_CALIBRATION mode: a host-driven camera calibration / ISP tuning
// session (see docs/camera_calibration_and_tuning.md).
//
// The device dials the host (CONFIG_REMOTE_HOST:CONFIG_CAM_CALIB_PORT, retrying until
// software/host_server/calibration is listening, and reconnecting if the link drops) and
// then obeys commands from it:
// capture N frames (with per-frame metadata), read/write/dump sensor registers,
// switch pixel format / resolution (incl. RAW8), exit. Everything on one
// persistent socket, framed as
//
//     uint32 len (LE) | uint8 type | body[len-1]
//
//   type 0x01 JSON   (both directions)
//   type 0x02 IMAGE  (device -> host): uint32 meta_len | meta JSON | frame bytes
//
// Nothing here runs unless the operator sends serial command 5; the streaming
// pipelines are torn down first (state_machine.c) so the camera is free for
// camera_init_ex() to reconfigure. Leaving the mode restores the normal
// streaming camera config.
// --------------------------------------------------------------------------

// Spawn cam_calib_task. Returns ESP_ERR_INVALID_STATE if already running.
esp_err_t cam_calib_start(void);

// Ask the task to finish and block (up to ~5 s) until it has released the
// camera/socket. Safe to call when not running.
void cam_calib_stop(void);

// True from cam_calib_start() until the task has fully cleaned up. The state
// machine polls this to drop back to IDLE when the host sends "exit".
bool cam_calib_active(void);
