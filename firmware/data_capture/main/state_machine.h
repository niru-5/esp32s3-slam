#pragma once

#include <stdbool.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// Owns the rig's runtime operating mode, driven by single-digit commands
// polled from the console (see docs/architecture.md "Runtime state machine"):
//
//   1 -> STREAM_WIFI        start camera+IMU capture, stream to the host
//   2 -> STREAM_SDCARD       start camera+IMU capture, log to the SD card
//   3 -> IDLE                stop streaming (either sink)
//   4 -> IMU_CALIBRATION     provisioning stub, see state_machine.c
//   5 -> CAMERA_CALIBRATION  provisioning stub, see state_machine.c
//
// 1<->2 switch directly (no need to send 3 first); 4/5 force an implicit
// teardown of whatever streaming pipeline is active first. Every transition
// creates the pipelines it needs fresh and deletes whatever was running
// before it -- see imu.h/camera.h/sysstats.h for the pipelines themselves and
// net_client.h/sdcard.h for the two sinks.
//
// main_state_machine_task is deliberately light: it polls stdin
// non-blockingly every CONFIG_STATE_MACHINE_POLL_MS. This is a
// human-in-the-loop bring-up/test control channel, not a real-time path.
// --------------------------------------------------------------------------

typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_STREAM_WIFI,
    APP_STATE_STREAM_SDCARD,
    APP_STATE_IMU_CALIBRATION,
    APP_STATE_CAMERA_CALIBRATION,
} app_state_t;

// Create main_state_machine_task (prio CONFIG_STATE_MACHINE_TASK_PRIORITY,
// core CONFIG_STATE_MACHINE_TASK_CORE). Call once at boot, after
// camera_init()/imu_init() have both succeeded. `sdcard_available` should be
// the result of sdcard_init() (or false if CONFIG_USE_SDCARD is off) --
// command 2 (STREAM_SDCARD) is rejected when false.
esp_err_t state_machine_start(bool sdcard_available);
