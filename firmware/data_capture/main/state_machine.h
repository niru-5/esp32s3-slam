#pragma once

#include <stdbool.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// Owns the rig's runtime operating mode, driven by single-digit commands
// polled from the console (see docs/architecture.md "Runtime state machine"):
//
//   1 -> STREAM_WIFI        start camera+IMU capture, stream to the host over HTTP
//   2 -> STREAM_SDCARD       start camera+IMU capture, log to the SD card
//   3 -> IDLE                stop streaming (any sink)
//   4 -> IMU_CALIBRATION     provisioning stub, see state_machine.c
//   5 -> CAMERA_CALIBRATION  host-driven camera calibration / ISP tuning session:
//                            the device dials software/host_server/calibration on
//                            the host (see cam_calib.h, docs/camera_calibration_and_tuning.md).
//                            Stays in this state until the host sends "exit" or serial 3.
//   6 -> STREAM_TCP          start camera+IMU capture, stream to the host over
//                            a raw TCP socket (see tcp_client.h) instead of
//                            HTTP -- lower per-message overhead, same capture
//                            pipelines as STREAM_WIFI.
//
// 1/2/6 switch directly between each other (no need to send 3 first); 4/5
// force an implicit teardown of whatever streaming pipeline is active first.
// Every transition creates the pipelines it needs fresh and deletes whatever
// was running before it -- see imu.h/camera.h/sysstats.h for the pipelines
// themselves and net_client.h/tcp_client.h/sdcard.h for the three sinks.
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
    APP_STATE_STREAM_TCP,
} app_state_t;

// Create main_state_machine_task (prio CONFIG_STATE_MACHINE_TASK_PRIORITY,
// core CONFIG_STATE_MACHINE_TASK_CORE). Call once at boot, after
// camera_init()/imu_init() have both succeeded. `sdcard_available` should be
// the result of sdcard_init() (or false if CONFIG_USE_SDCARD is off) --
// command 2 (STREAM_SDCARD) is rejected when false.
esp_err_t state_machine_start(bool sdcard_available);

// --------------------------------------------------------------------------
// Second command source: control_link.c (the always-on host control
// channel, see control_link.h). Both this and the serial console funnel
// into the exact same handle_command()/enter_*() transition functions --
// these just get a synthetic command into the queue main_state_machine_task
// already drains every CONFIG_STATE_MACHINE_POLL_MS.
// --------------------------------------------------------------------------

// Post one command byte, same encoding as a serial digit ('1'-'6'). Returns
// ESP_ERR_NO_MEM if the (8-deep) queue is full -- essentially unreachable at
// the rate a host UI would send these.
esp_err_t state_machine_post_command(char c);

// Answer the IMU-calibration gravity-axis prompt (state_machine.c
// select_gravity_axis()) from the control link instead of the serial
// console. Returns false if nothing is currently waiting for an answer
// (state isn't APP_STATE_IMU_CALIBRATION, or the prompt already got an
// answer) -- the caller should surface that as an error, not retry silently.
bool state_machine_post_imu_axis(char c);

// Best-effort snapshot of the current state, for control_link.c's
// get_status. Not mutex-guarded (s_state is a single aligned word written
// from exactly one task) -- same "eventually consistent" spirit as every
// other cross-task status read in this codebase.
app_state_t state_machine_get_state(void);

// Name matching what control_link.c's set_state command accepts, e.g.
// "stream_wifi". "unknown" for any value outside app_state_t's range.
const char *state_machine_state_name(app_state_t s);
