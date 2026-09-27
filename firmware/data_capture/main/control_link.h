#pragma once

#include <stdbool.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// Always-on device control channel — the network half of "host server as the
// primary control surface" (docs/camera_calibration_and_tuning.md "Control
// channel"). Unlike cam_calib.c's dedicated calibration socket (which only
// exists while APP_STATE_CAMERA_CALIBRATION is active), this one is dialed at
// boot and kept alive (with the same dial/retry/reconnect pattern cam_calib.c
// uses) for the device's entire runtime, independent of app_state_t.
//
// Device dials CONFIG_REMOTE_HOST:CONFIG_CONTROL_PORT; the host side is
// software/host_server/control (a small always-running http.server app with a
// browser UI). Same framing as cam_calib.c (uint32 len LE | uint8 type |
// body), but this channel only ever carries MSG_JSON.
//
// Commands:
//   ping
//   get_status                                    -> {"state": "...", ...}
//   set_state {"state": "idle"|"stream_wifi"|"stream_sdcard"|
//              "stream_tcp"|"camera_calibration"|"imu_calibration"}
//     Translates to the same single-digit commands the serial console sends
//     (state_machine.h) and posts it into the same command queue
//     main_state_machine_task drains -- there is exactly one place that ever
//     mutates the state, whether the request came from serial or here. The
//     reply just confirms the request was queued; the transition itself
//     happens asynchronously (within CONFIG_STATE_MACHINE_POLL_MS) -- poll
//     get_status to observe the result, same as the serial console's "->
//     STREAM_WIFI" log line is asynchronous relative to typing '1'.
//   imu_cal_axis {"axis": 1-6}   answers the gravity-axis prompt that
//                                imu_cal_preview (an event, not a reply) asks
//                                for -- only meaningful while state is
//                                imu_calibration; see state_machine.c
//                                select_gravity_axis().
//   imu_cal_abort                same effect as sending anything outside
//                                1-6 on serial: aborts the calibration.
//
// Events (device -> host, unprompted, no "id"): imu_cal_preview (the same
// 10-sample gravity-axis preview the serial console logs, see
// state_machine.c), imu_cal_report (the same before/after FOC report).
// --------------------------------------------------------------------------

// Spawn the control_link task (dial CONFIG_REMOTE_HOST:CONFIG_CONTROL_PORT,
// retry/reconnect forever). Call once at boot, after state_machine_start().
esp_err_t control_link_start(void);

// Send a pre-formatted JSON object as an event, if (and only if) a host is
// currently connected -- a no-op, not an error, otherwise. Used by
// state_machine.c to mirror its IMU-calibration prompts/report over the
// network alongside the existing ESP_LOG lines. NOT for replies to a
// request (those are handled internally by control_link.c itself).
bool control_link_send_event(const char *json_fmt, ...) __attribute__((format(printf, 1, 2)));
