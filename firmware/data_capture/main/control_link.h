#pragma once

#include <stdbool.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// Always-on device control channel — the network half of "one host server,
// one browser page" (docs/camera_calibration_and_tuning.md "Control
// channel"). Dialed at boot and kept alive (dial/retry/reconnect) for the
// device's entire runtime, independent of app_state_t. This is the device's
// ONLY network-control socket: what used to be cam_calib.c's separate
// dedicated calibration socket (CONFIG_CAM_CALIB_PORT) is merged in here --
// that split only existed so two separate host processes wouldn't contend
// for one port, and with a single host process (software/host_server) there
// is no more reason for a second one.
//
// Device dials CONFIG_REMOTE_HOST:CONFIG_CONTROL_PORT; the host side is
// software/host_server (one process, one browser page). Framing: uint32 len
// LE | uint8 type | body, type 0x01 JSON (both directions), type 0x02 IMAGE
// (device -> host, camera_calibration captures only: uint32 meta_len | meta
// JSON | frame bytes).
//
// Commands (always available):
//   ping
//   get_status                    -> {"state":..,"imu_available":bool,...}
//   set_state {"state": "idle"|"stream_wifi"|"stream_sdcard"|
//              "stream_tcp"|"camera_calibration"|"imu_calibration",
//              "fps": n, "framesize": "...", "include_imu": bool}
//     Translates to the same single-digit commands the serial console sends
//     (state_machine.h) and posts it into the same command queue
//     main_state_machine_task drains -- there is exactly one place that ever
//     mutates the state, whether the request came from serial or here.
//     fps/framesize/include_imu only matter for the stream_* states (see
//     state_machine_post_stream_params()); the reply just confirms the
//     request was queued -- poll get_status to observe the actual result.
//   imu_cal_axis {"axis": 1-6}   answers the gravity-axis prompt
//                                (imu_cal_preview event) -- see
//                                state_machine.c select_gravity_axis().
//   imu_cal_abort                same effect as sending anything outside
//                                1-6 on serial: aborts the calibration.
//
// Commands (only while state is camera_calibration -- "not in
// camera_calibration" otherwise): info, set_mode, reg_read, reg_write,
// reg_dump, set_orientation, fps_probe, capture, save_camera_regs,
// clear_camera_regs, get_camera_overrides, exit -- see
// docs/camera_calibration_and_tuning.md "Firmware side" for each one's
// shape; unchanged from the old cam_calib.c protocol.
//
// Events (device -> host, unprompted, no "id"): imu_cal_preview, imu_cal_report
// (state_machine.c's IMU calibration flow), reg_chunk (reg_dump's streamed
// chunks, carries the requesting command's "id").
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
