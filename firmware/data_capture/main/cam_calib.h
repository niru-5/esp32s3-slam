#pragma once

#include "esp_err.h"

// --------------------------------------------------------------------------
// Camera reconfigure for CAMERA_CALIBRATION mode, plus tracking of which
// format/size the calibration camera is currently in. This module owns no
// socket and no task -- the calibration protocol itself (info/set_mode/
// reg_read/write/dump/set_orientation/fps_probe/capture/save_camera_regs/
// clear_camera_regs/get_camera_overrides) lives in control_link.c, which
// calls into cam_calib_enter_mode()/cam_calib_exit_mode() on the
// CAMERA_CALIBRATION transition (state_machine.c) and into
// cam_calib_set_mode() whenever its own set_mode command changes the
// camera's format/size. See docs/camera_calibration_and_tuning.md.
// --------------------------------------------------------------------------

// Reconfigure the camera for a calibration session: JPEG SVGA, quality 6, 2
// frame buffers, grab-latest. Resets the tracked mode (cam_calib_fmt()/
// cam_calib_size()) to "jpeg"/"svga" to match. Called synchronously from
// state_machine.c's enter_camera_calibration() -- no task spawn needed,
// control_link.c's already-running task serves the protocol.
esp_err_t cam_calib_enter_mode(void);

// Restore the normal streaming camera config (camera_init()). Called from
// state_machine.c on leaving CAMERA_CALIBRATION (teardown_active_pipelines)
// and from control_link.c's "exit" command handler.
void cam_calib_exit_mode(void);

// Currently tracked format/size names (e.g. "jpeg"/"svga"), echoed in
// info/hello/capture metadata. Valid only while CAMERA_CALIBRATION is
// active; meaningless otherwise.
const char *cam_calib_fmt(void);
const char *cam_calib_size(void);

// Record a successful set_mode's new format/size (control_link.c calls this
// after camera_init_ex() succeeds in its cmd_set_mode handler).
void cam_calib_set_mode(const char *fmt, const char *size);
