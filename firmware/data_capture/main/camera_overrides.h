#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// Persisted OV5640 register overrides — the "save tuned registers" half of
// the host-driven camera calibration workflow (see cam_calib.c's
// save_camera_regs/clear_camera_regs commands and
// docs/camera_calibration_and_tuning.md "Persisting tuned registers").
//
// The OV5640 itself has no non-volatile register storage: every register
// resets to the driver's compiled-in defaults on power-cycle or
// esp_camera_init(). This module is the workaround — a small set of
// {addr, val} writes, stored in ESP32 NVS (namespace "cam_ovr", same
// nvs_open/nvs_set_blob/nvs_commit pattern as imu.c's FOC run counter), that
// camera_init() replays after every successful esp_camera_init() so a tuned
// exposure/AWB/orientation/etc. setting survives a reboot without any host
// involvement.
//
// This module does not decide *what* is safe to persist (resets, clocks,
// live AEC/AWB statistics, and resolution-specific timing registers should
// never be blindly replayed) -- it stores and applies whatever {addr, val}
// list it is given, verbatim. That filtering is the host's job: it already
// has UNSAFE_RESTORE / RegisterBank.volatile() in
// software/host_server/calibration/registers.py, reused via
// RegisterBank.save_candidates() to build the list passed to
// camera_overrides_save().
// --------------------------------------------------------------------------

typedef struct {
    uint16_t addr;
    uint8_t  val;
} camera_override_write_t;

// Upper bound on how many distinct register overrides can be stored at
// once. Comfortably covers a tuning session (cam_calib.c's own KEY_REGS
// list is ~35 registers) while keeping the NVS blob small.
#define CAMERA_OVERRIDES_MAX_REGS 128

// Re-apply every saved override to the current sensor (esp_camera_sensor_get()
// must already be up). Called from camera_init() right after a successful
// esp_camera_init() -- i.e. on every plain boot/streaming bring-up. Not
// called from camera_init_ex() (calibration-mode reinit), so a half-tuned
// live session never gets clobbered by its own previously-saved values.
// A no-op (ESP_OK) if nothing has been saved yet.
esp_err_t camera_overrides_apply(void);

// Merge `writes` into the saved set (last write per address wins), persist
// to NVS, and apply the merged set to the live sensor immediately. Returns
// ESP_ERR_NO_MEM if the merged set would exceed CAMERA_OVERRIDES_MAX_REGS
// distinct addresses -- the save is rejected whole (nothing partially
// written) so the caller can retry with a smaller list.
esp_err_t camera_overrides_save(const camera_override_write_t *writes, size_t n);

// Erase the saved set from NVS. Does not touch the live sensor -- the
// operator is expected to power-cycle or re-enter streaming mode to see
// defaults again (mirrors how the overrides themselves only take effect on
// the next camera_init()).
esp_err_t camera_overrides_clear(void);

// Copy up to `max` saved {addr, val} pairs into `out`. Returns the total
// number of saved overrides, which may exceed `max` -- only the first `max`
// were copied in that case.
size_t camera_overrides_get(camera_override_write_t *out, size_t max);
