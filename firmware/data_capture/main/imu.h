#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// BMI270 accel+gyro capture with per-sample sensor-time stamping.
//
// imu_init() brings up the I2C bus + BMI270 chip once at boot. The sampling
// pipeline itself only exists while STREAM_WIFI/STREAM_SDCARD is active --
// imu_pipeline_start()/imu_pipeline_stop() create/tear down imu_capture_task
// and imu_queue together (see docs/architecture.md "Task & queue
// architecture"). imu_capture_task is woken every CONFIG_IMU_CAPTURE_PERIOD_MS
// by an esp_timer callback that does nothing but notify the task -- NOT a
// FreeRTOS software timer (the FreeRTOS tick is 10ms, too coarse for a 1ms
// period, and timer callbacks all run on the single low-priority Timer
// Service task regardless of the "owning" task's priority -- see
// docs/architecture.md "Why esp_timer + task-notify").
//
// Each sample carries the BMI270's own 24-bit sensor-time tick (39.0625
// us/tick). imu_queue_drain() hands back a reference pair (ESP32 us + sensor
// tick captured at drain time) alongside the batch; callers reconstruct each
// sample's ESP32 timestamp as:
//
//   sample_esp_us = ref_esp_us + (int32)(sens_time - ref_sensor_ticks) * 39.0625
//
// This shared-clock scheme keeps camera and IMU timestamps comparable --
// keep it intact when touching this module. The on-wire layout below is
// unchanged from the previous ring-buffer implementation (software/host_server
// already decodes it), so touching it requires updating wire.py too.
// --------------------------------------------------------------------------

#define IMU_SENSORTIME_US 39.0625         // BMI270 tick resolution (us)

// On-wire IMU payload sizes (little-endian), used by imu_wifi_consumer_task
// (net_client.c) -- kept identical to software/host_server/wire.py:
//   header (16): u32 num_samples, i64 ref_esp_us, u32 ref_sensor_ticks
//   sample (28): u32 sens_time, f32 ax, ay, az, gx, gy, gz
#define IMU_WIRE_HEADER_LEN 16
#define IMU_WIRE_SAMPLE_LEN 28

typedef struct {
    uint32_t sens_time;        // raw BMI270 24-bit sensor clock ticks
    float    ax, ay, az;       // accelerometer (g)
    float    gx, gy, gz;       // gyroscope (deg/s)
} imu_sample_t;

// Bring up GPIO mode/address pins, I2C, and the BMI270 (accel+gyro, 1600Hz
// ODR -- see imu.c for why 1600Hz rather than the previous 100Hz). Does NOT
// start sampling -- call imu_pipeline_start() for that. Returns ESP_OK once
// the chip is ready.
esp_err_t imu_init(void);

// Create imu_queue (CONFIG_IMU_QUEUE_LEN deep) and imu_capture_task (prio
// CONFIG_IMU_CAPTURE_PRIORITY, core CONFIG_IMU_CAPTURE_CORE), driven by an
// esp_timer at CONFIG_IMU_CAPTURE_PERIOD_MS. Returns ESP_OK once running.
esp_err_t imu_pipeline_start(void);

// Delete imu_capture_task, stop+delete its esp_timer, and delete imu_queue
// (any samples still queued are discarded). Safe to call even if the
// pipeline isn't running.
void imu_pipeline_stop(void);

// Drain up to `max_count` samples from imu_queue into `out`, and capture a
// fresh (ESP32 us, BMI270 sensor tick) reference pair for the batch (one
// direct BMI270 read -- the shared-clock reference doesn't need to be
// per-sample, just aligned to *a* known instant). Returns the number of
// samples written to `out`. Non-blocking: returns 0 immediately if the queue
// is empty or the pipeline isn't running.
uint32_t imu_queue_drain(imu_sample_t *out, uint32_t max_count,
                         int64_t *ref_esp_us, uint32_t *ref_sensor_ticks);

// Current imu_queue depth / cumulative drop-oldest overflow count, for
// logging. Both read 0 if the pipeline isn't running.
uint32_t imu_queue_depth(void);
uint32_t imu_queue_overflow_count(void);

// Serialize a drained batch into the wire format documented above (header +
// per-sample, little-endian, no padding). Used identically by
// net_client.c's imu_wifi_consumer_task (POST body) and sdcard.c's
// imu_sdcard_consumer_task (raw file bytes) so STREAM_WIFI and STREAM_SDCARD
// produce byte-identical IMU records. `out` must be at least
// IMU_WIRE_HEADER_LEN + count*IMU_WIRE_SAMPLE_LEN bytes. Returns the number
// of bytes written.
size_t imu_serialize_wire(const imu_sample_t *samples, uint32_t count,
                          int64_t ref_esp_us, uint32_t ref_ticks, uint8_t *out);

// --------------------------------------------------------------------------
// Bias/offset calibration (state_machine.c command 4). Separate from the
// imu_queue pipeline above -- these poll the BMI270 directly, so only call
// them while imu_pipeline_start() is NOT running (state_machine.c already
// tears down any active pipeline before entering APP_STATE_IMU_CALIBRATION).
//
// This covers offset/bias only. Noise-density (white noise) and bias
// random-walk coefficients, and Allan-variance characterization, are future
// work -- see docs/calibration.md.
// --------------------------------------------------------------------------

typedef struct {
    uint32_t sample_count;
    float ax_mean, ay_mean, az_mean;   // g
    float gx_mean, gy_mean, gz_mean;   // deg/s
    float ax_std,  ay_std,  az_std;    // g      -- stationarity check
    float gx_std,  gy_std,  gz_std;    // deg/s  -- stationarity check
} imu_stationary_stats_t;

// Poll the BMI270 directly for duration_ms and return per-axis mean/stddev.
// The rig must be held flat and still for the result to mean anything --
// large stddev values mean it wasn't. Returns ESP_FAIL if no samples were
// read (chip not initialized, or I2C errors throughout the window).
esp_err_t imu_capture_stationary_stats(uint32_t duration_ms, imu_stationary_stats_t *out);

typedef struct {
    float ax, ay, az;   // g
    float gx, gy, gz;   // deg/s
} imu_raw_preview_sample_t;

// Poll the BMI270 directly for `count` raw samples, spaced period_ms apart,
// with no averaging -- used to show an operator a live look at each axis
// (e.g. 10 samples over 100ms) so they can tell which one reads ~1g before
// picking a gravity axis for imu_run_hw_foc_calibration(). Returns ESP_FAIL
// on the first I2C read failure.
esp_err_t imu_preview_raw_samples(imu_raw_preview_sample_t *out, uint32_t count, uint32_t period_ms);

// Which axis (and sign) the operator has identified as aligned with gravity
// with the rig held stationary -- fed into bmi2_perform_accel_foc. Values
// are deliberately 1-6 so a console digit '1'..'6' casts straight across
// (see state_machine.c imu_calibration_run()).
typedef enum {
    IMU_ACCEL_FOC_AXIS_POS_X = 1,
    IMU_ACCEL_FOC_AXIS_NEG_X,
    IMU_ACCEL_FOC_AXIS_POS_Y,
    IMU_ACCEL_FOC_AXIS_NEG_Y,
    IMU_ACCEL_FOC_AXIS_POS_Z,
    IMU_ACCEL_FOC_AXIS_NEG_Z,
} imu_accel_foc_axis_t;

// Human-readable label ("+X".."−Z") for logging/report text.
const char *imu_accel_foc_axis_label(imu_accel_foc_axis_t axis);

// One-shot hardware bias calibration:
//   1. capture stationary stats ("before")
//   2. bmi2_perform_accel_foc, using gravity_axis as the axis/sign gravity
//      is aligned with (the operator determines this by eye from
//      imu_preview_raw_samples() -- FOC has no way to detect orientation
//      itself, it only computes the offset needed to make the axis you tell
//      it read exactly +-1g) + bmi2_perform_gyro_foc (no orientation
//      dependency -- ideal stationary reading is 0 on every axis). Both
//      auto-enable offset compensation on the live data path as part of the
//      routine, so no other firmware change is needed for the correction to
//      take effect.
//   3. bmi2_nvm_prog to persist the resulting offset registers so they
//      survive power-cycles
//   4. capture stationary stats again ("after"), to confirm it helped
//
// BMI270 NVM write endurance is limited -- this bumps a persistent NVS
// counter (*foc_run_count_out) each call and logs a warning past the first,
// so it stays a rare, deliberate operation rather than something run on
// every boot. Returns ESP_OK only if FOC succeeded AND the NVM commit
// succeeded; if NVM commit failed the offsets are still active for this
// power cycle (FOC applies them immediately) but will NOT survive a reboot.
esp_err_t imu_run_hw_foc_calibration(uint32_t sample_window_ms,
                                      imu_accel_foc_axis_t gravity_axis,
                                      imu_stationary_stats_t *before_out,
                                      imu_stationary_stats_t *after_out,
                                      uint32_t *foc_run_count_out);
