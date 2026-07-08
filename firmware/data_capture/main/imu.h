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
