#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// BMI270 accel+gyro capture with per-sample sensor-time stamping.
//
// A dedicated FreeRTOS task (pinned to core 1) polls the BMI270 at 100 Hz into
// a ring buffer. Each sample carries the BMI270's own 24-bit sensor-time tick
// (39.0625 µs/tick). Callers drain the ring together with a reference pair
// (ESP32 µs + sensor tick captured at drain time); the host reconstructs each
// sample's ESP32 timestamp as:
//
//   sample_esp_us = ref_esp_us + (int32)(sens_time - ref_sensor_ticks) * 39.0625
//
// This shared-clock scheme keeps camera and IMU timestamps comparable — keep
// it intact when touching this module.
// --------------------------------------------------------------------------

#define IMU_RING_CAP 1000                 // 10 s at the current 100 Hz ODR;
                                           // sized for ~1 s once ODR is raised
                                           // toward 1 kHz (not yet done — the
                                           // BMI270 ODR in imu.c is still 100 Hz)
#define IMU_SENSORTIME_US 39.0625         // BMI270 tick resolution (µs)

// On-wire IMU payload sizes (little-endian), shared by the local server and
// the remote stream:
//   header (16): u32 num_samples, i64 ref_esp_us, u32 ref_sensor_ticks
//   sample (28): u32 sens_time, f32 ax, ay, az, gx, gy, gz
#define IMU_WIRE_HEADER_LEN 16
#define IMU_WIRE_SAMPLE_LEN 28
#define IMU_WIRE_MAXLEN (IMU_WIRE_HEADER_LEN + IMU_RING_CAP * IMU_WIRE_SAMPLE_LEN)

typedef struct {
    uint32_t sens_time;        // raw BMI270 24-bit sensor clock ticks
    float    ax, ay, az;       // accelerometer (g)
    float    gx, gy, gz;       // gyroscope (deg/s)
} imu_sample_t;

// Bring up GPIO mode/address pins, I2C, BMI270 (accel+gyro @ 100 Hz), and start
// the polling task. Returns ESP_OK once sampling is running.
esp_err_t imu_start(void);

// Drain the ring into `out` (which must hold at least IMU_RING_CAP samples),
// clear it, and capture the reference clock pair. Returns the sample count.
uint32_t imu_drain(imu_sample_t *out, int64_t *ref_esp_us,
                   uint32_t *ref_sensor_ticks);

// Drain the ring and serialize it into the on-wire payload described above.
// `out` must hold at least IMU_WIRE_MAXLEN bytes. Returns bytes written.
size_t imu_serialize(uint8_t *out);

// A second, independent drain of the same 100 Hz sample stream — same
// semantics as imu_drain() but backed by its own ring, so the SD-card logger
// can run concurrently with imu_drain()/imu_serialize() (e.g. the network
// stream) without the two stealing samples from each other.
uint32_t imu_sdlog_drain(imu_sample_t *out, int64_t *ref_esp_us,
                         uint32_t *ref_sensor_ticks);

// Non-destructive copy of the most recent sample (for a human-readable peek).
// Returns the number of samples currently buffered; `out` may be left zeroed
// when the ring is empty.
uint32_t imu_peek_latest(imu_sample_t *out);
