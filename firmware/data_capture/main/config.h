#pragma once

// --------------------------------------------------------------------------
// Build-time configuration for the data_capture app.
//
// Runtime behaviour is driven by main_state_machine_task (serial commands
// 1-5 select IDLE / STREAM_WIFI / STREAM_SDCARD / IMU_CALIBRATION /
// CAMERA_CALIBRATION) — see docs/architecture.md for the full design. This
// file holds every tunable the state machine and its producer/consumer/stats
// tasks read: task priorities, core pinning, periods, and queue depths.
// --------------------------------------------------------------------------

// WiFi (STA mode)
#define WIFI_SSID "Jarvis_slow"
#define WIFI_PASS "Someoneisusingmydata"

// Remote host the wifi consumer tasks push frames/IMU/stats to during
// STREAM_WIFI.
#define CONFIG_REMOTE_HOST "192.168.178.22"
#define CONFIG_REMOTE_PORT 8080

// Ports the tcp_client.c consumers connect to during STREAM_TCP (raw
// length-prefixed binary stream -- see tcp_client.h). One dedicated
// persistent connection per stream rather than sharing one socket: each
// connection has exactly one writer task, so no cross-stream mutex is
// needed, and a large/slow frame send can never head-of-line-block the
// small time-critical IMU/stats sends behind it. Distinct from
// CONFIG_REMOTE_PORT (STREAM_WIFI's HTTP port).
#define CONFIG_REMOTE_TCP_FRAME_PORT 8081
#define CONFIG_REMOTE_TCP_IMU_PORT   8082
#define CONFIG_REMOTE_TCP_STATS_PORT 8083

// --------------------------------------------------------------------------
// main_state_machine_task — owns the IDLE/STREAM_WIFI/STREAM_SDCARD/
// CALIBRATION state and creates/deletes the pipelines below on transition.
// Deliberately low-rate: polls the console UART non-blockingly, does not
// need to be woken faster than an operator can type.
// --------------------------------------------------------------------------
#define CONFIG_STATE_MACHINE_TASK_PRIORITY 10
#define CONFIG_STATE_MACHINE_TASK_CORE     0
#define CONFIG_STATE_MACHINE_POLL_MS       500

// --------------------------------------------------------------------------
// Feature toggles — disable IMU or camera capture/streaming entirely to
// isolate the other's performance (e.g. does turning off IMU logging speed
// up camera frame writes on SD?). CONFIG_ENABLE_IMU=0 stops imu_capture_task
// itself (imu.c) as well as both consumer paths (net_client.c, sdcard.c).
// CONFIG_ENABLE_CAMERA=0 only gates the consumer paths -- camera_capture_task
// (camera.c) keeps running at its already-low FPS regardless.
// --------------------------------------------------------------------------
#define CONFIG_ENABLE_IMU     1
#define CONFIG_ENABLE_CAMERA  1

// --------------------------------------------------------------------------
// IMU pipeline: imu_capture_task (esp_timer @ IMU_CAPTURE_PERIOD_MS wakes the
// task via task-notify — NOT a FreeRTOS software timer, see
// docs/architecture.md "Why esp_timer + task-notify") -> imu_queue ->
// imu_wifi_consumer_task / imu_sdcard_consumer_task (only one runs at a time).
// --------------------------------------------------------------------------
#define CONFIG_IMU_CAPTURE_PRIORITY   22
#define CONFIG_IMU_CAPTURE_CORE       1
#define CONFIG_IMU_CAPTURE_PERIOD_MS  2

#define CONFIG_IMU_QUEUE_LEN          1000   // ~200ms of samples at 1kHz

#define CONFIG_IMU_CONSUMER_PRIORITY  18
#define CONFIG_IMU_CONSUMER_CORE      0
#define CONFIG_IMU_CONSUMER_PERIOD_MS 500
#define CONFIG_IMU_CONSUMER_BATCH     500   // samples drained per wake

// --------------------------------------------------------------------------
// Camera pipeline: camera_capture_task (esp_timer @ derived period ->
// task-notify) -> camera_queue (holds {fb pointer, ts_us}, not frame data) ->
// camera_wifi_consumer_task / camera_sdcard_consumer_task.
// --------------------------------------------------------------------------
#define CONFIG_CAMERA_CAPTURE_PRIORITY 10
#define CONFIG_CAMERA_CAPTURE_CORE     1

// Valid range 1-20 fps (i.e. capture period clamped 1000ms-50ms).
#define CONFIG_CAMERA_CAPTURE_FPS      10
#if CONFIG_CAMERA_CAPTURE_FPS < 1 || CONFIG_CAMERA_CAPTURE_FPS > 30
#error "CONFIG_CAMERA_CAPTURE_FPS must be between 1 and 20"
#endif
#define CONFIG_CAMERA_CAPTURE_PERIOD_MS (1000 / CONFIG_CAMERA_CAPTURE_FPS)

#define CONFIG_CAMERA_QUEUE_LEN      2   // CONFIG_CAMERA_CAPTURE_FPS

// Frame buffers backing the camera driver (PSRAM). Every frame sitting in
// camera_queue holds one buffer checked out, so fb_count must be >=
// queue_len — otherwise camera_grab() blocks once fb_count frames are in
// flight, well before the queue's own drop-oldest backpressure ever kicks
// in, and the "queue depth of 10" becomes a lie. Kept equal to the queue
// length rather than picked independently.
#define CONFIG_CAMERA_FB_COUNT          CONFIG_CAMERA_QUEUE_LEN

#define CONFIG_CAMERA_CONSUMER_PRIORITY  17
#define CONFIG_CAMERA_CONSUMER_CORE      0
#define CONFIG_CAMERA_CONSUMER_PERIOD_MS 10
 
// --------------------------------------------------------------------------
// Stats pipeline: stats_producer_task samples sysstats and pushes to
// stats_queue; stats_writer_task drains it, branching on whichever sink
// (wifi/sdcard) is currently active — unlike IMU/camera it's one task, not a
// pair, since it's simple enough not to need reuse-by-splitting yet.
// --------------------------------------------------------------------------
#define CONFIG_STATS_PRODUCER_PRIORITY   12
#define CONFIG_STATS_PRODUCER_CORE       1
#define CONFIG_STATS_PRODUCER_PERIOD_MS  500

#define CONFIG_STATS_QUEUE_LEN           8

#define CONFIG_STATS_CONSUMER_PRIORITY   7
#define CONFIG_STATS_CONSUMER_CORE       0
#define CONFIG_STATS_CONSUMER_PERIOD_MS  500

// When set, log elapsed ms for each capture/send stage (camera + IMU) so you
// can see which stage dominates cycle time. See debug_time.h.
#define CONFIG_DEBUG_TIME 1

// SD card mount (compile-time: whether SD card support is built in at all —
// STREAM_SDCARD is only reachable if this is set).
//   CONFIG_USE_SDCARD          — mount the SD card at boot.
//   CONFIG_ENABLE_SDCARD_FORMAT — if the card fails to mount (blank or a
//                                 filesystem FatFs doesn't recognise), format
//                                 it instead of failing. Leave off once a
//                                 card has been formatted, to avoid wiping it
//                                 on an unrelated mount failure.
#define CONFIG_USE_SDCARD           1
#define CONFIG_ENABLE_SDCARD_FORMAT 0

