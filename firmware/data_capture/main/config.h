#pragma once

// --------------------------------------------------------------------------
// Build-time configuration for the data_capture app.
//
// The firmware always captures camera frames + IMU samples on a shared clock.
// How that data leaves the board is selected here — the two sinks are
// independent and can be enabled together:
//
//   CONFIG_ENABLE_LOCAL_SERVER  — run an on-board HTTP server the host pulls
//                                 from (GET /capture, /imu, ...). This is the
//                                 original behaviour.
//   CONFIG_ENABLE_REMOTE_STREAM — push frames + IMU to a computer at
//                                 CONFIG_REMOTE_HOST:CONFIG_REMOTE_PORT.
// --------------------------------------------------------------------------

// WiFi (STA mode)
#define WIFI_SSID "Jarvis_slow"
#define WIFI_PASS "Someoneisusingmydata"

// Data sinks — enable either or both.
#define CONFIG_ENABLE_LOCAL_SERVER  0
#define CONFIG_ENABLE_REMOTE_STREAM 1

// Remote host to push to when CONFIG_ENABLE_REMOTE_STREAM is set.
#define CONFIG_REMOTE_HOST "192.168.178.22"
#define CONFIG_REMOTE_PORT 8080

// Frame push rate for the remote stream (Hz). IMU is drained each cycle.
#define CONFIG_REMOTE_STREAM_FPS 3

// System-stats (CPU load / heap / temperature) push rate (Hz). Kept low so the
// telemetry adds negligible compute over the camera + IMU streaming.
#define CONFIG_REMOTE_STATS_HZ 1

// When set, log elapsed ms for each capture/send stage (camera + IMU) so you
// can see which stage dominates cycle time. See debug_time.h.
#define CONFIG_DEBUG_TIME 0

// SD card logging (camera frames + IMU samples to local storage, independent
// of the network sinks above).
//   CONFIG_USE_SDCARD          — mount the SD card and log frames/IMU to it.
//   CONFIG_ENABLE_SDCARD_FORMAT — if the card fails to mount (blank or a
//                                 filesystem FatFs doesn't recognise), format
//                                 it instead of failing. Leave off once a
//                                 card has been formatted, to avoid wiping it
//                                 on an unrelated mount failure.
#define CONFIG_USE_SDCARD           0
#define CONFIG_ENABLE_SDCARD_FORMAT 0

