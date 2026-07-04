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
#define CONFIG_ENABLE_LOCAL_SERVER  1
#define CONFIG_ENABLE_REMOTE_STREAM 0

// Remote host to push to when CONFIG_ENABLE_REMOTE_STREAM is set.
#define CONFIG_REMOTE_HOST "192.168.1.50"
#define CONFIG_REMOTE_PORT 8080

// Frame push rate for the remote stream (Hz). IMU is drained each cycle.
#define CONFIG_REMOTE_STREAM_FPS 10
