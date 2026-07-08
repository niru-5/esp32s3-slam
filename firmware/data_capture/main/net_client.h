#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// WiFi sink — pushes captured data to a computer at
// CONFIG_REMOTE_HOST:CONFIG_REMOTE_PORT over persistent HTTP/1.1 connections
// (see the comment on post_bytes() in net_client.c for why connections are
// kept open across calls instead of reconnecting per request). Wire formats
// are unchanged from before this pipeline rework and match
// software/host_server/wire.py exactly:
//
//   POST /frame  — body = JPEG, X-Timestamp-Us header
//   POST /imu    — body = IMU wire payload (see imu.h)
//   POST /stats  — body = sysstats wire payload (see sysstats.h)
//
// imu_wifi_consumer_task and camera_wifi_consumer_task each own a separate
// persistent connection (they run concurrently, and an esp_http_client
// handle isn't safe to drive from two tasks at once); net_client_send_stats()
// -- called from stats_writer_task in sysstats.c -- owns a third.
// --------------------------------------------------------------------------

// Create imu_wifi_consumer_task + camera_wifi_consumer_task (priorities/
// cores/periods from config.h). Returns ESP_OK once both are running.
esp_err_t net_client_pipeline_start(void);

// Delete the wifi consumer tasks and close all persistent connections.
void net_client_pipeline_stop(void);

// Push one system-stats snapshot payload (SYSSTATS_WIRE_LEN bytes). Called by
// stats_writer_task when the wifi sink is active.
esp_err_t net_client_send_stats(const uint8_t *payload, size_t len);
