#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// TCP sink — three independent persistent connections to
// CONFIG_REMOTE_HOST:{CONFIG_REMOTE_TCP_FRAME_PORT,_IMU_PORT,_STATS_PORT},
// one per stream, as a lower-overhead alternative to net_client.c's HTTP POST
// sink (no per-message HTTP request/response). Each connection has exactly
// one writer task (camera_tcp_consumer_task / imu_tcp_consumer_task / the
// sysstats.c stats_writer_task via tcp_client_send_stats), so — unlike an
// earlier single-shared-socket design — no mutex is needed, and a slow/large
// frame send can never head-of-line-block a small time-critical IMU/stats
// send behind it on the same socket.
//
// Wire framing per connection (little-endian, no padding): since each
// connection only ever carries one message type, there's no per-message type
// tag -- just a length prefix:
//
//   uint32_t len;    payload length that follows
//   uint8_t  payload[len];
//
// Payloads:
//   FRAME  -- int64 ts_us + JPEG bytes
//   IMU    -- imu_serialize_wire() output, unchanged (see imu.h)
//   STATS  -- sysstats_serialize_snapshot() output, unchanged (see sysstats.h)
//
// Matches software/host_server/tcp_ingest.py -- keep the two in sync.
// --------------------------------------------------------------------------

// Create imu_tcp_consumer_task + camera_tcp_consumer_task (priorities/cores/
// periods reused from config.h's IMU/CAMERA consumer settings — only one of
// the wifi/tcp/sdcard sinks runs at a time). Sockets connect lazily on first
// send, not here. Returns ESP_OK once both tasks are running.
esp_err_t tcp_client_pipeline_start(void);

// Delete the tcp consumer tasks and close all three connections.
void tcp_client_pipeline_stop(void);

// Push one system-stats snapshot payload (SYSSTATS_WIRE_LEN bytes) on the
// stats connection. Called by stats_writer_task when the tcp sink is active.
esp_err_t tcp_client_send_stats(const uint8_t *payload, size_t len);
