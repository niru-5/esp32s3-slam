#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// --------------------------------------------------------------------------
// TCP sink — raw framed binary stream to
// CONFIG_REMOTE_HOST:CONFIG_REMOTE_TCP_PORT, as a lower-overhead alternative
// to net_client.c's HTTP POST sink (no per-message HTTP request/response,
// just one persistent socket). One connection carries all three streams
// (frame/imu/stats), multiplexed by a small header per message:
//
//   uint8_t  type;   TCP_MSG_FRAME=1 / TCP_MSG_IMU=2 / TCP_MSG_STATS=3
//   uint32_t len;    payload length that follows (little-endian)
//   uint8_t  payload[len];
//
// Payloads:
//   FRAME  -- int64 ts_us + JPEG bytes
//   IMU    -- imu_serialize_wire() output, unchanged (see imu.h)
//   STATS  -- sysstats_serialize_snapshot() output, unchanged (see sysstats.h)
//
// camera_tcp_consumer_task and imu_tcp_consumer_task (this file) plus
// stats_writer_task (sysstats.c, via tcp_client_send_stats) all share one
// socket, so every send is wrapped in a mutex to keep a message's header+
// payload atomic on the wire. Matches software/host_server/tcp_ingest.py --
// keep the two in sync.
// --------------------------------------------------------------------------

#define TCP_MSG_FRAME 1
#define TCP_MSG_IMU   2
#define TCP_MSG_STATS 3

// Create imu_tcp_consumer_task + camera_tcp_consumer_task (priorities/cores/
// periods reused from config.h's IMU/CAMERA consumer settings — only one of
// the wifi/tcp/sdcard sinks runs at a time). The socket itself connects
// lazily on first send, not here. Returns ESP_OK once both tasks are running.
esp_err_t tcp_client_pipeline_start(void);

// Delete the tcp consumer tasks and close the socket + mutex.
void tcp_client_pipeline_stop(void);

// Push one system-stats snapshot payload (SYSSTATS_WIRE_LEN bytes). Called by
// stats_writer_task when the tcp sink is active.
esp_err_t tcp_client_send_stats(const uint8_t *payload, size_t len);
