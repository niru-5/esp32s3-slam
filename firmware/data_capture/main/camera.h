#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "esp_camera.h"

// --------------------------------------------------------------------------
// OV5640 camera capture (JPEG, VGA, CONFIG_CAMERA_FB_COUNT buffers in PSRAM).
//
// camera_init() brings up the sensor once at boot. camera_pipeline_start()/
// camera_pipeline_stop() create/tear down camera_capture_task -> camera_queue,
// mirroring the IMU pipeline: an esp_timer at the FPS-derived period notifies
// the capture task (see docs/architecture.md "Why esp_timer + task-notify"),
// which grabs a frame and pushes {fb, ts_us} onto the queue.
//
// fb_count is kept equal to the queue depth (CONFIG_CAMERA_FB_COUNT ==
// CONFIG_CAMERA_QUEUE_LEN, see config.h) -- every frame sitting in the queue
// holds one buffer checked out, so a smaller fb_count would make
// esp_camera_fb_get() block once that many frames are in flight, well
// before the queue's own drop-oldest backpressure ever triggers.
//
// Consumers must camera_release() every frame they drain via
// camera_queue_drain() -- including ones the queue itself evicted on
// overflow, which this module already releases internally.
// --------------------------------------------------------------------------

typedef struct {
    camera_fb_t *fb;
    int64_t      ts_us;   // esp_timer us at grab time -- shared-clock reference
} camera_frame_t;

// Initialize the camera hardware only. Returns ESP_OK when ready for
// camera_pipeline_start().
esp_err_t camera_init(void);

// Create camera_queue (CONFIG_CAMERA_QUEUE_LEN deep) and camera_capture_task
// (prio CONFIG_CAMERA_CAPTURE_PRIORITY, core CONFIG_CAMERA_CAPTURE_CORE),
// driven by an esp_timer at CONFIG_CAMERA_CAPTURE_PERIOD_MS. Returns ESP_OK
// once running.
esp_err_t camera_pipeline_start(void);

// Delete camera_capture_task, stop+delete its esp_timer, and
// camera_release() + delete camera_queue (any frames still queued are
// released, not leaked). Safe to call even if the pipeline isn't running.
void camera_pipeline_stop(void);

// Drain up to `max_count` frames from camera_queue into `out`. Returns the
// number written. Non-blocking: returns 0 immediately if the queue is empty
// or the pipeline isn't running. Caller owns every returned fb and MUST
// camera_release() each one once done.
uint32_t camera_queue_drain(camera_frame_t *out, uint32_t max_count);

// Return a frame buffer obtained from camera_queue_drain().
void camera_release(camera_fb_t *fb);

uint32_t camera_queue_depth(void);
uint32_t camera_queue_overflow_count(void);
