#pragma once

#include "config.h"

// --------------------------------------------------------------------------
// Lightweight per-stage timing, compiled out entirely unless CONFIG_DEBUG_TIME
// is set in config.h. Wrap a stage like:
//
//   DEBUG_TIME_START(t0);
//   camera_grab(&ts);
//   DEBUG_TIME_END(t0, TAG, "camera capture");
//
// which logs e.g. "CAM: camera capture took 12.34 ms".
// --------------------------------------------------------------------------

#if CONFIG_DEBUG_TIME

#include "esp_timer.h"
#include "esp_log.h"

#define DEBUG_TIME_START(var) int64_t var = esp_timer_get_time()
#define DEBUG_TIME_END(var, tag, label) \
    ESP_LOGI(tag, label " took %.2f ms", (esp_timer_get_time() - (var)) / 1000.0)

#else

#define DEBUG_TIME_START(var) do {} while (0)
#define DEBUG_TIME_END(var, tag, label) do {} while (0)

#endif
