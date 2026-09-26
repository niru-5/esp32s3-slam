#pragma once

// --------------------------------------------------------------------------
// Build-time configuration for the inference_on_esp32s3 app.
// See docs/inference_on_esp32s3.md for how the pieces fit together.
// --------------------------------------------------------------------------

// Board selection -- set exactly one to 1 (same two boards / same pin blocks
// as firmware/data_capture/main/config.h).
//   BOARD_ESP32S3_MAIN -- main rig: custom OV5640 wiring, microSD on SDMMC 1-bit.
//   BOARD_XIAO_ESP32S3 -- Seeed XIAO ESP32-S3 Sense: onboard camera, microSD on SPI.
#define BOARD_ESP32S3_MAIN   0
#define BOARD_XIAO_ESP32S3   1

#if (BOARD_ESP32S3_MAIN + BOARD_XIAO_ESP32S3) != 1
#error "config.h: set exactly one BOARD_* to 1"
#endif

#if BOARD_ESP32S3_MAIN
#define CONFIG_CAM_PWDN_GPIO  -1
#define CONFIG_CAM_RESET_GPIO -1
#define CONFIG_CAM_XCLK_GPIO  15
#define CONFIG_CAM_SIOD_GPIO   4
#define CONFIG_CAM_SIOC_GPIO   5
#define CONFIG_CAM_Y9_GPIO    16
#define CONFIG_CAM_Y8_GPIO    17
#define CONFIG_CAM_Y7_GPIO    18
#define CONFIG_CAM_Y6_GPIO    12
#define CONFIG_CAM_Y5_GPIO    10
#define CONFIG_CAM_Y4_GPIO     8
#define CONFIG_CAM_Y3_GPIO     9
#define CONFIG_CAM_Y2_GPIO    11
#define CONFIG_CAM_VSYNC_GPIO  6
#define CONFIG_CAM_HREF_GPIO   7
#define CONFIG_CAM_PCLK_GPIO  13
// microSD (only used if the model location is set to "sdcard" in menuconfig)
#define SD_BUS_SDMMC_1BIT     1
#define SD_PIN_CLK            39
#define SD_PIN_CMD            38
#define SD_PIN_D0             40
#elif BOARD_XIAO_ESP32S3
#define CONFIG_CAM_PWDN_GPIO  -1
#define CONFIG_CAM_RESET_GPIO -1
#define CONFIG_CAM_XCLK_GPIO  10
#define CONFIG_CAM_SIOD_GPIO  40
#define CONFIG_CAM_SIOC_GPIO  39
#define CONFIG_CAM_Y9_GPIO    48
#define CONFIG_CAM_Y8_GPIO    11
#define CONFIG_CAM_Y7_GPIO    12
#define CONFIG_CAM_Y6_GPIO    14
#define CONFIG_CAM_Y5_GPIO    16
#define CONFIG_CAM_Y4_GPIO    18
#define CONFIG_CAM_Y3_GPIO    17
#define CONFIG_CAM_Y2_GPIO    15
#define CONFIG_CAM_VSYNC_GPIO 38
#define CONFIG_CAM_HREF_GPIO  47
#define CONFIG_CAM_PCLK_GPIO  13
// XIAO Sense expansion-board microSD slot is SPI, not SDMMC.
#define SD_BUS_SDMMC_1BIT     0
#define SD_PIN_CS             21
#define SD_PIN_SCK             7
#define SD_PIN_MISO            8
#define SD_PIN_MOSI            9
#endif

// --------------------------------------------------------------------------
// Camera. esp-dl consumes raw pixels, so we ask the sensor for RGB565 (no JPEG
// decode step, unlike data_capture). QVGA 320x240 is plenty: the model's input
// is 224x224 and esp-dl letterboxes/resizes for us. Bigger frames only cost
// PSRAM bandwidth and preprocess time.
// --------------------------------------------------------------------------
#define INFER_FRAME_SIZE        FRAMESIZE_QVGA   // 320x240
#define INFER_FRAME_W           320
#define INFER_FRAME_H           240
#define INFER_CAM_XCLK_HZ       20000000
#define INFER_CAM_FB_COUNT      3    // 1 in inference + 1 being filled + 1 spare

// Byte order of RGB565 pixels as they land in the camera frame buffer. If the
// hand is never detected even in good light and the colours look wrong, flip
// this (see docs/inference_on_esp32s3.md "Troubleshooting").
//   1 -> tell esp-dl the buffer is RGB565BE; 0 -> RGB565LE.
#define INFER_RGB565_IS_BIG_ENDIAN 0

// --------------------------------------------------------------------------
// Tasks. Camera capture runs free on core 1 so the *camera* FPS is measured at
// sensor rate, independent of how slow inference is. Inference runs on core 0.
// The two are decoupled by a single "latest frame" slot: inference always
// processes the newest frame and older ones are dropped (lowest latency, no
// queue build-up).
// --------------------------------------------------------------------------
#define INFER_CAPTURE_PRIORITY   10
#define INFER_CAPTURE_CORE       1
#define INFER_CAPTURE_STACK      4096

#define INFER_INFERENCE_PRIORITY 5
#define INFER_INFERENCE_CORE     0
#define INFER_INFERENCE_STACK    12288   // esp-dl postprocess uses std::list/vector; be generous

#define INFER_STATS_PERIOD_MS    1000    // how often camera/inference FPS is logged

// --------------------------------------------------------------------------
// Hand detection + wave decision (see main/wave_detector.hpp).
// --------------------------------------------------------------------------
#define INFER_HAND_SCORE_THR     0.40f   // ignore detections below this confidence
                                          // (esp-dl's own default is 0.25; raised to cut false hands)

#define WAVE_WINDOW_MS           2000    // history used to count direction reversals
#define WAVE_MIN_REVERSALS       3       // right-left-right-left = 3 turning points
#define WAVE_AMP_FRAC            0.30f   // min swing as a fraction of the hand box width
#define WAVE_MIN_AMP_PX          6.0f    // absolute floor (detector box jitter)
#define WAVE_LOST_MS             600     // hand missing this long -> forget history
#define WAVE_HOLD_MS             800     // keep "WAVING" this long after the last reversal
