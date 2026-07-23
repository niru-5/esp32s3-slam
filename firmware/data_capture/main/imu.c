#include "imu.h"

#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs.h"

#include "config.h"
#include "bmi270.h"

// --------------------------------------------------------------------------
// BMI270 over I2C (IMU_I2C_PORT). All mode/address pins are driven by the
// ESP32, so the module needs only direct wiring — no breadboard pull resistors:
//   SDA→GPIO41  SCL→GPIO42  CSB→GPIO47 (high→I2C)  SA0→GPIO48 (low→0x68)
// --------------------------------------------------------------------------
#define IMU_I2C_PORT   I2C_NUM_0
#define IMU_SDA_GPIO   41
#define IMU_SCL_GPIO   42
#define IMU_I2C_FREQ   400000
#define BMI270_ADDR    BMI2_I2C_PRIM_ADDR  // 0x68 when SA0=low
#define IMU_CSB_GPIO   47                   // driven HIGH → I2C mode on BMI270
#define IMU_ADDR_GPIO  48                   // driven LOW  → I2C address 0x68 (SA0)

static const char *TAG = "IMU";

static struct bmi2_dev s_bmi2;
static float           s_raw_to_gs;       // int16 → g
static float           s_raw_to_dps;      // int16 → deg/s

static QueueHandle_t      s_imu_queue          = NULL;
static esp_timer_handle_t s_capture_timer      = NULL;
static TaskHandle_t       s_capture_task_handle = NULL;
static uint32_t           s_overflow_count      = 0;

// --------------------------------------------------------------------------
// BMI2 I2C read/write/delay callbacks
// --------------------------------------------------------------------------

static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *data,
                                             uint32_t len, void *intf_ptr) {
    if (len == 0) return BMI2_E_COM_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMI270_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMI270_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(IMU_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data,
                                              uint32_t len, void *intf_ptr) {
    if (len == 0) return BMI2_E_COM_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMI270_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(IMU_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static void bmi2_delay_us_cb(uint32_t period, void *intf_ptr) {
    // Busy-wait the full period. BMI270 init needs an accurate 2 ms settle
    // after soft-reset; routing sub-10 ms delays through
    // vTaskDelay(pdMS_TO_TICKS(...)) truncates to 0 ticks at the default
    // 100 Hz FreeRTOS tick and skips the wait (-> config upload COM_FAIL).
    esp_rom_delay_us(period);
}

static void i2c_bus_scan(void) {
    ESP_LOGI(TAG, "Scanning I2C%d on SDA=GPIO%d SCL=GPIO%d ...",
             IMU_I2C_PORT, IMU_SDA_GPIO, IMU_SCL_GPIO);
    int found = 0;
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(IMU_I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  -> ACK from address 0x%02X", (unsigned)addr);
            found++;
        }
    }
    if (found == 0)
        ESP_LOGW(TAG, "  no devices found (check SDA/SCL wiring, power, pull-ups)");
}

// --------------------------------------------------------------------------
// BMI270 init
// --------------------------------------------------------------------------

// bmi270_init + sensor_enable + ODR/range config, split out of
// bmi270_bringup() so it can be re-run standalone after bmi2_nvm_prog() --
// which ends with bmi2_soft_reset(), reverting the chip to power-on
// defaults (sensors disabled) -- without repeating the GPIO/I2C-driver setup
// that must only happen once. Recomputes s_raw_to_gs/s_raw_to_dps too, even
// though the values are the same, since it's config[]-derived either way.
static esp_err_t configure_bmi270_sensors(void) {
    int8_t err = bmi270_init(&s_bmi2);
    if (err != BMI2_OK) {
        ESP_LOGE(TAG, "bmi270_init failed: %d", err);
        return ESP_FAIL;
    }

    uint8_t sens_list[] = {BMI2_ACCEL, BMI2_GYRO};
    err = bmi270_sensor_enable(sens_list, 2, &s_bmi2);
    if (err != BMI2_OK) {
        ESP_LOGE(TAG, "sensor_enable failed: %d", err);
        return ESP_FAIL;
    }

    // Read defaults then override non-default values only
    struct bmi2_sens_config config[2];
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;
    err = bmi270_get_sensor_config(config, 2, &s_bmi2);
    if (err != BMI2_OK) {
        ESP_LOGE(TAG, "get_sensor_config failed: %d", err);
        return ESP_FAIL;
    }

    // ODR raised from 100Hz to 1600Hz (the BMI270's native rate closest to
    // and above our CONFIG_IMU_CAPTURE_PERIOD_MS=1ms capture cadence -- ODR
    // steps are powers of two, "1kHz" isn't one of them, and polling faster
    // than the sensor updates would just re-read stale samples). See
    // docs/architecture.md open questions.
    config[0].cfg.acc.odr   = BMI2_ACC_ODR_1600HZ;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;

    config[1].cfg.gyr.odr   = BMI2_GYR_ODR_1600HZ;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_500;

    err = bmi270_set_sensor_config(config, 2, &s_bmi2);
    if (err != BMI2_OK) {
        ESP_LOGE(TAG, "set_sensor_config failed: %d", err);
        return ESP_FAIL;
    }

    // Conversion scalars — must be computed from the ranges we just set
    // accel: raw int16 * scalar → g
    s_raw_to_gs  = (float)(2 << config[0].cfg.acc.range) / 32768.0f;
    // gyro: raw int16 * scalar → deg/s  (BMI2_GYR_RANGE_125 = 4)
    s_raw_to_dps = (125.0f * (float)(1 << (BMI2_GYR_RANGE_125 - config[1].cfg.gyr.range))) / 32768.0f;

    ESP_LOGI(TAG, "BMI270 ready — accel ±%dG  gyro ±%ddps  ODR 1600Hz",
             (int)(2 << config[0].cfg.acc.range),
             (int)(125 * (1 << (BMI2_GYR_RANGE_125 - config[1].cfg.gyr.range))));
    return ESP_OK;
}

static esp_err_t bmi270_bringup(void) {
    // Drive the BMI270 mode/address pins before any communication so the
    // module needs no external pull resistors:
    //   CSB high → I2C mode,  SA0 low → address 0x68
    // A falling edge on CSB latches the BMI270 into SPI mode until the next
    // power-on, so CSB must never glitch low. Preset the output latches BEFORE
    // switching the pins to output (otherwise output mode drives 0 first), and
    // enable a pull-up on CSB to bias it high through the boot window.
    gpio_set_level(IMU_CSB_GPIO, 1);   // preset CSB latch high
    gpio_set_level(IMU_ADDR_GPIO, 0);  // preset SA0 latch low
    gpio_config_t imu_pins = {
        .pin_bit_mask = (1ULL << IMU_CSB_GPIO) | (1ULL << IMU_ADDR_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&imu_pins);
    gpio_set_level(IMU_CSB_GPIO, 1);   // I2C mode
    gpio_set_level(IMU_ADDR_GPIO, 0);  // address 0x68
    vTaskDelay(pdMS_TO_TICKS(5)); // let BMI270 settle into I2C mode

    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = IMU_SDA_GPIO,
        .scl_io_num       = IMU_SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = IMU_I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(IMU_I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(IMU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    i2c_bus_scan();

    s_bmi2.read           = bmi2_i2c_read;
    s_bmi2.write          = bmi2_i2c_write;
    s_bmi2.delay_us       = bmi2_delay_us_cb;
    s_bmi2.intf_ptr       = NULL;
    s_bmi2.intf           = BMI2_I2C_INTF;
    s_bmi2.read_write_len = 32;

    return configure_bmi270_sensors();
}

// --------------------------------------------------------------------------
// imu_capture_task — woken by esp_timer every CONFIG_IMU_CAPTURE_PERIOD_MS
// via task-notify (see imu.h for why not a FreeRTOS software timer).
// --------------------------------------------------------------------------

static void capture_timer_cb(void *arg) {
    xTaskNotifyGive(s_capture_task_handle);
}

static void imu_capture_task(void *arg) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        struct bmi2_sens_data raw;
        memset(&raw, 0, sizeof(raw));
        if (bmi2_get_sensor_data(&raw, &s_bmi2) != BMI2_OK) continue;

        imu_sample_t s = {
            .sens_time = raw.sens_time,
            .ax = (float)raw.acc.x * s_raw_to_gs,
            .ay = (float)raw.acc.y * s_raw_to_gs,
            .az = (float)raw.acc.z * s_raw_to_gs,
            .gx = (float)raw.gyr.x * s_raw_to_dps,
            .gy = (float)raw.gyr.y * s_raw_to_dps,
            .gz = (float)raw.gyr.z * s_raw_to_dps,
        };

        // Producer never blocks on a full queue (a slow consumer must not
        // stall a priority-22 task) -- drop the oldest sample to make room.
        if (xQueueSend(s_imu_queue, &s, 0) != pdTRUE) {
            imu_sample_t discard;
            xQueueReceive(s_imu_queue, &discard, 0);
            xQueueSend(s_imu_queue, &s, 0);
            s_overflow_count++;
            if (s_overflow_count % 100 == 1)  // don't flood the log at 1kHz
                ESP_LOGW(TAG, "imu_queue full, dropped oldest (overflow #%lu)",
                         (unsigned long)s_overflow_count);
        }
    }
}

// --------------------------------------------------------------------------
// Public API
// --------------------------------------------------------------------------

esp_err_t imu_init(void) {
    esp_err_t err = bmi270_bringup();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed — check wiring (SDA=GPIO%d SCL=GPIO%d addr=0x%02X)",
                 IMU_SDA_GPIO, IMU_SCL_GPIO, BMI270_ADDR);
    }
    return err;
}

esp_err_t imu_pipeline_start(void) {
    s_overflow_count = 0;
    s_imu_queue = xQueueCreate(CONFIG_IMU_QUEUE_LEN, sizeof(imu_sample_t));
    if (!s_imu_queue) return ESP_ERR_NO_MEM;

    if (xTaskCreatePinnedToCore(imu_capture_task, "imu_cap", 4096, NULL,
                                CONFIG_IMU_CAPTURE_PRIORITY, &s_capture_task_handle,
                                CONFIG_IMU_CAPTURE_CORE) != pdPASS) {
        vQueueDelete(s_imu_queue);
        s_imu_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = capture_timer_cb,
        .name     = "imu_cap_timer",
    };
    if (esp_timer_create(&timer_args, &s_capture_timer) != ESP_OK ||
        esp_timer_start_periodic(s_capture_timer, CONFIG_IMU_CAPTURE_PERIOD_MS * 1000ULL) != ESP_OK) {
        vTaskDelete(s_capture_task_handle);
        s_capture_task_handle = NULL;
        vQueueDelete(s_imu_queue);
        s_imu_queue = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "capture pipeline started (%d ms period, queue depth %d)",
             CONFIG_IMU_CAPTURE_PERIOD_MS, CONFIG_IMU_QUEUE_LEN);
    return ESP_OK;
}

void imu_pipeline_stop(void) {
    if (s_capture_timer) {
        esp_timer_stop(s_capture_timer);
        esp_timer_delete(s_capture_timer);
        s_capture_timer = NULL;
    }
    if (s_capture_task_handle) {
        vTaskDelete(s_capture_task_handle);
        s_capture_task_handle = NULL;
    }
    if (s_imu_queue) {
        vQueueDelete(s_imu_queue);
        s_imu_queue = NULL;
    }
}

uint32_t imu_queue_drain(imu_sample_t *out, uint32_t max_count,
                         int64_t *ref_esp_us, uint32_t *ref_sensor_ticks) {
    uint32_t n = 0;
    if (s_imu_queue) {
        while (n < max_count && xQueueReceive(s_imu_queue, &out[n], 0) == pdTRUE) n++;
    }

    // Reference pair for the batch just drained -- a fresh BMI270 read taken
    // right now. The shared-clock math only needs *a* known (esp_us, tick)
    // instant, not one per sample.
    struct bmi2_sens_data ref;
    memset(&ref, 0, sizeof(ref));
    bmi2_get_sensor_data(&ref, &s_bmi2);
    if (ref_esp_us)       *ref_esp_us       = esp_timer_get_time();
    if (ref_sensor_ticks) *ref_sensor_ticks = ref.sens_time;
    return n;
}

uint32_t imu_queue_depth(void) {
    return s_imu_queue ? (uint32_t)uxQueueMessagesWaiting(s_imu_queue) : 0;
}

uint32_t imu_queue_overflow_count(void) {
    return s_overflow_count;
}

size_t imu_serialize_wire(const imu_sample_t *samples, uint32_t count,
                          int64_t ref_esp_us, uint32_t ref_ticks, uint8_t *out) {
    uint8_t *p = out;
    memcpy(p, &count,      4); p += 4;
    memcpy(p, &ref_esp_us, 8); p += 8;
    memcpy(p, &ref_ticks,  4); p += 4;
    for (uint32_t i = 0; i < count; i++) {
        memcpy(p, &samples[i].sens_time, 4); p += 4;
        memcpy(p, &samples[i].ax,        4); p += 4;
        memcpy(p, &samples[i].ay,        4); p += 4;
        memcpy(p, &samples[i].az,        4); p += 4;
        memcpy(p, &samples[i].gx,        4); p += 4;
        memcpy(p, &samples[i].gy,        4); p += 4;
        memcpy(p, &samples[i].gz,        4); p += 4;
    }
    return (size_t)(p - out);
}

// --------------------------------------------------------------------------
// Bias/offset calibration (see imu.h). Polls s_bmi2 directly -- only safe
// while imu_capture_task isn't also reading it, which state_machine.c
// guarantees by tearing down any active pipeline before entering
// APP_STATE_IMU_CALIBRATION.
// --------------------------------------------------------------------------

#define IMU_CALIB_NVS_NAMESPACE "imu_calib"
#define IMU_CALIB_NVS_KEY_FOC_COUNT "foc_count"

esp_err_t imu_capture_stationary_stats(uint32_t duration_ms, imu_stationary_stats_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(*out));

    double sum[6] = {0}, sumsq[6] = {0};
    uint32_t n = 0;
    int64_t deadline_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;

    // 10ms poll period: comfortably below the BMI270's 1600Hz ODR (no risk
    // of re-reading a stale sample) and, unlike a busy-wait, lets the
    // scheduler/idle task run each iteration -- this loop blocks the calling
    // task for seconds at a time, which is fine for a synchronous console
    // command but would trip the watchdog if done with esp_rom_delay_us.
    while (esp_timer_get_time() < deadline_us) {
        struct bmi2_sens_data raw;
        memset(&raw, 0, sizeof(raw));
        if (bmi2_get_sensor_data(&raw, &s_bmi2) == BMI2_OK) {
            float vals[6] = {
                (float)raw.acc.x * s_raw_to_gs,  (float)raw.acc.y * s_raw_to_gs,  (float)raw.acc.z * s_raw_to_gs,
                (float)raw.gyr.x * s_raw_to_dps, (float)raw.gyr.y * s_raw_to_dps, (float)raw.gyr.z * s_raw_to_dps,
            };
            for (int i = 0; i < 6; i++) {
                sum[i]   += vals[i];
                sumsq[i] += (double)vals[i] * (double)vals[i];
            }
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (n == 0) {
        ESP_LOGE(TAG, "stationary-stats capture read zero samples");
        return ESP_FAIL;
    }

    double mean[6], var[6];
    for (int i = 0; i < 6; i++) {
        mean[i] = sum[i] / n;
        var[i]  = sumsq[i] / n - mean[i] * mean[i];
        if (var[i] < 0) var[i] = 0;  // guard float rounding, not a real negative variance
    }

    out->sample_count = n;
    out->ax_mean = (float)mean[0]; out->ay_mean = (float)mean[1]; out->az_mean = (float)mean[2];
    out->gx_mean = (float)mean[3]; out->gy_mean = (float)mean[4]; out->gz_mean = (float)mean[5];
    out->ax_std  = sqrtf((float)var[0]); out->ay_std = sqrtf((float)var[1]); out->az_std = sqrtf((float)var[2]);
    out->gx_std  = sqrtf((float)var[3]); out->gy_std = sqrtf((float)var[4]); out->gz_std = sqrtf((float)var[5]);
    return ESP_OK;
}

esp_err_t imu_preview_raw_samples(imu_raw_preview_sample_t *out, uint32_t count, uint32_t period_ms) {
    if (!out || count == 0) return ESP_ERR_INVALID_ARG;

    for (uint32_t i = 0; i < count; i++) {
        struct bmi2_sens_data raw;
        memset(&raw, 0, sizeof(raw));
        if (bmi2_get_sensor_data(&raw, &s_bmi2) != BMI2_OK) return ESP_FAIL;

        out[i].ax = (float)raw.acc.x * s_raw_to_gs;
        out[i].ay = (float)raw.acc.y * s_raw_to_gs;
        out[i].az = (float)raw.acc.z * s_raw_to_gs;
        out[i].gx = (float)raw.gyr.x * s_raw_to_dps;
        out[i].gy = (float)raw.gyr.y * s_raw_to_dps;
        out[i].gz = (float)raw.gyr.z * s_raw_to_dps;

        if (i + 1 < count) vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
    return ESP_OK;
}

const char *imu_accel_foc_axis_label(imu_accel_foc_axis_t axis) {
    switch (axis) {
        case IMU_ACCEL_FOC_AXIS_POS_X: return "+X";
        case IMU_ACCEL_FOC_AXIS_NEG_X: return "-X";
        case IMU_ACCEL_FOC_AXIS_POS_Y: return "+Y";
        case IMU_ACCEL_FOC_AXIS_NEG_Y: return "-Y";
        case IMU_ACCEL_FOC_AXIS_POS_Z: return "+Z";
        case IMU_ACCEL_FOC_AXIS_NEG_Z: return "-Z";
        default:                       return "?";
    }
}

// Sign convention verified against bmi2.c's own validation, not just the doc
// comment: verify_foc_position() (bmi2.c ~line 11519) inverts the just-read
// sample (multiplies by -1) only when sign==1 for the matched axis, then
// checks the result against +1g -- i.e. sign==1 asserts "this axis currently
// reads negative" and sign==0 asserts "reads positive". (An earlier version
// of this code used the opposite convention, copied from
// SparkFun_BMI270_Arduino_Library.cpp's performAccelOffsetCalibration
// wrapper -- that wrapper's mapping does not match the underlying bmi2.c
// validation and caused BMI2_E_INVALID_FOC_POSITION on real hardware even
// with a correct axis/orientation.)
static struct bmi2_accel_foc_g_value accel_foc_g_value_from_axis(imu_accel_foc_axis_t axis) {
    switch (axis) {
        case IMU_ACCEL_FOC_AXIS_POS_X: return (struct bmi2_accel_foc_g_value){ .x = 1, .y = 0, .z = 0, .sign = 0 };
        case IMU_ACCEL_FOC_AXIS_NEG_X: return (struct bmi2_accel_foc_g_value){ .x = 1, .y = 0, .z = 0, .sign = 1 };
        case IMU_ACCEL_FOC_AXIS_POS_Y: return (struct bmi2_accel_foc_g_value){ .x = 0, .y = 1, .z = 0, .sign = 0 };
        case IMU_ACCEL_FOC_AXIS_NEG_Y: return (struct bmi2_accel_foc_g_value){ .x = 0, .y = 1, .z = 0, .sign = 1 };
        case IMU_ACCEL_FOC_AXIS_POS_Z: return (struct bmi2_accel_foc_g_value){ .x = 0, .y = 0, .z = 1, .sign = 0 };
        case IMU_ACCEL_FOC_AXIS_NEG_Z:
        default:                       return (struct bmi2_accel_foc_g_value){ .x = 0, .y = 0, .z = 1, .sign = 1 };
    }
}

// Bumps a persistent run counter in NVS and returns the new value (0 if NVS
// itself is unavailable -- calibration still proceeds, it just loses the
// "how many times has this chip been FOC'd" warning).
static uint32_t imu_foc_run_count_bump(void) {
    nvs_handle_t h;
    if (nvs_open(IMU_CALIB_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed -- can't track FOC run count");
        return 0;
    }
    uint32_t count = 0;
    nvs_get_u32(h, IMU_CALIB_NVS_KEY_FOC_COUNT, &count);  // leaves count=0 if key absent
    count++;
    nvs_set_u32(h, IMU_CALIB_NVS_KEY_FOC_COUNT, count);
    nvs_commit(h);
    nvs_close(h);
    return count;
}

esp_err_t imu_run_hw_foc_calibration(uint32_t sample_window_ms,
                                      imu_accel_foc_axis_t gravity_axis,
                                      imu_stationary_stats_t *before_out,
                                      imu_stationary_stats_t *after_out,
                                      uint32_t *foc_run_count_out) {
    if (!before_out || !after_out) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "capturing BEFORE stats (%lu ms) -- hold the rig flat and still",
             (unsigned long)sample_window_ms);
    esp_err_t err = imu_capture_stationary_stats(sample_window_ms, before_out);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "running accel FOC with gravity axis = %s", imu_accel_foc_axis_label(gravity_axis));
    struct bmi2_accel_foc_g_value g_dir = accel_foc_g_value_from_axis(gravity_axis);
    int8_t rslt = bmi2_perform_accel_foc(&g_dir, &s_bmi2);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "accel FOC failed: %d", rslt);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "accel FOC ok");

    rslt = bmi2_perform_gyro_foc(&s_bmi2);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "gyro FOC failed: %d", rslt);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "gyro FOC ok");

    rslt = bmi2_nvm_prog(&s_bmi2);
    bool nvm_ok = (rslt == BMI2_OK);
    if (!nvm_ok)
        ESP_LOGE(TAG, "bmi2_nvm_prog failed: %d -- offsets active this power cycle only, will NOT survive reboot", rslt);
    else
        ESP_LOGI(TAG, "offsets committed to BMI270 NVM -- persists across reboot");

    // bmi2_nvm_prog() ends with bmi2_soft_reset() whenever it reaches that
    // point (i.e. whenever nvm_ok is true, and possibly even on some failure
    // paths) -- a soft reset reverts the chip to power-on defaults, disabling
    // accel/gyro entirely. Without this, every read for the rest of the
    // session -- including the "after" snapshot below and the normal
    // streaming pipeline -- would silently come back as zeros until reboot.
    if (configure_bmi270_sensors() != ESP_OK) {
        ESP_LOGE(TAG, "failed to reconfigure BMI270 after NVM commit -- IMU is now dead until reboot");
        return ESP_FAIL;
    }

    uint32_t count = imu_foc_run_count_bump();
    if (foc_run_count_out) *foc_run_count_out = count;
    if (count > 1)
        ESP_LOGW(TAG, "this is FOC+NVM run #%lu on this chip -- NVM write endurance is limited, "
                       "recalibrate only when actually needed", (unsigned long)count);

    ESP_LOGI(TAG, "capturing AFTER stats (%lu ms)", (unsigned long)sample_window_ms);
    err = imu_capture_stationary_stats(sample_window_ms, after_out);
    if (err != ESP_OK) return err;

    return nvm_ok ? ESP_OK : ESP_ERR_INVALID_STATE;
}
