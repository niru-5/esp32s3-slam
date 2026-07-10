#include "imu.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

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
#if !CONFIG_ENABLE_IMU
    ESP_LOGW(TAG, "IMU pipeline disabled (CONFIG_ENABLE_IMU=0) — skipping capture");
    return ESP_OK;
#endif

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
