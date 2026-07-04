#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmi270.h"

// --------------------------------------------------------------------------
// BMI270 wiring — adjust to match your actual connections
// --------------------------------------------------------------------------
#define IMU_I2C_PORT   I2C_NUM_0
#define IMU_SDA_GPIO   41
#define IMU_SCL_GPIO   42
#define IMU_I2C_FREQ   400000
#define BMI270_ADDR    BMI2_I2C_PRIM_ADDR  // 0x68 when SA0=low, 0x69 if SA0=high
#define IMU_CSB_GPIO   47  // driven HIGH -> BMI270 in I2C mode
#define IMU_ADDR_GPIO  48  // driven LOW  -> I2C address 0x68 (SA0)

static struct bmi2_dev s_bmi2;
static float           s_raw_to_gs;   // int16 -> g
static float           s_raw_to_dps;  // int16 -> deg/s

static const char *TAG = "IMU_TEST";

// --------------------------------------------------------------------------
// BMI2 I2C read/write callbacks
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
    // Busy-wait for the full period. The BMI270 init path needs an accurate
    // 2 ms settle after soft-reset; routing sub-10 ms delays through
    // vTaskDelay(pdMS_TO_TICKS(...)) truncates to 0 ticks at the default
    // 100 Hz FreeRTOS tick and skips the wait (-> config upload COM_FAIL).
    // Init delays are short (<=10 ms) and infrequent, so busy-waiting is fine.
    esp_rom_delay_us(period);
}

// --------------------------------------------------------------------------
// IMU init
// --------------------------------------------------------------------------

static void i2c_bus_scan(void) {
    ESP_LOGI(TAG, "Scanning I2C bus on SDA=GPIO%d SCL=GPIO%d ...", IMU_SDA_GPIO, IMU_SCL_GPIO);
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
    if (found == 0) {
        ESP_LOGW(TAG, "  no devices found on the bus (check SDA/SCL wiring, power, pull-ups)");
    }
}

static esp_err_t imu_init(void) {
    // Drive the BMI270 mode/address pins before any communication so no
    // external pull resistors (breadboard) are needed:
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
    esp_err_t ret = i2c_param_config(IMU_I2C_PORT, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_driver_install(IMU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

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

    config[0].cfg.acc.odr   = BMI2_ACC_ODR_100HZ;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;

    config[1].cfg.gyr.odr   = BMI2_GYR_ODR_100HZ;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_500;

    err = bmi270_set_sensor_config(config, 2, &s_bmi2);
    if (err != BMI2_OK) {
        ESP_LOGE(TAG, "set_sensor_config failed: %d", err);
        return ESP_FAIL;
    }

    // Conversion scalars — must be computed from the ranges we just set
    s_raw_to_gs  = (float)(2 << config[0].cfg.acc.range) / 32768.0f;
    s_raw_to_dps = (125.0f * (float)(1 << (BMI2_GYR_RANGE_125 - config[1].cfg.gyr.range))) / 32768.0f;

    ESP_LOGI(TAG, "BMI270 ready - accel +/-%dG  gyro +/-%ddps  ODR 100Hz",
             (int)(2 << config[0].cfg.acc.range),
             (int)(125 * (1 << (BMI2_GYR_RANGE_125 - config[1].cfg.gyr.range))));
    return ESP_OK;
}

// --------------------------------------------------------------------------
// Entry point — poll at 10 Hz and print raw + scaled values
// --------------------------------------------------------------------------

void app_main(void) {
    ESP_LOGI(TAG, "Starting BMI270 raw read test (SDA=GPIO%d SCL=GPIO%d CSB=GPIO%d SA0=GPIO%d addr=0x%02X)",
             IMU_SDA_GPIO, IMU_SCL_GPIO, IMU_CSB_GPIO, IMU_ADDR_GPIO, (unsigned)BMI270_ADDR);

    if (imu_init() != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed - check wiring/pins");
        return;
    }

    while (1) {
        struct bmi2_sens_data raw;
        memset(&raw, 0, sizeof(raw));
        int8_t err = bmi2_get_sensor_data(&raw, &s_bmi2);
        if (err != BMI2_OK) {
            ESP_LOGW(TAG, "get_sensor_data failed: %d", err);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        printf("raw acc[%6d %6d %6d]  gyr[%6d %6d %6d]  |  g[%+.3f %+.3f %+.3f]  dps[%+7.2f %+7.2f %+7.2f]\n",
               raw.acc.x, raw.acc.y, raw.acc.z,
               raw.gyr.x, raw.gyr.y, raw.gyr.z,
               raw.acc.x * s_raw_to_gs, raw.acc.y * s_raw_to_gs, raw.acc.z * s_raw_to_gs,
               raw.gyr.x * s_raw_to_dps, raw.gyr.y * s_raw_to_dps, raw.gyr.z * s_raw_to_dps);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
