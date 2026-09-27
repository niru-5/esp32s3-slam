#include "camera_overrides.h"

#include <stdlib.h>
#include <string.h>
#include "esp_camera.h"
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "CAMOVR";

#define NVS_NAMESPACE   "cam_ovr"
#define NVS_KEY_REGS    "regs"
#define BLOB_VERSION    1

// On-disk shape: [uint8 version][uint16 count][count * {uint16 addr, uint8 val}],
// little-endian (native ESP32-S3 byte order -- this blob never leaves the device).
static size_t packed_size(size_t count) {
    return 1 + 2 + count * sizeof(camera_override_write_t);
}

// Static, not stack: called from whichever task is doing the save (cam_calib's
// task) or the apply (camera_init(), which may run on the main app_main/state
// machine task) -- keeping a ~387-byte buffer off both of their stacks.
static camera_override_write_t s_regs[CAMERA_OVERRIDES_MAX_REGS];

// Loads the saved set into s_regs. Returns the count (0 on any error or if
// nothing has been saved yet -- both are "no overrides" as far as callers
// are concerned).
static size_t load_locked(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
        return 0;

    size_t blob_len = 0;
    esp_err_t err = nvs_get_blob(h, NVS_KEY_REGS, NULL, &blob_len);
    if (err != ESP_OK || blob_len < 3) {
        nvs_close(h);
        return 0;
    }
    uint8_t *buf = malloc(blob_len);
    if (!buf) {
        ESP_LOGE(TAG, "oom loading %u-byte override blob", (unsigned)blob_len);
        nvs_close(h);
        return 0;
    }
    err = nvs_get_blob(h, NVS_KEY_REGS, buf, &blob_len);
    nvs_close(h);
    if (err != ESP_OK) {
        free(buf);
        return 0;
    }

    uint8_t version = buf[0];
    uint16_t count;
    memcpy(&count, buf + 1, 2);
    if (version != BLOB_VERSION || count > CAMERA_OVERRIDES_MAX_REGS ||
        blob_len != packed_size(count)) {
        ESP_LOGW(TAG, "stored override blob is malformed (version=%u count=%u len=%u) -- ignoring",
                 version, count, (unsigned)blob_len);
        free(buf);
        return 0;
    }
    memcpy(s_regs, buf + 3, count * sizeof(camera_override_write_t));
    free(buf);
    return count;
}

static esp_err_t store_locked(size_t count) {
    size_t len = packed_size(count);
    uint8_t *buf = malloc(len);
    if (!buf) return ESP_ERR_NO_MEM;
    buf[0] = BLOB_VERSION;
    uint16_t count16 = (uint16_t)count;
    memcpy(buf + 1, &count16, 2);
    memcpy(buf + 3, s_regs, count * sizeof(camera_override_write_t));

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, NVS_KEY_REGS, buf, len);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    free(buf);
    return err;
}

esp_err_t camera_overrides_apply(void) {
    size_t count = load_locked();
    if (count == 0) return ESP_OK;

    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) {
        ESP_LOGW(TAG, "no sensor handle -- can't apply %u saved override(s)", (unsigned)count);
        return ESP_FAIL;
    }
    for (size_t i = 0; i < count; i++)
        sensor->set_reg(sensor, s_regs[i].addr, 0xFF, s_regs[i].val);
    ESP_LOGI(TAG, "applied %u saved register override(s)", (unsigned)count);
    return ESP_OK;
}

esp_err_t camera_overrides_save(const camera_override_write_t *writes, size_t n) {
    size_t count = load_locked();

    for (size_t i = 0; i < n; i++) {
        size_t j;
        for (j = 0; j < count; j++) {
            if (s_regs[j].addr == writes[i].addr) { s_regs[j].val = writes[i].val; break; }
        }
        if (j == count) {
            if (count >= CAMERA_OVERRIDES_MAX_REGS) {
                ESP_LOGE(TAG, "save rejected: would exceed %d saved registers (have %u, adding %u)",
                         CAMERA_OVERRIDES_MAX_REGS, (unsigned)count, (unsigned)(n - i));
                return ESP_ERR_NO_MEM;
            }
            s_regs[count++] = writes[i];
        }
    }

    esp_err_t err = store_locked(count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS store failed (%s)", esp_err_to_name(err));
        return err;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor)
        for (size_t i = 0; i < count; i++)
            sensor->set_reg(sensor, s_regs[i].addr, 0xFF, s_regs[i].val);

    ESP_LOGI(TAG, "saved %u register override(s) to NVS (%u new/updated this call)",
             (unsigned)count, (unsigned)n);
    return ESP_OK;
}

esp_err_t camera_overrides_clear(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_erase_key(h, NVS_KEY_REGS);
    if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_commit(h);
        err = ESP_OK;
    }
    nvs_close(h);
    ESP_LOGI(TAG, "cleared saved register overrides (%s)", esp_err_to_name(err));
    return err;
}

size_t camera_overrides_get(camera_override_write_t *out, size_t max) {
    size_t count = load_locked();
    size_t n = count < max ? count : max;
    if (out && n) memcpy(out, s_regs, n * sizeof(camera_override_write_t));
    return count;
}
