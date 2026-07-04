# Learnings — BMI270 bring-up on ESP32-S3

Debugging notes from getting the BMI270 IMU working reliably in both
`firmware/imu_testing` and `firmware/data_capture`. Written newest-understanding-first
so the non-obvious traps are easy to re-find later.

---

## 1. `delay_us` callback must busy-wait — `vTaskDelay` truncates at the 100 Hz tick

**Symptom:** `bmi270_init` returns `-2` (`BMI2_E_COM_FAIL`) even though the I2C bus
scan ACKs `0x68`. Same Bosch driver works fine under Arduino.

**Cause:** the BMI270 init sequence does a soft-reset and then needs an accurate
**2 ms settle** before uploading its ~8 KB config file (`bmi2_soft_reset` →
`dev->delay_us(2000, …)`). Our callback routed any `period >= 1000` µs through
`vTaskDelay(pdMS_TO_TICKS(period_ms))`. With `CONFIG_FREERTOS_HZ = 100` (10 ms per
tick — the ESP-IDF default), `pdMS_TO_TICKS(2)` rounds down to **0 ticks**, so the
wait vanished. The config upload then hit a not-ready chip → NACK → COM_FAIL.
The bus scan passed because single-byte address probes don't depend on that delay.

Arduino's `delayMicroseconds(2000)` busy-waits a real 2 ms, which is why identical
driver code worked there.

**Fix:** make the delay callback a straight busy-wait. These init delays are short
(≤10 ms) and infrequent, so blocking is fine:
```c
#include "esp_rom_sys.h"
static void bmi2_delay_us_cb(uint32_t period, void *intf_ptr) {
    esp_rom_delay_us(period);
}
```

**General rule:** never feed sub-10 ms sensor timing delays through
`vTaskDelay(pdMS_TO_TICKS(...))` at a 100 Hz tick — anything under one tick becomes
0. Busy-wait, or raise `CONFIG_FREERTOS_HZ` to 1000.

---

## 2. BMI270 latches into SPI mode on a CSB falling edge — until power-on-reset

**Symptom:** IMU worked, then after a rewire the bus scan found **no devices at all**.
An ESP32 reset / reflash did not recover it; only unplugging power did.

**Cause:** the BMI270 auto-selects its interface from the CSB pin. CSB tied high =
I2C. **Any falling edge on CSB selects SPI, and it stays in SPI until the next
power-on-reset.** An ESP32 reset (EN toggle / esptool "hard reset") does *not*
power-cycle the sensor, so once latched it stays latched across resets.

Two things drove CSB low and latched it:
- CSB was wired only to a GPIO (no pull-up), so it **floats during the ~200 ms ROM
  boot window** before firmware runs.
- Our own init did `gpio_config(..., GPIO_MODE_OUTPUT)` *before* setting the level —
  output mode drives the latch's default **0** first, i.e. a brief CSB-low glitch,
  then raised it. That glitch is a falling edge.

**Fixes:**
- Preset the output latch high *before* enabling output, so the pin never drives low:
  ```c
  gpio_set_level(IMU_CSB_GPIO, 1);   // preset latch high (no pad effect yet)
  gpio_set_level(IMU_ADDR_GPIO, 0);  // preset SA0 low
  gpio_config(&imu_pins);            // enable output → drives preset values, no glitch
  ```
  Also enable an internal pull-up on CSB to bias it high through boot.
- **Recovery when already latched:** power-cycle the sensor (unplug USB). Reset alone
  won't do it.
- **Bulletproof hardware:** for I2C, CSB never needs to toggle — tie it **directly to
  3.3 V** (or a ~10 kΩ pull-up to 3.3 V). A GPIO can't cover the pre-firmware boot
  float window, so a GPIO-driven CSB is inherently a little racy.

**Wiring gotcha that caused it here:** CS and VDD were swapped — 3.3 V went to CS
(accidentally hardwiring it high, which "worked") and GPIO 47 fed the sensor's VDD.
After correcting to CS→GPIO 47, VDD→3.3 V, CSB was no longer guaranteed high at
power-on, exposing the latch behavior.

---

## 3. The camera SCCB owns I2C_NUM_1 — keep the IMU on I2C_NUM_0

**Symptom:** moving the IMU to `I2C_NUM_1` "keeps giving errors" after flashing.

**Cause:** in `data_capture`, the OV5640 camera's SCCB uses **I2C port 1** by default
(esp32-camera, on GPIO 4/5 — boot log shows `sccb: sccb_i2c_port=1`). Installing the
IMU driver on the same port collides (`i2c_driver_install` fails / conflicts).

**Rule:** BMI270 stays on **`I2C_NUM_0`** (GPIO 41/42). Port 1 is taken.

Side note: the two share the app but use *different* I2C driver generations — the IMU
uses the **legacy** `driver/i2c.h` (hence the "please migrate to `driver/i2c_master.h`"
warning), the camera component uses its own. It works, but is worth modernizing later.

---

## 4. A format-string bug hidden by a stale object file

**Symptom:** clean-looking source suddenly failed to build with `-Werror=format`
(`'%X' expects a matching 'unsigned int'`) once the file was actually recompiled.

**Cause:** a log line had **4 conversions but 3 arguments** (`CSB=GPIO%d` slot unfilled,
so `%02X` read off the stack). It had been compiled once into a cached object and never
rebuilt, so the running binary predated the bug — the serial log even showed a stale
pin value. Editing the file forced a recompile and surfaced it.

**Fixes / rules:**
- Match specifiers to arguments; cast `uint8_t` to `unsigned` for `%02X`.
- If a boot log prints values that don't match the current source (e.g. a pin number
  or the `app_init` "Compile time"), suspect a **stale / partially-rebuilt binary**.
  A forced rebuild is a cheap thing to rule out first.

---

## 5. Verified working setup (reference)

BMI270 wiring — all mode/address pins driven by the ESP32, no breadboard:
| Signal | Pin | Note |
|--------|-----|------|
| SDA | GPIO 41 | I2C_NUM_0 |
| SCL | GPIO 42 | I2C_NUM_0 |
| CSB | GPIO 47 (or 3.3 V) | must be **high** from power-on → I2C mode |
| SA0 | GPIO 48 | driven **low** → address 0x68 |
| VDD | 3.3 V | must be the real rail, not a GPIO |

- Accel/gyro config: ±4 G, ±500 dps, ODR 100 Hz.
- Scale factors are derived from the ranges actually read back after `set_sensor_config`,
  not the requested ones.
- Common clock: each sample stores the BMI270 sensor-time tick; the host reconstructs
  `sample_esp_us = ref_esp_us + (sens_time − ref_ticks) × 39.0625`, on the same clock as
  `/capture`'s `X-Timestamp-Us` header. This is what lets camera frames and IMU samples
  be aligned offline.

### `data_capture` HTTP endpoints
- `GET /` — MJPEG-ish viewer + live `/imu.json` readout.
- `GET /capture` — one JPEG, `X-Timestamp-Us` header (µs since boot at grab time).
- `GET /imu` — **binary, destructive** (drains the ring). 16-byte header
  (`count`, `ref_esp_us`, `ref_sensor_ticks`) + 28 bytes/sample
  (`sens_time` + `ax,ay,az,gx,gy,gz` float32), little-endian.
- `GET /imu.json` — **readable, non-destructive** peek at the latest sample; safe to poll.

---

## TL;DR checklist when the BMI270 won't init
1. Bus scan ACKs `0x68`? If **no** → CSB not high / latched in SPI / wiring / power →
   **power-cycle** (not just reset), and make sure CSB is high from power-on.
2. Scan ACKs but `bmi270_init` returns `-2`? → the `delay_us` callback is losing the
   soft-reset settle; busy-wait it.
3. In `data_capture`, IMU on `I2C_NUM_0` (port 1 belongs to the camera).
4. Boot log values don't match source? → force a clean rebuild (stale object).
