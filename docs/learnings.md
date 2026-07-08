# Learnings — ESP32-S3 SLAM firmware bring-up

Debugging notes from bringing up `data_capture` and `imu_testing` on the
ESP32-S3 board. Written newest-understanding-first so the non-obvious traps
are easy to re-find later. Sections 1-5 are BMI270-specific; later sections
cover the remote-stream network path and SD card logging.

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

---

## 6. WiFi modem-sleep power save adds 100s of ms to every request — but isn't the whole story

**Symptom:** `CONFIG_DEBUG_TIME` showed camera/IMU "sending" (the network POST) taking
300-600 ms per request, despite capture itself taking under 2 ms and excellent WiFi
signal.

**Cause:** ESP-IDF's default WiFi power save (modem sleep) duty-cycles the radio around
the AP's DTIM interval (here, 102.4 ms) to save power. Every request that has to round-trip
to the AP pays a variable tax from this duty cycle, unrelated to payload size.

**Fix:** `esp_wifi_set_ps(WIFI_PS_NONE)` right after `esp_wifi_start()`. This app streams
every cycle and isn't power-constrained, so there's no reason to duty-cycle the radio.

**Caveat — this alone did not fix the latency** (see §7 below): after disabling power
save, times got *worse*, not better, because the actual dominant cost was TCP connection
churn, not power save. Power save is still worth disabling for a latency-sensitive
streaming app, but don't assume it's the whole story if numbers don't improve — check
signal strength (RSSI) and connection-reuse patterns too before concluding it "didn't
work."

---

## 7. Per-request TCP churn exhausts LWIP's connection table — the real cause of multi-second stalls

**Symptom:** even after `WIFI_PS_NONE`, excellent RSSI (-42 dBm — about as good as WiFi
gets), and ruling out host-side disk I/O (`host_server --no-record`), POST latency
stayed erratic: 400-1900 ms, uncorrelated with payload size. Numbers got *worse* over a
longer test run, not better.

**Cause:** `net_client.c`'s `post_bytes()` did a fresh `esp_http_client_init()` →
`perform()` → `cleanup()` for **every single POST** (frame + imu + occasionally stats —
2-3 brand-new TCP connections every ~333 ms cycle). `esp_http_client_cleanup()` actively
closes the socket from the ESP32 side, so the ESP32 (not the server) becomes the TCP
active-closer and owns the resulting **TIME_WAIT** state. The project's sdkconfig has
`CONFIG_LWIP_MAX_ACTIVE_TCP=16` and `CONFIG_LWIP_TCP_MSL=60000` (60 s) — at this request
rate, accumulated TIME_WAIT entries exhaust the 16-slot connection table within seconds,
and once exhausted, new `esp_http_client_init()` calls stall waiting for a slot to free.
The specific ~1.5-1.9 s outliers matched `CONFIG_LWIP_TCP_RTO_TIME=1500` (the TCP
retransmit timeout) almost exactly — a strong tell that this is a *TCP-table*-scale
problem (seconds), not a *WiFi-power-save*-scale one (100s of ms).

**Fix:** keep one `esp_http_client` handle alive across the whole stream task and reuse
it for every POST — call `esp_http_client_set_url()`/`set_method()` between requests
instead of `init()`/`cleanup()`, and only tear down + reconnect on an actual `perform()`
failure. HTTP/1.1 keep-alive lets one TCP connection serve frame + imu + stats back to
back.

**General rule:** never `esp_http_client_init()`/`cleanup()` per-request in a tight
polling loop on ESP32 — reuse the handle. If request latency is large, *variable*, and
uncorrelated with payload size, suspect LWIP's TCP PCB table (`CONFIG_LWIP_MAX_ACTIVE_TCP`)
being starved by connection churn *before* suspecting WiFi/RF — RF-layer problems tend to
produce more uniformly bad numbers, not this kind of erratic bimodal pattern.

---

## 8. Identifying an undocumented SD card slot from its silkscreen + camera pinout

**Symptom:** the board's microSD slot came pre-wired from the vendor with no schematic —
needed the CLK/CMD/D0 GPIOs before writing any code.

**Method:** matched the board's silkscreen text (`ESP32-S3 N16R8 Development Board ...
N16R8 CAM`) and the camera pinout already reverse-engineered in `camera.c` (all 14
GPIOs) against public references for two differently-branded boards (GOOUUU Tech
ESP32-S3-CAM, keyestudio MB0184) — both turned out to describe the exact same whitelabel
reference design (same module variant, identical camera pins).

**Result (confirmed against real hardware):** SDMMC **1-bit** mode, `CLK=GPIO39`,
`CMD=GPIO38`, `D0=GPIO40`. Verified correct on first real flash — the SDMMC driver
successfully read the card's CID/CSD/SSR registers (`Name:`, `Type:`, `Size:` etc.
printed at boot) using exactly these pins.

**General rule:** for an unlabeled peripheral on a board bought without documentation,
matching its silkscreen text plus an already-known pinout (here, the camera) against
public references for the same reference design is a reliable identification method —
but cross-check at least two independent sources before wiring anything.

---

## 9. A 64 GB SD card ships exFAT by default — `FR_NO_FILESYSTEM` (13) on first mount

**Symptom:** first real-hardware mount attempt logged
`W vfs_fat_sdmmc: failed to mount card (13)`.

**Cause:** FRESULT `13` = `FR_NO_FILESYSTEM`. SD cards ≥64 GB (SDXC class) ship
pre-formatted **exFAT** per the SD Association spec. ESP-IDF's default FatFs build
doesn't understand exFAT, so it reports "no filesystem" even though the card itself is
fine — this is a card-capacity-class issue, not a wiring or card-quality problem.

**Fix:** `esp_vfs_fat_mount_config_t.format_if_mount_failed = true` (gated behind
`CONFIG_ENABLE_SDCARD_FORMAT` in this project) — on mount failure, ESP-IDF partitions and
reformats the card as FAT32 itself, guaranteeing compatibility with its own driver.
Confirmed working end to end: the same boot log showed `partitioning card` →
`formatting card, allocation unit size=32768` → `mounting again` → success, fully
automatically.

**Rule:** don't assume `FR_NO_FILESYSTEM` on a fresh card means bad wiring — check the
card's capacity class first. Leave `CONFIG_ENABLE_SDCARD_FORMAT` off again once a card
has been formatted once, so an unrelated future mount failure can't silently reformat
(and wipe) it.

---

## 10. FatFs defaults to 8.3 short filenames — descriptive filenames fail `mkdir`/`fopen` silently

**Symptom:** after the mount/format above succeeded, every
`mkdir("/sdcard/sess_NNNN")` call still failed. Worse, the code's own retry loop (meant
to skip past already-used session numbers) treated *every* `mkdir()` failure as "name
taken, try the next one," silently burning through all 10000 candidates and surfacing
only a generic "could not create a session directory" error — no hint of the real cause.

**Cause:** `CONFIG_FATFS_LFN_NONE` (ESP-IDF's default) restricts FatFs to classic 8.3
short names: 8 characters + a 3-character extension, no exceptions. `sess_0000` is 9
characters; `frame_<capture_ms>.jpg` and `imu_<dump_ms>.json` are far longer. Every
`mkdir`/`fopen` call with these names was doomed regardless of the retry logic.

**Fix:** switch to `CONFIG_FATFS_LFN_HEAP` (long filenames, with the temporary LFN work
buffer allocated from heap rather than growing every task's stack — the alternative,
`CONFIG_FATFS_LFN_STACK`, would require bumping the stack size of every task that
touches the filesystem).

**Compounding bug worth generalizing:** the retry loop's real flaw was checking only
`mkdir() == 0` and treating any nonzero return the same way. It should check `errno`
and only retry-with-next-candidate on `EEXIST`; anything else is a real error and should
fail loudly and immediately instead of masking itself behind thousands of silent
retries. Apply this any time "try the next candidate name" logic wraps a syscall.

---

## 11. Adding FATFS/SDMMC support needs headroom — bump the app partition ahead of time

**Symptom:** first build after adding `sdcard.c` failed at the size-check step:
`app partition is too small ... overflow 0x6cb0` (~27 KB over a 1 MB partition), even
though compilation itself succeeded.

**Cause:** the FATFS + SDMMC host driver code is not free — it pushed the binary over
the project's default `CONFIG_PARTITION_TABLE_SINGLE_APP` (1 MB app partition).

**Fix:** switched to `CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE` (2 MB app partition) —
safe headroom given this board's 16 MB flash (N16R8).

---

## 12. USB-Serial-JTAG console: log output works instantly, `stdin` silently never does

**Symptom:** `main_state_machine_task` polls `stdin` non-blockingly every 500ms and looks
completely healthy (a heartbeat log every tick proved the task itself was running fine),
but sending any byte over the port — `1`, `2`, any character — was never seen: no
`fgetc()` in the poll loop ever returned anything but `EOF`, indefinitely. This first
presented as what looked like a total system hang right after sending a command (nothing
printed for 10+ seconds), and burned a lot of time chasing task-priority/stack-overflow/
esp_timer theories before a heartbeat log proved the task was ticking the whole time —
it just never saw the byte.

**Cause:** this board exposes two USB-C ports — a "USB-OTG" port (the ESP32-S3's native
USB peripheral, appears as `/dev/ttyACM0`, also carries JTAG) and a separate "USB-UART"
port (a USB-to-UART bridge wired to the real UART0 pins). `sdkconfig` had
`CONFIG_ESP_CONSOLE_UART_DEFAULT=y` — UART0 as the **primary** console (the one bound to
`stdin`) — with USB-Serial-JTAG only as a **secondary**, output-only mirror (for boot-log
visibility over USB even when the primary is a UART nobody's watching). Connected via the
OTG port (needed for JTAG), every log line still appeared (secondary mirrors output
fine), but bytes typed into that port had nowhere to go — `stdin` was listening on UART0,
whose pins aren't wired to this port at all.

Switching the primary console to `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` was *necessary
but not sufficient*: log output kept working (USB-Serial-JTAG's default VFS mode already
handles TX + a simple/limited RX path used automatically), but `stdin` reads still
silently stayed empty. The remaining piece: `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG` alone
does not install the full interrupt-driven USB-Serial-JTAG driver — that install is
**not called anywhere in ESP-IDF's own startup path** (confirmed by grepping
`esp_system`/`vfs`/`esp_driver_usb_serial_jtag` — the only callers in the whole SDK tree
are that driver's own test apps). Without it, incoming USB OUT (host→device) transfers
are never serviced, so every byte sent is dropped at the USB level before it ever reaches
`stdin` — not a buffering or timing issue, the byte truly never arrives anywhere in the
firmware.

**Fix:** two parts.
1. `sdkconfig`: `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` (primary), `CONFIG_ESP_CONSOLE_SECONDARY_NONE=y`
   (drop the UART0-primary / USB-secondary split entirely).
2. In code, before the first `stdin` read (`state_machine.c::main_state_machine_task`):
   ```c
   #include "driver/usb_serial_jtag.h"
   #include "driver/usb_serial_jtag_vfs.h"

   usb_serial_jtag_driver_config_t usj_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
   usb_serial_jtag_driver_install(&usj_cfg);
   usb_serial_jtag_vfs_use_driver();   // switches stdin/stdout to the interrupt-driven driver

   // usb_serial_jtag_vfs_use_driver() makes reads blocking — reapply O_NONBLOCK
   // so fgetc() returns EOF immediately instead of blocking the task.
   int flags = fcntl(fileno(stdin), F_GETFL, 0);
   fcntl(fileno(stdin), F_SETFL, flags | O_NONBLOCK);
   ```

**How this was actually diagnosed** (worth reusing next time input silently vanishes):
add a heartbeat log to the suspected task's loop (`ESP_LOGI("tick %lu", n++)` every
iteration) *before* assuming a hang. If the heartbeat keeps ticking right through the
moment input was sent, the task is alive and the bug is in the input path specifically,
not a crash/deadlock — which redirects debugging effort immediately instead of chasing
task-priority or stack-overflow theories that a live heartbeat already rules out.

**General rule:** on boards with two USB connectors (native USB/OTG+JTAG vs a
UART-bridge), decide which one owns the *interactive* console up front, and remember that
selecting a console peripheral via `CONFIG_ESP_CONSOLE_*` only guarantees log **output**
works — interactive **input** over USB-Serial-JTAG additionally needs the driver
installed and switched to explicitly in application code.

---

## TL;DR checklist: data_capture streaming is slow / SD card won't mount
1. Network POST taking 100s of ms and it's not obviously the WiFi signal (check RSSI via
   `/stats`)? → try `WIFI_PS_NONE` first, but don't stop there.
2. Still slow/erratic after that, uncorrelated with payload size? → suspect TCP
   connection churn exhausting `CONFIG_LWIP_MAX_ACTIVE_TCP` before blaming the radio.
   Reuse one `esp_http_client` handle instead of `init()`/`cleanup()` per request.
3. SD card mount fails with `FR_NO_FILESYSTEM` (13) on a card ≥64 GB? → it shipped
   exFAT; `format_if_mount_failed` reformats it as FAT32 automatically.
4. `mkdir()`/`fopen()` failing on a freshly-mounted card with no clear reason? → check
   `CONFIG_FATFS_LFN_NONE` — descriptive (>8.3) filenames need `CONFIG_FATFS_LFN_HEAP`.
5. Build suddenly overflows the app partition after adding a new driver (FATFS, SDMMC,
   etc.)? → switch to `CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE` if flash size allows.
