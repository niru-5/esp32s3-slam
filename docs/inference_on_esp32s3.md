# `inference_on_esp32s3` — on-device hand detection and wave recognition

`firmware/inference_on_esp32s3/` is a standalone ESP-IDF app that reads the OV5640
camera, runs a neural network **on the ESP32-S3 itself** (using Espressif's
[esp-dl](https://github.com/espressif/esp-dl)), and prints over serial:

- the **camera FPS** (how fast the sensor delivers frames),
- the **inference FPS** and milliseconds per inference,
- whether a **hand** is visible (score + bounding box), and
- when a hand is **waving** (`>>> WAVE DETECTED <<<`).

It is the stepping stone for Goal 2 in `TODO.md` (run vision on the ESP32-S3 instead
of streaming everything to a host).

> **Status:** builds clean for both boards, in flash-model and SD-model mode, and the
> wave logic is unit-tested on the host. **Not yet run on hardware** — the FPS numbers
> below are Espressif's published figures, not measurements from this rig. See
> [Things to verify on hardware](#things-to-verify-on-hardware).

---

## 1. Quick start

```bash
source ~/.espressif/tools/activate_idf_v5.3.5.sh
cd firmware/inference_on_esp32s3
idf.py build                       # first build downloads esp-dl + the hand model (needs internet)
idf.py -p /dev/ttyACM0 flash monitor
```

Board choice is in `main/config.h` (`BOARD_XIAO_ESP32S3` is selected by default, same as
`data_capture`; flip to `BOARD_ESP32S3_MAIN` for the main rig).

Expected serial output once running (format; numbers are illustrative):

```
I (…) INFER: cam 25.0 fps | infer 7.4 fps (127 ms/frame, 18 dropped) | no hand
I (…) INFER: cam 25.0 fps | infer 7.3 fps (129 ms/frame, 18 dropped) | hand 0.81 box[102,60,190,170] rev=1 | -
W (…) INFER: >>> WAVE DETECTED <<<
I (…) INFER: cam 25.0 fps | infer 7.2 fps (130 ms/frame, 17 dropped) | hand 0.78 box[80,58,172,171] rev=4 | WAVING
```

`rev` is the number of left/right direction reversals in the last 2 s (see §4).
`dropped` is how many camera frames were skipped that second because inference was busy.

---

## 2. Which model files do I need? (nothing to copy by default)

**Default: no SD card needed.** The single model file is baked into the firmware image at
build time, so `idf.py flash` "pushes it from the host" for you.

| File | Size | What it is | Needed? |
|---|---|---|---|
| `espdet_pico_224_224_hand.espdl` | ~497 KB | ESPDet-Pico hand detector, 224×224 input, INT8, built for ESP32-S3 | **Yes — the only model** |

It comes from the `espressif/hand_detect` component (esp-dl repo path
`models/hand_detect/models/s3/`), which the IDF component manager downloads into
`managed_components/` on the first build. You don't fetch it by hand.

(esp-dl also ships `mobilenetv2_0_5_128_128_gesture.espdl`, a static-gesture classifier —
one/two/…/five/like/ok. This app deliberately does **not** use it; see
[Why only the detector](#why-only-the-detector).)

### Three ways to deliver the model (esp-dl supports all three)

Chosen in `idf.py menuconfig` → **models: hand_detect** → **model location**:

| Location | How the model gets on the board | When to use |
|---|---|---|
| **flash rodata** (default) | Linked into the app binary; `idf.py flash` writes it | Simplest. Model change = reflash (~1 min). |
| **flash partition** | Written to a partition named `hand_det` (you must add it to `partitions.csv`) | Update model without relinking the app. |
| **sdcard** | You copy the `.espdl` file onto a FAT32 card | Swap models without reflashing. |

**SD-card route, exactly what to copy:**

```
<SD card root>/models/s3/espdet_pico_224_224_hand.espdl
```

Helper (run `idf.py build` once first so the component is downloaded):

```bash
firmware/inference_on_esp32s3/tools/prepare_sdcard.sh /media/$USER/<your-card>
```

Then `idf.py menuconfig` → models: hand_detect → model location → sdcard, rebuild, flash.
The file name must not be changed (esp-dl looks it up by exact name). Card must be FAT32.
The firmware mounts `/sdcard` itself: SDMMC 1-bit on the main rig (CLK=39, CMD=38, D0=40),
SPI on the XIAO (CS=21, SCK=7, MISO=8, MOSI=9) — pins in `main/config.h`.

---

## 3. How it works

### 3.1 Pipeline

```
 OV5640 ──RGB565 QVGA 320x240──▶ capture_task (core 1, prio 10)
                                        │  blocks on esp_camera_fb_get(), counts every frame
                                        ▼
                             ┌──── latest-frame slot ────┐   (mutex-guarded, holds ONE frame;
                             │  new frame replaces old   │    a replaced frame is returned to the
                             └───────────┬───────────────┘    camera driver and counted "dropped")
                                         ▼  task-notify
                          inference_task (core 0, prio 5)
                            1. esp-dl HandDetect::run(img)
                                 preprocess → INT8 net → decode boxes → NMS
                            2. pick highest-score hand → (centre-x, width)
                            3. WaveDetector.update(timestamp, x, width)
                                         ▼
                          app_main: every 1 s log camera fps / inference fps / status
```

Design choices, and why:

- **Camera and inference are decoupled.** The camera FPS you see is the *sensor's* rate,
  measured in `capture_task`; it's not throttled by inference. Inference always takes the
  *newest* frame, so latency stays about one inference long instead of growing with a queue.
  The cost is that the neural net only sees a subset of frames — the `dropped` counter shows
  how many.
- **RGB565, not JPEG.** `data_capture` uses JPEG (small, good for streaming). Neural nets
  need raw pixels, so decoding JPEG per frame would add CPU time. The sensor can output
  RGB565 directly.
- **QVGA (320×240) is enough.** The model input is 224×224. esp-dl's preprocessor
  letterbox-resizes the frame (keeping aspect ratio, padding grey) and quantises to INT8 in
  one pass; a bigger frame only costs PSRAM bandwidth and preprocess time.
- **Core split.** Camera DMA + capture on core 1, the heavy math on core 0. esp-dl runs
  on the calling task's core (it uses the S3's vector/PIE instructions there).
- **Frame buffers live in PSRAM** (`fb_count = 3`: one in inference, one filling, one
  spare). PSRAM must be **Octal @ 80 MHz** on this board (already in `sdkconfig.defaults`).

### 3.2 What esp-dl is doing

esp-dl is Espressif's inference runtime for ESP chips. The workflow it supports is:

1. Train a model in PyTorch/ONNX.
2. **Quantise** it to INT8 with ESP-PPQ and export an `.espdl` file (a FlatBuffer holding the
   graph + INT8 weights + quantisation scales). Espressif already did this for the hand model.
3. On the chip, `dl::Model` loads the `.espdl` (from rodata / a partition / SD) and runs the
   graph with kernels hand-optimised for the S3's SIMD instructions.

`HandDetect` (from `espressif/hand_detect`) wraps model + image preprocessor + postprocessor:

- **Model:** ESPDet-Pico — a tiny YOLO-style single-shot detector at 224×224, three output
  scales (strides 8/16/32).
- **Postprocessor:** turns raw grid outputs into boxes, applies a score threshold
  (we use 0.40, esp-dl's default is 0.25) and non-max suppression.
- **Result:** a list of `{category, score, box[x1,y1,x2,y2]}` **in the original frame's
  coordinates** (esp-dl undoes the letterbox), so we can use pixels directly.

Espressif's published numbers for this model on ESP32-S3 (224×224): preprocess 7.7 ms +
model 123.6 ms + postprocess 1.5 ms ≈ **133 ms → roughly 7 fps**.

### 3.3 Why only the detector

A detector answers "is there a hand and where, **in this frame**". *Waving* is a property of
motion **over time**, so no single-frame model — including esp-dl's gesture classifier — can
recognise it. We therefore:

1. use the detector every frame to get the hand's box, and
2. decide "waving" from how that box moves (§4).

The gesture classifier would add ~115 ms per frame on the S3 (≈ 4 fps total), which is too
slow to sample a 1–2 Hz wave, and it recognises static poses (it has no "wave" class).
Adding it later as a *confirmation* (e.g. require "five"/open palm while waving, run only
when the box is moving) is a natural extension — `espressif/hand_gesture_recognition`
depends on `hand_detect`, so it is a one-line manifest change.

---

## 4. Wave recognition (`main/wave_detector.hpp`)

Pure C++, no ESP or esp-dl dependency, so it's unit-tested on the host.

**Idea:** a wave is the hand moving right, then left, then right, … Track the box's
horizontal centre `x` over a sliding 2-second window and count **direction reversals**
(turning points). A reversal only counts once `x` has moved back by more than a threshold
from the last extreme — this *hysteresis* ignores detector jitter.

```
x(px)
 │      ●            ●            ●          reversals: ▲ ▼ ▲ ▼ = 3
 │    ●   ●        ●   ●        ●   ●        every swing ≥ threshold  →  WAVING
 │  ●       ●    ●       ●    ●       ●
 │           ●●●           ●●●
 └──────────────────────────────────── time (2 s window)
```

Rules:

| Parameter (`config.h`) | Default | Meaning |
|---|---|---|
| `WAVE_WINDOW_MS` | 2000 | history considered |
| `WAVE_MIN_REVERSALS` | 3 | turning points needed (≈1.5 back-and-forth cycles) |
| `WAVE_AMP_FRAC` | 0.30 | minimum swing = 30 % of the hand's box width |
| `WAVE_MIN_AMP_PX` | 6 | absolute floor for tiny/far hands |
| `WAVE_LOST_MS` | 600 | hand missing this long → forget history |
| `WAVE_HOLD_MS` | 800 | keep saying "WAVING" this long after the last reversal (no flicker) |

The swing threshold scales with the hand's box width, so a far-away (small) hand and a
close (big) hand are judged the same way.

**Timing limit:** at ~7 fps you get a sample every ~140 ms. A wave at 2 Hz has a half-swing of
250 ms → ~2 samples per swing, which is the practical minimum. Very fast waves (>2.5 Hz)
will alias and may be missed; a normal "hello" wave (1–2 Hz) is fine. Faster inference
(smaller input, or a lower-latency detector) directly improves this.

**Tests:** `firmware/inference_on_esp32s3/test/test_wave_detector.cpp` (8 cases: real waves at
8 fps and 4 fps, stationary jitter, a single sweep, small tremor, hold/expiry, hand lost, far
hand). Run on the host:

```bash
cd firmware/inference_on_esp32s3
# (use a PATH without the ESP toolchain first, or g++ picks up the xtensa `as`)
PATH=/usr/bin:/bin g++ -std=c++17 -Wall -I main test/test_wave_detector.cpp -o test/test_wave_detector && ./test/test_wave_detector
```

---

## 5. Files

| Path | Purpose |
|---|---|
| `main/inference_on_esp32s3.cpp` | app: camera init, capture task, inference task, stats loop, optional SD mount |
| `main/wave_detector.hpp` | temporal wave decision (host-testable) |
| `main/config.h` | board pins, camera settings, task placement, wave tunables |
| `main/idf_component.yml` | deps: `espressif/esp32-camera`, `espressif/hand_detect ~0.2.0` (pulls `esp-dl 3.3.x`) |
| `sdkconfig.defaults` | Octal PSRAM, 8 MB flash, perf optimisation, model-in-rodata |
| `partitions.csv` | 4 MB app slot (default 1 MB is too small for esp-dl) |
| `tools/prepare_sdcard.sh` | copy the model onto an SD card |
| `test/test_wave_detector.cpp` | host unit tests |

Build size: ~2.5 MB image (model included) in a 4 MB slot.
`sdkconfig.defaults` is only applied when `sdkconfig` doesn't exist — delete `sdkconfig`
(gitignored) to re-apply after editing it.

---

## 6. Things to verify on hardware

Not yet run on a board. In rough order of likelihood to need attention:

1. **RGB565 byte order.** `INFER_RGB565_IS_BIG_ENDIAN` in `config.h` tells esp-dl how to read
   camera pixels; the default (`0`, little-endian) is my best reading of how
   esp32-camera lays out RGB565 on the S3, but I couldn't confirm it without a board.
   *Symptom of the wrong setting:* hands are never (or rarely) detected in good light.
   *Fix:* flip it and rebuild.
2. **Camera FPS.** RGB565 QVGA at a 20 MHz XCLK should be sensor-limited, but the actual
   figure is unmeasured. If it's low, try a higher `INFER_CAM_XCLK_HZ` (OV5640 tolerates up to
   ~24 MHz) — but watch for corruption.
3. **Inference FPS** should land near Espressif's ~7 fps; the extra cost here is
   letterboxing a 320×240 RGB565 frame instead of a 224×224 RGB888 one.
4. **Wave thresholds.** `WAVE_AMP_FRAC` / `WAVE_MIN_REVERSALS` are reasoned defaults;
   tune with the `rev=` value in the log (watch it while waving vs holding still).
5. **Image orientation.** If the camera is mounted rotated/mirrored, "left/right" flips but
   wave detection is unaffected (it only counts reversals).
6. **Memory.** The inference task has a 12 KB stack; esp-dl allocates its working tensors
   from PSRAM. If you see a stack overflow or heap error at first inference, raise
   `INFER_INFERENCE_STACK`.

## 7. Troubleshooting

| Symptom | Likely cause |
|---|---|
| Boot crash right away | PSRAM mode wrong — must be **Octal, 80 MHz** (`sdkconfig.defaults`) |
| `camera init failed` | wrong `BOARD_*` in `config.h`, or camera ribbon not seated |
| `SD mount failed` (SD mode) | card not FAT32 / not inserted / wrong board selected |
| Model load error mentioning file not found (SD mode) | file not at `/sdcard/models/s3/espdet_pico_224_224_hand.espdl` |
| Never detects a hand | byte order (item 1), poor light, hand too small in frame (<~40 px) |
| Detects hand, never "WAVING" | swing too small vs `WAVE_AMP_FRAC`; wave wider, or lower it |
| False WAVING on a still hand | detector box jittering; raise `WAVE_AMP_FRAC` or `INFER_HAND_SCORE_THR` |

## 8. References

- esp-dl: <https://github.com/espressif/esp-dl> (`examples/hand_detect`, `models/hand_detect`)
- Model docs: `models/hand_detect/README.md`, `models/hand_gesture_recognition/README.md`
- Component registry: `espressif/esp-dl` (3.3.12 at time of writing), `espressif/hand_detect` (0.2.x)
