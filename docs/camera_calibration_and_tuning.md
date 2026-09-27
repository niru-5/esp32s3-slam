# Camera calibration and ISP tuning

Host-driven workflow for two jobs on the OV5640:

1. **Intrinsic calibration** with a checkerboard — capture N views on demand, gate them, solve on the fly, store everything so it can be re-solved later.
2. **ISP tuning** — the ten steps of [`OV5640_tuning_playbook_bench_procedures.md`](OV5640_tuning_playbook_bench_procedures.md), each one backing up the sensor registers first, applying variants, saving frames + register state, and telling the operator what to expect. Theory is in [`OV5640 _ISP_theory_from_first_principles.md`](OV5640%20_ISP_theory_from_first_principles.md).

```
 ESP32-S3 (firmware/data_capture)                          host (software/)
┌──────────────────────────────────┐   WiFi, TCP 8084    ┌──────────────────────────────────────────┐
│ serial '5' → CAMERA_CALIBRATION  │ ───────────────────►│ python -m host_server.calibration run     │
│  cam_calib_task (device = client)│  device dials host  │  ├─ link.py       framing, request/reply │
│   capture N frames + metadata    │ ────── frames ────► │  ├─ registers.py  dump/undo/restore      │
│   reg read / write / dump        │ ────── replies ───► │  ├─ flow.py       intrinsics capture+solve│
│   set_mode (jpeg/raw8/…, size)   │                     │  ├─ tuning.py     10 guided steps        │
│   set_orientation, fps_probe     │                     │  ├─ session.py    on-disk layout         │
└──────────────────────────────────┘                     │  └─ liveview.py   http://localhost:8090/ │
                                                         └──────────────────────────────────────────┘
```

(`serial '5'` above is one of two ways in now — `software/host_server/control`'s browser UI can
also flip the device into `CAMERA_CALIBRATION` over its own always-on socket, TCP 8085; see
"Control channel" below.)

## Quick start

```bash
# one-off: host tools need numpy + OpenCV (the streaming server itself stays stdlib-only)
cd software && python3 -m venv .venv && .venv/bin/pip install -r requirements-calib.txt

# firmware: build + flash (>= 45 s if using the flash_monitor wrapper, see docs/learnings.md)
.claude/skills/esp32-idf/scripts/build.sh data_capture
.claude/skills/esp32-idf/scripts/flash_monitor.sh data_capture /dev/ttyACM0 45

# session: listens on :8084, waits for the boot log, sends '5' over serial, waits for the device to dial in
cd software && .venv/bin/python -m host_server.calibration run --serial /dev/ttyACM0
#   or, with no serial cable at all: if software/host_server/control (see below) is already
#   running, `run` asks it to flip the device into calibration mode over the control channel
.venv/bin/python -m host_server.calibration run
#   or, if the device is already in mode 5 by some other means -- it keeps retrying until the host is up:
.venv/bin/python -m host_server.calibration run --no-control
```

On connect the tool creates `software/calib_data/<timestamp>/`, **backs up all 12 288 sensor registers** to `settings/00_startup.json` (~9 s), and opens an interactive `calib>` prompt (`help` lists everything). `http://localhost:8090/` shows the latest frame with detected corners and the running status. `exit` sends the device back to normal streaming config; `quit` leaves the tool and keeps the device in calibration mode. Serial `3` also aborts the mode on the device.

> **Connection direction:** the device dials the host (`CONFIG_REMOTE_HOST:CONFIG_CAM_CALIB_PORT`, 8084) and retries every ~3 s until the host tool is listening, and re-dials if the link drops — the host never has to poll for it. The host firewall must allow inbound TCP **8084** (like 8080–8083 for streaming, and 8085 for the control channel below). If the device log shows `connect to <host>:<port> failed: errno 116/113`, it's usually a firewall/VPN dropping the SYN — NordVPN's firewall with LAN Discovery off does this, and so does a plain `ufw`/`iptables` host firewall that simply has no rule for that port yet (verified on real hardware while building the control channel: `sudo ufw allow <port>/tcp` fixed it in one shot — a brand-new port needs its own rule even when an existing one already covers a neighbouring port).

## Control channel (mode switching + IMU calibration, no serial needed)

`software/host_server/control` (`python -m host_server.control`, default ports 8085 device- /
8091 browser-facing) is a small always-running app with a browser page: toggle Streaming vs.
Calibration, pick a streaming sink or calibration target (Camera/IMU), and run IMU
calibration's gravity-axis prompt from the page instead of the serial console. It talks to
the device over `control_link.c`'s always-on socket (`CONFIG_CONTROL_PORT`, distinct from
this tool's own `CONFIG_CAM_CALIB_PORT` on purpose — see `docs/architecture.md` "Control
channel" for why they're kept separate). Run it once and leave it running; `cli.py run` (no
`--serial`) uses it to enter `camera_calibration` remotely, as shown above.

It does **not** duplicate register-level tuning — that stays in this tool (`cli.py`), on
purpose. What it adds for the camera side is just the mode toggle and a note pointing back
here; the IMU side is fully interactive there (axis-picker buttons, before/after report).

## Intrinsic calibration (checkerboard)

1. Print a board: `python -m host_server.calibration make-board --cols 9 --rows 6 --square-mm 25 board.png` → **print at 100 % scale** (A4, landscape is chosen automatically when the board is wider than portrait A4), stick it on something rigid and flat, measure a square with calipers and pass the true size with `--square-mm`. The cols/rows are **inner corners** (a board with 10×7 squares has 9×6 inner corners). Change on the fly with `board <cols> <rows> <square_mm>`.
2. Fix the camera settings you will run with (focus, resolution, `freeze` for constant exposure) — **intrinsics are only valid for the mode they were captured in** (the tool refuses mixed image sizes).
3. Capture, either way:

| Command | What it does |
| --- | --- |
| `cal capture 8 700` | trigger: device grabs 8 frames 700 ms apart and streams them with metadata; each is gated and the running solve is printed |
| `cal auto 20` | polls the camera; keeps a frame whenever a sharp, *novel* board pose is visible, until 20 views |
| `cal solve [--save]` | solve now; `--save` writes `intrinsics_<ts>.json` and a Kalibr-style `camchain_<ts>.yaml` |
| `cal status` / `cal reset` | inspect / clear the in-memory view set (files on disk are kept) |

Each frame is **rejected** (and saved under `intrinsics/rejected/` with the reason) if the board is not found, is smaller than 4 % of the frame, is blurry (Laplacian variance < 20) or is nearly identical to an already accepted pose. From 8 accepted views on, every new view triggers a solve and prints RMS reprojection error, `fx fy cx cy`, FOV, distortion, the worst images, and an image-coverage map (`####/..##/…` = which of 4×3 image cells the corners have visited) plus tilt count. Aim for 15–25 views, corners in every cell, several strongly tilted (±30°) views, RMS < 0.5 px.

**Later / offline:** `python -m host_server.calibration solve calib_data/<session>` re-detects every stored image and solves again (different board size, pruning, …).

### Session layout

```
calib_data/<YYYYmmdd_HHMMSS>/
  session.json  timeline.jsonl  device.log        device hello, board, action log, firmware serial log
  settings/00_startup.json                          full register backup at connect
  settings/<step>_before_<HHMMSS>.json              backup before every tuning step
  captures/<tag>/frame_NNNN.jpg|.pgm + .json        ad-hoc `capture` output
  intrinsics/images/img_0001.jpg + .json            accepted views (+ detection stats)
  intrinsics/rejected/rej_0001.jpg + .json          rejected views with reason
  intrinsics/results/intrinsics_*.json camchain_*.yaml
  tuning/<step>/NN_variant/frame_*.jpg|pgm + .json  frames + per-frame metadata
  tuning/<step>/NN_variant/regs_full.json           all registers, for raw steps
  tuning/<step>/analysis.json  report.md            metrics, register changes, expectations
```

Per-frame sidecar JSON: `seq`, `ts_us` (esp_timer when the request was made), `frame_ts_us` (the driver's frame-complete timestamp, same clock — use this one for sync), `fmt`, `w`, `h`, `len`, `size`, `tag` and `regs` (≈45 key registers: exposure, gain, AWB gains, orientation, HTS/VTS, banding, BLC, ISP enables, AWB readback). RAW8 frames are stored as binary PGM (a Bayer mosaic, not a grey image). A ROS bag export (for Kalibr) is not implemented; `camchain_*.yaml` carries the intrinsics in Kalibr's format.

## ISP tuning

```
calib> tune                      # list the steps
calib> tune notes black_level    # what to set up physically, what to expect
calib> tune black_level          # run it (asks for Enter once the scene is ready; --yes skips)
```

Every `tune <step>` does the same thing:

1. **Backup** of all registers → `settings/<step>_before_*.json`. (`restore <file> [--apply]` writes back only the registers that differ, skipping resets/clock/PLL, statistics blocks and every *volatile* register — it takes two dumps in a row and ignores whatever changes on its own (~18 s) — so it shows real setting changes, not live AEC/AWB statistics; `diff <a> [<b>]` compares dumps; `undo` reverts every write of the session; a camera `mode` switch re-initialises the sensor and clears the undo log.)
2. **Prints the physical set-up** (lens cap, flat field, grey card, mains lamp, …) and waits.
3. **Runs variants** — each applies register changes, flushes 3–6 frames (the newest-frame buffer can still hold pre-write frames), captures, saves files, computes metrics, then **reverts** the registers it touched (`--keep` leaves the last variant applied). A step that switches between JPEG and RAW8 puts the camera back in the mode it started in.
4. Prints the **expected result** and writes `report.md` / `analysis.json`.

| Step | Variants / measurements | Needs |
| --- | --- | --- |
| `orientation` | normal / mirror / flip / both in JPEG (contact sheet, also in the live view) and RAW8 (Bayer phase guess); checks the ISP bits [2:1] follow | asymmetric scene, red object |
| `timing` | HTS, VTS, measured fps from driver timestamps → PCLK, t_row, computed B50/B60 vs the registers | – |
| `black_level` | RAW8 dark frames over exposure × gain grid + BLC off: per-Bayer-plane mean/std/zero-clip, dark-current slope, full reg dump | **lens cap** |
| `lens_shading` | RAW8 flat field averaged: falloff, optical centre, R/G B/G colour shading, 6×6 green gain grid (`shading_map.json`); LENC on/off in JPEG | uniform white field |
| `awb` | RAW8 grey card → g_R, g_B (register = round(1024·g)) per illuminant (`illuminant=tungsten`, accumulates `awb_table.json`); auto / manual-unity / manual-measured in JPEG | grey card |
| `ccm` | ColorChecker: default matrix, matrix bypass (expected to break colours — the block also does RGB→YUV) + linear RAW8 with register dumps for the offline fit | ColorChecker |
| `gamma` | gamma on/off at frozen exposure: percentiles of luma | tonal-range scene |
| `dpc` | long-exposure dark RAW8 with DPC off / white / black / both: hot-pixel counts, static hot map | lens cap |
| `cip` | sharpen off / default / strong, denoise strong: Laplacian sharpness and temporal noise | textured static scene |
| `aec_banding` | computes B50/B60 from the measured t_row, writes them + max bands + 50/60 Hz, compares banding off/on: row-banding amplitude, exposure/band ratio | mains lamp (`mains_hz=60` for 60 Hz) |

Order matters (each step assumes the earlier ones are right) — same as the playbook. Options are `key=value` after the step name.

**What is automated and what is not:** shading gain *computation* is automated, writing the LENC register table is not; the CCM fit (24-patch constrained least squares) is left to offline analysis of the saved `chart_raw` capture; gamma knots are compared on/off, not fitted. The register meanings for CIP/DPC/BLC follow the playbook; if a variant shows no change, the register may be mode-dependent or FAE-locked — the saved before/after register dumps let you check.

## Persisting tuned registers

The OV5640 has no non-volatile register storage of its own — everything resets to the
driver's compiled-in defaults on power-cycle or `esp_camera_init()`. `camera_overrides.c`
works around this: it stores a small `{addr, val}` set in ESP32 NVS and re-applies it after
every `camera_init()` (the plain boot/streaming path only — not the calibration mode's own
`camera_init_ex()` reinit, so a live tuning session never gets clobbered by its own
previously-saved values).

```
calib> save            # dry run: shows what would be saved (this session's regw/freeze/orient
                        #   writes, deduped to the last value per address, minus anything on
                        #   the UNSAFE_RESTORE list -- resets/clocks/live statistics)
calib> save --apply    # persist it to the device's NVS, and apply it immediately
calib> save status     # show what's currently saved on the device
calib> save clear      # erase it -- defaults return after the next camera_init() (exit/reboot)
```

`save`'s write-list comes from the session's own undo log (every `regw`/`freeze`/`orient`
write this session made), not a diff against a baseline dump — it's already exactly the set
of registers the operator intentionally changed. Device-side commands: `save_camera_regs
{writes:[{a,v}]}`, `clear_camera_regs`, `get_camera_overrides` (added to `cam_calib.c`'s
existing command set, valid only while `CAMERA_CALIBRATION` is active).

## Firmware side (`cam_calib.c`)

* Entered with serial `5` or `set_state camera_calibration` over the control channel (`state_machine.c`; see `docs/architecture.md` "Control channel"), streaming pipelines are torn down first (the device then dials the host); the camera is re-initialised as JPEG SVGA, quality 6, 2 frame buffers, *grab latest*. Leaving the mode (`exit`, serial `3`) restores the normal streaming camera config (and re-applies any saved register overrides, see "Persisting tuned registers").
* Wire format: `uint32 len | uint8 type | body`, type `0x01` JSON (both ways), `0x02` IMAGE (`uint32 meta_len | meta JSON | frame bytes`). Commands: `ping`, `info`, `set_mode {format,framesize,quality,fb_count}`, `reg_read`, `reg_write {writes:[{a,v,m}]}` (replies old/new bytes → undo log), `reg_dump {ranges}`, `set_orientation {mirror,flip}` (through the driver, which also fixes the 0x4514 black-level-line registers), `capture {n,interval_ms,flush,regs,tag}`, `fps_probe {n}`, `save_camera_regs {writes:[{a,v}]}`, `clear_camera_regs`, `get_camera_overrides`, `exit`.
* Formats: `jpeg`, `raw8`, `gray`, `rgb565`, `yuv422`. **RAW8** — the ESP32-S3 camera driver has no RAW path, so the sensor is brought up as GRAYSCALE (1 byte/pixel) and then `0x501F=0x03`, `0x4300=0x00` switch the OV5640 to Bayer output. It is the **top 8 of 10 bits** (a DN step is 4× the sensor's own black-level resolution) and it **bypasses the ISP scaler**, so only native readouts are coherent: `hd` (1280×720, the default for raw steps) and `vga` (640×480). At svga/xga/sxga the rows come out scrambled (the stream is really 1280 pixels wide); the tool warns when it sees this (it stays quiet on dark/flat frames, where there is nothing to measure).
* A mode switch needs a ~32 KB contiguous internal-DRAM block for the camera DMA buffer, so the command/TX buffers are allocated in PSRAM.
* The IMU is not part of this mode (`CONFIG_ENABLE_IMU` is 0 in the current build, and camera–IMU extrinsics are a separate calibration).

## Findings on the real module (verified 2026-09-25)

* Sensor PID 0x5640; SVGA JPEG runs **HTS 2060, VTS 984, 22.20 fps (±0.4 ms), PCLK 45 MHz, t_row 45.8 µs**. The stored banding steps (B50 = 295, B60 = 246, the 5 MP defaults) are **stale for this mode**: this row time needs B50 ≈ 218, B60 ≈ 182 — step 10 recomputes them.
* Bayer tile in normal orientation is **B G / G R** (confirmed against the JPEG colours); demosaic with `cv2.COLOR_BayerRG2BGR` (OpenCV names the code by the second row/column).
* RAW8 black level reads exactly 4.0 DN in all four planes at short exposure (BLC target 0x10 at 10 bit ÷ 4), std ≈ 0.01 DN.
* `set_orientation` produces `0x3820[2:1]` / `0x3821[2:1]` as the playbook describes.
* The module is mounted rotated ≈90°, which mirror/flip cannot correct (only 180° via mirror+flip).
* A full 12 288-register dump takes ≈9 s; a 1280×720 RAW8 frame (0.9 MB) takes ≈0.4 s over WiFi.

## Tests

```bash
cd software && .venv/bin/python -m unittest tests.test_calibration -v
```

`tests/fake_device.py` is an in-process stand-in for `cam_calib.c` (same protocol, fake register file, renders a distorted checkerboard from known intrinsics). The tests check that detection + solve recover the ground-truth `fx fy cx cy` and distortion, that the burst flow accepts/rejects/saves/re-solves offline, duplicates are rejected, and that backup / restore / undo / freeze behave. Everything hardware-specific (RAW8 geometry, register semantics, timing) was verified on the real device — see above.

## Ports and known issues

* Host ports: **8084** (this tool, inbound from the device), 8080 (`STREAM_WIFI` HTTP), 8081–8083 (`STREAM_TCP` frame/IMU/stats), **8085** (`software/host_server/control`, inbound from the device), 8091 (that app's browser page, only needed inbound if browsing from another host on the LAN). All must be open inbound on the host firewall -- see the `ufw`/VPN note under Quick start; each port needs its own allow rule, one working port doesn't imply the next one will.
* Streaming consumers (`net_client.c`, `tcp_client.c`, the stats writer) shut down cooperatively — see `docs/learnings.md` §14. Verified on hardware: after a calibration session `STREAM_WIFI` and `STREAM_TCP` stream normally (≈22 fps) and stop cleanly, also with the host unreachable.
