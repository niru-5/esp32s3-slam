# Camera calibration and ISP tuning

Host-driven workflow for two jobs on the OV5640, both reachable from **one browser page**
(`python -m host_server`, see `docs/architecture.md` "Control channel") or from a terminal
(`cli.py`, a thin client of that same running server — see below):

1. **Intrinsic calibration** with a checkerboard — capture N views on demand, gate them, solve on the fly, store everything so it can be re-solved later.
2. **ISP tuning** — the ten steps of [`OV5640_tuning_playbook_bench_procedures.md`](OV5640_tuning_playbook_bench_procedures.md), each one backing up the sensor registers first, applying variants, saving frames + register state, and telling the operator what to expect. Theory is in [`OV5640 _ISP_theory_from_first_principles.md`](OV5640%20_ISP_theory_from_first_principles.md).

```
 ESP32-S3 (firmware/data_capture)                    host (software/, ONE process)
┌───────────────────────────────┐  TCP 8085 (only  ┌───────────────────────────────────────┐
│ control_link.c (device=client)│  control socket) │ python -m host_server                  │
│  set_state camera_calibration │◄─────────────────┤  http://localhost:8080/  (the one page)│
│  capture N frames + metadata  │──── frames ──────►│  ├─ app.py            HTTP + /command  │
│  reg read / write / dump      │◄─── replies ──────┤  ├─ device_session.py device connection│
│  set_mode, set_orientation,   │                   │  ├─ calibration/console.py  worker     │
│  fps_probe, save_camera_regs  │                   │  ├─ calibration/{registers,flow,tuning,│
└───────────────────────────────┘                   │  │   intrinsics,session}.py  (unchanged)│
                                                     │  └─ calibration/cli.py  App + terminal │
                                                     │     client (ConsoleClient)             │
                                                     └───────────────────────────────────────┘
```

Camera calibration used to need its own dedicated socket/port purely so two separate host
processes (a streaming server and a calibration tool) wouldn't contend for one port. With a
single host process there's no more reason for that split: mode switching, calibration, and
streaming all go over the one control socket the device dials.

## Quick start

```bash
# one-off: host tools need numpy + OpenCV
cd software && python3 -m venv .venv && .venv/bin/pip install -r requirements-calib.txt

# firmware: build + flash (>= 45 s if using the flash_monitor wrapper, see docs/learnings.md)
.claude/skills/esp32-idf/scripts/build.sh data_capture
.claude/skills/esp32-idf/scripts/flash_monitor.sh data_capture /dev/ttyACM0 45

# host: the one command, one URL
cd software && .venv/bin/python -m host_server
# -> open http://localhost:8080/ in a browser: Mode -> Calibration -> Camera
```

The browser page's Calibration panel is the primary way in: click "Camera", which puts the
device into `CAMERA_CALIBRATION` and starts a session (`software/calib_data/<timestamp>/`,
**backs up all 12 288 sensor registers** to `settings/00_startup.json`, ~9 s). From there you
get a live capture image, a "Capture single image" button, quick-launch buttons for the ISP
tuning steps and intrinsics capture, save/save-status/save-clear buttons, and a command
console that runs any of the commands below (`reg`, `regw`, `tune <step>`, `cal ...`,
everything in `help`'s output) with live output.

**Prefer a terminal?** `cli.py` (`host_server.calibration run`) is the exact same session, as
a client instead of a browser tab — it does not own its own device connection anymore, it
just talks to the already-running `python -m host_server`:

```bash
cd software && .venv/bin/python -m host_server.calibration run
#   or, pointed at a server running elsewhere:
.venv/bin/python -m host_server.calibration run --server http://otherhost:8080
```

`exit` (either the browser or `cli.py`) sends the device back to normal streaming config;
`quit` (`cli.py` only) leaves the terminal but keeps the device in calibration mode — the
browser page (or a later `cli.py run`) picks the same session back up. Serial `3` also
aborts the mode on the device, from either.

> **Connection direction:** the device dials the host (`CONFIG_REMOTE_HOST:CONFIG_CONTROL_PORT`,
> 8085) and retries every ~3 s until the host is listening, and re-dials if the link drops —
> the host never has to poll for it. The host firewall must allow inbound TCP **8085** (like
> 8080–8083 for streaming). If the device log shows `connect to <host>:<port> failed: errno
> 116/113`, it's usually a firewall/VPN dropping the SYN — NordVPN's firewall with LAN
> Discovery off does this, and so does a plain `ufw`/`iptables` host firewall that simply has
> no rule for that port yet (`sudo ufw allow 8085/tcp` — a brand-new port needs its own rule
> even when a neighbouring one is already open).

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

## Firmware side (`control_link.c`, camera reconfigure in `cam_calib.c`)

* Entered with serial `5` or `set_state camera_calibration` over the control channel (`state_machine.c`; see `docs/architecture.md` "Control channel"), streaming pipelines are torn down first; `cam_calib_enter_mode()` re-initialises the camera as JPEG SVGA, quality 6, 2 frame buffers, *grab latest* — synchronously, no separate task. Leaving the mode (`exit`, serial `3`) calls `cam_calib_exit_mode()`, restoring the normal streaming camera config (and re-applying any saved register overrides, see "Persisting tuned registers").
* Wire format: `uint32 len | uint8 type | body`, type `0x01` JSON (both ways), `0x02` IMAGE (`uint32 meta_len | meta JSON | frame bytes`) — all on `control_link.c`'s one always-on socket, gated on `CAMERA_CALIBRATION` being the active state. Commands: `ping`, `info`, `set_mode {format,framesize,quality,fb_count}`, `reg_read`, `reg_write {writes:[{a,v,m}]}` (replies old/new bytes → undo log), `reg_dump {ranges}`, `set_orientation {mirror,flip}` (through the driver, which also fixes the 0x4514 black-level-line registers), `capture {n,interval_ms,flush,regs,tag}`, `fps_probe {n}`, `save_camera_regs {writes:[{a,v}]}`, `clear_camera_regs`, `get_camera_overrides`, `exit`.
* Formats: `jpeg`, `raw8`, `gray`, `rgb565`, `yuv422`. **RAW8** — the ESP32-S3 camera driver has no RAW path, so the sensor is brought up as GRAYSCALE (1 byte/pixel) and then `0x501F=0x03`, `0x4300=0x00` switch the OV5640 to Bayer output. It is the **top 8 of 10 bits** (a DN step is 4× the sensor's own black-level resolution) and it **bypasses the ISP scaler**, so only native readouts are coherent: `hd` (1280×720, the default for raw steps) and `vga` (640×480). At svga/xga/sxga the rows come out scrambled (the stream is really 1280 pixels wide); the tool warns when it sees this (it stays quiet on dark/flat frames, where there is nothing to measure).
* A mode switch needs a ~32 KB contiguous internal-DRAM block for the camera DMA buffer, so the command/TX buffers are allocated in PSRAM.
* The IMU is not part of this mode (camera–IMU extrinsics are a separate calibration). Whether IMU hardware is present at all is now a runtime fact (`imu_available()`, see `docs/calibration.md`), not a compile-time toggle — irrelevant here either way.

## Findings on the real module (verified 2026-09-25)

* Sensor PID 0x5640; SVGA JPEG runs **HTS 2060, VTS 984, 22.20 fps (±0.4 ms), PCLK 45 MHz, t_row 45.8 µs**. The stored banding steps (B50 = 295, B60 = 246, the 5 MP defaults) are **stale for this mode**: this row time needs B50 ≈ 218, B60 ≈ 182 — step 10 recomputes them.
* Bayer tile in normal orientation is **B G / G R** (confirmed against the JPEG colours); demosaic with `cv2.COLOR_BayerRG2BGR` (OpenCV names the code by the second row/column).
* RAW8 black level reads exactly 4.0 DN in all four planes at short exposure (BLC target 0x10 at 10 bit ÷ 4), std ≈ 0.01 DN.
* `set_orientation` produces `0x3820[2:1]` / `0x3821[2:1]` as the playbook describes.
* The module is mounted rotated ≈90°, which mirror/flip cannot correct (only 180° via mirror+flip).
* A full 12 288-register dump takes ≈9 s; a 1280×720 RAW8 frame (0.9 MB) takes ≈0.4 s over WiFi.

## Tests

```bash
cd software && .venv/bin/python -m unittest tests.test_calibration -v   # registers/flow/tuning/intrinsics logic
cd software && .venv/bin/python -m unittest tests.test_control -v      # the unified host_server.app
```

`tests/fake_device.py` is an in-process stand-in speaking the calibration protocol only (same
shape `control_link.c` carries now, previously `cam_calib.c`'s own socket) — fake register
file, renders a distorted checkerboard from known intrinsics. Used by `test_calibration.py`
to check that detection + solve recover the ground-truth `fx fy cx cy` and distortion, that
the burst flow accepts/rejects/saves/re-solves offline, duplicates are rejected, and that
backup/restore/undo/freeze behave — exercising `App`/`RegisterBank`/`IntrinsicFlow` directly,
independent of the HTTP layer. `tests/fake_control_device.py` speaks the *merged* protocol
(state + calibration + images) and backs `test_control.py`, which exercises the whole stack
through HTTP: `/status.json`, `/command`, and a full calibration-console lifecycle (startup
backup, `info`/`reg`/`capture` via the console, a `tune` step's readline/`/calib/answer`
path, `exit` torn down by the background poller, on-demand `STREAM_TCP` ports). Everything
hardware-specific (RAW8 geometry, register semantics, timing) was verified on the real
device — see above.

## Ports and known issues

* Host ports: **8080** (the one browser page + `STREAM_WIFI` HTTP ingest, must be reachable to open the page at all), **8085** (the device's control channel, inbound from the device — calibration, mode switching, IMU calibration all go over this one), 8081–8083 (`STREAM_TCP` frame/IMU/stats — opened only while `STREAM_TCP` is the active mode, closed otherwise). All must be open inbound on the host firewall -- see the `ufw`/VPN note under Quick start; each port needs its own allow rule, one working port doesn't imply the next one will.
* Streaming consumers (`net_client.c`, `tcp_client.c`, the stats writer) shut down cooperatively — see `docs/learnings.md` §14. Verified on hardware: after a calibration session `STREAM_WIFI` and `STREAM_TCP` stream normally (≈22 fps) and stop cleanly, also with the host unreachable.
