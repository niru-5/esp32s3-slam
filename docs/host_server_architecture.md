# `host_server` architecture

The host side of the rig is **one process, one browser page**:

```bash
cd software && .venv/bin/python -m host_server
# -> open http://localhost:8080/ in a browser
```

This document walks through how that one process is put together. For the device side, see
[`architecture.md`](architecture.md) (`data_capture`'s runtime state machine, task/queue
layout, control channel). For the calibration/ISP-tuning protocol and workflow specifically,
see [`camera_calibration_and_tuning.md`](camera_calibration_and_tuning.md) — this document
covers the *host process's* shape, not the calibration commands themselves.

## Why one process

Earlier rounds of this project had three separate host processes (streaming ingest, a
mode-toggle control server, and `cli.py` as its own calibration tool), each binding its own
port. That was real friction in practice — multiple ports to open in the firewall, and the
browser page telling the operator to go run a second terminal command for anything beyond
basic mode switching. Consolidating to one process removes an entire class of "which of my
three things is actually listening right now" bugs, at the cost of that one process now
owning several different jobs. The rest of this document is about how those jobs stay
cleanly separated *inside* the one process.

## Module map

```
software/host_server/
├── __main__.py           entry point: parses args, builds Server, runs the HTTP server
├── app.py                Server (owns everything below) + the HTTP request handler
├── device_session.py     DeviceSession: the device's one always-on control channel, host side
├── hub.py                Hub: shared latest-frame/latest-IMU/stats state for STREAM_WIFI
├── wire.py                decodes the STREAM_WIFI/STREAM_TCP binary frame/IMU/stats payloads
├── tcp_ingest.py         TcpIngestManager: the on-demand raw-TCP listeners for STREAM_TCP
├── bag_recorder.py       BagRecorder / RecordingSlot: optional ROS 2 bag writer
├── index.html            the one browser page
└── calibration/
    ├── cli.py            App (the calibration session), HELP, terminal client (ConsoleClient)
    ├── console.py        CalibrationConsole: runs App on a worker thread for the browser
    ├── registers.py      RegisterBank: register read/write/undo-log, orientation tracking
    ├── tuning.py         the ten ISP tuning steps (Ctx, Tuner, snapshot_current)
    ├── analysis.py       image statistics (Bayer-plane extraction, debayer, etc.)
    ├── intrinsics.py      checkerboard detection + calibrateCamera solve
    ├── flow.py            IntrinsicFlow: the capture/gate/solve loop for `cal auto`
    ├── liveview.py        LiveView: the calibration panel's live + "current" image buffers
    ├── session.py          Session: one calibration run's on-disk folder (backups, captures,
    │                       reports), plus decode()/save_image()
    └── link.py             DeviceLink: the offline/test transport (fake_device.py uses this;
                            the live server uses DeviceSession instead, same .call() shape)
```

`cli.py`'s classes (`App`, `RegisterBank`, `IntrinsicFlow`, `Tuner`) are unchanged from when
`cli.py` used to own its own device connection — only the transport underneath them changed
(`DeviceSession` live, `DeviceLink` in offline tests). This is deliberate: it is what let the
*entire* register/tuning/intrinsics workflow become a browser feature without rewriting any
of that logic — see "Camera calibration" below.

## The control channel: `DeviceSession`

The device dials the host (never the other way around) on one always-on TCP socket
(`CONFIG_CONTROL_PORT`, 8085) and stays connected for its whole runtime, independent of
which `app_state_t` it's currently in. `DeviceSession` is this socket's host-side listener:

- A background thread (`_accept_loop`) accepts the device's connection and keeps reading
  from it (`_read_loop`) for as long as the process runs. If the device reconnects (reboot,
  WiFi drop), a new connection simply replaces the old one.
- Every JSON message with an `"evt"` key and no request id is an unprompted **event**
  (`hello` on connect, `imu_cal_preview`/`imu_cal_report` during IMU calibration) — stored as
  `self.last_event`, picked up by `/status.json` and rendered by the browser's IMU panel.
- `call(cmd, **kwargs)` sends a framed JSON request with a fresh id, blocks on a
  `threading.Event` until a reply with a matching id arrives (or `timeout`), and returns a
  `Reply`. **Only one `call()` may be in flight at a time** (`self._call_lock`) — the device
  itself only ever answers one command before reading the next, so this lock is what keeps
  the calibration console's worker thread and the browser's `/command` handler from
  interleaving their requests on the wire.
- `MSG_IMAGE` frames (camera-calibration captures) are collected onto whichever `call()` is
  currently pending, and `reg_chunk` events (a `reg_dump`'s streamed register data) the same
  way — this is what lets `RegisterBank.dump()` and `Ctx.capture()` just call `.call(...)`
  and get back a `Reply` with `.images`/`.chunks` already populated, matching the wire
  protocol described in `camera_calibration_and_tuning.md`.

## Streaming ingest: `Hub`, `wire.py`, `tcp_ingest.py`

Two independent paths deliver frames/IMU/stats from the device to the browser, matching the
device's `STREAM_WIFI` / `STREAM_TCP` states (see `architecture.md`):

- **STREAM_WIFI**: the device `POST`s to this same process's HTTP port (`/frame`, `/imu`,
  `/stats` — see routes table below). `wire.py` decodes each payload; `Hub.put_frame()` /
  `put_imu()` / `put_stats()` update the shared latest-value state and running-rate EMAs.
  These handlers are always listening (can't meaningfully be "on demand" without also taking
  the browser page's own port down) — they just no-op if the device isn't actually in
  `STREAM_WIFI`, since nothing will be POSTing to them.
- **STREAM_TCP**: `TcpIngestManager` opens three raw-TCP listener sockets (frame/IMU/stats,
  default 8081-8083) **only** while `STREAM_TCP` is the device's active state — opened by
  `Server._set_state()` right before telling the device to enter that state, closed right
  after leaving it. This one genuinely opens/closes on demand, since nothing else needs
  those ports. Received data feeds into the same `Hub` (and `BagRecorder`, if recording).

The browser's live preview (`<img id="cam" src="/stream.mjpg">`) is `Hub.wait_frame()`
turned into a `multipart/x-mixed-replace` stream by `_Handler._stream_mjpeg()` — it blocks on
`Hub`'s condition variable until a newer frame arrives, so it's push-driven, not polled.

## Recording: `RecordingSlot` / `BagRecorder`

`BagRecorder` wraps a `rosbag2_py.SequentialWriter`, importable only in a sourced ROS 2
environment (`bag_recorder.ros_available()`); if the import fails, the server still runs as
a live viewer, just without recording. `RecordingSlot` is the swappable holder every ingest
handler keeps a reference to (`recorder.write_frame(...)` etc. are always safe to call, even
with nothing recording — they just no-op) — `POST /command {"cmd":"set_recording",...}`
creates/destroys the actual `BagRecorder` instance inside it at runtime. `/status.json`'s
`ros_available`/`ros_import_error` fields let the browser grey out the "record to ROS
bag" checkbox with a reason, the same pattern `imu_available` uses for "include IMU".

## Camera calibration: `CalibrationManager`, `CalibrationConsole`, `App`

This is the part worth understanding in the most depth, since it's where a whole
interactive terminal-style workflow (register access, the ten ISP tuning steps, checkerboard
intrinsics) becomes a browser feature.

```
 browser                    Server                      CalibrationManager      App (cli.py)
┌─────────┐  POST /command  ┌──────────────┐  enter()   ┌──────────────────┐   ┌───────────┐
│ Camera  │ ───────────────►│ _set_state() │───────────►│ Session, LiveView│──►│ RegisterBank
│ button  │  set_state:     │              │            │ CalibrationConsole│   │ IntrinsicFlow
└─────────┘  camera_calib.  └──────────────┘            └──────────────────┘   │ Tuner
                                                                  │             └───────────┘
                          POST /calib/line {"line":"tune awb --yes"}
┌─────────┐ ───────────────────────────────────────────────────► console.submit(line)
│ console │                                                              │
│ (or a   │ GET /calib/output.json?since=N  (poll every 600ms)           ▼
│ quick-  │ ◄─────────────────────────────────────────────  worker thread runs
│ launch  │  {"lines":[...], "busy":..., "live_version":...,  App.run_line(line)
│ button) │   "before_version":...}                           unchanged from the
└─────────┘                                                    interactive terminal
```

**`CalibrationManager`** (in `app.py`) owns the one `Session`/`App`/`CalibrationConsole` for
as long as the device is in `camera_calibration`.

- `enter()` builds a `Session` (the on-disk folder for this run), a `LiveView`, and a
  `CalibrationConsole` — whose constructor synchronously runs `App.startup_backup()` (backs
  up all ~12,288 registers, ~9s) and `tuning.snapshot_current()` (one frame captured in the
  sensor's *actual current* state — see "Recent fixes" below) before the console is
  considered ready. `enter()`/`leave()` are guarded by a `threading.Lock` — the
  check-then-act window across that ~9s backup is wide enough that two independent callers
  (the HTTP handler driving a just-issued `set_state`, and the background poller noticing
  the same state change) really did both build their own session on real hardware before
  this lock existed.
- `leave()` calls `CalibrationConsole.close()`, which stops the worker accepting new
  commands, unblocks it if it's waiting on a `tune` step's readline prompt, and **waits**
  (bounded, 20s) for whatever command is currently running to actually finish before
  returning. See "Recent fixes" below for why this matters.

**`CalibrationConsole`** (`calibration/console.py`) is the bridge between a request/response
HTTP API and `App.run_line()`'s blocking, synchronous, terminal-shaped interface:

- A background worker thread pulls lines off a queue and runs
  `self.app.run_line(line)` — **completely unchanged** from the interactive `cli.py` REPL.
  Output goes into a version-counted list via `App`'s `out` callback instead of `print()`.
- The one blocking call `cli.py`/`tuning.py` makes that doesn't fit request/response —
  `Tuner.run()`'s "press Enter when the scene is ready" — goes through an injectable
  `app.readline()` (see `App.__init__`'s `readline` param) instead of the builtin, backed by
  a second queue that `POST /calib/answer` fills.
- `cli.py` itself (`python -m host_server.calibration run`) is a thin HTTP client of these
  same `/calib/line` / `/calib/output` / `/calib/answer` endpoints now (`ConsoleClient`), not
  a second device connection — so the interactive terminal and the browser drive the exact
  same session.

**`App`** (`cli.py`) is the calibration session itself: `RegisterBank` (register
read/write/undo-log, orientation tracking — see `sync_orientation()`/`.mirror`/`.flip`),
`IntrinsicFlow` (checkerboard capture/gate/solve), `Tuner` (the ten ISP steps). None of this
changed when the transport moved from an owned `DeviceLink` socket to `DeviceSession` calls —
only `App.__init__`'s `link` argument's shape matters (`.call(cmd, **kwargs) -> Reply`).

## Recent fixes (2026-09-27, live-hardware bug report)

Four issues reported from actually using the browser page, each traced to a specific gap:

1. **Calibration panel flicker.** `pollCalib()` unconditionally reassigned both images'
   `.src` every 600ms poll, even when the version hadn't changed — reassigning `.src` to the
   exact same string still re-triggers a visible reload in most browsers. Fixed by only
   reassigning when the version actually changes, and showing a placeholder instead of an
   empty-body 200 response (broken-image icon) before any image exists. The calibration
   panel's controls are also now visibly disabled (`#camera-controls.loading`) until the
   console reports `active:true`, instead of looking interactive during the ~9s entry
   backup.
2. **"Before" picture showing stale/default state.** Two real gaps, not one: (a)
   `RegisterBank.__init__` assumed `mirror=flip=False` instead of reading what the sensor
   actually has (e.g. NVS-persisted orientation from an earlier `save --apply`, re-applied
   by every `camera_init()`) — fixed by `RegisterBank.sync_orientation()`, called from
   `App.__init__`. (b) the "before" picture only ever got captured at the start of a `tune`
   step, leaving the panel blank until the operator ran one — fixed by calling the same
   capture (renamed `tuning.snapshot_current()`) right after `CalibrationManager.enter()`'s
   startup backup too, and relabelling it "Current" in the UI (it was always "current state
   right now", "Before" was just a confusing name for it).
3. **No graceful exit from calibration.** Switching to Streaming while a `tune` step was
   still running used to tear the console down immediately
   (`CalibrationConsole.close()` just flagged a bool) while `Server._set_state()` had
   *already* told the device to leave `camera_calibration` — so the orphaned worker
   thread's next register/capture call failed `"not in camera_calibration"`, and its own
   cleanup (`finally` blocks reverting registers) then failed too. Fixed two ways: `close()`
   now actually waits (with a bounded timeout, unblocking any pending readline first) for
   the in-flight command to finish, and `_set_state()` now closes the console **before**
   telling the device to change state, not after — so an in-flight command gets a clean
   chance to finish while the device is still actually in `camera_calibration`.
4. **ROS bag recording checkbox silently unchecking itself.** `set_recording` was failing
   (ROS 2 not sourced into this `host_server` process) with no visible explanation — the
   checkbox just reverted a moment later when the next `/status.json` poll re-synced it.
   Fixed by surfacing `ros_available`/`ros_import_error` in `/status.json` (same pattern as
   `imu_available`) so the browser greys out the checkbox with a reason, and by showing the
   `set_recording` error immediately instead of waiting for the next poll to silently fix it.

## Recent fixes (2026-09-28, UI/workflow review)

Eight issues from a review of the browser page + firmware, tracked in
`docs/ui_fixes_todo.md`:

1. **Mode label.** "Streaming" → "Stream" (copy only, `index.html`).
2. **fps/resolution defaults hardcoded in the browser.** `index.html` hardcoded `fps=25`/
   `svga` independent of the firmware's actual compiled defaults
   (`CONFIG_CAMERA_CAPTURE_FPS`/`CONFIG_CAMERA_DEFAULT_FRAMESIZE` in `config.h`). Fixed by
   adding both to `get_status`'s reply (`control_link.c`) and having the browser seed its
   fps/resolution fields from them on first load (`seedStreamDefaults()`) instead of a second,
   driftable copy of the same numbers.
3. **Resolution dropdown showed enum names, not pixel sizes.** `camera.c`'s `SIZES[]` was
   already the curated "feasible" list (12 of `sensor.h`'s 24 `framesize_t` values); the
   dropdown just needed pixel-dimension labels (`svga` → `800×600`, etc.) — the wire value
   (`"svga"`) is unchanged.
4. **ROS bag recording never worked despite ROS 2 being installed.** `software/.venv` has
   `include-system-site-packages=false`; sourcing `/opt/ros/<distro>/setup.bash` only patches
   `PYTHONPATH`, and the venv was still missing `PyYAML`, which `rclpy` imports — so
   `rosbag2_py`'s import chain (and `ros_available()`) failed even with ROS 2 correctly
   sourced. No sudo/apt install needed: `pip install pyyaml` into the venv fixed it; added to
   `requirements-calib.txt`.
5. **Stream stats box showed in Calibration mode too.** The right column's "Stream" panel had
   no mode gating. Gave it an id and hid/showed it alongside the Stream/Calibration panel
   toggle.
6. **No graceful exit from calibration mode, stale UI on re-entry.** Two related gaps beyond
   the 2026-09-27 round's close-ordering fix (#3 above, which only fixed the console's
   internal race): there was no dedicated "leave calibration" affordance (switching to Stream
   worked, but nothing stopped an operator from doing that mid-run without realizing it should
   go through a clean exit), and the *browser's own* console/image state (`consoleSince`,
   `lastLiveVersion`, etc.) was never reset between sessions, so re-entering calibration
   showed the previous session's console log and pictures until new events overwrote them.
   Fixed with an explicit "Exit calibration mode" button (`set_state idle`, same path as
   switching to a stream sink), the Stream mode button disabled for the duration
   (`setCalibrationActive()`, driven off `get_status`, so it also reacts to calibration
   entered/left via serial or another tab), and `resetCalibrationUI()` wiping the console/
   image state on every fresh entry and exit.
7. **Streaming showed a flipped image; calibration's "current" picture looked normal.** Real
   root cause, one level under the 2026-09-27 `RegisterBank.sync_orientation()` fix (which
   made the *analysis* code correctly read whatever orientation registers actually hold — it
   didn't touch what the hardware registers actually get set to): `cam_calib_enter_mode()`
   (`cam_calib.c`) calls `camera_init_ex()` directly and never called
   `camera_overrides_apply()`, unlike `camera_init_streaming()` (`camera.c`), which does. So
   entering calibration left the sensor's *actual* registers at the driver's un-tuned
   defaults — including orientation — while streaming reflected the saved NVS overrides. The
   "current" picture was truthfully unflipped because the hardware genuinely was, at that
   point, unflipped; only streaming re-applied the saved override. Fixed by calling
   `camera_overrides_apply()` in `cam_calib_enter_mode()` too, so both paths start from the
   same persisted register state.
8. **No operator-facing guidance for the calibration workflow.** The technical detail already
   existed in this doc and the tuning playbook, but nothing pointed at it from the UI itself.
   Added: a short intro in the Calibration panel, a description of the intrinsics flow
   (`cal auto 20` → `cal status` → `cal solve --save`) above those buttons, and a per-step
   "ⓘ" button next to each `tune` step that runs `tune notes <step>` — reusing the session's
   own notes command instead of a second copy of the setup text that could drift from it.

## HTTP routes

```
GET  /                    the one browser page (index.html)
GET  /status.json         device status + streaming stats + recording + ros_available
GET  /stream.mjpg         multipart MJPEG of the latest frames (STREAM_WIFI/STREAM_TCP)
POST /frame  /imu  /stats device -> host streaming ingest (STREAM_WIFI only)
POST /command             {"cmd": ...} forwarded to the device, or a host-only
                          pseudo-command (set_recording; set_state is special-cased)

GET  /calib/live.jpg      latest calibration-session capture ("Live / After")
GET  /calib/before.jpg    the "Current" picture (see "Recent fixes" #2)
GET  /calib/output.json   {"lines","next","busy","waiting_for_answer","active",
                           "live_version","live_text","before_version","before_text"}
                          -- poll with ?since=<next from the previous poll>
POST /calib/line          {"line": "..."} -- run one cli.py command
POST /calib/answer        {"line": "..."} -- answers a pending readline() prompt
```

## The browser page

`index.html` is one page, two polling loops:

- `poll()` (1s): `/status.json` — connection dot, device state, uptime/heap/PSRAM, IMU
  availability, streaming stats (frame/IMU rate, CPU load, RSSI), recording checkbox state.
- `pollCalib()` (600ms): `/calib/output.json` — console output lines, the two calibration
  images (version-gated, see "Recent fixes" #1), and the loading/ready state of the
  calibration controls.

Mode switching (`Streaming` / `Calibration` buttons) only toggles which panel is *shown* —
the actual device state change happens on the sink/target buttons underneath
(`sink-wifi`/`sink-tcp`/…, `target-camera`/`target-imu`), each a `POST /command`.

## Where to look

- Device-side runtime, tasks, control channel: [`architecture.md`](architecture.md)
- Calibration protocol, the ten ISP tuning steps, register persistence:
  [`camera_calibration_and_tuning.md`](camera_calibration_and_tuning.md)
- IMU calibration/bring-up: [`calibration.md`](calibration.md)
- Tests: `software/tests/test_control.py` (this process, against a fake device),
  `software/tests/test_calibration.py` (the calibration logic, against a different fake).
