# UI/workflow fixes tracker (2026-09-28)

Tracks the 8 issues from the 2026-09-28 UI review of `firmware/data_capture` +
`software/host_server`. Root-cause notes captured during investigation, kept here
since they explain *why* the fix looks the way it does.

| # | Issue | Root cause | Status |
|---|-------|------------|--------|
| 1 | Mode button says "Streaming", should be "Stream" | Copy only (`index.html`) | done |
| 2 | fps/resolution defaults should come from `config.h`, not hardcoded in `index.html` | `index.html` hardcoded `fps=25`/`svga`; device never exposed its compiled defaults | done — hw-verified |
| 3 | Resolution dropdown should show pixel dimensions, only feasible sizes | `camera.c`'s `SIZES[]` already curates the feasible list (12 of `sensor.h`'s 24 `framesize_t` values) — just needed pixel labels in the UI | done — hw spot-check (5mp) |
| 4 | ROS bag recording doesn't work despite ROS 2 installed | `software/.venv` has `include-system-site-packages=false`, and even with `/opt/ros/jazzy/setup.bash` sourced (which only patches `PYTHONPATH`), the venv was missing `PyYAML` — `rclpy` imports it and failed, so `rosbag2_py`'s import chain failed and `ros_available()` was always `False`. No sudo needed — fixed with `.venv/bin/pip install pyyaml` (added to `requirements-calib.txt`). | done — hw-verified (real bag recorded, `ros2 bag info` confirmed CompressedImage/Imu/DiagnosticArray) |
| 5 | "Stream" stats box should only show in Stream mode | `index.html`'s right column showed the Stream panel unconditionally | done |
| 6 | No way to exit calibration mode gracefully; stale UI on re-entry | No explicit "exit calibration" affordance; Stream mode button stayed clickable mid-calibration; front-end console/image state (`consoleSince`, `lastLiveVersion`, etc.) was never reset on a fresh calibration session | done — hw-verified (exit via `set_state idle` cleanly returns to idle) |
| 7 | Streaming shows a flipped image, calibration's "current" picture looks normal | `cam_calib_enter_mode()` (`cam_calib.c`) calls `camera_init_ex()` directly and never calls `camera_overrides_apply()`, unlike `camera_init_streaming()` (`camera.c`) which does — so calibration mode's actual sensor registers never got the saved orientation override applied, while streaming mode did | done — hw-verified (register 0x3820/0x4514 in calibration mode now matches the 2 persisted overrides) |
| 8 | Calibration workflow needs operator-facing guidance (when to start/stop `cal auto 20`, `cal solve --save`, tune steps) | UX/docs gap — technical detail existed in `docs/camera_calibration_and_tuning.md` but nothing operator-facing was in the UI itself | done — copy revised per independent UI-friendliness review |
| — | End-to-end hardware test of all of the above | — | done — see hw-verified notes above; #1/#5/#6's UI-click paths were verified by JS syntax check + logic read + independent code review (no browser available in this environment for an actual click-through) |

**Independent review results:**
- Code-correctness review (fresh agent, full `git diff`): no bugs found. Explicitly ruled out
  5 things that looked suspicious at first glance (dropped `selected`/`value` HTML defaults,
  `tune notes <step>` command existing, the `CONFIG_CAMERA_DEFAULT_FRAMESIZE` fallback not
  masking errors, `camera_overrides_apply()` placement, and `poll()`/`pollCalib()` timer
  ordering) — see conversation for detail.
- UI-friendliness review (fresh agent, acting as a first-time operator): found 3 real gaps in
  the new calibration guidance copy, all fixed: the ⓘ info-button didn't say its output lands
  in the console below; the intrinsics paragraph didn't mention that some views get rejected
  (could read as a failure); the exit-calibration paragraph buried the consequence of not
  exiting cleanly. A 4th note (minor jargon: "intrinsic calibration", "accepted view") was
  judged acceptable given the surrounding concrete instructions, so left as-is.

See `docs/work_log.md` / git log for the detailed change notes once committed.
