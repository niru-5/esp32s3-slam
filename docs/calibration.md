# Calibration of sensors for SLAM

## IMU bias/offset calibration (implemented)

Triggered by console command `4` on `data_capture` (`APP_STATE_IMU_CALIBRATION`,
`state_machine.c:imu_calibration_run()`), which calls
`imu_run_hw_foc_calibration()` (`firmware/data_capture/main/imu.c`). Procedure:

1. **Guided gravity-axis selection** (`state_machine.c:select_gravity_axis()`).
   `bmi2_perform_accel_foc` has no way to detect the rig's orientation itself
   — it needs to be told which axis (and sign) gravity is aligned with, and
   computes whatever offset makes *that* axis read exactly ±1g. So the
   console prints 10 raw accel+gyro samples over ~100ms, tells the operator
   whichever axis reads closest to ±1.0 is the one gravity is aligned with,
   and blocks for one digit (`1`=+X `2`=-X `3`=+Y `4`=-Y `5`=+Z `6`=-Z,
   anything else aborts before touching the chip). Get this wrong and FOC
   still "succeeds" — it just bakes in the wrong bias, silently, since the
   chip has no independent way to check the claimed orientation.
2. Capture ~`CONFIG_IMU_CALIBRATION_WINDOW_MS` of stationary samples (direct
   BMI270 polls, bypassing the streaming pipeline) and compute per-axis
   mean/stddev — the "before" snapshot. Stddev is a stationarity check: large
   values mean the rig moved during the window and the run should be
   discarded/redone.
3. Run the BMI270's built-in Fast Offset Compensation (`bmi2_perform_accel_foc`
   with the operator-selected gravity axis, and `bmi2_perform_gyro_foc`, which
   has no orientation dependency — ideal stationary reading is 0 on every
   axis). Both auto-enable hardware offset compensation as part of the
   routine, so every subsequent raw read — including the normal streaming
   pipeline — is corrected with no other firmware change.
4. `bmi2_nvm_prog()` persists the resulting offset registers into the BMI270's
   own NVM so they survive power-cycles. NVM write endurance is limited, so
   this is meant to be a rare, deliberate operation, not something run every
   boot — a persistent NVS counter tracks how many times it's been invoked on
   this chip and logs a warning past the first.
5. Capture another stationary window (same duration) — the "after" snapshot —
   so the before/after comparison shows whether the correction actually
   helped.

Output: the full report (run count, before/after mean+stddev per axis) is
logged over serial, and — if the SD card is available — written to
`/sdcard/imu_calibration_<timestamp>.txt` (`sdcard_write_imu_calibration_report()`).

Notes / known simplifications:
- Single-position FOC bias/offset only — the operator picks one axis per run
  (whichever currently reads ~1g), not a multi-position scale/misalignment
  (tumble) calibration.
- The BMI270 FOC targets a fixed factory-trimmed 1.000g on the selected axis;
  true local gravity (`CONFIG_LOCAL_GRAVITY_MPS2`, currently a rough Belgium
  estimate) is only used to annotate the report in physical units, not fed
  into the FOC call itself.
- Gyro FOC only needs stillness (no orientation dependency).

## Future work (not yet implemented)

These need long stationary logs (30min-3hr+) and are analysis, not something
the ESP32 itself needs to compute — the plan is to log raw stationary
accel/gyro data (via the existing STREAM_SDCARD or STREAM_WIFI sinks, not the
calibration routine above) and run the analysis offline in Python/ROS tooling
(e.g. `allan_variance_ros`, `imu_utils`, or a small custom script), the same
way camera calibration will lean on existing tools (Kalibr/ROS
camera_calibration) rather than reinventing them on-device.

- **White noise coefficients** (velocity/angle random walk, "N" term) —
  derived from the short-tau slope of the Allan deviation curve.
- **Bias instability / random walk coefficients** ("B"/"K" terms) — derived
  from the flat/long-tau region of the same curve.
- **Allan variance plots** — compute Allan deviation vs. cluster time from a
  long raw stationary log per axis (accel + gyro), plot log-log, and read off
  the above coefficients. Output format should end up compatible with
  whatever the SLAM pipeline expects (e.g. Kalibr's `imu.yaml` fields
  `accelerometer_noise_density` / `accelerometer_random_walk` /
  `gyroscope_noise_density` / `gyroscope_random_walk`).
- Multi-position/tumble accelerometer calibration (scale + misalignment, not
  just bias) if FOC-only bias turns out insufficient for SLAM accuracy.

## Camera calibration

Not started. `firmware/camera_calibration/` currently holds only target PDFs
(ChArUco/circles/Kalibr boards); `APP_STATE_CAMERA_CALIBRATION` in
`state_machine.c` remains a stub. Plan is to use existing tooling (Kalibr or
ROS `camera_calibration`) against captured frames rather than implementing
calibration logic on-device.
