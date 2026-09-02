# esp32s3-slam

An ESP32-S3-based visual-inertial SLAM rig, and — more broadly — a small,
self-contained camera + IMU capture module I want to reuse anywhere I need
synchronized image/motion data off a cheap microcontroller: as a sensor head
for other robotics projects, or mounted on my Sesame robot.

## Goals

1. **SLAM on commodity hardware.** Capture camera frames and IMU samples with
   reliable, jointly-referenced timestamps, calibrate the camera and IMU,
   stream/record everything to a ROS 2 bag, and run SLAM (currently a
   monocular visual-odometry pipeline, with fused VIO planned) offline with
   standard ROS tooling.
2. **A robust, portable capture module.** The firmware itself — camera + IMU
   sampling, timestamp alignment, WiFi streaming — is meant to stand on its
   own as a general-purpose data-logging front end, not just a SLAM-specific
   one-off. The intent is to be able to drop this onto other robots (starting
   with Sesame) as an off-the-shelf sensing module.
3. **Longer-term:** run a sparse/fast SLAM pipeline directly on the ESP32-S3,
   rather than only offline on a host machine.

The project is currently at the firmware + offline-pipeline stage: the
ESP32-S3 captures and streams data, a host-side Python server records it to a
ROS 2 bag, and a replay pipeline runs monocular VO against the recorded bag.

## Repo layout

- **`firmware/`** — ESP-IDF apps that run on the ESP32-S3.
  - `data_capture/` — the main app: OV5640 camera + BMI270 IMU + WiFi, streams
    both to a host over the network (HTTP or raw TCP) with samples timestamped
    on a shared clock.
  - `imu_testing/` — standalone BMI270 bring-up app, for debugging IMU wiring
    /config in isolation from the camera/WiFi stack.
  - `camera_calibration/` — calibration target PDFs (ChArUco, circles, Kalibr
    boards).
  - `esp-idf/` — vendored ESP-IDF SDK checkout (gitignored; a local toolchain
    install, not project source).
- **`software/`** — host-side tooling that runs off the board.
  - `host_server/` — receives the firmware's camera/IMU stream, records it to
    a ROS 2 bag, and serves a live browser view of the feed.
  - `slam_replay/` — replays a recorded bag, visualizes it in RViz, and runs a
    monocular visual-odometry pipeline against it.
  - `bags/` — recorded ROS 2 bags.
- **`hardware/`** — FreeCAD enclosure/mount designs for the board and camera
  (currently for the ESP32-S3 and Seeed XIAO ESP32-S3 form factors).
- **`SparkFun_BMI270_Arduino_Library/`** — vendored BMI270 driver; only the
  core driver files are compiled directly into the firmware apps.
- **`docs/`** — architecture notes, calibration notes, running work log, and
  ESP32-S3 reference datasheets.

See `CLAUDE.md` for build/flash/monitor commands and firmware architecture
details, and each subdirectory's own README for specifics.
