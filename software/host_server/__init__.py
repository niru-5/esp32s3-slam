"""Host-side receiver for the ESP32-S3 visual-inertial capture rig.

Accepts the camera + IMU stream the firmware pushes over HTTP, records it to a
ROS 2 bag, and serves a live browser view. See ``README.md`` in ``software/``.
"""
