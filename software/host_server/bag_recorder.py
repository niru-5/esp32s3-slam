"""Record incoming frames + IMU samples to a ROS 2 bag (rosbag2).

Uses the native ``rosbag2_py`` writer and ``sensor_msgs`` types from a sourced
ROS 2 environment (developed against Jazzy). If those imports fail — e.g. the
server is run without ``source /opt/ros/<distro>/setup.bash`` — recording is
disabled and the server still runs as a live viewer.

Topics written:
    /camera/image_raw/compressed   sensor_msgs/msg/CompressedImage  (jpeg)
    /imu/data                      sensor_msgs/msg/Imu

Timestamps
----------
The firmware stamps every frame and IMU sample with esp_timer microseconds on
one shared clock. We keep that relative timing intact and anchor it to wall
time using the first timestamp seen across *either* stream:

    ros_ns = t0_wall_ns + (esp_us - t0_esp_us) * 1000

so camera and IMU stay mutually synchronised in the bag.
"""

from __future__ import annotations

import math
import threading
import time

# Standard gravity / deg->rad, to emit IMU in SI units as ROS expects.
_G_TO_MS2 = 9.80665
_DPS_TO_RADS = math.pi / 180.0

IMAGE_TOPIC = "/camera/image_raw/compressed"
IMU_TOPIC = "/imu/data"

try:  # ROS 2 (sourced environment) — optional.
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CompressedImage, Imu
    _ROS_AVAILABLE = True
    _ROS_IMPORT_ERROR = ""
except Exception as exc:  # pragma: no cover - depends on environment
    _ROS_AVAILABLE = False
    _ROS_IMPORT_ERROR = str(exc)


def ros_available() -> bool:
    return _ROS_AVAILABLE


def ros_import_error() -> str:
    return _ROS_IMPORT_ERROR


def _stamp_from_ns(msg_stamp, ns: int) -> None:
    msg_stamp.sec = ns // 1_000_000_000
    msg_stamp.nanosec = ns % 1_000_000_000


class BagRecorder:
    """Thread-safe wrapper around a rosbag2 SequentialWriter."""

    def __init__(self, bag_uri: str, storage_id: str = "sqlite3",
                 image_frame_id: str = "camera", imu_frame_id: str = "imu") -> None:
        self.enabled = _ROS_AVAILABLE
        self.bag_uri = bag_uri
        self._image_frame_id = image_frame_id
        self._imu_frame_id = imu_frame_id
        self._lock = threading.Lock()
        self._t0_esp_us: int | None = None
        self._t0_wall_ns: int = 0
        self._writer = None
        self._next_topic_id = 0

        if not self.enabled:
            return

        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(
            rosbag2_py.StorageOptions(uri=bag_uri, storage_id=storage_id),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        self._create_topic(IMAGE_TOPIC, "sensor_msgs/msg/CompressedImage")
        self._create_topic(IMU_TOPIC, "sensor_msgs/msg/Imu")

    def _create_topic(self, name: str, msg_type: str) -> None:
        # Jazzy's TopicMetadata takes a leading integer id.
        meta = rosbag2_py.TopicMetadata(
            id=self._next_topic_id, name=name, type=msg_type,
            serialization_format="cdr",
        )
        self._next_topic_id += 1
        self._writer.create_topic(meta)

    def _ros_ns(self, esp_us: int) -> int:
        """Map an esp_timer microsecond stamp to wall-anchored ROS nanoseconds."""
        if self._t0_esp_us is None:
            self._t0_esp_us = esp_us
            self._t0_wall_ns = time.time_ns()
        return self._t0_wall_ns + (esp_us - self._t0_esp_us) * 1000

    def write_frame(self, esp_us: int, jpeg: bytes) -> None:
        if not self.enabled:
            return
        with self._lock:
            ns = self._ros_ns(esp_us)
            msg = CompressedImage()
            _stamp_from_ns(msg.header.stamp, ns)
            msg.header.frame_id = self._image_frame_id
            msg.format = "jpeg"
            msg.data = jpeg if isinstance(jpeg, bytes) else bytes(jpeg)
            self._writer.write(IMAGE_TOPIC, serialize_message(msg), ns)

    def write_imu(self, sample) -> None:
        if not self.enabled:
            return
        with self._lock:
            ns = self._ros_ns(sample.esp_us)
            msg = Imu()
            _stamp_from_ns(msg.header.stamp, ns)
            msg.header.frame_id = self._imu_frame_id
            msg.linear_acceleration.x = sample.ax * _G_TO_MS2
            msg.linear_acceleration.y = sample.ay * _G_TO_MS2
            msg.linear_acceleration.z = sample.az * _G_TO_MS2
            msg.angular_velocity.x = sample.gx * _DPS_TO_RADS
            msg.angular_velocity.y = sample.gy * _DPS_TO_RADS
            msg.angular_velocity.z = sample.gz * _DPS_TO_RADS
            # No orientation estimate from a raw IMU — flag it per REP-145.
            msg.orientation_covariance[0] = -1.0
            self._writer.write(IMU_TOPIC, serialize_message(msg), ns)

    def close(self) -> None:
        with self._lock:
            if self._writer is not None:
                # SequentialWriter finalizes on destruction; drop the ref.
                del self._writer
                self._writer = None
