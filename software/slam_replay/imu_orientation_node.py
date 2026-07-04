#!/usr/bin/env python3
"""IMU orientation estimator for the ESP32-S3 rig.

Fuses the BMI270 gyro + accel from `/imu/data` with a complementary filter and
publishes the device orientation so it can be seen moving in RViz:

  /imu/orientation  geometry_msgs/PoseStamped
  TF: map -> imu_link

This is orientation only (roll/pitch/yaw drift-corrected by gravity); it does
NOT estimate position -- double-integrating this consumer-grade accel to a
position diverges in seconds. Position comes from the visual odometry node.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def normalize(q):
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([0.0, 0.0, 0.0, 1.0])


class ImuOrientation(Node):
    def __init__(self):
        super().__init__('imu_orientation')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('alpha', 0.98)   # gyro weight in the blend
        self.map_frame = self.get_parameter('map_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.alpha = self.get_parameter('alpha').value

        self.q = np.array([0.0, 0.0, 0.0, 1.0])  # x,y,z,w
        self.last_t = None

        self.pub = self.create_publisher(PoseStamped, '/imu/orientation', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.on_imu, 50)
        self.get_logger().info('imu_orientation running (complementary filter)')

    def on_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0 or dt > 0.5:
            return

        g = msg.angular_velocity
        wx, wy, wz = g.x, g.y, g.z
        # integrate gyro: dq = 0.5 * q (x) omega
        omega = np.array([wx, wy, wz, 0.0])
        self.q = normalize(self.q + 0.5 * quat_mul(self.q, omega) * dt)

        # gravity correction: tilt from accel, blended in
        a = msg.linear_acceleration
        av = np.array([a.x, a.y, a.z])
        n = np.linalg.norm(av)
        if 5.0 < n < 15.0:                     # plausibly ~1 g, not high accel
            ax, ay, az = av / n
            roll = np.arctan2(ay, az)
            pitch = np.arctan2(-ax, np.sqrt(ay * ay + az * az))
            cr, sr = np.cos(roll / 2), np.sin(roll / 2)
            cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
            q_acc = np.array([sr * cp, cr * sp, -sr * sp, cr * cp])
            self.q = normalize(self.alpha * self.q + (1 - self.alpha) * q_acc)

        self.publish(msg.header.stamp)

    def publish(self, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.imu_frame
        tf.transform.translation.x = 1.0   # offset so it doesn't overlap the VO path
        tf.transform.rotation.x = float(self.q[0])
        tf.transform.rotation.y = float(self.q[1])
        tf.transform.rotation.z = float(self.q[2])
        tf.transform.rotation.w = float(self.q[3])
        self.tf.sendTransform(tf)

        p = PoseStamped()
        p.header.stamp = stamp
        p.header.frame_id = self.map_frame
        p.pose.position.x = 1.0
        p.pose.orientation.x = float(self.q[0])
        p.pose.orientation.y = float(self.q[1])
        p.pose.orientation.z = float(self.q[2])
        p.pose.orientation.w = float(self.q[3])
        self.pub.publish(p)


def main():
    rclpy.init()
    node = ImuOrientation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
