#!/usr/bin/env python3
"""Monocular visual-odometry node for the ESP32-S3 SLAM rig.

Subscribes to the camera stream recorded in the bag
(`/camera/image_raw/compressed`, JPEG 640x480), runs a classic ORB feature
odometry pipeline (detect -> match -> essential matrix -> recover pose) and
publishes an incremental trajectory that can be visualised in RViz:

  /vo/path      nav_msgs/Path        camera trajectory (grows over time)
  /vo/pose      geometry_msgs/PoseStamped   latest camera pose
  /vo/features  sensor_msgs/Image    debug image with tracked keypoints drawn
  TF: map -> camera_optical          latest pose, so an Axes/Camera moves in 3D

Because this is a *monocular* camera the absolute scale of the translation is
unobservable: each step is normalised to unit length, so the trajectory shape
is meaningful but its metric size is arbitrary. The rig is also uncalibrated,
so the pinhole intrinsics are an *estimate* from the image size and an assumed
field of view. Override fx/fy/cx/cy with real calibration for better results.

No external SLAM package is required -- only OpenCV + cv_bridge, which ship
with the ROS 2 desktop install.
"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster


def rot_to_quat(R):
    """3x3 rotation matrix -> (x, y, z, w) quaternion."""
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class MonoVO(Node):
    def __init__(self):
        super().__init__('mono_vo')

        # --- parameters ------------------------------------------------------
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('compressed', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_optical')
        # Uncalibrated pinhole estimate: 640x480, assumed ~60 deg horizontal FOV
        # -> f = (W/2) / tan(FOV/2) ~= 550 px. Override with real calibration.
        self.declare_parameter('fx', 550.0)
        self.declare_parameter('fy', 550.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('orb_features', 1500)
        self.declare_parameter('min_matches', 15)
        self.declare_parameter('min_inliers', 12)

        gp = self.get_parameter
        self.map_frame = gp('map_frame').value
        self.cam_frame = gp('camera_frame').value
        self.compressed = gp('compressed').value
        self.min_matches = gp('min_matches').value
        self.min_inliers = gp('min_inliers').value
        self.K = np.array([[gp('fx').value, 0.0, gp('cx').value],
                           [0.0, gp('fy').value, gp('cy').value],
                           [0.0, 0.0, 1.0]])

        self.orb = cv2.ORB_create(int(gp('orb_features').value))
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.bridge = CvBridge()

        # --- accumulated pose (world = first frame's optical frame) ----------
        self.R_wc = np.eye(3)
        self.t_wc = np.zeros((3, 1))
        self.prev_kp = None
        self.prev_des = None
        self.n_updates = 0
        self.n_frames = 0

        # --- ROS I/O ---------------------------------------------------------
        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.pub_path = self.create_publisher(Path, '/vo/path', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/vo/pose', 10)
        self.pub_feat = self.create_publisher(Image, '/vo/features', 5)
        self.tf = TransformBroadcaster(self)

        topic = gp('image_topic').value
        if self.compressed:
            self.create_subscription(CompressedImage, topic, self.on_compressed, 10)
        else:
            self.create_subscription(Image, topic, self.on_raw, 10)
        self.get_logger().info(
            f'mono_vo listening on {topic} '
            f'({"compressed" if self.compressed else "raw"}), '
            f'fx={self.K[0,0]:.1f} cx={self.K[0,2]:.1f}')

    # -- image callbacks -----------------------------------------------------
    def on_compressed(self, msg: CompressedImage):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        self.process(img, msg.header.stamp)

    def on_raw(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process(img, msg.header.stamp)

    # -- core VO -------------------------------------------------------------
    def process(self, bgr, stamp):
        self.n_frames += 1
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        kp, des = self.orb.detectAndCompute(gray, None)

        drawn = 0
        if (self.prev_des is not None and des is not None
                and len(kp) > 8 and len(self.prev_kp) > 8):
            matches = self.bf.match(self.prev_des, des)
            matches = sorted(matches, key=lambda m: m.distance)[:400]
            if len(matches) >= self.min_matches:
                p_prev = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
                p_cur = np.float32([kp[m.trainIdx].pt for m in matches])
                E, mask = cv2.findEssentialMat(
                    p_cur, p_prev, self.K, cv2.RANSAC, 0.999, 1.0)
                if E is not None and E.shape == (3, 3):
                    _, R, t, pose_mask = cv2.recoverPose(
                        E, p_cur, p_prev, self.K, mask=mask)
                    inliers = int(pose_mask.sum())
                    if inliers >= self.min_inliers:
                        # compose: world pose of the new camera
                        self.t_wc = self.t_wc + self.R_wc @ t
                        self.R_wc = R @ self.R_wc
                        self.n_updates += 1
                    # draw inlier matches on the debug image
                    for i, m in enumerate(matches):
                        if pose_mask[i]:
                            x, y = kp[m.trainIdx].pt
                            cv2.circle(bgr, (int(x), int(y)), 3, (0, 255, 0), 1)
                            drawn += 1

        self.prev_kp, self.prev_des = kp, des
        self.publish(stamp)
        self.publish_debug(bgr, stamp, drawn)

    # -- publishers ----------------------------------------------------------
    def publish(self, stamp):
        qx, qy, qz, qw = rot_to_quat(self.R_wc.T)   # camera orientation in world
        c = (-self.R_wc.T @ self.t_wc).flatten()      # camera centre in world

        tfmsg = TransformStamped()
        tfmsg.header.stamp = stamp
        tfmsg.header.frame_id = self.map_frame
        tfmsg.child_frame_id = self.cam_frame
        tfmsg.transform.translation.x = float(c[0])
        tfmsg.transform.translation.y = float(c[1])
        tfmsg.transform.translation.z = float(c[2])
        tfmsg.transform.rotation.x = qx
        tfmsg.transform.rotation.y = qy
        tfmsg.transform.rotation.z = qz
        tfmsg.transform.rotation.w = qw
        self.tf.sendTransform(tfmsg)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = float(c[0])
        pose.pose.position.y = float(c[1])
        pose.pose.position.z = float(c[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pub_pose.publish(pose)

        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        self.pub_path.publish(self.path)

    def publish_debug(self, bgr, stamp, n_inliers):
        cv2.putText(bgr, f'inliers:{n_inliers} updates:{self.n_updates}/{self.n_frames}',
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        img = self.bridge.cv2_to_imgmsg(bgr, 'bgr8')
        img.header.stamp = stamp
        img.header.frame_id = self.cam_frame
        self.pub_feat.publish(img)


def main():
    rclpy.init()
    node = MonoVO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'VO done: {node.n_updates} pose updates over {node.n_frames} frames')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
