#!/usr/bin/env python3
"""Replay the bag AND run monocular visual odometry + IMU orientation, with the
resulting trajectory visualised live in RViz.

  ros2 launch software/slam_replay/launch/slam.launch.py
  ros2 launch software/slam_replay/launch/slam.launch.py bag:=/path/to/bag rate:=1.0

Starts everything replay.launch.py does, plus:
  * mono_vo_node.py         -> /vo/path, /vo/pose, /vo/features, TF map->camera
  * imu_orientation_node.py -> /imu/orientation, TF map->imu_link
  * rviz2 with the SLAM config (image + trajectory + TF)

Notes / honest limitations:
  * Monocular VO => trajectory scale is arbitrary (unit step per keyframe).
  * The rig is uncalibrated => intrinsics are estimated; pass real fx/fy/cx/cy
    via -p for better accuracy (see README).
  * Camera runs at ~2.3 fps, so large inter-frame motion sometimes breaks the
    essential-matrix estimate; those frames are skipped (see the inlier count
    drawn on /vo/features).
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
PKG = os.path.dirname(HERE)
SOFTWARE = os.path.dirname(PKG)
DEFAULT_BAG = os.path.join(SOFTWARE, 'bags', 'slam_20260704_172019')
RVIZ_CFG = os.path.join(PKG, 'rviz', 'slam.rviz')
VO_NODE = os.path.join(PKG, 'mono_vo_node.py')
IMU_NODE = os.path.join(PKG, 'imu_orientation_node.py')


def generate_launch_description():
    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value=DEFAULT_BAG,
                              description='rosbag2 directory to play'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='playback rate multiplier'),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate],
            output='screen'),

        Node(
            package='image_transport', executable='republish',
            name='image_republisher',
            parameters=[{'in_transport': 'compressed', 'out_transport': 'raw'}],
            remappings=[('in/compressed', '/camera/image_raw/compressed'),
                        ('out', '/camera/image_raw')]),

        ExecuteProcess(
            cmd=['python3', VO_NODE, '--ros-args',
                 '-p', 'use_sim_time:=true'],
            output='screen'),

        ExecuteProcess(
            cmd=['python3', IMU_NODE, '--ros-args',
                 '-p', 'use_sim_time:=true'],
            output='screen'),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', RVIZ_CFG],
            parameters=[{'use_sim_time': True}]),
    ])
