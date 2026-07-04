#!/usr/bin/env python3
"""Replay the recorded bag and visualise the raw streams in RViz.

  ros2 launch software/slam_replay/launch/replay.launch.py
  ros2 launch software/slam_replay/launch/replay.launch.py bag:=/path/to/bag rate:=2.0

Starts:
  * ros2 bag play  (publishes /camera/image_raw/compressed + /imu/data on /clock)
  * image_transport republish  (compressed -> /camera/image_raw for RViz)
  * rviz2 with the replay config (camera image view)
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
RVIZ_CFG = os.path.join(PKG, 'rviz', 'replay.rviz')


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

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', RVIZ_CFG],
            parameters=[{'use_sim_time': True}]),
    ])
