#!/usr/bin/env python3
"""Standalone preview of the camera mount: robot_state_publisher + joint_state_publisher_gui
+ rviz2. No hardware involved -- drag the GUI sliders to sanity-check the URDF (joint
axes/limits, TF tree) before ever touching the real servos.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('camera_mount_description'), 'urdf', 'camera_mount.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])
