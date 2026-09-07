#!/usr/bin/env python3
"""Standalone preview of the camera mount: robot_state_publisher + joint_state_publisher_gui
+ rviz2. No hardware involved -- drag the GUI sliders to sanity-check the URDF (joint
axes/limits, TF tree) before ever touching the real servos.

Loads camera_mount_standalone.urdf.xacro, which instantiates the camera_mount macro
(camera_mount.urdf.xacro) under a free-floating "world" link. Once the mount is
attached to the G1's own URDF, that URDF includes camera_mount.urdf.xacro directly
and this standalone wrapper is only needed for bench testing like this.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('camera_mount_description')
    xacro_path = os.path.join(share_dir, 'urdf', 'camera_mount_standalone.urdf.xacro')
    rviz_config_path = os.path.join(share_dir, 'rviz', 'camera_mount.rviz')

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
            arguments=['-d', rviz_config_path],
        ),
    ])
