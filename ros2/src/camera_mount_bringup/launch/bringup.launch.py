#!/usr/bin/env python3
"""Real hardware + TF: camera_mount_bridge (talks to the servos) plus
robot_state_publisher (camera_mount_description's URDF), so /tf is available
alongside live control. rviz2 is optional (rviz:=true).
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('camera_mount_description'), 'urdf', 'camera_mount.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    device_arg = DeclareLaunchArgument('device', default_value='/dev/ttyUSB0')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')

    return LaunchDescription([
        device_arg,
        rviz_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='camera_mount_bridge',
            executable='bridge_node',
            output='screen',
            parameters=[{'device': LaunchConfiguration('device')}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
