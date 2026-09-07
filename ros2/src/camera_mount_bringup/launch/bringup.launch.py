#!/usr/bin/env python3
"""Real hardware + TF: camera_mount_bridge (talks to the servos) plus
robot_state_publisher (camera_mount_description's URDF), so /tf is available
alongside live control. rviz2 is optional (rviz:=true).

Loads camera_mount_standalone.urdf.xacro (mount attached under a free-floating
"world" link) rather than camera_mount.urdf.xacro directly, since the latter is
now a bare xacro:macro meant to be included from a parent robot's URDF -- once
the mount is attached to the G1's own URDF, this launch file should load that
URDF instead.
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
    description_share = get_package_share_directory('camera_mount_description')
    xacro_path = os.path.join(description_share, 'urdf', 'camera_mount_standalone.urdf.xacro')
    rviz_config_path = os.path.join(description_share, 'rviz', 'camera_mount.rviz')
    robot_description = xacro.process_file(xacro_path).toxml()

    device_arg = DeclareLaunchArgument('device', default_value='/dev/dynamixel_pan_tilt')
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
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
