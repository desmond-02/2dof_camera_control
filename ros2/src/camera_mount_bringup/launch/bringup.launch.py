#!/usr/bin/env python3
"""Real hardware + TF: camera_mount_bridge (talks to the servos) plus
robot_state_publisher (camera_mount_description's URDF), so /tf is available
alongside live control. rviz2 is optional (rviz:=true).

Loads camera_mount_standalone.urdf.xacro (mount attached under a free-floating
"world" link) rather than camera_mount.urdf.xacro directly, since the latter is
now a bare xacro:macro meant to be included from a parent robot's URDF -- once
the mount is attached to the G1's own URDF, this launch file should load that
URDF instead.

Also starts neck_zmq_bridge (enable_neck_zmq_bridge:=false to skip -- e.g. a
dev machine with no g1_dataset_recorder to talk to), which exposes
/joint_states over ZMQ so the recorder's NeckZmqReader can see this mount
without any ROS2 dependency of its own. See neck_zmq_bridge.py for the wire
format and why the command direction stays passive for now.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('camera_mount_description')
    xacro_path = os.path.join(description_share, 'urdf', 'camera_mount_standalone.urdf.xacro')
    rviz_config_path = os.path.join(description_share, 'rviz', 'camera_mount.rviz')
    robot_description = xacro.process_file(xacro_path).toxml()

    device_arg = DeclareLaunchArgument('device', default_value='/dev/dynamixel_pan_tilt')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false')
    home_yaw_arg = DeclareLaunchArgument('home_yaw_deg', default_value='0.0')
    home_pitch_arg = DeclareLaunchArgument('home_pitch_deg', default_value='0.0')
    enable_neck_zmq_arg = DeclareLaunchArgument('enable_neck_zmq_bridge', default_value='true')
    neck_state_bind_host_arg = DeclareLaunchArgument('neck_state_bind_host', default_value='0.0.0.0')
    neck_state_bind_port_arg = DeclareLaunchArgument('neck_state_bind_port', default_value='5562')
    neck_cmd_host_arg = DeclareLaunchArgument('neck_cmd_host', default_value='192.168.123.222')
    neck_cmd_port_arg = DeclareLaunchArgument('neck_cmd_port', default_value='5556')

    return LaunchDescription([
        device_arg,
        rviz_arg,
        home_yaw_arg,
        home_pitch_arg,
        enable_neck_zmq_arg,
        neck_state_bind_host_arg,
        neck_state_bind_port_arg,
        neck_cmd_host_arg,
        neck_cmd_port_arg,
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
            parameters=[{
                'device': LaunchConfiguration('device'),
                'home_yaw_deg': ParameterValue(LaunchConfiguration('home_yaw_deg'), value_type=float),
                'home_pitch_deg': ParameterValue(LaunchConfiguration('home_pitch_deg'), value_type=float),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        Node(
            package='camera_mount_bridge',
            executable='neck_zmq_bridge',
            output='screen',
            parameters=[{
                'state_bind_host': LaunchConfiguration('neck_state_bind_host'),
                'state_bind_port': ParameterValue(LaunchConfiguration('neck_state_bind_port'), value_type=int),
                'cmd_host': LaunchConfiguration('neck_cmd_host'),
                'cmd_port': ParameterValue(LaunchConfiguration('neck_cmd_port'), value_type=int),
            }],
            condition=IfCondition(LaunchConfiguration('enable_neck_zmq_bridge')),
        ),
    ])
