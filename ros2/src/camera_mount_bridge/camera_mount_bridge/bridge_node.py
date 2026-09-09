#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS2 bridge for the 2-DOF (yaw/pitch) Dynamixel camera mount.
#
# Subscribes: joint_command (sensor_msgs/JointState) -- name + position (radians)
# Publishes:  joint_states  (sensor_msgs/JointState) -- current name + position, on a timer
#
# No URDF/TF dependency: this node only needs joint_states to exist on the topic;
# robot_state_publisher (added later, once the mount's geometry is available) is
# what turns that into TF, entirely independently of this node.

import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState

from camera_mount_bridge import dynamixel_io as dxl

RANGE_TOLERANCE_DEG = 0.05


class CameraMountBridge(Node):

    def __init__(self):
        super().__init__('camera_mount_bridge')
        self._shutdown_lock = threading.Lock()
        self._shutdown_done = False

        default_config = os.path.join(
            get_package_share_directory('camera_mount_bridge'), 'config', 'servos.json')

        self.declare_parameter('device', '/dev/dynamixel_pan_tilt')
        self.declare_parameter('baudrate', dxl.BAUDRATE)
        self.declare_parameter('servos_config_path', default_config)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('profile_velocity', 60)
        self.declare_parameter('profile_acceleration', 20)
        self.declare_parameter('home_yaw_deg', 0.0)
        self.declare_parameter('home_pitch_deg', 0.0)

        device = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value
        config_path = self.get_parameter('servos_config_path').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        profile_velocity = self.get_parameter('profile_velocity').value
        profile_acceleration = self.get_parameter('profile_acceleration').value
        home_angles_deg = {
            'yaw': self.get_parameter('home_yaw_deg').value,
            'pitch': self.get_parameter('home_pitch_deg').value,
        }

        with open(config_path) as f:
            self.joints = json.load(f)  # name -> {id, center_tick, min_limit, max_limit, reversed}

        self.port_handler, self.packet_handler = dxl.connect(device, baudrate)
        self.enabled_joints = set()

        for name, joint in self.joints.items():
            dxl_id = joint['id']
            if not dxl.check_hardware_error(self.port_handler, self.packet_handler, dxl_id, self.get_logger()):
                self.get_logger().error(
                    "'%s' (ID %d) has a hardware fault -- will not be commanded until resolved." % (name, dxl_id))
                continue

            self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id, dxl.ADDR_PROFILE_VELOCITY, profile_velocity)
            self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id, dxl.ADDR_PROFILE_ACCELERATION, profile_acceleration)

            dxl.set_torque(self.port_handler, self.packet_handler, dxl_id, True)
            self.enabled_joints.add(name)
            low, high = dxl.joint_angle_range(joint)
            self.get_logger().info(
                "'%s' (ID %d) ready. Allowed range: %.1f to %.1f deg" % (name, dxl_id, low, high))

            # Home to the configured position on startup (home_yaw_deg/home_pitch_deg,
            # default 0.0 = center) -- non-blocking, same as any other commanded move:
            # joint_states reports the approach as it happens, nothing here waits for it
            # to arrive. Out-of-range configured home falls back to center rather than
            # silently clamping to the limit, same reasoning as on_joint_command's warn.
            home_deg = home_angles_deg[name]
            if home_deg < low - RANGE_TOLERANCE_DEG or home_deg > high + RANGE_TOLERANCE_DEG:
                self.get_logger().error(
                    "Configured home for '%s' (%.1f deg) is out of range [%.1f, %.1f] -- homing to center instead" %
                    (name, home_deg, low, high))
                home_tick = joint['center_tick']
            else:
                home_tick = dxl.angle_to_tick(joint, home_deg)
            dxl.write_goal_position(self.port_handler, self.packet_handler, dxl_id, home_tick)

        self.command_sub = self.create_subscription(
            JointState, 'joint_command', self.on_joint_command, 10)
        self.state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_joint_states)

    def on_joint_command(self, msg):
        for name, position_rad in zip(msg.name, msg.position):
            if name not in self.enabled_joints:
                self.get_logger().warn("Ignoring command for unknown/disabled joint '%s'" % name)
                continue

            joint = self.joints[name]
            angle_deg = math.degrees(position_rad)
            low, high = dxl.joint_angle_range(joint)
            if angle_deg < low - RANGE_TOLERANCE_DEG or angle_deg > high + RANGE_TOLERANCE_DEG:
                self.get_logger().warn(
                    "'%s' commanded %.1f deg is out of range [%.1f, %.1f] -- ignored" %
                    (name, angle_deg, low, high))
                continue

            goal_tick = dxl.angle_to_tick(joint, angle_deg)
            try:
                dxl.write_goal_position(self.port_handler, self.packet_handler, joint['id'], goal_tick)
            except RuntimeError as e:
                self.get_logger().error(str(e))

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name in self.enabled_joints:
            joint = self.joints[name]
            try:
                tick = dxl.read_present_position(self.port_handler, self.packet_handler, joint['id'])
            except RuntimeError as e:
                self.get_logger().error(str(e))
                continue
            msg.name.append(name)
            msg.position.append(math.radians(dxl.tick_to_angle(joint, tick)))
        self.state_pub.publish(msg)

    def shutdown(self):
        # Idempotent (and thread-safe): called from context.on_shutdown() (the reliable
        # path -- rclpy invokes this as part of its own shutdown sequence, e.g. on
        # SIGINT) and again from main()'s finally as a backup.
        with self._shutdown_lock:
            if self._shutdown_done:
                return
            self._shutdown_done = True

            # SIGINT can interrupt a read/write mid-flight (e.g. the joint_states timer),
            # which leaves PortHandler.is_using stuck True forever -- it's only ever
            # reset on a call's normal completion path, which an interrupted call never
            # reaches. Every subsequent call, including this cleanup, would otherwise
            # fail immediately with "Port is in use!" against that stale flag. We're
            # intentionally reclaiming the port here, so clear it first.
            self.port_handler.is_using = False

            # Same interruption can also leave stale/partial bytes sitting in the OS
            # serial receive buffer (e.g. a status packet the timer's read was cut off
            # partway through). Left there, they corrupt the framing of the very next
            # read -- this cleanup's own torque-disable status-packet response -- which
            # surfaces as "Incorrect status packet!" rather than a clean result.
            #
            # A single immediate clear can still race a reply that was in flight on the
            # wire (request already sent, response not yet arrived) when SIGINT hit --
            # it can land microseconds after the clear and still corrupt the next read.
            # Give it a moment to finish arriving, then clear, and retry on failure
            # (clearing again each time) rather than accepting one attempt as final.
            time.sleep(0.05)
            self.port_handler.is_using = False
            self.port_handler.clearPort()

            for name in self.enabled_joints:
                dxl_id = self.joints[name]['id']
                for attempt in range(3):
                    try:
                        dxl.set_torque(self.port_handler, self.packet_handler, dxl_id, False)
                        break
                    except Exception as e:
                        if attempt == 2:
                            self.get_logger().error(
                                "Could not confirm torque disabled for '%s': %s" % (name, e))
                        else:
                            self.port_handler.is_using = False
                            self.port_handler.clearPort()
                            time.sleep(0.02)
            self.port_handler.closePort()


def main(args=None):
    rclpy.init(args=args)
    node = CameraMountBridge()
    node.context.on_shutdown(node.shutdown)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
