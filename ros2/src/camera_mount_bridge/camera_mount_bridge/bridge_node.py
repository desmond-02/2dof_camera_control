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
import signal
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState

from camera_mount_bridge import dynamixel_io as dxl

RANGE_TOLERANCE_DEG = 0.05
# How far outside its calibrated range a joint may sit and still be trusted. A
# joint resting a little past a limit is normal (pitch drops onto its down stop,
# ~2 deg past max_limit, whenever torque is off); anything further means
# servos.json doesn't describe this hardware (swapped IDs, another robot's
# calibration) or a servo rebooted mid-session and lost its turn offset.
TRUSTED_RANGE_MARGIN_DEG = 15.0
# ~1 s of joint_states ticks at the default 30 Hz.
MAX_CONSECUTIVE_READ_FAILURES = 30


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
        # name -> whole-turn shift: calibrated tick = raw servo tick + turn_offsets[name]
        self.turn_offsets = {}
        self._read_failures = {name: 0 for name in self.joints}
        self.fault = None

        # Every servo must answer before any torque goes on -- at boot the U2D2 can
        # enumerate before the servos' 12 V supply is up. Raising here exits non-zero,
        # and the launch's on_exit=Shutdown plus comp-cap.service's Restart=always
        # retry until it is.
        for name, joint in self.joints.items():
            _, comm_result, _ = self.packet_handler.ping(self.port_handler, joint['id'])
            if comm_result != dxl.COMM_SUCCESS:
                self.port_handler.closePort()
                raise RuntimeError("'%s' (ID %d) not responding (%s) -- servo power off or bus disconnected?" %
                                   (name, joint['id'], self.packet_handler.getTxRxResult(comm_result)))

        for name, joint in self.joints.items():
            dxl_id = joint['id']
            if not dxl.check_hardware_error(self.port_handler, self.packet_handler, dxl_id, self.get_logger()):
                self.get_logger().error(
                    "'%s' (ID %d) has a hardware fault -- will not be commanded until resolved." % (name, dxl_id))
                continue

            # Extended Position Mode + a per-session turn offset, so a joint whose
            # travel straddles the encoder's 4095->0 wrap still moves the short way:
            # in single-turn Position Mode a wrapped reading (pitch on its down stop
            # reads ~5, not ~4101) can only reach the calibrated range by driving a
            # near-full turn through the hard stop. See dxl.turn_offset.
            if dxl.ensure_extended_position_mode(self.port_handler, self.packet_handler, dxl_id):
                self.get_logger().warn(
                    "'%s' (ID %d) switched to Extended Position Mode (one-time EEPROM change)" % (name, dxl_id))
            raw_tick = dxl.read_present_position(self.port_handler, self.packet_handler, dxl_id)
            offset = dxl.turn_offset(joint, raw_tick)
            low, high = dxl.joint_angle_range(joint)
            start_deg = dxl.tick_to_angle(joint, raw_tick + offset)
            if start_deg < low - TRUSTED_RANGE_MARGIN_DEG or start_deg > high + TRUSTED_RANGE_MARGIN_DEG:
                self.get_logger().error(
                    "'%s' (ID %d) starts at %.1f deg, more than %.0f deg outside [%.1f, %.1f] -- servos.json "
                    "doesn't match this hardware; leaving it limp." %
                    (name, dxl_id, start_deg, TRUSTED_RANGE_MARGIN_DEG, low, high))
                continue
            if offset:
                self.get_logger().info(
                    "'%s' (ID %d) raw tick %d is %+d turn(s) from its calibration; using tick %d" %
                    (name, dxl_id, raw_tick, offset // dxl.TICKS_PER_TURN, raw_tick + offset))
            self.turn_offsets[name] = offset

            self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id, dxl.ADDR_PROFILE_VELOCITY, profile_velocity)
            self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id, dxl.ADDR_PROFILE_ACCELERATION, profile_acceleration)

            dxl.set_torque(self.port_handler, self.packet_handler, dxl_id, True)
            self.enabled_joints.add(name)
            self.get_logger().info(
                "'%s' (ID %d) ready at %.1f deg. Allowed range: %.1f to %.1f deg" %
                (name, dxl_id, start_deg, low, high))

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
            dxl.write_goal_position(self.port_handler, self.packet_handler, dxl_id, home_tick - offset)

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
                dxl.write_goal_position(
                    self.port_handler, self.packet_handler, joint['id'], goal_tick - self.turn_offsets[name])
            except RuntimeError as e:
                self.get_logger().error(str(e))

    def publish_joint_states(self):
        if self.fault:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name in self.enabled_joints:
            joint = self.joints[name]
            try:
                tick = dxl.read_present_position(
                    self.port_handler, self.packet_handler, joint['id']) + self.turn_offsets[name]
            except RuntimeError as e:
                self.get_logger().error(str(e))
                self._read_failures[name] += 1
                if self._read_failures[name] >= MAX_CONSECUTIVE_READ_FAILURES:
                    self._fail("'%s' stopped responding" % name)
                    return
                continue
            self._read_failures[name] = 0
            angle_deg = dxl.tick_to_angle(joint, tick)
            low, high = dxl.joint_angle_range(joint)
            if angle_deg < low - TRUSTED_RANGE_MARGIN_DEG or angle_deg > high + TRUSTED_RANGE_MARGIN_DEG:
                self._fail("'%s' reads %.1f deg, far outside [%.1f, %.1f] -- servo rebooted (power blip?) "
                           "and lost its turn offset" % (name, angle_deg, low, high))
                return
            msg.name.append(name)
            msg.position.append(math.radians(angle_deg))
        self.state_pub.publish(msg)

    def _fail(self, reason):
        # Exit non-zero and let systemd restart the whole bring-up: a fresh start
        # re-checks the servos, re-derives turn offsets and re-homes -- the only
        # sound recovery once a servo has dropped off the bus or rebooted.
        self.fault = reason
        self.get_logger().error('%s -- shutting down so comp-cap.service restarts the bring-up' % reason)
        self.context.try_shutdown()

    def shutdown(self):
        # Idempotent (and thread-safe): called from context.on_shutdown() (the reliable
        # path -- rclpy invokes this as part of its own shutdown sequence, e.g. on
        # SIGINT) and again from main()'s finally as a backup.
        #
        # Ignore any further SIGINT from here on: Ctrl+C in a terminal reaches both
        # ros2 launch and this process, and launch then forwards its own -- that second
        # one used to land in the sleep below as a KeyboardInterrupt and skip
        # torque-off entirely, leaving the servos energized after every stop.
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except ValueError:
            pass  # not on the main thread; signal dispositions can only be set there
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
    try:
        node = CameraMountBridge()
    except Exception as e:
        # Expected at boot (U2D2 not enumerated yet, servo supply still off): one line
        # instead of a traceback, then a non-zero exit -- the launch's on_exit=Shutdown
        # and comp-cap.service's Restart=always retry until the hardware is there.
        get_logger('camera_mount_bridge').error(
            'Startup failed: %s: %s -- exiting so comp-cap.service retries' % (type(e).__name__, e))
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)
    node.context.on_shutdown(node.shutdown)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(1 if node.fault else 0)


if __name__ == '__main__':
    main()
