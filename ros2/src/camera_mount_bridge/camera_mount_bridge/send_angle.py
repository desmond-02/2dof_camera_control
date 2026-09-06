#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# One-shot CLI: command a joint by degrees instead of radians.
# /joint_command itself stays radians (sensor_msgs/JointState, ROS convention) --
# this is just a human-friendly way to publish to it without hand-converting.
#
# Usage: ros2 run camera_mount_bridge send_angle <joint> <degrees> [<joint> <degrees> ...]
# Example: ros2 run camera_mount_bridge send_angle yaw 50 pitch -20

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

DISCOVERY_TIMEOUT_SEC = 5.0


def main(args=None):
    rclpy.init(args=args)
    argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]

    if len(argv) < 2 or len(argv) % 2 != 0:
        print("Usage: send_angle <joint> <degrees> [<joint> <degrees> ...]")
        print("Example: send_angle yaw 50 pitch -20")
        rclpy.shutdown()
        return 1

    names = argv[0::2]
    try:
        degrees = [float(d) for d in argv[1::2]]
    except ValueError:
        print("Degrees must be numbers.")
        rclpy.shutdown()
        return 1

    node = Node('send_angle')
    pub = node.create_publisher(JointState, 'joint_command', 10)

    # A publish before bridge_node's subscription is discovered is silently dropped
    # (volatile QoS, no late-joiner delivery) -- wait for a real subscriber first
    # rather than hoping a single "fire and forget" publish lands in time.
    deadline = time.time() + DISCOVERY_TIMEOUT_SEC
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if pub.get_subscription_count() == 0:
        node.get_logger().error(
            "No subscriber on 'joint_command' after %.0fs -- is bridge_node running?" %
            DISCOVERY_TIMEOUT_SEC)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = names
    msg.position = [math.radians(d) for d in degrees]
    pub.publish(msg)

    for name, deg, rad in zip(names, degrees, msg.position):
        node.get_logger().info("Sent '%s' -> %.1f deg (%.4f rad)" % (name, deg, rad))

    # Let the publish actually leave before the node (and its publisher) is torn down.
    rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
