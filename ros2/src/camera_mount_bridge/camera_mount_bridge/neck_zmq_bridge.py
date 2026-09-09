#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS2 <-> ZMQ bridge so g1_dataset_recorder's NeckZmqReader (readers/neck_zmq.py)
# can see this mount without knowing anything about ROS2.
#
# Direction 1 (state, always on): camera_mount_bridge's /joint_states (measured
# yaw/pitch, radians) -> ZMQ PUB, bound on this robot, topic "neck_state". The
# workstation recorder connects with --neck-state-zmq-host <this robot's IP>
# --neck-state-zmq-port <state_bind_port> and fills states/neck/joint_position.
#
# Direction 2 (command, passive): a ZMQ SUB connected to wherever a future
# neck-commander publishes (default: the same host:port g1_dataset_recorder's
# PicoVRReader already connects to on this robot -- see relay logs, "SONIC neck_cmd"
# in record.py's --neck-cmd-zmq-host help), topic "neck_cmd" -> republished onto
# /joint_command so camera_mount_bridge actually moves. Nothing publishes this
# topic yet (no component drives the neck in real time as of 2026-09), so this
# side sits idle until one exists -- wiring it now means g1_dataset_recorder's
# states/neck/cmd_position will start reflecting real commands the moment a
# publisher shows up, no recorder-side changes needed.
#
# Wire format matches NeckZmqReader._decode_sample exactly: raw bytes = topic
# name, optionally followed by a single space, then a msgpack- or JSON-encoded
# payload. Publishing plain JSON (no msgpack dependency needed here) is enough --
# NeckZmqReader's msgpack.unpackb attempt fails safely on JSON bytes and it falls
# through to json.loads. Payload shape: {"yaw": <rad>, "pitch": <rad>,
# "timestamp_monotonic": <float>} -- _extract_neck_values checks top-level
# yaw/pitch keys when none of neck/neck_cmd/neck_state/action.neck/
# observation.state.neck are present, and timestamp_monotonic is one of the
# sender-time keys it looks for.

import json
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import zmq

try:
    import msgpack
except ImportError:  # pragma: no cover - optional, JSON works without it
    msgpack = None


class NeckZmqBridge(Node):

    def __init__(self):
        super().__init__('neck_zmq_bridge')

        self.declare_parameter('state_bind_host', '0.0.0.0')
        self.declare_parameter('state_bind_port', 5562)
        self.declare_parameter('state_topic', 'neck_state')
        self.declare_parameter('cmd_host', '192.168.123.222')
        self.declare_parameter('cmd_port', 5556)
        self.declare_parameter('cmd_topic', 'neck_cmd')

        state_bind_host = self.get_parameter('state_bind_host').value
        state_bind_port = self.get_parameter('state_bind_port').value
        self._state_topic = self.get_parameter('state_topic').value.encode('utf-8')
        cmd_host = self.get_parameter('cmd_host').value
        cmd_port = self.get_parameter('cmd_port').value
        self._cmd_topic = self.get_parameter('cmd_topic').value
        cmd_topic_bytes = self._cmd_topic.encode('utf-8')

        self._zmq_ctx = zmq.Context.instance()

        self._state_pub_lock = threading.Lock()
        self._state_pub = self._zmq_ctx.socket(zmq.PUB)
        self._state_pub.bind(f'tcp://{state_bind_host}:{state_bind_port}')
        self.get_logger().info(
            "Publishing neck_state on tcp://%s:%s (topic=%s)" %
            (state_bind_host, state_bind_port, self._state_topic.decode()))

        self.joint_command_pub = self.create_publisher(JointState, 'joint_command', 10)
        self.create_subscription(JointState, 'joint_states', self._on_joint_states, 10)

        self._cmd_sub = self._zmq_ctx.socket(zmq.SUB)
        self._cmd_sub.setsockopt(zmq.SUBSCRIBE, cmd_topic_bytes)
        self._cmd_sub.setsockopt(zmq.CONFLATE, 1)
        self._cmd_sub.setsockopt(zmq.RCVTIMEO, 200)
        self._cmd_sub.connect(f'tcp://{cmd_host}:{cmd_port}')
        self.get_logger().info(
            "Listening for neck_cmd on tcp://%s:%s (topic=%s) -- idle until a "
            "publisher exists" % (cmd_host, cmd_port, self._cmd_topic))

        self._stop = threading.Event()
        self._cmd_thread = threading.Thread(target=self._run_cmd_listener, daemon=True)
        self._cmd_thread.start()

    def _on_joint_states(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if 'yaw' not in positions or 'pitch' not in positions:
            return
        stamp = msg.header.stamp
        sender_time = stamp.sec + stamp.nanosec * 1e-9
        payload = json.dumps({
            'yaw': positions['yaw'],
            'pitch': positions['pitch'],
            'timestamp_monotonic': sender_time,
        }).encode('utf-8')
        with self._state_pub_lock:
            self._state_pub.send(self._state_topic + b' ' + payload)

    def _run_cmd_listener(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self._cmd_sub.recv()
            except zmq.Again:
                continue
            except zmq.ZMQError as exc:
                self.get_logger().warn('neck_cmd recv error: %s' % exc)
                continue
            decoded = self._decode_cmd(raw)
            if decoded is None:
                continue
            yaw, pitch = decoded
            out = JointState()
            out.header.stamp = self.get_clock().now().to_msg()
            out.name = ['yaw', 'pitch']
            out.position = [float(yaw), float(pitch)]
            self.joint_command_pub.publish(out)

    def _decode_cmd(self, raw: bytes):
        topic_bytes = self._cmd_topic.encode('utf-8')
        payload = raw
        if payload.startswith(topic_bytes):
            payload = payload[len(topic_bytes):]
        if payload.startswith(b' '):
            payload = payload[1:]
        if not payload:
            return None

        decoded = None
        if msgpack is not None:
            try:
                decoded = msgpack.unpackb(payload, raw=False)
            except Exception:
                decoded = None
        if decoded is None:
            try:
                decoded = json.loads(payload.decode('utf-8'))
            except Exception:
                return None

        if isinstance(decoded, dict):
            value = (
                decoded.get('neck')
                or decoded.get('neck_cmd')
                or decoded.get('action.neck')
            )
            if value is None and 'yaw' in decoded and 'pitch' in decoded:
                value = [decoded['yaw'], decoded['pitch']]
        else:
            value = decoded

        if value is None or len(value) < 2:
            return None
        return float(value[0]), float(value[1])

    def shutdown(self) -> None:
        self._stop.set()
        if self._cmd_thread.is_alive():
            self._cmd_thread.join(timeout=1.0)
        self._cmd_sub.close(linger=0)
        with self._state_pub_lock:
            self._state_pub.close(linger=0)


def main(args=None):
    rclpy.init(args=args)
    node = NeckZmqBridge()
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
