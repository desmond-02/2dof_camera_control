#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Dynamixel driver for camera_mount_bridge. Deliberately independent from
# 2dof_camera_control/python/dxl_common.py + control.py (a project decision: this
# package stays self-contained for `colcon build`, no cross-repo pip dependency) --
# but the logic below is ported from those already-debugged versions, not
# reimplemented from scratch, since the angle/tick math in particular went through
# real bug fixes there (Homing Offset's +/-1024 tick limit in Position Control Mode,
# a Moving-flag race right after writing a new goal). The blocking "poll until
# settled" logic from control.py is NOT ported here: it doesn't apply to a ROS
# node -- joint_states is published continuously on a timer, so callers observe
# convergence over the topic instead of a single command blocking for it.

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PROTOCOL_VERSION = 2.0
BAUDRATE = 57600

ADDR_TORQUE_ENABLE        = 64
ADDR_GOAL_POSITION        = 116
ADDR_PRESENT_POSITION     = 132
ADDR_PRESENT_VOLTAGE      = 144
ADDR_HARDWARE_ERROR       = 70
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY     = 112

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

POSITION_MAX  = 4095   # one full revolution
ANGLE_MAX_DEG = 360.0


def connect(devicename, baudrate=BAUDRATE):
    port_handler = PortHandler(devicename)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        raise RuntimeError('Failed to open port %s' % devicename)
    if not port_handler.setBaudRate(baudrate):
        port_handler.closePort()
        raise RuntimeError('Failed to set baudrate %d' % baudrate)

    return port_handler, packet_handler


def check_hardware_error(port_handler, packet_handler, dxl_id, logger=None):
    """Log voltage + any latched hardware fault for dxl_id. Returns True if healthy."""
    voltage, comm_result, _ = packet_handler.read2ByteTxRx(port_handler, dxl_id, ADDR_PRESENT_VOLTAGE)
    if comm_result != COMM_SUCCESS:
        if logger:
            logger.error('[ID:%d] %s' % (dxl_id, packet_handler.getTxRxResult(comm_result)))
        return False
    if logger:
        logger.info('[ID:%d] Present Input Voltage: %.1f V' % (dxl_id, voltage / 10.0))

    hw_error, comm_result, _ = packet_handler.read1ByteTxRx(port_handler, dxl_id, ADDR_HARDWARE_ERROR)
    if comm_result != COMM_SUCCESS:
        if logger:
            logger.error('[ID:%d] %s' % (dxl_id, packet_handler.getTxRxResult(comm_result)))
        return False

    if hw_error != 0:
        if logger:
            logger.error('[ID:%d] Hardware Error Status = 0x%02X' % (dxl_id, hw_error))
        return False

    return True


def set_torque(port_handler, packet_handler, dxl_id, enable):
    comm_result, error = packet_handler.write1ByteTxRx(
        port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE if enable else TORQUE_DISABLE)
    if comm_result != COMM_SUCCESS:
        raise RuntimeError('[ID:%d] %s' % (dxl_id, packet_handler.getTxRxResult(comm_result)))
    if error != 0:
        raise RuntimeError('[ID:%d] %s' % (dxl_id, packet_handler.getRxPacketError(error)))


def read_present_position(port_handler, packet_handler, dxl_id):
    position, comm_result, error = packet_handler.read4ByteTxRx(port_handler, dxl_id, ADDR_PRESENT_POSITION)
    if comm_result != COMM_SUCCESS:
        raise RuntimeError('[ID:%d] %s' % (dxl_id, packet_handler.getTxRxResult(comm_result)))
    if error != 0:
        raise RuntimeError('[ID:%d] %s' % (dxl_id, packet_handler.getRxPacketError(error)))
    return position


def write_goal_position(port_handler, packet_handler, dxl_id, tick):
    comm_result, error = packet_handler.write4ByteTxRx(port_handler, dxl_id, ADDR_GOAL_POSITION, tick)
    if comm_result != COMM_SUCCESS:
        raise RuntimeError('[ID:%d] %s' % (dxl_id, packet_handler.getTxRxResult(comm_result)))
    if error != 0:
        raise RuntimeError('[ID:%d] %s' % (dxl_id, packet_handler.getRxPacketError(error)))


def angle_to_tick(joint, angle_deg):
    signed_angle = -angle_deg if joint['reversed'] else angle_deg
    tick = joint['center_tick'] + int(round(signed_angle / ANGLE_MAX_DEG * POSITION_MAX))
    return max(joint['min_limit'], min(joint['max_limit'], tick))


def tick_to_angle(joint, tick):
    signed_angle = (tick - joint['center_tick']) / POSITION_MAX * ANGLE_MAX_DEG
    return -signed_angle if joint['reversed'] else signed_angle


def joint_angle_range(joint):
    angle_at_min_tick = tick_to_angle(joint, joint['min_limit'])
    angle_at_max_tick = tick_to_angle(joint, joint['max_limit'])
    return min(angle_at_min_tick, angle_at_max_tick), max(angle_at_min_tick, angle_at_max_tick)
