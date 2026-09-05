#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Shared constants and connection helpers for the 2-DOF camera mount (XC330-T288-T x2).

from dynamixel_sdk import *

PROTOCOL_VERSION = 2.0
DEVICENAME       = '/dev/ttyUSB1'
BAUDRATE         = 57600

ADDR_ID                 = 7
ADDR_HARDWARE_ERROR     = 70
ADDR_TORQUE_ENABLE      = 64
ADDR_PRESENT_VOLTAGE    = 144
ADDR_HOMING_OFFSET      = 20
ADDR_MIN_POSITION_LIMIT = 52
ADDR_MAX_POSITION_LIMIT = 48
ADDR_PRESENT_POSITION   = 132
ADDR_GOAL_POSITION      = 116
ADDR_MOVING             = 122

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

POSITION_MAX = 4095   # one full revolution
ANGLE_MAX    = 360.0
CENTER_TICK  = 2048   # midpoint of the position range


def connect(devicename=DEVICENAME, baudrate=BAUDRATE):
    portHandler = PortHandler(devicename)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        raise RuntimeError("Failed to open port %s" % devicename)
    if not portHandler.setBaudRate(baudrate):
        portHandler.closePort()
        raise RuntimeError("Failed to set baudrate %d" % baudrate)

    return portHandler, packetHandler


def check_hardware_error(portHandler, packetHandler, dxl_id):
    """Print voltage + any latched hardware fault for dxl_id. Returns True if healthy."""
    voltage, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_VOLTAGE)
    if dxl_comm_result != COMM_SUCCESS:
        print("[ID:%d] %s" % (dxl_id, packetHandler.getTxRxResult(dxl_comm_result)))
        return False
    print("[ID:%d] Present Input Voltage: %.1f V" % (dxl_id, voltage / 10.0))

    hw_error, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, dxl_id, ADDR_HARDWARE_ERROR)
    if dxl_comm_result != COMM_SUCCESS:
        print("[ID:%d] %s" % (dxl_id, packetHandler.getTxRxResult(dxl_comm_result)))
        return False

    if hw_error != 0:
        print("[ID:%d] Hardware Error Status = 0x%02X (nonzero)" % (dxl_id, hw_error))
        if hw_error & 0x01:
            print("  -> Input Voltage Error")
        if hw_error & 0x04:
            print("  -> Overheating Error")
        if hw_error & 0x08:
            print("  -> Motor Encoder Error")
        if hw_error & 0x10:
            print("  -> Electrical Shock Error")
        if hw_error & 0x20:
            print("  -> Overload Error")
        return False

    return True


def set_torque(portHandler, packetHandler, dxl_id, enable):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE if enable else TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError("[ID:%d] %s" % (dxl_id, packetHandler.getTxRxResult(dxl_comm_result)))
    if dxl_error != 0:
        raise RuntimeError("[ID:%d] %s" % (dxl_id, packetHandler.getRxPacketError(dxl_error)))


def read_present_position(portHandler, packetHandler, dxl_id):
    position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError("[ID:%d] %s" % (dxl_id, packetHandler.getTxRxResult(dxl_comm_result)))
    if dxl_error != 0:
        raise RuntimeError("[ID:%d] %s" % (dxl_id, packetHandler.getRxPacketError(dxl_error)))
    return position
