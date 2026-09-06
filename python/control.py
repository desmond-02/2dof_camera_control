#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Interactive control for the 2-DOF camera mount, using config/servos.json
# (written by calibrate.py) to map degrees-from-center to each servo's raw ticks.
#
# Usage: <joint> <angle_degrees>   e.g.  "yaw 30" / "y 30"   or  "pitch -15" / "p -15"
#        "center" or "c" moves both joints to 0
#        "q" quits

import json
import os
import time

from dynamixel_sdk import *
import dxl_common as dxl

CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'config', 'servos.json')

PROFILE_VELOCITY     = 60   # moderate speed, in the servo's own velocity units
PROFILE_ACCELERATION = 20

ADDR_PROFILE_VELOCITY     = 112
ADDR_PROFILE_ACCELERATION = 108

POLL_TIMEOUT_LOOPS  = 200
POLL_INTERVAL_SEC   = 0.02
SETTLE_THRESHOLD_TICKS = 8   # ~0.7 deg, loosened from 4 since a loaded joint (pitch, holding
                             # the camera against gravity) may have a small persistent steady-
                             # state error that never quite closes to within a tighter tolerance
RANGE_DISPLAY_TOLERANCE_DEG = 0.05   # matches the rounding of the printed "%.1f" range


def load_config():
    with open(CONFIG_PATH) as f:
        return json.load(f)


def angle_to_tick(joint, angle_deg):
    signed_angle = -angle_deg if joint['reversed'] else angle_deg
    tick = joint['center_tick'] + int(round(signed_angle / dxl.ANGLE_MAX * dxl.POSITION_MAX))
    return max(joint['min_limit'], min(joint['max_limit'], tick))


def tick_to_angle(joint, tick):
    signed_angle = (tick - joint['center_tick']) / dxl.POSITION_MAX * dxl.ANGLE_MAX
    return -signed_angle if joint['reversed'] else signed_angle


def joint_angle_range(joint):
    angle_at_min_tick = tick_to_angle(joint, joint['min_limit'])
    angle_at_max_tick = tick_to_angle(joint, joint['max_limit'])
    return min(angle_at_min_tick, angle_at_max_tick), max(angle_at_min_tick, angle_at_max_tick)


config = load_config()
# Single-letter shortcuts, e.g. 'y' -> 'yaw', 'p' -> 'pitch' (falls back to the full name if
# two joints happen to share a first letter, so nothing is silently ambiguous).
joint_aliases = {}
for name in config:
    joint_aliases.setdefault(name[0].lower(), name)

portHandler, packetHandler = dxl.connect()
enabled_ids = []

try:
    for name, joint in config.items():
        dxl_id = joint['id']
        if not dxl.check_hardware_error(portHandler, packetHandler, dxl_id):
            print("Resolve the fault on '%s' (ID %d) before continuing." % (name, dxl_id))
            raise SystemExit

        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION)

        dxl.set_torque(portHandler, packetHandler, dxl_id, True)
        enabled_ids.append(dxl_id)
        low, high = joint_angle_range(joint)
        print("'%s' (ID %d) ready. Allowed range: %.1f to %.1f deg" % (name, dxl_id, low, high))

    alias_hint = ", ".join("'%s' for %s" % (alias, name) for alias, name in joint_aliases.items())
    print("\nCommands: '<joint> <angle>' e.g. 'yaw 30' or 'y 30' (%s), 'center' or 'c' for both to 0, 'q' to quit." %
          alias_hint)

    while True:
        line = input("> ").strip()
        if not line:
            continue
        if line.lower() == 'q':
            break

        if line.lower() in ('center', 'c'):
            targets = [(name, 0.0) for name in config]
        else:
            parts = line.split()
            joint_name = joint_aliases.get(parts[0].lower(), parts[0])
            if len(parts) != 2 or joint_name not in config:
                print("Unrecognized command. Try '<joint> <angle>', 'center'/'c', or 'q'. Known joints: %s" %
                      list(config.keys()))
                continue
            try:
                targets = [(joint_name, float(parts[1]))]
            except ValueError:
                print("Angle must be a number.")
                continue

        valid_targets = []
        for name, angle_deg in targets:
            joint = config[name]
            low, high = joint_angle_range(joint)
            if angle_deg < low - RANGE_DISPLAY_TOLERANCE_DEG or angle_deg > high + RANGE_DISPLAY_TOLERANCE_DEG:
                print("[%s] %.1f deg is out of range. Please input between %.1f and %.1f deg." %
                      (name, angle_deg, low, high))
                continue
            valid_targets.append((name, angle_deg))

        if not valid_targets:
            continue
        targets = valid_targets

        for name, angle_deg in targets:
            joint = config[name]
            dxl_id = joint['id']
            goal_tick = angle_to_tick(joint, angle_deg)

            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                portHandler, dxl_id, dxl.ADDR_GOAL_POSITION, goal_tick)
            if dxl_comm_result != COMM_SUCCESS:
                print("[%s] %s" % (name, packetHandler.getTxRxResult(dxl_comm_result)))
                continue
            elif dxl_error != 0:
                print("[%s] %s" % (name, packetHandler.getRxPacketError(dxl_error)))
                continue

        for name, angle_deg in targets:
            joint = config[name]
            dxl_id = joint['id']
            goal_tick = angle_to_tick(joint, angle_deg)

            for _ in range(POLL_TIMEOUT_LOOPS):
                present_tick = dxl.read_present_position(portHandler, packetHandler, dxl_id)
                if abs(goal_tick - present_tick) <= SETTLE_THRESHOLD_TICKS:
                    break
                time.sleep(POLL_INTERVAL_SEC)
            else:
                remaining_deg = tick_to_angle(joint, present_tick) - tick_to_angle(joint, goal_tick)
                print("[%s] timed out waiting to reach goal (still %.1f deg / %d ticks away, at %.1f deg)" %
                      (name, abs(remaining_deg), abs(goal_tick - present_tick), tick_to_angle(joint, present_tick)))

finally:
    for dxl_id in enabled_ids:
        try:
            dxl.set_torque(portHandler, packetHandler, dxl_id, False)
        except Exception as e:
            print("Could not confirm torque was disabled for ID %d: %s" % (dxl_id, e))
    portHandler.closePort()
    print("Torque disabled, port closed.")
