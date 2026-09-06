#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Interactive calibration for one servo/joint of the 2-DOF camera mount:
#   - records the raw tick of the pose you choose as "center" (center is tracked in
#     software/config, not via the servo's Homing Offset register — Homing Offset is
#     capped at +/-1024 ticks (~90 deg) in Position Control Mode and silently ignored
#     beyond that, per Robotis's control table docs, which is too small for some joints)
#   - records the safe min/max travel range you demonstrate by hand
# Run once per joint (yaw, pitch). Torque is disabled throughout so you can move it by hand.

import json
import os

from dynamixel_sdk import *
import dxl_common as dxl

CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'config', 'servos.json')


def load_config():
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH) as f:
            return json.load(f)
    return {}


def save_config(config):
    os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
    with open(CONFIG_PATH, 'w') as f:
        json.dump(config, f, indent=2)


def wait_for_enter(prompt):
    input(prompt)


portHandler, packetHandler = dxl.connect()
dxl_id = None

try:
    joint_name = input("Joint name for this servo (e.g. yaw / pitch): ").strip()
    dxl_id = int(input("Servo ID: ").strip())

    if not dxl.check_hardware_error(portHandler, packetHandler, dxl_id):
        print("Resolve the fault above before calibrating.")
        raise SystemExit

    dxl.set_torque(portHandler, packetHandler, dxl_id, False)
    print("Torque disabled — you can now move the joint freely by hand.")

    # Reset any leftover Homing Offset from earlier attempts (Wizard or a previous
    # version of this script) so Present Position purely reflects the raw encoder,
    # and "center" is tracked as an explicit tick value in config instead.
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, dxl.ADDR_HOMING_OFFSET, 0)
    if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
        print("Failed to reset Homing Offset")
        raise SystemExit

    # --- Center calibration ---
    wait_for_enter("Move the mount to the CENTER pose you want (camera facing forward / level), then press Enter...")
    center_tick = dxl.read_present_position(portHandler, packetHandler, dxl_id)
    print("Center recorded at raw tick %d." % center_tick)

    # --- Travel range (Min/Max Position Limit) ---
    wait_for_enter("Move the joint to one SAFE travel limit, then press Enter...")
    raw_limit_a = dxl.read_present_position(portHandler, packetHandler, dxl_id)

    wait_for_enter("Move the joint to the OTHER safe travel limit, then press Enter...")
    raw_limit_b = dxl.read_present_position(portHandler, packetHandler, dxl_id)

    min_limit = min(raw_limit_a, raw_limit_b)
    max_limit = max(raw_limit_a, raw_limit_b)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, dxl.ADDR_MIN_POSITION_LIMIT, min_limit)
    if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
        print("Failed to write Min Position Limit")
        raise SystemExit

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, dxl.ADDR_MAX_POSITION_LIMIT, max_limit)
    if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
        print("Failed to write Max Position Limit")
        raise SystemExit

    print("Position limits set: min=%d, max=%d" % (min_limit, max_limit))

    # --- Direction (handled in software, not firmware, so it's easy to flip later) ---
    reversed_input = input(
        "With the mount at center, does commanding a HIGHER angle move it in the direction "
        "you want to call positive? (y/n): ").strip().lower()
    reversed_axis = reversed_input == 'n'

    config = load_config()
    config[joint_name] = {
        'id': dxl_id,
        'center_tick': center_tick,
        'min_limit': min_limit,
        'max_limit': max_limit,
        'reversed': reversed_axis,
    }
    save_config(config)
    print("Saved calibration for '%s' to %s" % (joint_name, CONFIG_PATH))

finally:
    # Runs on normal completion, an error, or Ctrl+C — the servo never gets left energized.
    if dxl_id is not None:
        try:
            dxl.set_torque(portHandler, packetHandler, dxl_id, False)
            print("Torque disabled.")
        except Exception as e:
            print("Could not confirm torque was disabled: %s" % e)
    portHandler.closePort()
