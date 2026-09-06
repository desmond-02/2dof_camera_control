#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# One-time setup: give a single connected XC330 a unique bus ID.
# Connect ONLY ONE servo at a time when running this.

from dynamixel_sdk import *
import dxl_common as dxl

portHandler, packetHandler = dxl.connect()

print("Scanning bus for connected servos...")
data_list, dxl_comm_result = packetHandler.broadcastPing(portHandler)
if dxl_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(dxl_comm_result))
    portHandler.closePort()
    quit()

if len(data_list) == 0:
    print("No servo detected. Check power and the TTL cable, then try again.")
    portHandler.closePort()
    quit()

if len(data_list) > 1:
    print("More than one servo responded: %s" % list(data_list.keys()))
    print("This script is for setting up ONE servo at a time — disconnect the others and re-run.")
    portHandler.closePort()
    quit()

current_id = list(data_list.keys())[0]
model_number = data_list[current_id][0]
print("Found servo: ID=%d, Model Number=%d" % (current_id, model_number))

if not dxl.check_hardware_error(portHandler, packetHandler, current_id):
    print("Resolve the fault above before continuing.")
    portHandler.closePort()
    quit()

try:
    new_id_input = input("Enter the new ID for this servo (1-252): ").strip()

    try:
        new_id = int(new_id_input)
    except ValueError:
        print("Not a number.")
        raise SystemExit

    if new_id < 1 or new_id > 252:
        print("ID must be between 1 and 252.")
        raise SystemExit

    if new_id == current_id:
        print("Servo is already ID %d, nothing to do." % current_id)
        raise SystemExit

    dxl.set_torque(portHandler, packetHandler, current_id, False)

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, current_id, dxl.ADDR_ID, new_id)
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
        raise SystemExit
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))
        raise SystemExit

    model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, new_id)
    if dxl_comm_result != COMM_SUCCESS:
        print("ID change may have failed — could not ping new ID %d" % new_id)
        print(packetHandler.getTxRxResult(dxl_comm_result))
    else:
        print("Success: servo now responds at ID %d" % new_id)

finally:
    # Runs on normal completion, an error, or Ctrl+C — the servo never gets left energized.
    try:
        dxl.set_torque(portHandler, packetHandler, current_id, False)
    except Exception as e:
        print("Could not confirm torque was disabled: %s" % e)
    portHandler.closePort()
