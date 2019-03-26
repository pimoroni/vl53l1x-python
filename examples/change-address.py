#!/usr/bin/env python

import os
import time
import sys
import signal
import argparse

import VL53L1X


addr_current = 0x29
addr_desired = 0x33

def auto_int(x):
    return int(x, 0)

parser = argparse.ArgumentParser(description='Change address options.')
parser.add_argument('--current', type=auto_int, help='The current VL53L1X i2c address.', default=addr_current)
parser.add_argument('--desired', type=auto_int, help='The desired VL53L1X i2c address.', default=addr_desired)
args = parser.parse_args()

addr_current = args.current
addr_desired = args.desired

print("""change-address.py

Change address then display the distance read from the sensor.

Current address: {:02x}
Desired address: {:02x}

Press Ctrl+C to exit.

""".format(addr_current, addr_desired))


"""
Open and start the VL53L1X ranging sensor
"""
tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=addr_current)
tof.open() # Initialise the i2c bus and configure the sensor
tof.change_address(addr_desired)
tof.close()
tof.open()
tof.start_ranging(1) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range


running = True

def exit_handler(signal, frame):
    global running
    running = False
    tof.stop_ranging() # Stop ranging
    print()
    sys.exit(0)

signal.signal(signal.SIGINT, exit_handler)

while running:
    distance_in_mm = tof.get_distance() # Grab the range in mm
    print("Distance: {}mm".format(distance_in_mm))
    time.sleep(0.1)

