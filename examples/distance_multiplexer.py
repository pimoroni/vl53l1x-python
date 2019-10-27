#!/usr/bin/env python

import os
import time
import sys
import signal

import VL53L1X


print("""distance.py

Display the distance read from the sensor.

Press Ctrl+C to exit.

""")


"""
Open and start the VL53L1X ranging sensor for each channel of the TCA9548A
"""
tof1 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29, tca9548a_num=2, tca9548a_addr=0x70)
tof2 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29, tca9548a_num=4, tca9548a_addr=0x70)
tof1.open() # Initialise the i2c bus and configure the sensor
tof1.start_ranging(3) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
tof2.open() # Initialise the i2c bus and configure the sensor
tof2.start_ranging(3) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range


running = True

def exit_handler(signal, frame):
    global running
    running = False
    tof1.stop_ranging() # Stop ranging
    tof2.stop_ranging()  # Stop ranging
    print()
    sys.exit(0)

signal.signal(signal.SIGINT, exit_handler)

while running:
    distance_in_mm = tof1.get_distance() # Grab the range in mm
    print("Sensor 1 distance: {}mm".format(distance_in_mm))
    distance_in_mm = tof2.get_distance() # Grab the range in mm
    print("Sensor 2 distance: {}mm".format(distance_in_mm))
    time.sleep(0.1)

