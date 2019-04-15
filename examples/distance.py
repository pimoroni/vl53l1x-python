#!/usr/bin/env python

import time
import sys
import signal

import VL53L1X


print("""distance.py

Display the distance read from the sensor.

Press Ctrl+C to exit.

""")


"""
Open and start the VL53L1X ranging sensor
"""
tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()            # Initialise the i2c bus and configure the sensor
tof.start_ranging(1)  # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range

running = True


def exit_handler(signal, frame):
    global running
    running = False
    tof.stop_ranging()
    print()
    sys.exit(0)


signal.signal(signal.SIGINT, exit_handler)

while running:
    distance_in_mm = tof.get_distance()
    print("Distance: {}mm".format(distance_in_mm))
    time.sleep(0.1)
