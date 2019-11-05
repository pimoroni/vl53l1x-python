#!/usr/bin/env python

import os
import time
import sys
import signal

import VL53L1X

MAX_DISTANCE_MM = 800  # Distance at which our bar is full
BAR_CHAR = u'\u2588'   # Unicode FULL BLOCK

ANSI_COLOR_RED = "\x1b[31m"
ANSI_COLOR_YELLOW = "\x1b[33m"
ANSI_COLOR_GREEN = "\x1b[32m"
ANSI_COLOR_RESET = "\x1b[0m"


UPDATE_TIME_MICROS = 66000
INTER_MEASUREMENT_PERIOD_MILLIS = 70

print("""graph.py

Display a bar graph that ranges up to 80cm and turns yellow/red as the range decreases.

Press Ctrl+C to exit.

""")

"""
Grab the width/height of the terminal using `stty size`
"""
try:
    rows, cols = [int(c) for c in os.popen("stty size", "r").read().split()]
except ValueError:
    print("Cannot get size of tty! Try running in Terminal.")
    sys.exit(1)

"""
Open and start the VL53L1X ranging sensor
"""
tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()  # Initialise the i2c bus and configure the sensor

# tof.set_distance_mode(3)

# Lower timing budgets allow for faster updates, but sacrifice accuracy
tof.set_timing(UPDATE_TIME_MICROS, INTER_MEASUREMENT_PERIOD_MILLIS)

# Start ranging, mode 0 to leave timing unchanged
tof.start_ranging(0)

sys.stdout.write("\n")

running = True


def exit_handler(signal, frame):
    global running
    running = False
    tof.stop_ranging()
    sys.stdout.write("\n")
    sys.exit(0)


signal.signal(signal.SIGINT, exit_handler)

while running:
    distance_in_mm = tof.get_distance()  # Grab the range in mm
    distance_in_mm = min(MAX_DISTANCE_MM, distance_in_mm)                    # Cap at our MAX_DISTANCE
    bar_size = int((distance_in_mm / float(MAX_DISTANCE_MM)) * (cols - 10))  # Scale bar_size to our terminal width
    bar = BAR_CHAR * bar_size            # Create a bar out of `bar_size` unicode FULL BLOCK characters
    bar = bar.ljust(cols - 7, u' ')      # Pad the bar to the full with of the terminal, minus the "00.00cm " prefix
    sys.stdout.write("\r")               # Return the cursor to the beginning of the current line
    sys.stdout.flush()
    color = ANSI_COLOR_GREEN
    if distance_in_mm < MAX_DISTANCE_MM * 0.6:
        color = ANSI_COLOR_YELLOW
    if distance_in_mm < MAX_DISTANCE_MM * 0.3:
        color = ANSI_COLOR_RED
    sys.stdout.write(color)
    sys.stdout.write(u"{:04.1f}cm {}".format(distance_in_mm / 10.0, bar))     # Output our measurement and bar
    sys.stdout.write(ANSI_COLOR_RESET)
    sys.stdout.flush()                   # Flush the output buffer, since we're overdrawing the last line
    time.sleep(INTER_MEASUREMENT_PERIOD_MILLIS / 1000.0)
