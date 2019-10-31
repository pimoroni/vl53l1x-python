#!/usr/bin/env python

import sys
sys.path.insert(0, "build/lib.linux-armv7l-2.7/")

import VL53L1X
import time
from datetime import datetime

tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
print("Python: Initialized")
tof.open()
print("Python: Opened")

tof.start_ranging(1)

try:
    while True:
        distance_mm = tof.get_distance()
        print("Time: {} Distance: {}mm".format(datetime.utcnow().strftime("%S.%f"), distance_mm))
        time.sleep(0.001)
except KeyboardInterrupt:
    tof.stop_ranging()
