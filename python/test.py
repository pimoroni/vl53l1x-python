import VL53L1X
import time

tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
print("Python: Initialized")
tof.open()
print("Python: Opened")

tof.start_ranging()

for x in range(4):
    distance_mm = tof.get_distance()
    print("Distance: {}mm".format(distance_mm))
    time.sleep(0.1)

tof.stop_ranging()