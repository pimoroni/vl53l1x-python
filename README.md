# vl53l1x-python

Python library for the VL53L1X Laser Ranger.

https://shop.pimoroni.com/products/vl53l1x-breakout

# Installing

```
sudo pip install smbus2
sudo pip install vl53l1x
```

# Usage

```python
import VL53L1X

# Open and start the VL53L1X sensor.
# If you've previously used change-address.py then you
# should use the new i2c address here.
# If you're using a software i2c bus (ie: HyperPixel4) then
# you should `ls /dev/i2c-*` and use the relevant bus number.
tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()

# Optionally set an explicit timing budget
# These values are measurement time in microseconds,
# and inter-measurement time in milliseconds.
# If you uncomment the line below to set a budget you
# should use `tof.start_ranging(0)`
# tof.set_timing(66000, 70)

tof.start_ranging(1)  # Start ranging
                      # 0 = Unchanged
                      # 1 = Short Range
                      # 2 = Medium Range
                      # 3 = Long Range

# Grab the range in mm, this function will block until
# a reading is returned.
distance_in_mm = tof.get_distance()

tof.stop_ranging()
```

See examples for more advanced usage.
