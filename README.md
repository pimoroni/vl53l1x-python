# vl53l1x-python

Python library for the VL53L1X Laser Ranger.

https://shop.pimoroni.com/products/vl53l1x-breakout

# Installing

```
pip install vl53l1x
```

On Raspberry Pi OS Bookworm and newer the system Python is marked externally managed. Either install into a virtual environment:

```
python3 -m venv --system-site-packages ~/.virtualenvs/vl53l1x
source ~/.virtualenvs/vl53l1x/bin/activate
pip install vl53l1x
```

or override the check:

```
sudo pip install --break-system-packages vl53l1x
```

## Dependencies

At runtime the library needs [smbus2](https://pypi.org/project/smbus2/), which pip installs for you. I2C must be enabled, via `sudo raspi-config` under "Interface Options".

If there's no wheel for your platform, pip builds from source and you'll need a compiler:

```
sudo apt install build-essential python3-dev
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

# Building from source

```
git clone https://github.com/pimoroni/vl53l1x-python
cd vl53l1x-python
pip install .
```

# Using the library from C

```
make            # builds libvl53l1x.so
make examples   # builds examples/distance
./examples/distance
```

`libvl53l1x.so` exposes the functions declared in `python_lib/vl53l1x_python.h`. It has no I2C transport of its own. Before calling `initialise()` you must hand it read, write and multiplexer callbacks with `VL53L1_set_i2c()`, declared in `api/platform/vl53l1_platform.h`. Skipping this is what produces `i2c bus read not set.` and `i2c bus write not set.` at runtime. `examples/distance.c` shows a complete set of callbacks driving `/dev/i2c-1` through `ioctl(I2C_RDWR)`.

Compiling against the library outside this source tree:

```
cc -Ipath/to/api/core -Ipath/to/api/platform -Ipath/to/python_lib \
   -o distance distance.c -Lpath/to/vl53l1x-python -lvl53l1x
```

