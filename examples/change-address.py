#!/usr/bin/env python

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


tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=addr_current)
tof.open()
tof.change_address(addr_desired)
tof.close()
