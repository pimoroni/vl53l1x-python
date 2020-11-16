#!/usr/bin/python

# MIT License
#
# Copyright (c) 2017 John Bryan Moore
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from ctypes import CDLL, CFUNCTYPE, POINTER, c_int, c_uint, pointer, c_ubyte, c_uint8, c_uint32, c_uint16
from smbus2 import SMBus, i2c_msg
import os
import site
import glob


class VL53L1xError(RuntimeError):
    pass


class VL53L1xDistanceMode:
    NONE = 0
    SHORT = 1
    MEDIUM = 2
    LONG = 3


class VL53L1xUserRoi:
    def __init__(self, tlx=0, tly=0, brx=15, bry=15):
        self.top_left_x = tlx
        self.top_left_y = tly
        self.bot_right_x = brx
        self.bot_right_y = bry


# Read/write function pointer types.
_I2C_MULTI_FUNC = CFUNCTYPE(c_int, c_ubyte, c_uint16)
_I2C_READ_FUNC = CFUNCTYPE(c_int, c_ubyte, c_uint16, POINTER(c_ubyte), c_ubyte)
_I2C_WRITE_FUNC = CFUNCTYPE(c_int, c_ubyte, c_uint16, POINTER(c_ubyte), c_ubyte)

# Load VL53L1X shared lib
_POSSIBLE_LIBRARY_LOCATIONS = [os.path.dirname(os.path.realpath(__file__))]

try:
    _POSSIBLE_LIBRARY_LOCATIONS += site.getsitepackages()
except AttributeError:
    pass

try:
    _POSSIBLE_LIBRARY_LOCATIONS += [site.getusersitepackages()]
except AttributeError:
    pass

for lib_location in _POSSIBLE_LIBRARY_LOCATIONS:
    files = glob.glob(lib_location + "/vl53l1x_python*.so")
    if len(files) > 0:
        lib_file = files[0]
        try:
            _TOF_LIBRARY = CDLL(lib_file)
            # print("Using: " + lib_location + "/vl51l1x_python.so")
            break
        except OSError:
            # print(lib_location + "/vl51l1x_python.so not found")
            pass
else:
    raise OSError('Could not find vl53l1x_python.so')


class VL53L1X:
    """VL53L1X ToF."""
    def __init__(self, i2c_bus=1, i2c_address=0x29, tca9548a_num=255, tca9548a_addr=0):
        """Initialize the VL53L1X ToF Sensor from ST"""
        self._i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self._tca9548a_num = tca9548a_num
        self._tca9548a_addr = tca9548a_addr

        self._i2c = SMBus()
        if tca9548a_num == 255:
            try:
                self._i2c.open(bus=self._i2c_bus)
                self._i2c.read_byte_data(self.i2c_address, 0x00)
            except IOError:
                raise RuntimeError("VL53L1X not found on adddress: {:02x}".format(self.i2c_address))
            finally:
                self._i2c.close()

        self._dev = None
        # Register Address
        self.ADDR_UNIT_ID_HIGH = 0x16  # Serial number high byte
        self.ADDR_UNIT_ID_LOW = 0x17   # Serial number low byte
        self.ADDR_I2C_ID_HIGH = 0x18   # Write serial number high byte for I2C address unlock
        self.ADDR_I2C_ID_LOW = 0x19    # Write serial number low byte for I2C address unlock
        self.ADDR_I2C_SEC_ADDR = 0x8a  # Write new I2C address after unlock

    def open(self, reset=False):
        self._i2c.open(bus=self._i2c_bus)
        self._configure_i2c_library_functions()
        self._dev = _TOF_LIBRARY.initialise(self.i2c_address, self._tca9548a_num, self._tca9548a_addr, reset)

    def close(self):
        self._i2c.close()
        self._dev = None

    def _configure_i2c_library_functions(self):
        # I2C bus read callback for low level library.
        def _i2c_read(address, reg, data_p, length):
            ret_val = 0

            msg_w = i2c_msg.write(address, [reg >> 8, reg & 0xff])
            msg_r = i2c_msg.read(address, length)

            self._i2c.i2c_rdwr(msg_w, msg_r)

            if ret_val == 0:
                for index in range(length):
                    data_p[index] = ord(msg_r.buf[index])

            return ret_val

        # I2C bus write callback for low level library.
        def _i2c_write(address, reg, data_p, length):
            ret_val = 0
            data = []

            for index in range(length):
                data.append(data_p[index])

            msg_w = i2c_msg.write(address, [reg >> 8, reg & 0xff] + data)

            self._i2c.i2c_rdwr(msg_w)

            return ret_val

        # I2C bus write callback for low level library.
        def _i2c_multi(address, reg):
            ret_val = 0

            # Write to the multiplexer
            self._i2c.write_byte(address, reg)

            return ret_val

        # Pass i2c read/write function pointers to VL53L1X library.
        self._i2c_multi_func = _I2C_MULTI_FUNC(_i2c_multi)
        self._i2c_read_func = _I2C_READ_FUNC(_i2c_read)
        self._i2c_write_func = _I2C_WRITE_FUNC(_i2c_write)
        _TOF_LIBRARY.VL53L1_set_i2c(self._i2c_multi_func, self._i2c_read_func, self._i2c_write_func)

    # The ROI is a square or rectangle defined by two corners: top left and bottom right.
    # Default ROI is 16x16 (indices 0-15). The minimum ROI size is 4x4.
    def set_user_roi(self, user_roi):
        """Set Region Of Interest (ROI)"""
        _TOF_LIBRARY.setUserRoi(self._dev,
                                user_roi.top_left_x,
                                user_roi.top_left_y,
                                user_roi.bot_right_x,
                                user_roi.bot_right_y)

    def start_ranging(self, mode=VL53L1xDistanceMode.LONG):
        """Start VL53L1X ToF Sensor Ranging"""
        _TOF_LIBRARY.startRanging(self._dev, mode)

    def set_distance_mode(self, mode):
        """Set distance mode

        :param mode: One of 1 = Short, 2 = Medium or 3 = Long

        """
        _TOF_LIBRARY.setDistanceMode(self._dev, mode)

    def stop_ranging(self):
        """Stop VL53L1X ToF Sensor Ranging"""
        _TOF_LIBRARY.stopRanging(self._dev)

    def get_distance(self):
        """Get distance from VL53L1X ToF Sensor"""
        return _TOF_LIBRARY.getDistance(self._dev)

    def set_timing(self, timing_budget, inter_measurement_period):
        """Set the timing budget and inter measurement period.

        A higher timing budget results in greater measurement accuracy,
        but also a higher power consumption.

        The inter measurement period must be >= the timing budget, otherwise
        it will be double the expected value.

        :param timing_budget: Timing budget in microseconds
        :param inter_measurement_period: Inter Measurement Period in milliseconds

        """
        if (inter_measurement_period * 1000) < timing_budget:
            raise ValueError("The Inter Measurement Period must be >= Timing Budget")

        self.set_timing_budget(timing_budget)
        self.set_inter_measurement_period(inter_measurement_period)

    def set_timing_budget(self, timing_budget):
        """Set the timing budget in microseocnds"""
        _TOF_LIBRARY.setMeasurementTimingBudgetMicroSeconds(self._dev, timing_budget)

    def set_inter_measurement_period(self, period):
        """Set the inter-measurement period in milliseconds"""
        _TOF_LIBRARY.setInterMeasurementPeriodMilliSeconds(self._dev, period)

    # This function included to show how to access the ST library directly
    # from python instead of through the simplified interface
    def get_timing(self):
        budget = c_uint(0)
        budget_p = pointer(budget)
        status = _TOF_LIBRARY.VL53L1_GetMeasurementTimingBudgetMicroSeconds(self._dev, budget_p)
        if status == 0:
            return budget.value + 1000
        else:
            return 0

    def change_address(self, new_address):
        status = _TOF_LIBRARY.setDeviceAddress(self._dev, new_address)
        if status == 0:
            self.i2c_address = new_address
        else:
            raise RuntimeError("change_address failed with code: {}".format(status))
        return True
