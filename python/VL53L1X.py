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
from ctypes import CDLL, CFUNCTYPE, POINTER, c_int, c_uint, pointer, c_ubyte, c_uint8, c_uint32
import pkg_resources
SMBUS='smbus'
for dist in pkg_resources.working_set:
    #print(dist.project_name, dist.version)
    if dist.project_name == 'smbus':
        break
    if dist.project_name == 'smbus2':
        SMBUS='smbus2'
        break
if SMBUS == 'smbus':
    import smbus
elif SMBUS == 'smbus2':
    import smbus2 as smbus
import site


class Vl53l0xError(RuntimeError):
    pass


class Vl53l0xAccuracyMode:
    GOOD = 0        # 33 ms timing budget 1.2m range
    BETTER = 1      # 66 ms timing budget 1.2m range
    BEST = 2        # 200 ms 1.2m range
    LONG_RANGE = 3  # 33 ms timing budget 2m range
    HIGH_SPEED = 4  # 20 ms timing budget 1.2m range


class Vl53l0xDeviceMode:
    SINGLE_RANGING = 0
    CONTINUOUS_RANGING = 1
    SINGLE_HISTOGRAM = 2
    CONTINUOUS_TIMED_RANGING = 3
    SINGLE_ALS = 10
    GPIO_DRIVE = 20
    GPIO_OSC = 21


class Vl53l0xGpioAlarmType:
    OFF = 0
    THRESHOLD_CROSSED_LOW = 1
    THRESHOLD_CROSSED_HIGH = 2
    THRESHOLD_CROSSED_OUT = 3
    NEW_MEASUREMENT_READY = 4


class Vl53l0xInterruptPolarity:
    LOW = 0
    HIGH = 1


# Read/write function pointer types.
_I2C_READ_FUNC = CFUNCTYPE(c_int, c_ubyte, c_ubyte, POINTER(c_ubyte), c_ubyte)
_I2C_WRITE_FUNC = CFUNCTYPE(c_int, c_ubyte, c_ubyte, POINTER(c_ubyte), c_ubyte)

# Load VL53L1X shared lib
_POSSIBLE_LIBRARY_LOCATIONS = ['../build/lib.linux-armv7l-2.7'] + site.getsitepackages()
for lib_location in _POSSIBLE_LIBRARY_LOCATIONS:
    #try:
        _TOF_LIBRARY = CDLL(lib_location + "/vl53l1x_python.so")
        break
    #except OSError:
    #    pass
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
        self._i2c = smbus.SMBus()
        self._dev = None
        # Resgiter Address
        self.ADDR_UNIT_ID_HIGH = 0x16 # Serial number high byte
        self.ADDR_UNIT_ID_LOW = 0x17 # Serial number low byte
        self.ADDR_I2C_ID_HIGH = 0x18 # Write serial number high byte for I2C address unlock
        self.ADDR_I2C_ID_LOW = 0x19 # Write serial number low byte for I2C address unlock
        self.ADDR_I2C_SEC_ADDR = 0x8a # Write new I2C address after unlock

    def open(self):
        self._i2c.open(bus=self._i2c_bus)
        self._configure_i2c_library_functions()
        self._dev = _TOF_LIBRARY.initialise(self.i2c_address)

    def close(self):
        self._i2c.close()
        self._dev = None

    def _configure_i2c_library_functions(self):
        # I2C bus read callback for low level library.
        def _i2c_read(address, reg, data_p, length):
            ret_val = 0
            result = []

            print("Read {} bytes from {}.".format(length, reg))

            try:
                for i in range(0, length, 32):
                    read_length = min(length - i, 32)
                    result += self._i2c.read_i2c_block_data(address, reg + i, read_length)
            except IOError:
                ret_val = -1

            print(result)

            if ret_val == 0:
                for index in range(length):
                    data_p[index] = result[index]

            return ret_val

        # I2C bus write callback for low level library.
        def _i2c_write(address, reg, data_p, length):
            ret_val = 0
            data = []

            print("Write {} bytes to {}.".format(length, reg))

            for index in range(length):
                data.append(data_p[index])
            print(data)
            try:
                for i in range(0, len(data), 32):
                    self._i2c.write_i2c_block_data(address, reg + i, data[i:i+32])
            except IOError:
                ret_val = -1

            return ret_val

        # Pass i2c read/write function pointers to VL53L1X library.
        self._i2c_read_func = _I2C_READ_FUNC(_i2c_read)
        self._i2c_write_func = _I2C_WRITE_FUNC(_i2c_write)
        _TOF_LIBRARY.VL53L1_set_i2c(self._i2c_read_func, self._i2c_write_func)

    def start_ranging(self, mode=Vl53l0xAccuracyMode.GOOD):
        """Start VL53L1X ToF Sensor Ranging"""
        _TOF_LIBRARY.startRanging(self._dev, mode)

    def stop_ranging(self):
        """Stop VL53L1X ToF Sensor Ranging"""
        _TOF_LIBRARY.stopRanging(self._dev)

    def get_distance(self):
        """Get distance from VL53L1X ToF Sensor"""
        return _TOF_LIBRARY.getDistance(self._dev)

    # This function included to show how to access the ST library directly
    # from python instead of through the simplified interface
    def get_timing(self):
        budget = c_uint(0)
        budget_p = pointer(budget)
        status = _TOF_LIBRARY.VL53L1X_GetMeasurementTimingBudgetMicroSeconds(self._dev, budget_p)
        if status == 0:
            return budget.value + 1000
        else:
            return 0

    def configure_gpio_interrupt(
            self, proximity_alarm_type=Vl53l0xGpioAlarmType.THRESHOLD_CROSSED_LOW,
            interrupt_polarity=Vl53l0xInterruptPolarity.HIGH, threshold_low_mm=250, threshold_high_mm=500):
        """
        Configures a GPIO interrupt from device, be sure to call "clear_interrupt" after interrupt is processed.
        """
        pin = c_uint8(0)  # 0 is only GPIO pin.
        device_mode = c_uint8(Vl53l0xDeviceMode.CONTINUOUS_RANGING)
        functionality = c_uint8(proximity_alarm_type)
        polarity = c_uint8(interrupt_polarity)
        status = _TOF_LIBRARY.VL53L1X_SetGpioConfig(self._dev, pin, device_mode, functionality, polarity)
        if status != 0:
            raise Vl53l0xError('Error setting VL53L1X GPIO config')

        threshold_low = c_uint32(threshold_low_mm << 16)
        threshold_high = c_uint32(threshold_high_mm << 16)
        status = _TOF_LIBRARY.VL53L1X_SetInterruptThresholds(self._dev, device_mode, threshold_low, threshold_high)
        if status != 0:
            raise Vl53l0xError('Error setting VL53L1X thresholds')

        # Ensure any pending interrupts are cleared.
        self.clear_interrupt()

    def clear_interrupt(self):
        mask = c_uint32(0)
        status = _TOF_LIBRARY.VL53L1X_ClearInterruptMask(self._dev, mask)
        if status != 0:
            raise Vl53l0xError('Error clearing VL53L1X interrupt')

    def change_address(self, new_address):
        if self._dev is not None:
            raise Vl53l0xError('Error changing VL53L1X address')

        self._i2c.open(bus=self._i2c_bus)

        if new_address == None:
            return
        elif new_address == self.i2c_address:
            return
        else:
            # read value from 0x16,0x17
            high = self._i2c.read_byte_data(self.i2c_address, self.ADDR_UNIT_ID_HIGH)
            low = self._i2c.read_byte_data(self.i2c_address, self.ADDR_UNIT_ID_LOW)

            # write value to 0x18,0x19
            self._i2c.write_byte_data(self.i2c_address, self.ADDR_I2C_ID_HIGH, high)
            self._i2c.write_byte_data(self.i2c_address, self.ADDR_I2C_ID_LOW, low)

            # write new_address to 0x1a
            self._i2c.write_byte_data(self.i2c_address, self.ADDR_I2C_SEC_ADDR, new_address)

            self.i2c_address = new_address

        self._i2c.close()