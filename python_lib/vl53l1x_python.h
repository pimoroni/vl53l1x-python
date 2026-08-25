/*
MIT License

Copyright (c) 2017 John Bryan Moore

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 * A simplified wrapper around the parts of the ST VL53L1X API needed to take
 * distance readings. Both the Python module (via ctypes) and the C examples
 * call this rather than the ST API directly.
 *
 * Callers must supply the I2C transport by calling VL53L1_set_i2c(), declared
 * in vl53l1_platform.h, before calling initialise(). Without it every bus
 * access fails with "i2c bus read/write not set."
 */

#ifndef _VL53L1X_PYTHON_H_
#define _VL53L1X_PYTHON_H_

#include "vl53l1_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Returns a handle the caller owns and should free() once ranging has stopped.
 * TCA9548A_Device is a multiplexer channel, or 255 when none is in use.
 * perform_reset resets the sensor first, returning it to address 0x29.
 */
VL53L1_DEV initialise(uint8_t i2c_address, uint8_t TCA9548A_Device, uint8_t TCA9548A_Address, uint8_t perform_reset);

/* Changes the sensor's I2C address, and the address the handle talks to. */
VL53L1_Error setDeviceAddress(VL53L1_DEV dev, int i2c_address);

/* mode: 1 = short range, 2 = medium range, 3 = long range. */
VL53L1_Error setDistanceMode(VL53L1_DEV dev, int mode);

/* Region of interest corners, in a 16x16 grid of SPADs. Minimum size is 4x4. */
VL53L1_Error setUserRoi(VL53L1_DEV dev, int topLeftX, int topLeftY, int botRightX, int botRightY);

/* mode: 0 = leave distance mode unchanged, otherwise as setDistanceMode(). */
VL53L1_Error startRanging(VL53L1_DEV dev, int mode);

/* A longer timing budget trades power and update rate for accuracy. */
VL53L1_Error setMeasurementTimingBudgetMicroSeconds(VL53L1_DEV dev, int timing_budget);

/* Should be >= the timing budget, or the sensor ranges at half the rate asked for. */
VL53L1_Error setInterMeasurementPeriodMilliSeconds(VL53L1_DEV dev, int period);

/*
 * Blocks until a reading is ready, then returns the distance in mm. The
 * reading means nothing unless getStatus() and getRangeStatus() agree it
 * succeeded, so a failed measurement still reports a number.
 */
int32_t getDistance(VL53L1_DEV dev);

/* Both describe the last getDistance(), whichever device that was for. */
VL53L1_Error getStatus(void);
uint8_t getRangeStatus(void);

void stopRanging(VL53L1_DEV dev);

#ifdef __cplusplus
}
#endif

#endif
