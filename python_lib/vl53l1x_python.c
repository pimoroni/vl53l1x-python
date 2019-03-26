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

#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <time.h>
#include "vl53l1_api.h"
#include "vl53l1_platform.h"

static VL53L1_RangingMeasurementData_t RangingMeasurementData;
static VL53L1_RangingMeasurementData_t *pRangingMeasurementData = &RangingMeasurementData;

/******************************************************************************
 * @brief   Initialises the device.
 *  @param  i2c_address - I2C Address to set for this device
 *  @param  TCA9548A_Device - Device number on TCA9548A I2C multiplexer if
 *              being used. If not being used, set to 255.
 *  @param  TCA9548A_Address - Address of TCA9548A I2C multiplexer if
 *              being used. If not being used, set to 0.
 * @retval  The Dev Object to pass to other library functions.
 *****************************************************************************/
VL53L1_DEV *initialise(uint8_t i2c_address, uint8_t perform_reset)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    VL53L1_Version_t                   Version;
    VL53L1_Version_t                  *pVersion   = &Version;
    VL53L1_DeviceInfo_t                DeviceInfo;
    int32_t status_int;

    VL53L1_Dev_t *dev = (VL53L1_Dev_t *)malloc(sizeof(VL53L1_Dev_t));
    memset(dev, 0, sizeof(VL53L1_Dev_t));

    dev->I2cDevAddr = i2c_address;

    if(perform_reset){
        Status = VL53L1_software_reset(dev);
        dev->I2cDevAddr = 0x29; // Resetting will reset i2c address back to default
        Status = VL53L1_WaitDeviceBooted(dev);
    }
    Status = VL53L1_DataInit(dev);
    Status = VL53L1_StaticInit(dev);
    //if(Status == VL53L1_ERROR_NONE){
        Status = VL53L1_GetDeviceInfo(dev, &DeviceInfo);
        if(Status == VL53L1_ERROR_NONE){
            printf("VL53L0X_GetDeviceInfo:\n");
            printf("Device Name : %s\n", DeviceInfo.Name);
            printf("Device Type : %s\n", DeviceInfo.Type);
            printf("Device ID : %s\n", DeviceInfo.ProductId);
            printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
            printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);
        }
    //}

    VL53L1_PerformRefSpadManagement(dev);
    VL53L1_SetXTalkCompensationEnable(dev, 0); // Disable crosstalk compensation (bare sensor)

    return dev;
}

VL53L1_Error setDeviceAddress(VL53L1_Dev_t *dev, int i2c_address)
{
    printf("Set addr: %x", i2c_address);
    VL53L1_Error Status = VL53L1_SetDeviceAddress(dev, i2c_address << 1);
    if(Status == 0){
        dev->I2cDevAddr = i2c_address;
    }
    return Status;
}

/******************************************************************************
 * @brief   Start Ranging
 * @param   mode - ranging mode
 *              0 - Good Accuracy mode
 *              1 - Better Accuracy mode
 *              2 - Best Accuracy mode
 *              3 - Longe Range mode
 *              4 - High Speed mode
 * @note Mode Definitions
 *   Good Accuracy mode
 *       33 ms timing budget 1.2m range
 *   Better Accuracy mode
 *       66 ms timing budget 1.2m range
 *   Best Accuracy mode
 *       200 ms 1.2m range
 *   Long Range mode (indoor,darker conditions)
 *       33 ms timing budget 2m range
 *   High Speed Mode (decreased accuracy)
 *       20 ms timing budget 1.2m range
 * @retval  Error code, 0 for success.
 *****************************************************************************/
VL53L1_Error startRanging(VL53L1_Dev_t *dev, int mode)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    Status = VL53L1_SetDistanceMode(dev, mode);
    Status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 66000);
    Status = VL53L1_SetInterMeasurementPeriodMilliSeconds(dev, 70);
    Status = VL53L1_StartMeasurement(dev);
    return Status;
}

/******************************************************************************
 * @brief   Get current distance in mm
 * @return  Current distance in mm or -1 on error
 *****************************************************************************/
int32_t getDistance(VL53L1_Dev_t *dev)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t current_distance = -1;
    Status = VL53L1_WaitMeasurementDataReady(dev);
    Status = VL53L1_GetRangingMeasurementData(dev, pRangingMeasurementData);
    current_distance = pRangingMeasurementData->RangeMilliMeter;
    VL53L1_ClearInterruptAndStartMeasurement(dev);
    return current_distance;
}

/******************************************************************************
 * @brief   Stop Ranging
 *****************************************************************************/
void stopRanging(VL53L1_Dev_t *dev)
{
    return VL53L1_StopMeasurement(dev);
}
