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
VL53L1_DEV *initialise(uint8_t i2c_address, uint8_t TCA9548A_Device, uint8_t TCA9548A_Address, uint8_t perform_reset)
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
    uint8_t ModelId, ModuleType, MaskRev;


#ifdef DEBUG
    if (TCA9548A_Device < 8)
    {
        printf ("VL53L1X Start Ranging Address 0x%02X TCA9548A Device %d TCA9548A Address 0x%02X\n\n",
                    i2c_address, TCA9548A_Device, TCA9548A_Address);
    }
    else
    {
        printf ("VL53L1X Start Ranging Address 0x%02X\n\n", i2c_address);
    }
#endif

    VL53L1_Dev_t *dev = (VL53L1_Dev_t *)malloc(sizeof(VL53L1_Dev_t));
    memset(dev, 0, sizeof(VL53L1_Dev_t));

    dev->I2cDevAddr = i2c_address;
    dev->TCA9548A_Device = TCA9548A_Device;
    dev->TCA9548A_Address = TCA9548A_Address;

    if(perform_reset){
        Status = VL53L1_software_reset(dev);
        dev->I2cDevAddr = 0x29; // Resetting will reset i2c address back to default
        Status = VL53L1_WaitDeviceBooted(dev);
    }

    Status = VL53L1_RdByte(dev, 0x010F, &ModelId);
    Status = VL53L1_RdByte(dev, 0x0110, &ModuleType);
    Status = VL53L1_RdByte(dev, 0x0111, &MaskRev);

#ifdef DEBUG
    printf("VL53L1X I2C-Validity:\n");
    printf("Model ID (should be 0xEA): 0x%X\n", ModelId);
    printf("Module Type (should be 0xCC): 0x%X\n", ModuleType);
    printf("Mask Revision (should be 0x10): 0x%X\n\n", MaskRev);

    // Check valid I2C interface as shown in VL53L1X data sheet page 22
    if ((ModelId != 0xEA) || (ModuleType != 0xCC) || (MaskRev != 0x10))
    {
      printf("Warning: I2C interface invalid! Data might be corrupted!");
    }
#endif

    Status = VL53L1_DataInit(dev);
    Status = VL53L1_StaticInit(dev);
    //if(Status == VL53L1_ERROR_NONE){
#ifdef DEBUG
        Status = VL53L1_GetDeviceInfo(dev, &DeviceInfo);
        if(Status == VL53L1_ERROR_NONE){
            printf("VL53L0X_GetDeviceInfo:\n");
            printf("Device Name : %s\n", DeviceInfo.Name);
            printf("Device Type : %s\n", DeviceInfo.Type);
            printf("Device ID : %s\n", DeviceInfo.ProductId);
            printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
            printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);
        }
#endif
    //}

    VL53L1_PerformRefSpadManagement(dev);
    VL53L1_SetXTalkCompensationEnable(dev, 0); // Disable crosstalk compensation (bare sensor)

    return dev;
}

VL53L1_Error setDeviceAddress(VL53L1_Dev_t *dev, int i2c_address)
{
#ifdef DEBUG
    printf("Set addr: %x", i2c_address);
#endif
    VL53L1_Error Status = VL53L1_SetDeviceAddress(dev, i2c_address << 1);
    if(Status == 0){
        dev->I2cDevAddr = i2c_address;
    }
    return Status;
}

/******************************************************************************
 * @brief   Set Distance Mode
 * @param   mode - ranging mode
 *              1 - Short-range mode
 *              2 - Medium-range mode
 *              3 - Long-range mode
 * @retval  Error code, 0 for success.
 *****************************************************************************/
VL53L1_Error setDistanceMode(VL53L1_Dev_t *dev, int mode)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    Status = VL53L1_SetDistanceMode(dev, mode);
    return Status;
}

/******************************************************************************
 * @brief   Set User ROI (Region Of Interest)
 * @param   userRoi - user ROI
 * @retval  Error code, 0 for success.
 *****************************************************************************/
VL53L1_Error setUserRoi(VL53L1_Dev_t *dev, int topLeftX, int topLeftY, int botRightX, int botRightY)
{
    VL53L1_UserRoi_t userRoi;
    userRoi.TopLeftX = topLeftX;
    userRoi.TopLeftY = topLeftY;
    userRoi.BotRightX = botRightX;
    userRoi.BotRightY = botRightY;

    VL53L1_Error Status = VL53L1_ERROR_NONE;
    Status = VL53L1_SetUserROI(dev, &userRoi);
    return Status;
}


/******************************************************************************
 * @brief   Start Ranging
 * @param   mode - ranging mode
 *              0 - Unchanged
 *              1 - Short-range mode
 *              2 - Medium-range mode
 *              3 - Long-range mode
 * @retval  Error code, 0 for success.
 *****************************************************************************/
VL53L1_Error startRanging(VL53L1_Dev_t *dev, int mode)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    if (mode > 0) {
        Status = setDistanceMode(dev, mode);
    }
    Status = VL53L1_StartMeasurement(dev);
    return Status;
}

/******************************************************************************
 * @brief   Set the measurement timing budget in microseconds
 * @return  Error code, 0 for success.
 *****************************************************************************/
VL53L1_Error setMeasurementTimingBudgetMicroSeconds(VL53L1_Dev_t *dev, int timing_budget) {
    return VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, timing_budget);
}

/******************************************************************************
 * @brief   Set the inter-measurement period in milliseconds
 * @return  Error code, 0 for success.
 *****************************************************************************/
VL53L1_Error setInterMeasurementPeriodMilliSeconds(VL53L1_Dev_t *dev, int period) {
    return VL53L1_SetInterMeasurementPeriodMilliSeconds(dev, period);
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
    VL53L1_StopMeasurement(dev);
}
