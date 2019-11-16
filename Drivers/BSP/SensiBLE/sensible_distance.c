#include "sensible_distance.h"
#include "stm32l4xx_I2C.h"
#include "vl53l0x_api.h"

// I2C address
#define VL53L0X_I2C_ADDRESS    (0x52)
#define VL53L0X_ID             (0xEEAA)

static VL53L0X_Dev_t VL53L0XDev = {0};
static VL53L0X_RangingMeasurementData_t RangingMeasurementData = {0};
static int LeakyFactorFix8 = (int)( 0.6 *256); //(int)( 0.6 *256);

static void VL53L0X_Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange);
static DrvStatusTypeDef VL53L0X_Sensor_Detect(VL53L0X_Dev_t *pDev);
static DrvStatusTypeDef VL53L0X_Sensor_Setup(VL53L0X_Dev_t *pDev, Distance_RangingConfig rangeConfig);

/**
 * @brief Initialize a distance sensor
 * @param  rangeConfig     value of Distance_RangingConfig,
 *                         distance measurement mode
 * @retval COMPONENT_OK    in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_DISTANCE_Init(Distance_RangingConfig rangeConfig)
{
    
    DrvStatusTypeDef status = COMPONENT_OK;

    if (I2C_Global_Init() != HAL_OK) {
        status = COMPONENT_ERROR;
        goto done;
    }

    status = VL53L0X_Sensor_Detect(&VL53L0XDev);
    if (status != COMPONENT_OK) {
        goto done;
    }

    status = VL53L0X_Sensor_Setup(&VL53L0XDev, rangeConfig);

    done: return status;
}

/**
 * @brief Deinitialize a distance sensor
 * @retval COMPONENT_OK     in case of success
 * @retval COMPONENT_ERROR  in case of failure
 */
DrvStatusTypeDef BSP_DISTANCE_DeInit()
{

    DrvStatusTypeDef status = COMPONENT_OK;

    status = VL53L0X_ResetDevice(&VL53L0XDev);
    if(VL53L0X_ResetDevice(&VL53L0XDev) != VL53L0X_ERROR_NONE){
        status = COMPONENT_ERROR;
    }

    memset(&VL53L0XDev, 0, sizeof(VL53L0XDev));
    memset(&RangingMeasurementData, 0, sizeof(RangingMeasurementData));

    return status;
}

/**
 * @brief  Check if the distance sensor is initialized
 * @param  status     the pointer to the initialization status
 * @retval uint8_t    the initialization status, can be 1 or 0
 */
uint8_t BSP_DISTANCE_IsInitalized()
{
    return VL53L0XDev.Present;
}

/**
 * @brief  Get the distance value
 * @param  pData            pointer where the distance value is written [mm]
 * @retval COMPONENT_OK     in case of success
 * @retval COMPONENT_ERROR  in case of failure
 */
DrvStatusTypeDef BSP_DISTANCE_GetValue(uint16_t *pData)
{
    DrvStatusTypeDef status = COMPONENT_ERROR;
    if (pData == NULL) {
        goto done;
    }

    if (!VL53L0XDev.Present) {
        goto done;
    }
    
#ifdef PROX_DEBUG
  printf("\r\n\tBSP_DISTANCE_GetValue()\r\n");
#endif
    
    if (VL53L0X_PerformSingleRangingMeasurement(&VL53L0XDev,
            &RangingMeasurementData) != VL53L0X_ERROR_NONE) {
        goto done;
    }
    
    /* Store new ranging distance */
    VL53L0X_Sensor_SetNewRange(&VL53L0XDev, &RangingMeasurementData);
    *pData = RangingMeasurementData.RangeMilliMeter;
    status = COMPONENT_OK;
    
#ifdef PROX_DEBUG
  printf("\tdistance = %u\r\n", RangingMeasurementData.RangeMilliMeter);
#endif

    done: return status;
}

/**
 * @brief  Set the power mode for a distance sensor
 * @param  powerMode        the value of Distance_PowerModesTypeDef,
 *                          One of two available power modes
 * @retval COMPONENT_OK     in case of success
 * @retval COMPONENT_ERROR  in case of failure
 */
DrvStatusTypeDef BSP_DISTANCE_SetPowerMode(Distance_PowerModesTypeDef powerMode)
{
    if (VL53L0X_SetPowerMode(&VL53L0XDev, (VL53L0X_PowerModes)powerMode) == VL53L0X_ERROR_NONE){
        return COMPONENT_OK;
    }
    return COMPONENT_ERROR;
}

/**
 * @brief  Get the power mode for a distance sensor
 * @param  pPowerMode       The pointer where the power mode is written,
 *                          value of Distance_PowerModesTypeDef
 * @retval COMPONENT_OK     in case of success
 * @retval COMPONENT_ERROR  in case of failure
 */
DrvStatusTypeDef BSP_DISTANCE_GetPowerMode(Distance_PowerModesTypeDef *pPowerMode)
{
    *pPowerMode = (Distance_PowerModesTypeDef)VL53L0XDev.Data.PowerMode;
    return COMPONENT_OK;
}

/**
 * @brief  Reads the Device information for a distance sensor
 * @param  pVL53L0X_DeviceInfo  The pointer where the device info is written,
 *                              value of Distance_DeviceInfoTypeDef
 * @retval COMPONENT_OK         in case of success
 * @retval COMPONENT_ERROR      in case of failure
 */
DrvStatusTypeDef BSP_DISTANCE_GetDeviceInfo(
        Distance_DeviceInfoTypeDef *pVL53L0X_DeviceInfo)
{
    if (VL53L0X_GetDeviceInfo(&VL53L0XDev,
            (VL53L0X_DeviceInfo_t*) pVL53L0X_DeviceInfo) == VL53L0X_ERROR_NONE) {
        return COMPONENT_OK;
    }
    return COMPONENT_ERROR;
}

/**
 * @brief  Search for a distance sensor and initialize values in the case of success
 * @param  pDev             The pointer to the device descriptor,
 *                          type of VL53L0X_Dev_t
 * @retval COMPONENT_OK     in case of success
 * @retval COMPONENT_ERROR  in case of failure
 */
static DrvStatusTypeDef VL53L0X_Sensor_Detect(VL53L0X_Dev_t *pDev)
{
    DrvStatusTypeDef status = COMPONENT_ERROR;

    pDev->I2cDevAddr = VL53L0X_I2C_ADDRESS;
    pDev->I2cHandle = &I2CHandle;
    pDev->Present = 0;
    uint16_t Id = 0;

    HAL_Delay(40); //delay before initialization
    if (VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID,
            &Id) != VL53L0X_ERROR_NONE) {
        goto done;
    }
    if (Id == VL53L0X_ID) {
        if (VL53L0X_DataInit(pDev) == VL53L0X_ERROR_NONE) {
            pDev->Present = 1;
        } else {
            goto done;
        }
    } else {
        goto done;
    }
    status = COMPONENT_OK;
    done: return status;
}

/**
 * @brief  Setup the distance sensor
 * @param  pDev             The pointer to the device descriptor,
 *                          type of VL53L0X_Dev_t
 * @param  rangeConfig      value of Distance_RangingConfig,
 *                          distance measurement mode
 * @retval COMPONENT_OK     in case of success
 * @retval COMPONENT_ERROR  in case of failure
 */
static DrvStatusTypeDef VL53L0X_Sensor_Setup(VL53L0X_Dev_t *pDev, Distance_RangingConfig rangeConfig)
{
    DrvStatusTypeDef status = COMPONENT_ERROR;

    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint32_t signalLimit = (uint32_t) (0.25 * 65536);
    uint32_t sigmaLimit = (uint32_t) (18 * 65536);
    uint32_t timingBudget = 33000;
    uint8_t preRangeVcselPeriod = 14;
    uint8_t finalRangeVcselPeriod = 10;

    /* Setup sensors in single mode */
    HAL_Delay(1);
    if (VL53L0XDev.Present) {
        if (VL53L0X_StaticInit(pDev) != VL53L0X_ERROR_NONE) {
            printf("VL53L0X_StaticInit failed\n");
            goto done;
        }
        
        if (VL53L0X_PerformRefCalibration(pDev, &VhvSettings,
                &PhaseCal) != VL53L0X_ERROR_NONE) {
            printf("VL53L0X_PerformRefCalibration failed\n");
            goto done;
        }
        
        if (VL53L0X_PerformRefSpadManagement(pDev, &refSpadCount,
                &isApertureSpads) != VL53L0X_ERROR_NONE) {
            printf("VL53L0X_PerformRefSpadManagement failed\n");
            goto done;
        }
        
        // Setup in single ranging mode
        if (VL53L0X_SetDeviceMode(pDev,
                VL53L0X_DEVICEMODE_SINGLE_RANGING) != VL53L0X_ERROR_NONE) {
            printf("VL53L0X_SetDeviceMode failed\n");
            goto done;
        }
        
        // Enable Sigma limit
        if (VL53L0X_SetLimitCheckEnable(pDev,
                VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1) != VL53L0X_ERROR_NONE) {
            printf("VL53L0X_SetLimitCheckEnable(SIGMA) failed\n");
            goto done;
        }
        
        // Enable Signa limit
        if (VL53L0X_SetLimitCheckEnable(pDev,
                VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                1) != VL53L0X_ERROR_NONE) {
            printf("VL53L0X_SetLimitCheckEnable(SIGNAL) failed\n");
            goto done;
        }
        
        /* Ranging configuration */
        switch (rangeConfig) {
            case LONG_RANGE:
                signalLimit = (uint32_t) (0.1 * 65536);
                sigmaLimit = (uint32_t) (60 * 65536);
                timingBudget = 33000;
                preRangeVcselPeriod = 18;
                finalRangeVcselPeriod = 14;
                break;
            case HIGH_ACCURACY:
                signalLimit = (uint32_t) (0.25 * 65536);
                sigmaLimit = (uint32_t) (18 * 65536);
                timingBudget = 200000;
                preRangeVcselPeriod = 14;
                finalRangeVcselPeriod = 10;
                break;
            case HIGH_SPEED:
                signalLimit = (uint32_t) (0.25 * 65536);
                sigmaLimit = (uint32_t) (32 * 65536);
                timingBudget = 20000;
                preRangeVcselPeriod = 14;
                finalRangeVcselPeriod = 10;
                break;
            default:
                printf("Not Supported Range");
                goto done;
        }
        
        if (VL53L0X_SetLimitCheckValue(pDev,
                VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                signalLimit) != VL53L0X_ERROR_NONE) {
            printf("VL53L0X_SetLimitCheckValue(SIGNAL) failed\n");
            goto done;
        }
        
        if (VL53L0X_SetLimitCheckValue(pDev,
                VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                sigmaLimit) != VL53L0X_ERROR_NONE) {
            printf("VL53L0X_SetLimitCheckValue(SIGMA) failed\n");
            goto done;
        }
        
        if (VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDev,
                timingBudget) != VL53L0X_ERROR_NONE) {
            printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed\n");
            goto done;
        }
        
        if (VL53L0X_SetVcselPulsePeriod(pDev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                preRangeVcselPeriod) != VL53L0X_ERROR_NONE) {
            printf("VL53L0X_SetVcselPulsePeriod(PRE_RANGE) failed\n");
            goto done;
        }
        
        if (VL53L0X_SetVcselPulsePeriod(pDev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                finalRangeVcselPeriod) != VL53L0X_ERROR_NONE) {
            printf("VL53L0X_SetVcselPulsePeriod(FINAL_RANGE) failed\n");
            goto done;
        }
        
        if (VL53L0X_PerformRefCalibration(pDev, &VhvSettings,
                &PhaseCal) != VL53L0X_ERROR_NONE) {
            printf("VL53L0X_PerformRefCalibration failed\n");
            goto done;
        }
        
        pDev->LeakyFirst = 1;
        status = COMPONENT_OK;
    }

    done: return status;
}

/**
 * @brief  Store new ranging data into the device structure,
 *         apply leaky integrator if needed
 * @param  pDev             The pointer to the device descriptor, type of VL53L0X_Dev_t
 * @param  pRange           The pointer to VL53L0X_RangingMeasurementData_t,
 *                          Range measurement data.
 * @retval COMPONENT_OK     in case of success
 * @retval COMPONENT_ERROR  in case of failure
 */
static void VL53L0X_Sensor_SetNewRange(VL53L0X_Dev_t *pDev,
        VL53L0X_RangingMeasurementData_t *pRange)
{
    if (pRange->RangeStatus == 0) {
        pDev->RangeStatus = 0;
        if (pDev->LeakyFirst) {
            pDev->LeakyFirst = 0;
            pDev->LeakyRange = pRange->RangeMilliMeter;
        } else {
            pDev->LeakyRange = (pDev->LeakyRange * LeakyFactorFix8
                    + (256 - LeakyFactorFix8) * pRange->RangeMilliMeter) >> 8;
        }
    } else {
        pDev->RangeStatus = pRange->RangeStatus;
        pDev->LeakyFirst = 1;
    }
}

/************************ (C) COPYRIGHT SensiEdge LDT *************************/
