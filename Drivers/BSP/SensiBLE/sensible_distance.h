/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSOR_BOARD_DISTANCE_H
#define __SENSOR_BOARD_DISTANCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32l4xx_nucleo.h"
#include "distance.h"
#include "vl53l0x_def.h"

//Available distance measurement modes
typedef enum {
    LONG_RANGE      = 0, /*!< Long range mode */
    HIGH_SPEED      = 1, /*!< High speed mode */
    HIGH_ACCURACY   = 2, /*!< High accuracy mode */
} Distance_RangingConfig;

//Available Power modes
typedef enum {
    POWERMODE_STANDBY = 0,
    POWERMODE_IDLE    = 2,
} Distance_PowerModesTypeDef;

//Defines the parameters of the GetDeviceInfo Functions
typedef struct {
    char Name[VL53L0X_MAX_STRING_LENGTH];
        /*!< Name of the Device e.g. Left_Distance */
    char Type[VL53L0X_MAX_STRING_LENGTH];
        /*!< Type of the Device e.g VL53L0X */
    char ProductId[VL53L0X_MAX_STRING_LENGTH];
        /*!< Product Identifier String  */
    uint8_t ProductType;
        /*!< Product Type, VL53L0X = 1, VL53L1 = 2 */
    uint8_t ProductRevisionMajor0;
        /*!< Product revision major */
    uint8_t ProductRevisionMinor;
        /*!< Product revision minor */
} Distance_DeviceInfoTypeDef;


/* Public function prototypes */
DrvStatusTypeDef BSP_DISTANCE_Init(Distance_RangingConfig rangeConfig);
DrvStatusTypeDef BSP_DISTANCE_DeInit();
uint8_t BSP_DISTANCE_IsInitalized();
DrvStatusTypeDef BSP_DISTANCE_GetValue(uint16_t *pData);
DrvStatusTypeDef BSP_DISTANCE_SetPowerMode(Distance_PowerModesTypeDef powerMode);
DrvStatusTypeDef BSP_DISTANCE_GetPowerMode(Distance_PowerModesTypeDef *pPowerMode);
DrvStatusTypeDef BSP_DISTANCE_GetDeviceInfo(Distance_DeviceInfoTypeDef *pVL53L0X_DeviceInfo);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_BOARD_HUMIDITY_H */

/************************ (C) COPYRIGHT SensiEdge LDT *************************/
