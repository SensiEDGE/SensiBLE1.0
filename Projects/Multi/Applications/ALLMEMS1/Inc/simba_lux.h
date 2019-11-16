#ifndef __SIMBA_LUX_H__
#define __SIMBA_LUX_H__

#include <stdint.h>

typedef enum
{
  LUX_OK = 0,
  LUX_ERROR = 1,
  LUX_TIMEOUT = 2,
  LUX_NOT_IMPLEMENTED = 3
} LUX_StatusTypeDef;


LUX_StatusTypeDef   BSP_LUX_Init(void);
uint8_t             BSP_LUX_IsInitalized(void);

LUX_StatusTypeDef   BSP_LUX_PowerON(void);
LUX_StatusTypeDef   BSP_LUX_PowerOFF(void);

uint8_t             BSP_LUX_IsDataReady(void);
LUX_StatusTypeDef   BSP_LUX_GetValue(uint16_t *pData);


#endif //__SIMBA_LUX_H__
