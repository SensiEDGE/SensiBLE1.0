#include "simba_lux.h"
#include "x_nucleo_iks01a1.h"
#include "stm32l4xx_I2C.h"


// I2C address
#define APDS9250_I2C_ADDRESS    (0x52<<1)

// Register definitions
#define APDS9250_REG_MAIN_CTRL          0x00
#define APDS9250_REG_LS_MEAS_RATE       0x04
#define APDS9250_REG_LS_GAIN            0x05
#define APDS9250_REG_PART_ID            0x06
#define APDS9250_REG_MAIN_STATUS        0x07
#define APDS9250_REG_DATA_IR_0          0x0A
#define APDS9250_REG_DATA_IR_1          0x0B
#define APDS9250_REG_DATA_IR_2          0x0C
#define APDS9250_REG_DATA_GREEN_0       0x0D
#define APDS9250_REG_DATA_GREEN_1       0x0E
#define APDS9250_REG_DATA_GREEN_2       0x0F
#define APDS9250_REG_DATA_BLUE_0        0x10
#define APDS9250_REG_DATA_BLUE_1        0x11
#define APDS9250_REG_DATA_BLUE_2        0x12
#define APDS9250_REG_DATA_RED_0         0x13
#define APDS9250_REG_DATA_RED_1         0x14
#define APDS9250_REG_DATA_RED_2         0x15
#define APDS9250_REG_INT_CFG            0x19
#define APDS9250_REG_INT_PERSISTENCE    0x1A
#define APDS9250_REG_THRES_UP_0         0x21
#define APDS9250_REG_THRES_UP_1         0x22
#define APDS9250_REG_THRES_UP_2         0x23
#define APDS9250_REG_THRES_LOW_0        0x24
#define APDS9250_REG_THRES_LOW_1        0x25
#define APDS9250_REG_THRES_LOW_2        0x26
#define APDS9250_REG_THRES_VAR          0x27

// Register values
#define APDS9250_PART_ID        0xB2


static uint8_t LuxInitialized = 0;


static LUX_StatusTypeDef APDS9250_IO_Init(void);
static LUX_StatusTypeDef APDS9250_IO_Read(uint8_t reg, uint8_t *buf, uint16_t count);
static LUX_StatusTypeDef APDS9250_IO_Write(uint8_t reg, uint8_t *buf, uint16_t count);


LUX_StatusTypeDef BSP_LUX_Init(void)
{
    if (!LuxInitialized)
    {
        uint8_t gain = 0x01; // gain = 3;
        uint8_t meas_rate = 0x22; // 18bit, 100 mS

        if (APDS9250_IO_Init() != LUX_OK)
        {
            return LUX_ERROR;
        }

        HAL_Delay(100);

        // Set gain
        if (APDS9250_IO_Write(APDS9250_REG_LS_GAIN, &gain, 1) != LUX_OK)
        {
            return LUX_ERROR;
        }
        // Set measurement rate
        if (APDS9250_IO_Write(APDS9250_REG_LS_MEAS_RATE, &meas_rate, 1) != LUX_OK)
        {
            return LUX_ERROR;
        }
        LuxInitialized = 1;
    }

    return LUX_OK;
}


uint8_t BSP_LUX_IsInitalized(void)
{
    return LuxInitialized;
}


LUX_StatusTypeDef BSP_LUX_PowerON(void)
{
    uint8_t main_ctrl = 0x02; // ALS, active
    //uint8_t main_ctrl = 0x06; // RGB, active
    return APDS9250_IO_Write(APDS9250_REG_MAIN_CTRL, &main_ctrl, 1);
}


LUX_StatusTypeDef BSP_LUX_PowerOFF(void)
{
    uint8_t main_ctrl = 0;
    return APDS9250_IO_Write(APDS9250_REG_MAIN_CTRL, &main_ctrl, 1);
}


uint8_t BSP_LUX_IsDataReady(void)
{
    LUX_StatusTypeDef ret;
    uint8_t main_status;
#ifdef PROX_DEBUG
  printf("\r\n\tBSP_LUX_IsDataReady()\r\n");
#endif
    ret = APDS9250_IO_Read(APDS9250_REG_MAIN_STATUS, &main_status, 1);
    if (ret != LUX_OK)
    {
        return 0;
    }

    return (main_status & 0x08) ? 1 : 0;
}


LUX_StatusTypeDef BSP_LUX_GetValue(uint16_t *pData)
{
    LUX_StatusTypeDef ret;
    uint8_t adc_data[6];

#ifdef PROX_DEBUG
  printf("\r\n\tBSP_LUX_GetValue()\r\n");
#endif
    
    ret = APDS9250_IO_Read(APDS9250_REG_DATA_IR_0, adc_data, 6);
    if (ret == LUX_OK)
    {
        uint32_t ir_value    = adc_data[0] | (adc_data[1] << 8) | (adc_data[2] << 16);
        uint32_t green_value = adc_data[3] | (adc_data[4] << 8) | (adc_data[5] << 16);

        uint32_t factor = ir_value > green_value ? 35 : 46;

        uint32_t lux = ((green_value * factor) / 3) / 100;
        *pData = (uint16_t)lux;
        
        #ifdef PROX_DEBUG
          printf("\tLUX = %u\r\n", (uint16_t)lux);
        #endif
        
    }
    
    return ret;
}



LUX_StatusTypeDef APDS9250_IO_Init(void)
{
    // Init I2C
    if(I2C_Global_Init() != HAL_OK)
    {
        return LUX_ERROR;
    }
    return LUX_OK;
}


static LUX_StatusTypeDef APDS9250_IO_Read(uint8_t reg, uint8_t *buf, uint16_t count)
{
    LUX_StatusTypeDef ret_val = LUX_OK;
    
    if (HAL_I2C_Mem_Read(&I2CHandle, APDS9250_I2C_ADDRESS, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, buf, count, 5000) != HAL_OK)
    {
        ret_val = LUX_ERROR;
    }
    
#ifdef PROX_DEBUG
  printf("\t\t<<< %02X %02X", APDS9250_I2C_ADDRESS, reg);
  for(uint16_t i = 0; i < count; i++){
    printf(" %02X", buf[i]);
  }
  printf("\r\n");
#endif

    return ret_val;
}


static LUX_StatusTypeDef APDS9250_IO_Write(uint8_t reg, uint8_t *buf, uint16_t count)
{
    LUX_StatusTypeDef ret_val = LUX_OK;

#ifdef PROX_DEBUG
  printf("\t\t>>> %02X %02X", APDS9250_I2C_ADDRESS, reg);
  for(uint16_t i = 0; i < count; i++){
    printf(" %02X", buf[i]);
  }
  printf("\r\n");
#endif

    
    if (HAL_I2C_Mem_Write(&I2CHandle, APDS9250_I2C_ADDRESS, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, buf, count, 5000) != HAL_OK)
    {
        ret_val = LUX_ERROR;
    }

    return ret_val;
}
