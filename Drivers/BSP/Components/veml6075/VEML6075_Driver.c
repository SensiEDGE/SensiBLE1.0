/**
 ******************************************************************************
 * @file    VEML6075_Driver.c
 * @author  
 * @version V1.0
 * @date    09-August-2017
 * @brief   VEML6075 driver file
 ******************************************************************************
 * @attention
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "VEML6075_Driver.h"

#ifdef  USE_FULL_ASSERT_VEML6075
#include <stdio.h>
#endif


/** @addtogroup Environmental_Sensor
* @{
*/

/** @defgroup VEML6075_DRIVER
* @brief VEML6075 DRIVER
* @{
*/

/** @defgroup VEML6075_Imported_Function_Prototypes
* @{
*/

extern uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
extern uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );

extern void HAL_NVIC_SystemReset(void);
/**
* @}
*/

/** @defgroup VEML6075_Private_Function_Prototypes
* @{
*/

/**
* @}
*/

/** @defgroup VEML6075_Private_Functions
* @{
*/

/**
* @}
*/

/** @defgroup VEML6075_Public_Functions
* @{
*/

/*******************************************************************************
* Function Name : VEML6075_ReadReg
* Description   : Generic Reading function. It must be fullfilled with
*               : I2C reading functions
* Input         : Register Address
* Output        : Data Read
* Return        : None
*******************************************************************************/
VEML6075_Error_et VEML6075_ReadReg( void *handle, uint8_t RegAddr, 
                                    uint16_t NumByteToRead, uint8_t *Data )
{

  if ( Sensor_IO_Read( handle, RegAddr, Data, NumByteToRead ) )
    return VEML6075_ERROR;
  else
    return VEML6075_OK;
}


/*******************************************************************************
* Function Name : VEML6075_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*               : I2C or SPI writing function
* Input         : Register Address, Data to be written
* Output        : None
* Return        : None
*******************************************************************************/
VEML6075_Error_et VEML6075_WriteReg( void *handle, uint8_t RegAddr, 
                                    uint16_t NumByteToWrite, uint8_t *Data )
{
  if ( Sensor_IO_Write( handle, RegAddr, Data, NumByteToWrite ) )
    return VEML6075_ERROR;
  else
    return VEML6075_OK;
}


/**
* @brief  Get the version of this driver.
* @param  pxVersion pointer to a VEML6075_DriverVersion_st structure that 
*         contains the version information.
*         This parameter is a pointer to @ref VEML6075_DriverVersion_st.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Get_DriverVersion(VEML6075_DriverVersion_st* version)
{
  version->Major = VEML6075_DRIVER_VERSION_MAJOR;
  version->Minor = VEML6075_DRIVER_VERSION_MINOR;
  version->Point = VEML6075_DRIVER_VERSION_POINT;

  return VEML6075_OK;
}


/**
* @brief  Get device type ID.
* @param  *handle Device handle.
* @param  deviceid pointer to the returned device type ID.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Get_DeviceID(void *handle, uint8_t* deviceid)
{
  if(VEML6075_ReadReg(handle, VEML6075_WHO_AM_I_REG, 2, deviceid))
    return VEML6075_ERROR;

  return VEML6075_OK;
}


/**
* @brief  De initialization function for VEML6075.
*         This function put the VEML6075 in power down, make a memory boot and clear the data output flags.
* @param  *handle Device handle.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_DeInit(void *handle)
{
  VEML6075_DeActivate(handle);

  return VEML6075_OK;
}


/**
* @brief  Read VEML6075 output registers, and calculate ultraviolet.
* @param  *handle Device handle.
* @param  ultraviolet pointer to the returned ultraviolet value in units
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Get_Measurement(void *handle, uint16_t* ultraviolet)
{
  if ( VEML6075_Get_Ultraviolet( handle, ultraviolet ) == VEML6075_ERROR ) return VEML6075_ERROR;

  return VEML6075_OK;
}


/**
* @brief  Read VEML6075 Ultraviolet output registers
* @param  *handle Device handle.
* @param  Pointer to the returned ultraviolet value
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Get_Ultraviolet(void *handle, uint16_t* value)
{
  uint16_t uva_data, uvb_data;
  uint8_t buffer[2];

#ifdef PROX_DEBUG
  printf("\r\n\tVEML6075_Get_Ultraviolet()\r\n");
#endif
  
  if(VEML6075_ReadReg(handle, VEML6075_UVA_DATA_REG, 2, buffer))
    return VEML6075_ERROR;
  uva_data = ((uint16_t) buffer[0]) | (((uint16_t) buffer[1]) << 8);
  
  if(VEML6075_ReadReg(handle, VEML6075_UVB_DATA_REG, 2, buffer))
    return VEML6075_ERROR;
  uvb_data = ((uint16_t) buffer[0]) | (((uint16_t) buffer[1]) << 8);
  
  /* Calculate average value */
  *value = (uva_data + uvb_data) / 2;

#ifdef PROX_DEBUG
  printf("\tUV = %u\r\n", *value);
#endif
  
  return VEML6075_OK;
}


/**
* @brief  Exit from power down mode.
* @param  *handle Device handle.
* @param  void.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Activate(void *handle)
{
  uint8_t tmp, tmp1;

  if(VEML6075_ReadReg(handle, VEML6075_UV_CONF_REG1, 2, &tmp))
    return VEML6075_ERROR;

  tmp &= ~VEML6075_SD_MASK;

  if(VEML6075_WriteReg(handle, VEML6075_UV_CONF_REG1, 2, &tmp))
    return VEML6075_ERROR;

  return VEML6075_OK;
}

/**
* @brief  Put the sensor in power down mode.
* @param  *handle Device handle.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_DeActivate(void *handle)
{
  uint8_t tmp, tmp1;

  if(VEML6075_ReadReg(handle, VEML6075_UV_CONF_REG1, 2, &tmp))
    return VEML6075_ERROR;

  tmp |= VEML6075_SD_MASK;

  if(VEML6075_WriteReg(handle, VEML6075_UV_CONF_REG1, 2, &tmp))
    return VEML6075_ERROR;

  return VEML6075_OK;
}


/**
* @brief  Enter or exit from power down mode.
* @param  *handle Device handle.
* @param  status can be VEML6075_SET: VEML6075 in power down mode.
* @param  status can be VEML6075_REET: VEML6075 in active mode.
*         This parameter is a @ref VEML6075_BitStatus_et.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Set_PowerDownMode(void *handle, VEML6075_BitStatus_et status)
{
  uint8_t tmp, tmp1;

  VEML6075_assert_param(IS_VEML6075_BitStatus(status));

  if(VEML6075_ReadReg(handle, VEML6075_UV_CONF_REG1, 2, &tmp))
    return VEML6075_ERROR;

  tmp |= VEML6075_SD_MASK;

  if(VEML6075_WriteReg(handle, VEML6075_UV_CONF_REG1, 2, &tmp))
    return VEML6075_ERROR;

  return VEML6075_OK;
}

/**
* @brief  Get if VEML6075 is in active mode or in power down mode.
* @param  *handle Device handle.
* @param  Pointer to the returned value with VEML6075 status.
* @retval Error code [VEML6075_OK, VEML6075_ERROR].
*/
VEML6075_Error_et VEML6075_Get_PowerDownMode(void *handle, VEML6075_BitStatus_et* status)
{
  uint8_t tmp, tmp1;

  if(VEML6075_ReadReg(handle, VEML6075_UV_CONF_REG1, 2, &tmp))
    return VEML6075_ERROR;

  *status = (VEML6075_BitStatus_et)((tmp & VEML6075_SD_MASK));

  return VEML6075_OK;
}


#ifdef  USE_FULL_ASSERT_VEML6075
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void VEML6075_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters value: file %s on line %d\r\n", file, (int)line);

  /* Infinite loop */
  while (1)
  {
      #warning "Removed hang up here!"
      // Try to reset MCU
      HAL_NVIC_SystemReset();
  }
}
#endif

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/***************************************************************END OF FILE****/
