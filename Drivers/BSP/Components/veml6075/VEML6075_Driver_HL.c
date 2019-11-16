/**
 ******************************************************************************
 * @file    VEML6075_Driver_HL.c
 * @author  
 * @version V1.0.0
 * @date    08-August-2017
 * @brief   This file provides a set of high-level functions needed to manage
            the VEML6075 sensor
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
#include "VEML6075_Driver_HL.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup VEML6075 VEML6075
 * @{
 */

/** @addtogroup VEML6075_Callable_Private_FunctionPrototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef VEML6075_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef VEML6075_U_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef VEML6075_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef VEML6075_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef VEML6075_U_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef VEML6075_U_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef VEML6075_Get_Uv( DrvContextTypeDef *handle, uint16_t *ultraviolet );
static DrvStatusTypeDef VEML6075_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef VEML6075_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef VEML6075_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef VEML6075_Check_WhoAmI( DrvContextTypeDef *handle );

/**
 * @}
 */

/** @addtogroup VEML6075_Private_Function_Prototypes Private function prototypes
 * @{
 */

/**
 * @}
 */

/** @addtogroup VEML6075_Private_Variables Private variables
 * @{
 */

/**
 * @brief VEML6075 humidity driver structure
 */
ULTRAVIOLET_Drv_t VEML6075_Drv =
{
  VEML6075_Init,
  VEML6075_U_DeInit,
  VEML6075_Sensor_Enable,
  VEML6075_Sensor_Disable,
  VEML6075_U_Get_WhoAmI,
  VEML6075_U_Check_WhoAmI,
  VEML6075_Get_Uv,
  VEML6075_Read_Reg,
  VEML6075_Write_Reg
};


/**
 * @brief VEML6075 combo data structure definition
 */
VEML6075_Combo_Data_t VEML6075_Combo_Data[VEML6075_SENSORS_MAX_NUM];

/**
 * @}
 */

/** @addtogroup VEML6075_Callable_Private_Functions Callable private functions
 * @{
 */


/**
 * @brief Deinitialize the VEML6075 ultraviolet sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef VEML6075_U_DeInit( DrvContextTypeDef *handle )
{

  /* Check if the VEML6075 ultraviolet sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if ((((VEML6075_Data_t *)(((ULTRAVIOLET_Data_t *)(handle->pData))->pComponentData))->comboData->isUvInitialized == 0))
  {
    if(VEML6075_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((VEML6075_Data_t *)(((ULTRAVIOLET_Data_t *)(handle->pData))->pComponentData))->comboData->isUvInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the VEML6075 humidity sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef VEML6075_U_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{
  return VEML6075_Get_WhoAmI( handle, who_am_i );
}


/**
 * @brief Check the WHO_AM_I ID of the VEML6075 humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef VEML6075_U_Check_WhoAmI( DrvContextTypeDef *handle )
{
  return VEML6075_Check_WhoAmI( handle );
}


/**
 * @brief Get the humidity value of the VEML6075 humidity sensor
 * @param handle the device handle
 * @param humidity pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef VEML6075_Get_Uv( DrvContextTypeDef *handle, uint16_t *ultraviolet )
{
  return VEML6075_Get_Ultraviolet( handle, ultraviolet );
}


/**
 * @}
 */

/** @addtogroup VEML6075_Private_Functions Private functions
 * @{
 */

/**
 * @brief Initialize the VEML6075 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef VEML6075_Init( DrvContextTypeDef *handle )
{
  if ( VEML6075_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Power down the device */
  if ( VEML6075_DeActivate( (void *)handle ) == VEML6075_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  handle->isInitialized = 1;

  return COMPONENT_OK;
}

/**
 * @brief Enable the VEML6075 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef VEML6075_Sensor_Enable( DrvContextTypeDef *handle )
{

  /* Power up the device */
  if ( VEML6075_Activate( (void *)handle ) == VEML6075_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Disable the VEML6075 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef VEML6075_Sensor_Disable( DrvContextTypeDef *handle )
{

  /* Power down the device */
  if ( VEML6075_DeActivate( (void *)handle ) == VEML6075_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the VEML6075 sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef VEML6075_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  /* Read WHO AM I register */
  if ( VEML6075_Get_DeviceID( (void *)handle, who_am_i ) == VEML6075_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the VEML6075 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef VEML6075_Check_WhoAmI( DrvContextTypeDef *handle )
{

  uint8_t who_am_i = 0x00;

  if ( VEML6075_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( who_am_i != handle->who_am_i )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef VEML6075_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( VEML6075_ReadReg( (void *)handle, reg, 1, data ) == VEML6075_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef VEML6075_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( VEML6075_WriteReg( (void *)handle, reg, 1, &data ) == VEML6075_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @}
 */

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
