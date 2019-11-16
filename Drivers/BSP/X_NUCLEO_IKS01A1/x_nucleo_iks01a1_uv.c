/**
 ******************************************************************************
 * @file    x_nucleo_iks01a1_uv.c
 * @author  
 * @version V1.0.0
 * @date    08-August-2017
 * @brief   This file provides a set of functions needed to manage
 *          the ultraviolet sensor
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
#include "x_nucleo_iks01a1_uv.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1 X_NUCLEO_IKS01A1
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_ULTRAVIOLET Ultraviolet
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_ULTRAVIOLET_Private_Variables Private variables
 * @{
 */

static DrvContextTypeDef ULTRAVIOLET_SensorHandle[ ULTRAVIOLET_SENSORS_MAX_NUM ];
static ULTRAVIOLET_Data_t ULTRAVIOLET_Data[ ULTRAVIOLET_SENSORS_MAX_NUM ]; // Ultraviolet - all.
static VEML6075_Data_t VEML6075_0_Data; // Ultraviolet - sensor 0.

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_ULTRAVIOLET_Private_FunctionPrototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef BSP_VEML6075_ULTRAVIOLET_Init( void **handle );

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_ULTRAVIOLET_Public_Functions Public functions
 * @{
 */

/**
 * @brief Initialize a ultraviolet sensor
 * @param id the ultraviolet sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ULTRAVIOLET_Init( ULTRAVIOLET_ID_t id, void **handle )
{
  *handle = NULL;

  switch(id)
  {
    case ULTRAVIOLET_SENSORS_AUTO:
    default:
    {
      if( BSP_VEML6075_ULTRAVIOLET_Init(handle)  == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
    case VEML6075_0:
    {
      if( BSP_VEML6075_ULTRAVIOLET_Init(handle)  == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
  }

  return COMPONENT_OK;
}



static DrvStatusTypeDef BSP_VEML6075_ULTRAVIOLET_Init( void **handle )
{
  ULTRAVIOLET_Drv_t *driver = NULL;

  if(ULTRAVIOLET_SensorHandle[ VEML6075_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }

  if ( Sensor_IO_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  ULTRAVIOLET_SensorHandle[ VEML6075_0 ].who_am_i      = VEML6075_WHO_AM_I_VAL;
  ULTRAVIOLET_SensorHandle[ VEML6075_0 ].address       = VEML6075_ADDRESS_DEFAULT;
  ULTRAVIOLET_SensorHandle[ VEML6075_0 ].instance      = VEML6075_0;
  ULTRAVIOLET_SensorHandle[ VEML6075_0 ].isInitialized = 0;
  ULTRAVIOLET_SensorHandle[ VEML6075_0 ].isEnabled     = 0;
  ULTRAVIOLET_SensorHandle[ VEML6075_0 ].isCombo       = 0;
  ULTRAVIOLET_SensorHandle[ VEML6075_0 ].pData         = ( void * )&ULTRAVIOLET_Data[ VEML6075_0 ];
  ULTRAVIOLET_SensorHandle[ VEML6075_0 ].pVTable       = ( void * )&VEML6075_Drv;
  ULTRAVIOLET_SensorHandle[ VEML6075_0 ].pExtVTable    = 0;

  VEML6075_0_Data.comboData = &VEML6075_Combo_Data[0];
  ULTRAVIOLET_Data[ VEML6075_0 ].pComponentData = ( void * )&VEML6075_0_Data;
  ULTRAVIOLET_Data[ VEML6075_0 ].pExtData       = 0;

  *handle = (void *)&ULTRAVIOLET_SensorHandle[ VEML6075_0 ];

  driver = ( ULTRAVIOLET_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;

  if ( driver->Init == NULL )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if ( driver->Init( (DrvContextTypeDef *)(*handle) ) == COMPONENT_ERROR )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize a ultraviolet sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ULTRAVIOLET_DeInit( void **handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  ULTRAVIOLET_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ULTRAVIOLET_Drv_t * )ctx->pVTable;

  if ( driver->DeInit == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->DeInit( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  memset(ctx, 0, sizeof(DrvContextTypeDef));

  *handle = NULL;

  return COMPONENT_OK;
}



/**
 * @brief Enable ultraviolet sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ULTRAVIOLET_Sensor_Enable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ULTRAVIOLET_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ULTRAVIOLET_Drv_t * )ctx->pVTable;

  if ( driver->Sensor_Enable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Sensor_Enable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable ultraviolet sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ULTRAVIOLET_Sensor_Disable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ULTRAVIOLET_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ULTRAVIOLET_Drv_t * )ctx->pVTable;

  if ( driver->Sensor_Disable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Sensor_Disable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Check if the ultraviolet sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ULTRAVIOLET_IsInitialized( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isInitialized;

  return COMPONENT_OK;
}


/**
 * @brief Check if the ultraviolet sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ULTRAVIOLET_IsEnabled( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isEnabled;

  return COMPONENT_OK;
}


/**
 * @brief Check if the ultraviolet sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ULTRAVIOLET_IsCombo( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isCombo;

  return COMPONENT_OK;
}


/**
 * @brief Get the ultraviolet sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ULTRAVIOLET_Get_Instance( void *handle, uint8_t *instance )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( instance == NULL )
  {
    return COMPONENT_ERROR;
  }

  *instance = ctx->instance;

  return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the ultraviolet sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ULTRAVIOLET_Get_WhoAmI( void *handle, uint8_t *who_am_i )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ULTRAVIOLET_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ULTRAVIOLET_Drv_t * )ctx->pVTable;

  if ( who_am_i == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_WhoAmI( ctx, who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the ultraviolet sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ULTRAVIOLET_Check_WhoAmI( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ULTRAVIOLET_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ULTRAVIOLET_Drv_t * )ctx->pVTable;

  if ( driver->Check_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Check_WhoAmI( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the ultraviolet value
 * @param handle the device handle
 * @param ultraviolet pointer where the value is written [%]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ULTRAVIOLET_Get_Uv( void *handle, uint16_t *ultraviolet )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ULTRAVIOLET_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ULTRAVIOLET_Drv_t * )ctx->pVTable;

  if ( ultraviolet == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Uv == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Uv( ctx, ultraviolet ) == COMPONENT_ERROR )
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
DrvStatusTypeDef BSP_ULTRAVIOLET_Read_Reg( void *handle, uint8_t reg, uint8_t *data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ULTRAVIOLET_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ULTRAVIOLET_Drv_t * )ctx->pVTable;

  if(data == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Read_Reg == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Read_Reg( ctx, reg, data ) == COMPONENT_ERROR )
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
DrvStatusTypeDef BSP_ULTRAVIOLET_Write_Reg( void *handle, uint8_t reg, uint8_t data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  ULTRAVIOLET_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( ULTRAVIOLET_Drv_t * )ctx->pVTable;

  if ( driver->Write_Reg == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Write_Reg( ctx, reg, data ) == COMPONENT_ERROR )
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

/******************************END OF FILE*************************************/
