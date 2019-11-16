/**
 ******************************************************************************
 * @file    x_nucleo_iks01a1_uv.h
 * @author  
 * @version V1.0.0
 * @date    08-August-2017
 * @brief   This file contains definitions for x_nucleo_iks01a1_uv.c
 ******************************************************************************
 * @attention
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __X_NUCLEO_IKS01A1_ULTRAVIOLET_H
#define __X_NUCLEO_IKS01A1_ULTRAVIOLET_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "VEML6075_Driver_HL.h"
#include "x_nucleo_iks01a1.h"


/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1 X_NUCLEO_IKS01A1
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_ULTRAVIOLET Ultraviolet
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_ULTRAVIOLET_Public_Types Public types
  * @{
  */

typedef enum
{
  ULTRAVIOLET_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
  VEML6075_0                        /* Default on board. */
} ULTRAVIOLET_ID_t;

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_ULTRAVIOLET_Public_Defines Public defines
  * @{
  */

#define ULTRAVIOLET_SENSORS_MAX_NUM 1

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_ULTRAVIOLET_Public_Function_Prototypes Public function prototypes
 * @{
 */

/* Sensor Configuration Functions */
DrvStatusTypeDef BSP_ULTRAVIOLET_Init( ULTRAVIOLET_ID_t id, void **handle );
DrvStatusTypeDef BSP_ULTRAVIOLET_DeInit( void **handle );
DrvStatusTypeDef BSP_ULTRAVIOLET_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_ULTRAVIOLET_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_ULTRAVIOLET_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ULTRAVIOLET_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ULTRAVIOLET_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ULTRAVIOLET_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_ULTRAVIOLET_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_ULTRAVIOLET_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_ULTRAVIOLET_Get_Uv( void *handle, uint16_t *ultraviolet );
DrvStatusTypeDef BSP_ULTRAVIOLET_Get_ODR( void *handle, float *odr );
DrvStatusTypeDef BSP_ULTRAVIOLET_Set_ODR( void *handle, SensorOdr_t odr );
DrvStatusTypeDef BSP_ULTRAVIOLET_Set_ODR_Value( void *handle, float odr );
DrvStatusTypeDef BSP_ULTRAVIOLET_Read_Reg( void *handle, uint8_t reg, uint8_t *data );
DrvStatusTypeDef BSP_ULTRAVIOLET_Write_Reg( void *handle, uint8_t reg, uint8_t data );
DrvStatusTypeDef BSP_ULTRAVIOLET_Get_DRDY_Status( void *handle, uint8_t *status );

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

#ifdef __cplusplus
}
#endif

#endif /* __X_NUCLEO_IKS01A1_ULTRAVIOLET_H_H */

/************************************************************************END OF FILE****/
