/**
 ******************************************************************************
 * @file    VEML6075_Driver_HL.h
 * @author
 * @version V1.0.0
 * @date    08-August-2017
 * @brief   This file contains definitions for the VEML6075_Driver_HL.c
 *          firmware driver
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VEML6075_DRIVER_HL_H
#define __VEML6075_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "ultraviolet.h"

/* Include sensor component drivers. */
#include "VEML6075_Driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup VEML6075 VEML6075
 * @{
 */

/** @addtogroup VEML6075_Public_Constants Public constants
 * @{
 */

#define VEML6075_SENSORS_MAX_NUM  1     /**< VEML6075 max number of instances */

/** @addtogroup VEML6075_I2C_Addresses VEML6075 I2C Addresses
 * @{
 */

#define VEML6075_ADDRESS_DEFAULT  0x20  /**< VEML6075 I2C Address */

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup VEML6075_Public_Types VEML6075 Public Types
 * @{
 */

/**
 * @brief VEML6075 combo specific data internal structure definition
 */

typedef struct
{
  uint8_t isHumInitialized;
  uint8_t isTempInitialized;
  uint8_t isHumEnabled;
  uint8_t isTempEnabled;
  uint8_t isUvInitialized;
} VEML6075_Combo_Data_t;

/**
 * @brief VEML6075 humidity specific data internal structure definition
 */

typedef struct
{
  VEML6075_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} VEML6075_H_Data_t;


/**
 * @brief VEML6075 temperature specific data internal structure definition
 */

typedef struct
{
  VEML6075_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} VEML6075_Data_t;

/**
 * @}
 */

/** @addtogroup VEML6075_Public_Variables Public variables
 * @{
 */

extern ULTRAVIOLET_Drv_t VEML6075_Drv;
extern VEML6075_Combo_Data_t VEML6075_Combo_Data[VEML6075_SENSORS_MAX_NUM];

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

#endif /* __VEML6075_DRIVER_HL_H */

/***************************************************************END OF FILE****/
