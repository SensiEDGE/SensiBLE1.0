/**
 ******************************************************************************
 * @file    VEML6075_Driver.h
 * @author  
 * @version V1.0
 * @date    9-August-2017
 * @brief   VEML6075 driver header file
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
#ifndef __VEML6075_DRIVER__H
#define __VEML6075_DRIVER__H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Uncomment the line below to expanse the "assert_param" macro in the drivers code */
#define USE_FULL_ASSERT_VEML6075

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT_VEML6075

/**
* @brief  The assert_param macro is used for function's parameters check.
* @param  expr: If expr is false, it calls assert_failed function which reports
*         the name of the source file and the source line number of the call
*         that failed. If expr is true, it returns no value.
* @retval None
*/
#define VEML6075_assert_param(expr) ((expr) ? (void)0 : VEML6075_assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void VEML6075_assert_failed(uint8_t* file, uint32_t line);
#else
#define VEML6075_assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT_VEML6075 */

/** @addtogroup Environmental_Sensor
* @{
*/

/** @addtogroup VEML6075_DRIVER
* @{
*/

/* Exported Types -------------------------------------------------------------*/
/** @defgroup VEML6075_Exported_Types
* @{
*/


/**
* @brief  Error code type.
*/
typedef enum {VEML6075_OK = (uint8_t)0, VEML6075_ERROR = !VEML6075_OK} VEML6075_Error_et;

/**
* @brief  State type.
*/
typedef enum {VEML6075_DISABLE = (uint8_t)0, VEML6075_ENABLE = !VEML6075_DISABLE} VEML6075_State_et;
#define IS_VEML6075_State(MODE) ((MODE == VEML6075_ENABLE) || (MODE == VEML6075_DISABLE))

/**
* @brief  Bit status type.
*/
typedef enum {VEML6075_RESET = (uint8_t)0, VEML6075_SET = !VEML6075_RESET} VEML6075_BitStatus_et;
#define IS_VEML6075_BitStatus(MODE) ((MODE == VEML6075_RESET) || (MODE == VEML6075_SET))

/**
* @brief  Ultraviolet integration time
*/
typedef enum
{
  VEML6075_UV_IT_50MS     = (uint8_t)0x00,
  VEML6075_UV_IT_100MS    = (uint8_t)0x10,
  VEML6075_UV_IT_200MS    = (uint8_t)0x20,
  VEML6075_UV_IT_400MS    = (uint8_t)0x30,
  VEML6075_UV_IT_800MS    = (uint8_t)0x40
} VEML6075_UvIt_et;

#define IS_VEML6075_UV_IT(UV_IT) ((UV_IT == VEML6075_UV_IT_50MS) \
                                || (UV_IT == VEML6075_UV_IT_100MS) \
                                || (UV_IT == VEML6075_UV_IT_200MS) \
                                || (UV_IT == VEML6075_UV_IT_400MS) \
                                || (UV_IT == VEML6075_UV_IT_800MS)

/**
* @brief  Dynamic settings
*/
typedef enum
{
  VEML6075_NORMAL_DYNAMIC = (uint8_t)0x00,  /*!< Normal dynamic setting */
  VEML6075_HIGH_DYNAMIC   = (uint8_t)0x08,  /*!< High dynamic setting */
} VEML6075_Hd_et;
#define IS_VEML6075_DYMAMIC(HD) ((HD == VEML6075_NORMAL_DYNAMIC) || (HD == VEML6075_HIGH_DYNAMIC)


/**
* @brief  Force mode trigger
*/
typedef enum
{
  VEML6075_UV_TRIG_NO_FORCE = (uint8_t)0x00,
  VEML6075_UV_TRIG_ONE      = (uint8_t)0x04
} VEML6075_ForceTrig_et;
#define IS_VEML6075_UV_TRIG(TRIG_MODE) ((TRIG_MODE == VEML6075_UV_TRIG_NO_FORCE) || (TRIG_MODE == VEML6075_UV_TRIG_ONE))


/**
* @brief  Active Force mode
*/
typedef enum
{
  VEML6075_NORMAL_MODE      = (uint8_t)0x00,
  VEML6075_FORCE_MODE       = (uint8_t)0x02    /*!< LOW state level for DRDY pin */
} VEML6075_Mode_et;
#define IS_VEML6075_UV_AF(MODE) ((MODE == VEML6075_NORMAL_MODE) || (MODE == VEML6075_FORCE_MODE))


/**
* @brief  Shut down/Power on
*/
typedef enum
{
  VEML6075_POWERED          = (uint8_t)0x00,
  VEML6075_SHUT_DOWN        = (uint8_t)0x02    /*!< LOW state level for DRDY pin */
} VEML6075_Power_et;
#define IS_VEML6075_PWR_SD() ((MODE == VEML6075_NORMAL_MODE) || (MODE == VEML6075_FORCE_MODE))


/**
* @brief  Driver Version Info structure definition.
*/
typedef struct
{
  uint8_t   Major;
  uint8_t   Minor;
  uint8_t   Point;
} VEML6075_DriverVersion_st;


/**
* @brief  VEML6075 Init structure definition.
*/
typedef struct
{
  VEML6075_UvIt_et        it_uv;            /*!< Ultraviolet integration time*/
  VEML6075_Hd_et          hd;               /*!< Dynamic setting */
  VEML6075_ForceTrig_et   force_trig;       /*!< VEML6075_ENABLE/VEML6075_DISABLE the block data update */
  VEML6075_Mode_et        mode;             /*!< Active force mode */
  VEML6075_Power_et       sd;               /*!< Shut down */
} VEML6075_Init_st;

/**
* @}
*/


/* Exported Constants ---------------------------------------------------------*/
/** @defgroup VEML6075_Exported_Constants
* @{
*/

/**
* @brief  Bitfield positioning.
*/
#define VEML6075_BIT(x) ((uint8_t)x)

/**
* @brief  I2C address.
*/
#define VEML6075_I2C_ADDRESS  (uint8_t)0x10

/**
* @brief  Driver version.
*/
#define VEML6075_DRIVER_VERSION_MAJOR (uint8_t)1
#define VEML6075_DRIVER_VERSION_MINOR (uint8_t)1
#define VEML6075_DRIVER_VERSION_POINT (uint8_t)0

/**
* @addtogroup VEML6075_Registers
* @{
*/


/**
* @brief Device Identification register.
* \code
* Read
* Default value: 0x0C
* 7:0 This read-only register contains the device identifier for VEML6075.
* \endcode
*/
#define VEML6075_WHO_AM_I_REG          (uint8_t)0x0C

/**
* @brief Device Identification value.
*/
#define VEML6075_WHO_AM_I_VAL         (uint8_t)0x26


#define VEML6075_UV_CONF_REG1         (uint8_t)0x00

//#define VEML6075_PD_BIT          VEML6075_BIT(7)
//#define VEML6075_BDU_BIT         VEML6075_BIT(2)
//#define VEML6075_ODR_BIT         VEML6075_BIT(0)
//
#define VEML6075_SD_MASK        (uint8_t)0x01
//#define VEML6075_BDU_MASK       (uint8_t)0x04
//#define VEML6075_ODR_MASK       (uint8_t)0x03
//
///**
//* @brief Control register 2.
//* \code
//* Read/write
//* Default value: 0x00
//* 7 BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content. Self-cleared upon completation.
//* 6:2 Reserved.
//* 1 HEATHER: 0: heater enable; 1: heater disable.
//* 0 ONE_SHOT: 0: waiting for start of conversion; 1: start for a new dataset. Self-cleared upon completation.
//* \endcode
//*/
//#define VEML6075_CTRL_REG2      (uint8_t)0x21
//
//#define VEML6075_BOOT_BIT        VEML6075_BIT(7)
//#define VEML6075_HEATHER_BIT     VEML6075_BIT(1)
//#define VEML6075_ONESHOT_BIT     VEML6075_BIT(0)
//
//#define VEML6075_BOOT_MASK      (uint8_t)0x80
//#define VEML6075_HEATHER_MASK   (uint8_t)0x02
//#define VEML6075_ONE_SHOT_MASK  (uint8_t)0x01
//
///**
//* @brief Control register 3.
//* \code
//* Read/write
//* Default value: 0x00
//* 7 DRDY_H_L: Interrupt edge. 0: active high, 1: active low.
//* 6 PP_OD: Push-Pull/OpenDrain selection on interrupt pads. 0: push-pull; 1: open drain.
//* 5:3 Reserved.
//* 2 DRDY: interrupt config. 0: disable, 1: enable.
//* \endcode
//*/
//#define VEML6075_CTRL_REG3      (uint8_t)0x22
//
//#define VEML6075_DRDY_H_L_BIT    VEML6075_BIT(7)
//#define VEML6075_PP_OD_BIT       VEML6075_BIT(6)
//#define VEML6075_DRDY_BIT        VEML6075_BIT(2)
//
//#define VEML6075_DRDY_H_L_MASK  (uint8_t)0x80
//#define VEML6075_PP_OD_MASK     (uint8_t)0x40
//#define VEML6075_DRDY_MASK      (uint8_t)0x04
//
///**
//* @brief  Status register.
//* \code
//* Read
//* Default value: 0x00
//* 7:2 Reserved.
//* 1 H_DA: Humidity data available. 0: new data for humidity is not yet available; 1: new data for humidity is available.
//* 0 T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
//* \endcode
//*/
//#define VEML6075_STATUS_REG    (uint8_t)0x27
//
//#define VEML6075_H_DA_BIT       VEML6075_BIT(1)
//#define VEML6075_T_DA_BIT       VEML6075_BIT(0)
//
//#define VEML6075_HDA_MASK      (uint8_t)0x02
//#define VEML6075_TDA_MASK      (uint8_t)0x01
//


/**
* @brief  UVA_data (LSB_MSB).
* \code
* Read
* Default value: 0x00.
* \endcode
*/
#define VEML6075_UVA_DATA_REG           (uint8_t)0x07

/**
* @brief  UVB_data (LSB_MSB).
* \code
* Read
* Default value: 0x00.
* \endcode
*/
#define VEML6075_UVB_DATA_REG           (uint8_t)0x09

/**
* @brief  UVCOMP1_data (LSB_MSB).
* \code
* Read
* Default value: 0x00.
* \endcode
*/
#define VEML6075_UVCOMP1_DATA_REG       (uint8_t)0x0A

/**
* @brief  UVCOMP2_data (LSB_MSB).
* \code
* Read
* Default value: 0x00.
* \endcode
*/
#define VEML6075_UVCOMP2_DATA_REG       (uint8_t)0x0A


/**
* @}
*/


/**
* @}
*/


/* Exported Functions -------------------------------------------------------------*/
/** @defgroup VEML6075_Exported_Functions
* @{
*/

VEML6075_Error_et VEML6075_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
VEML6075_Error_et VEML6075_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data );

VEML6075_Error_et VEML6075_Get_DriverVersion(VEML6075_DriverVersion_st* version);
VEML6075_Error_et VEML6075_Get_DeviceID(void *handle, uint8_t* deviceid);

VEML6075_Error_et VEML6075_DeInit(void *handle);

VEML6075_Error_et VEML6075_Get_Measurement(void *handle, uint16_t* ultraviolet);
VEML6075_Error_et VEML6075_Get_Ultraviolet(void *handle, uint16_t* value);
VEML6075_Error_et VEML6075_Activate(void *handle);
VEML6075_Error_et VEML6075_DeActivate(void *handle);

VEML6075_Error_et VEML6075_Set_PowerDownMode(void *handle, VEML6075_BitStatus_et status);
VEML6075_Error_et VEML6075_Get_PowerDownMode(void *handle, VEML6075_BitStatus_et* status);

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

#endif /* __VEML6075_DRIVER__H */

/***************************************************************END OF FILE****/
