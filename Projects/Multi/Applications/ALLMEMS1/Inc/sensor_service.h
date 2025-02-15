/**
  ******************************************************************************
  * @file    sensor_service.h 
  * @author  Central LAB
  * @version V3.0.0
  * @date    12-May-2017
  * @brief   Sensors services APIs
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
#ifndef _SENSOR_SERVICE_H_
#define _SENSOR_SERVICE_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"

#include "hal_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "gp_timer.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"   
#include "hci.h"
#include "hal.h"
#include "sm.h"
#include "debug.h"
#include "uuid_ble_service.h"

#include <stdlib.h>
#include "main.h"

/* Exported functions ------------------------------------------------------- */
extern tBleStatus Add_HW_SW_ServW2ST_Service(void);
extern tBleStatus AccGyroMag_Update(SensorAxes_t *Acc,SensorAxes_t *Gyro,SensorAxes_t *Mag);
extern tBleStatus AccEvent_Notifi(uint8_t event);
extern tBleStatus AccEventSteps_Notify(uint8_t event, uint16_t steps);
extern tBleStatus AccEventSteps_Notifi(uint16_t steps);
extern tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1);

/* Code for MotionFX integration - Start Section */
extern tBleStatus Quat_Update(SensorAxes_t *data);
extern tBleStatus ECompass_Update(uint16_t Angle);
/* Code for MotionFX integration - End Section */

extern tBleStatus AudioLevel_Update(uint16_t *Mic);

extern tBleStatus LED_Update(uint8_t LedStatus);
extern tBleStatus Prox_Update(uint16_t ProxValue);
#if ENABLE_UV_SENSOR == 1
extern tBleStatus Uv_Update(uint16_t UvValue);
extern tBleStatus Co_Lux_Update(uint32_t CoValue, uint16_t LuxValue);
#else
extern tBleStatus Lux_Update(uint16_t LuxValue);
#endif

#ifdef USE_SENSIBLE
extern tBleStatus BAT_Update(uint16_t soc, uint16_t voltage, uint16_t current, uint8_t status);
#endif /* USE_SENSIBLE */

#ifdef STM32_SENSORTILE
extern tBleStatus GG_Update(uint32_t soc, uint32_t voltage, int32_t current);
#endif /* STM32_SENSORTILE */

/* Code for MotionAR integration - Start Section */
extern tBleStatus ActivityRec_Update(MAR_output_t ActivityCode);
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
extern tBleStatus CarryPosRec_Update(MCP_output_t CarryPositionCode);
#endif /* ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONGR
extern tBleStatus GestureRec_Update(MGR_output_t GestureCode);
#endif /* ALLMEMS1_MOTIONGR */

#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION
extern tBleStatus AudioSourceLocalization_Update(uint16_t Angle);
#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */

extern tBleStatus Add_ConsoleW2ST_Service(void);
extern tBleStatus Stderr_Update(uint8_t *data,uint8_t length);
extern tBleStatus Term_Update(uint8_t *data,uint8_t length);

extern tBleStatus Add_ConfigW2ST_Service(void);
extern tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t val);

extern void       stopAdvertise(void);
extern void       disconnect(void);
extern void       setConnectable(void);

extern void       setIbeacon(void);

extern void       HCI_Event_CB(void *pckt);

/* Exported variables --------------------------------------------------------*/

/* For enabling the capability to handle BlueNRG Congestion */
#define ACC_BLUENRG_CONGESTION

#ifdef ACC_BLUENRG_CONGESTION
/* For defining how many events skip when there is a congestion */
#define ACC_BLUENRG_CONGESTION_SKIP 30
#endif /* ACC_BLUENRG_CONGESTION */

/*************** Don't Change the following defines *************/

/* Define the Max dimesion of the Bluetooth characteristics
for each packet used for Console Service */
#define W2ST_CONSOLE_MAX_CHAR_LEN 20

/* Define the symbol used for defining each termination string
used in Console service */
#define W2ST_CONSOLE_END_STRING "\0"

/* @brief  Scale factor. It is used to scale acceleration from mg to g */ 
#define FROM_MG_TO_G    0.001

/* Feature mask for Sensor fusion short precision */
#define FEATURE_MASK_SENSORFUSION_SHORT 0x00000100

/* Feature mask for e-compass */
#define FEATURE_MASK_ECOMPASS 0x00000040

/* Feature mask for hardware events */
#define FEATURE_MASK_ACC_EVENTS 0x00000400

/* Feature mask for Temperature1 */
#define FEATURE_MASK_TEMP1 0x00040000

/* Feature mask for Temperature2 */
#define FEATURE_MASK_TEMP2 0x00010000

/* Feature mask for Pressure */
#define FEATURE_MASK_PRESS 0x00100000

/* Feature mask for Humidity */
#define FEATURE_MASK_HUM   0x00080000

/* Feature mask for Accelerometer */
#define FEATURE_MASK_ACC   0x00800000

/* Feature mask for Gyroscope */
#define FEATURE_MASK_GRYO  0x00400000

/* Feature mask for Magnetometer */
#define FEATURE_MASK_MAG   0x00200000

/* Feature mask for Microphone */
#define FEATURE_MASK_MIC   0x04000000

/* Feature mask for Led switch */
#define FEATURE_MASK_LED_SWITCH  0x20000000

/* W2ST command for asking the calibration status */
#define W2ST_COMMAND_CAL_STATUS 0xFF
/* W2ST command for resetting the calibration */
#define W2ST_COMMAND_CAL_RESET  0x00
/* W2ST command for stopping the calibration process */
#define W2ST_COMMAND_CAL_STOP   0x01


/* BLE Characteristic connection control */
/* Environmental Data */
#define W2ST_CONNECT_ENV           (1   )

/* Acceleration/Gyroscope/Magneto */
#define W2ST_CONNECT_ACC_GYRO_MAG  (1<<2)
/* Mic */
#define W2ST_CONNECT_AUDIO_LEVEL   (1<<3)

#define W2ST_CONNECT_LED           (1<<1 )



/* Code for MotionFX integration - Start Section */
/* Quaternions */
#define W2ST_CONNECT_QUAT          (1<<4)
/* ECompass Feature */
#define W2ST_CONNECT_EC            (1<<5)
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
#define W2ST_CONNECT_AR            (1<<6 )
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
#define W2ST_CONNECT_CP            (1<<7 )
#endif /* ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONGR
#define W2ST_CONNECT_GR            (1<<8 )
#endif /* ALLMEMS1_MOTIONGR */

/* Audio Source Localization */
#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION
  #define W2ST_CONNECT_SL          (1<<9)
#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */

/* Standard Terminal */
#define W2ST_CONNECT_STD_TERM      (1<<10)
/* Standard Error */
#define W2ST_CONNECT_STD_ERR       (1<<11)
/* HW Advance Features */
#define W2ST_CONNECT_ACC_EVENT     (1<<12)

/* Code for BlueVoice integration - Start Section */
#define W2ST_CONNECT_BV_AUDIO      (1<<13)
#define W2ST_CONNECT_BV_SYNC       (1<<14)
/* Code for BlueVoice integration - End Section */

/* Gas Gouge Feature */
#define W2ST_CONNECT_GG_EVENT      (1<<15)
#define W2ST_CONNECT_PROX          (1<<16)
#define W2ST_CONNECT_UV            (1<<17)
#define W2ST_CONNECT_CO            (1<<18)

#define W2ST_CONNECT_BAT_EVENT      (1 << 29)
/* Lux sensor */
#define W2ST_CONNECT_LUX           (1 << 30) //add2



#define W2ST_CHECK_CONNECTION(BleChar) (((ConnectionBleStatus)&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    ((ConnectionBleStatus)|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   ((ConnectionBleStatus)&=(~BleChar))

#ifdef __cplusplus
}
#endif

#endif /* _SENSOR_SERVICE_H_ */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
