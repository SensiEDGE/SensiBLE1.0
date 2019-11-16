/**
 ******************************************************************************
 * @file    AudioBV_Manager.c
 * @author  Central LAB
 * @version V3.0.0
 * @date    12-May-2017
 * @brief   This file includes BlueVoice interface functions
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

/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"

/* Code for BlueVoice integration - Start Section */

/* Imported Variables -------------------------------------------------------------*/
extern uint16_t PCM_Buffer[];
extern BV_ADPCM_ProfileHandle_t BLUEVOICE_tx_handle;

/* Private Variables -------------------------------------------------------------*/
volatile uint8_t SendBlueVoiceADPCM;
static uint32_t led_toggle_count = 0; /*!< Variable used to handle led toggling.*/
BV_ADPCM_Config_t BLUEVOICE_Config;
BV_ADPCM_Status bvStat;

/* Private Defines -------------------------------------------------------------*/
#define LED_TOGGLE_STREAMING  100


/**
* @brief  Initialises BlueVoice manager
* @param  None
* @retval None
*/
void AudioBV_Manager_init(void)
{
  BluevoiceADPCM_Initialize();
  
#ifdef STM32_NUCLEO
  BLUEVOICE_Config.sampling_frequency = FR_8000;
  BLUEVOICE_Config.channel_in = 1;
  BLUEVOICE_Config.channel_tot = 2;
  bvStat = BluevoiceADPCM_SetConfig(&BLUEVOICE_Config);
  if (bvStat != BV_ADPCM_SUCCESS) {
    goto fail;
  }
#endif /* STM32_NUCLEO */
  
#ifdef STM32_SENSORTILE
  BLUEVOICE_Config.sampling_frequency = FR_8000;
  BLUEVOICE_Config.channel_in = 1;
  BLUEVOICE_Config.channel_tot = 1;
  bvStat = BluevoiceADPCM_SetConfig(&BLUEVOICE_Config);
  if (bvStat != BV_ADPCM_SUCCESS) {
    goto fail;
  }
#endif /* STM32_SENSORTILE */
  
  bvStat = BluevoiceADPCM_SetTxHandle(&BLUEVOICE_tx_handle);
  if (bvStat != BV_ADPCM_SUCCESS) {
    goto fail;
  }
  
  /* If everything is ok */
  TargetBoardFeatures.AudioBVIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized ST BlueVoiceADPCM v2.0.0\r\n");

  return;
  
  fail:
    #warning "Removed hang up here!"
    return;
    // while(1){}
}

/**
* @brief  User function that is called when the PCM_Buffer is full and ready to send.
* @param  none
* @retval None
*/
void AudioProcess_BV(void)
{
    BV_ADPCM_Status status;

    if (BluevoiceADPCM_IsProfileConfigured()) {
      status = BluevoiceADPCM_AudioIn((uint16_t*) PCM_Buffer, BV_PCM_AUDIO_IN_SAMPLES);
      if (led_toggle_count++ >= LED_TOGGLE_STREAMING) {
        led_toggle_count = 0;
        LedToggleTargetPlatform();
      }
      if(status==BV_ADPCM_OUT_BUF_READY) {
        SendBlueVoiceADPCM = 1;
      }
    }
}

/* Code for BlueVoice integration - End Section */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
