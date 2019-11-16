 /**
 ******************************************************************************
 * @file    AcousticSL_Manager.h
 * @author  Central Lab
 * @version V3.0.0
 * @date    12-May-2017
 * @brief   Header for AcousticSL_Manager.c
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
#ifndef _ACOUSTICSL_MANAGER_H_
#define _ACOUSTICSL_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Define Acoustic Source Localization for static parameters */
//#define DIAGONAL        57
//#define SIDE            40
// 
//#define TOP_RIGHT_MIC 1
//#define TOP_LEFT_MIC 3
//#define BOTTOM_RIGHT_MIC 0
//#define BOTTOM_LEFT_MIC 2
  
//#define TOP_RIGHT_MIC 2
//#define TOP_LEFT_MIC 3
//#define BOTTOM_RIGHT_MIC 0
//#define BOTTOM_LEFT_MIC 1

//#define TOP_LEFT_MIC 3
//#define TOP_RIGHT_MIC 0
//#define BOTTOM_LEFT_MIC 2
//#define BOTTOM_RIGHT_MIC 1
  
/* Define Acoustic Source Localization for dynamic parameters */
#define MIC_LEVEL_THRESHOLD     24  /* 800 for old SL Version */
#define ANGLE_RESOLUTION        10
  
/* Define Source Localization for static parameters 
   - Do not change: depends on to Acoustic Source Localization static parameters */
#define SOURCE_LOCALIZATION_MEMORY_SIZE   0x43E6
  
//#define PCM_TEMP_BUFF_SIZE      AUDIO_SAMPLING_FREQUENCY/1000*1

#define ACOUSTIC_SL_NO_AUDIO_DETECTED              -100


/* Exported Functions Prototypes ---------------------------------------------*/
extern void AcousticSL_Manager_init(void);
extern void SW_Task2_Callback(void);
extern void AudioProcess_SL(void);


#ifdef __cplusplus
}
#endif

#endif //_ACOUSTICSL_MANAGER_H_

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

