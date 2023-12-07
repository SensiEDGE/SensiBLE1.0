/**
 ******************************************************************************
 * @file    FlashManager.h
 * @date    07-October-2022
 * @brief   The FlashManager description.
 *
 ******************************************************************************
 *
 * COPYRIGHT(c) 2022 SensiEDGE
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of SensiEDGE LTD nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
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
#ifndef __FLASH_MANAGER_H_
#define __FLASH_MANAGER_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct Data {
    uint8_t mIsSett;
    uint8_t mAccFullSc;
    uint16_t mAccOdr;
    uint16_t mGyroFullSc;
    uint16_t mGyroOdr;
    uint8_t mMagFullSc;
    uint16_t mMagOdr;
    uint16_t mPressOdr;
    uint16_t mHumTempOdr;
} FlashManegerData_t;

/**
 * @brief  Clear Flash Memory
 * @param  None
 * @retval None
 */
void FlashMemoryClearAll(void);

/**
 * @brief  Initializes the FlashManager
 * @param  None
 * @retval None
 */
void InitFlashM();

/**
* @brief  Get settings Accelerometer sensor
* @param  fullScale - pointer to uint8_t variable
*         odr - pointer to uint8_t variable
* @retval true - successful, false - error
*/
bool FlashMemoryGetAccSett(uint16_t *fullScale, uint16_t *odr);

/**
* @brief  Get settings Gyroscope sensor
* @param  fullScale - pointer to uint8_t variable
*         odr - pointer to uint8_t variable
* @retval true - successful, false - error
*/
bool FlashMemoryGetGyroSett(uint16_t *fullScale, uint16_t *odr);

/**
* @brief  Read settings Magnetic sensor
* @param  odr - pointer to uint8_t variable
* @retval true - successful, false - error
*/
bool FlashManagerGetMagSett(uint8_t *fullScale, float *odr);

/**
 * @brief  Read status Pressure sensor
 * @param  odr - pointer to uint8_t variable
 * @retval true - successful, false - error
 */
bool FlashManagerGetPressureSett(float *odr);

/**
 * @brief  Read setttings Humidity sensor
 * @param  avg - pointer to uint8_t variable
 *         odr - pointer to uint8_t variable
 * @retval true - successful, false - error
 */
bool FlashManagerGetHumTempSett(float *odr);

/**
 * @brief  Set default settings
 * @param  None
 * @retval None
 */
void FlashManagersetDefaultSett(void);

FlashManegerData_t* FlashManagerGetParam(void);

void FlashManagerSetParam(uint8_t *data, uint16_t length);

#endif /* __FLASH_MANAGER_H_ */
