/**
 ******************************************************************************
 * @file    FlashManager.c
 * @date    07-October-2022
 * @brief   The Source file.
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "AT25XE041B_Driver.h"
#include "FlashManager.h"
#include "LSM6DS3_ACC_GYRO_driver.h"
#include "LIS3MDL_MAG_driver.h"
#include "HTS221_Driver.h"
#include "LPS25HB_Driver.h"

#define FLASH_ADDRESS         0
#define FLASH_FLAG_SETT       127

//#define FLASH_LOG_ENABLE

#ifdef FLASH_LOG_ENABLE
#define FLASH_LOG(...)              APP_LOG(TS_ON, VLEVEL_M, ##__VA_ARGS__)
#else
#define FLASH_LOG(...)
#endif



static FlashManegerData_t mData;

#ifdef FLASH_LOG_ENABLE
static void printData(void);
#endif

static void readData(void);
static void saveData(void);

/**
 * @brief  Initializes the FlashManager
 * @param  None
 * @retval None
 */
void InitFlashM()
{
    static bool initFlag = false;

    if (!initFlag) {
        initFlag = true;

        AT25XE041B_Init();
        readData();

        if (mData.mIsSett != FLASH_FLAG_SETT) {
            FlashManagersetDefaultSett();
        }

#ifdef FLASH_LOG_ENABLE
        printData();
#endif
        AT25XE041B_EnterUltraDeepPowerDown();
    }
}

/**
 * @brief  Clear Flash Memory
 * @param  None
 * @retval None
 */
void FlashMemoryClearAll(void)
{
    AT25XE041B_ExitUltraDeepPowerDown();
    AT25XE041B_EraseAll();
    AT25XE041B_EnterUltraDeepPowerDown();
}

/**
* @brief  Get settings Accelerometer sensor
* @param  fullScale - pointer to uint8_t variable
*         odr - pointer to uint8_t variable
* @retval true - successful, false - error
*/
bool FlashMemoryGetAccSett(uint16_t *fullScale, uint16_t *odr)
{
    if (fullScale == NULL || odr == NULL) {
        FLASH_LOG("FLASH_ERROR, Accelerometer NULL");
        return false;
    }
    *fullScale = mData.mAccFullSc;
    *odr = mData.mAccOdr;

	return true;
}

/**
* @brief  Get settings Gyroscope sensor
* @param  fullScale - pointer to uint8_t variable
*         odr - pointer to uint8_t variable
* @retval true - successful, false - error
*/
bool FlashMemoryGetGyroSett(uint16_t *fullScale, uint16_t *odr)
{
   if (fullScale == NULL || odr == NULL) {
       FLASH_LOG("FLASH_ERROR, Gyroscope NULL");
       return false;
   }
   *fullScale = mData.mGyroFullSc;
   *odr = mData.mGyroOdr;

   return true;
}

/**
* @brief  Read settings Magnetic sensor
* @param  fullScale - pointer to uint8_t variable
*         odr - pointer to uint8_t variable
* @retval true - successful, false - error
*/
bool FlashManagerGetMagSett(uint8_t *fullScale, float *odr)
{
    if (fullScale == NULL || odr == NULL) {
        FLASH_LOG("FLASH_ERROR, Magnetic NULL");
        return false;
    }
    *fullScale = mData.mMagFullSc;
    *odr = (float)mData.mMagOdr / 100.0;

    return true;
}

/**
 * @brief  Read status Pressure sensor
 * @param  odr - pointer to uint8_t variable
 * @retval true - successful, false - error
 */
bool FlashManagerGetPressureSett(float *odr)
{
    if (odr == NULL) {
        FLASH_LOG("FLASH_ERROR, Pressure NULL");
        return false;
    }

    *odr = (float)mData.mPressOdr / 100.0;

    return true;
}

/**
 * @brief  Read settings Humidity sensor
 * @param  odr - pointer to uint8_t variable
 * @retval true - successful, false - error
 */
bool FlashManagerGetHumTempSett(float *odr)
{
    if (odr == NULL) {
        FLASH_LOG("FLASH_ERROR, Humidity NULL");
        return false;
    }

    *odr = (float)mData.mHumTempOdr / 100.0;

    return true;
}

/**
 * @brief  Set default settings
 * @param  None
 * @retval None
 */
void FlashManagersetDefaultSett(void)
{
    mData.mIsSett = FLASH_FLAG_SETT;
    mData.mAccFullSc = 2;
    mData.mAccOdr = 13;
    mData.mGyroFullSc = 245;
    mData.mGyroOdr = 13;
    mData.mMagFullSc = 4;
    mData.mMagOdr = 1.25 * 100;
    mData.mPressOdr = 12.5 * 100;
    mData.mHumTempOdr = 12.5 * 100;
    saveData();
    FLASH_LOG("FLASH, Set Default Settings\r\n");
}

/**
 * @brief  Read data with EEPROM
 * @param  None
 * @retval None
 */
void readData(void)
{
    AT25XE041B_ExitUltraDeepPowerDown();
    AT25XE041B_ReadByteArray(FLASH_ADDRESS, (uint8_t*)&mData, sizeof(mData));
    AT25XE041B_EnterUltraDeepPowerDown();
}

/**
 * @brief  Save data to EEPROM
 * @param  None
 * @retval None
 */
void saveData(void)
{
    uint16_t len = 0;
    AT25XE041B_ExitUltraDeepPowerDown();
    AT25XE041B_WriteByteArray(FLASH_ADDRESS, (uint8_t*)&mData, &len, sizeof(mData));
    AT25XE041B_EnterUltraDeepPowerDown();
}

FlashManegerData_t* FlashManagerGetParam(void)
{
    return &mData;
}

void FlashManagerSetParam(uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0) {
        return;
    }

    if (length >= sizeof(mData) - 2) {
        mData.mAccFullSc = data[0];
        mData.mAccOdr = ((uint16_t)data[2] << 8) | data[1];
        mData.mGyroFullSc = ((uint16_t)data[4] << 8) | data[3];
        mData.mGyroOdr = ((uint16_t)data[6] << 8) | data[5];
        mData.mMagFullSc = data[7];
        mData.mMagOdr = ((uint16_t)data[9] << 8) | data[8];
        mData.mPressOdr = ((uint16_t)data[11] << 8) | data[10];
        mData.mHumTempOdr = ((uint16_t)data[13] << 8) | data[12];
        saveData();
    }
}

#ifdef FLASH_LOG_ENABLE
/**
 * @brief  Print Data Flash
 * @param  None
 * @retval None
 */
void printData(void)
{
    FLASH_LOG("...Print Data Flash...\r\n");
    FLASH_LOG("FLASH_ACC.EN=%d;FS=%d;ODR=%d\r\n", mData.mAccEn, mData.mAccFullSc, mData.mAccOdr);
    FLASH_LOG("FLASH_GYRO.EN=%d;FS=%d;ODR=%d\r\n", mData.mGyroEn, mData.mGyroFullSc, mData.mGyroOdr);
    FLASH_LOG("FLASH_MAG.EN=%d;ODR=%d\r\n", mData.mMagEn, mData.mMagOdr);
    FLASH_LOG("FLASH_PRESS.EN=%d;ODR=%d\r\n", mData.mPressEn, mData.mPressOdr);
    FLASH_LOG("FLASH_HUM.EN=%d;ODR=%d;AVG=%d\r\n", mData.mHumEn, mData.mHumOdr, mData.mHumAvg);
    FLASH_LOG("FLASH_TEMP.EN=%d;AVG=%d\r\n", mData.mTempEn, mData.mTempAvg);
    FLASH_LOG("FLASH_LIGHT.EN=%d;BIT=%d;MS=%d\r\n", mData.mLightEn, mData.mLightBit, mData.mLightMs);
}
#endif
