/**
 ******************************************************************************
 * @file    AT25XE041B_Driver.c
 * @date    02-July-2019
 * @brief   AT25XE041B driver file.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2019 SensiEDGE LTD
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

#include "AT25XE041B_Driver.h"

#ifdef SENSIBLE_2_0
    #include "sensible20_spi.h"
    #include "BlueNRG1_gpio.h"
    #include "BlueNRG1_sysCtrl.h"
    #include "BlueNRG1_spi.h"
#endif

#ifdef APOS
    #include "main.h"
    #include "SDK_EVAL_SPI.h"
    #include "BlueNRG1_gpio.h"
    #include "BlueNRG1_sysCtrl.h"
    #include "BlueNRG1_spi.h"
#endif

#ifdef USE_STM32F4XX_NUCLEO
    #include "stm32f4xx_periph_conf.h"
    #include "stm32f4xx_hal.h"
    #include "stm32f4xx_nucleo.h"
    #include "stm32f4xx_SPI.h"
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
    #include "stm32l4xx_periph_conf.h"
    #include "stm32l4xx_hal.h"
    #include "stm32l4xx_nucleo.h"
    #include "stm32l4xx_SPI.h"
#endif /* USE_STM32L4XX_NUCLEO */

#ifdef USE_STM32L0XX_NUCLEO
    #include "stm32l0xx_periph_conf.h"
    #include "stm32l0xx_hal.h"
    #include "stm32l0xx_nucleo.h"
    #include "stm32l0xx_SPI.h"
#endif /* USE_STM32L0XX_NUCLEO */

#ifdef USE_SENSI_ULE
    #include "stm32l4xx_hal.h"
    #include "stm32l4xx_nucleo.h"
    #include "stm32l4xx_SPI.h"
#endif /* USE_STM32L4XX_NUCLEO */

#ifdef  USE_SENSI_NBIOT
    #include "stm32l4xx_hal.h"
    #include "common.h"
    #include "cmsis_os.h"
#endif

/* Private defines begin -----------------------------------------------------*/
#ifdef SENSIBLE_2_0
    #define AT25XE041B_CS_PIN GPIO_Pin_1

    #define AT25XE041B_CS_ON()   GPIO_ResetBits(GPIO_Pin_1)
    #define AT25XE041B_CS_OFF()  GPIO_SetBits(GPIO_Pin_1)

    #define AT25XE041B_CS_INIT() do {                            \
        GPIO_InitType init = {                                   \
            .GPIO_Pin = AT25XE041B_CS_PIN,                       \
            .GPIO_Mode = GPIO_Output,                            \
            .GPIO_Pull = ENABLE,                                 \
            .GPIO_HighPwr = ENABLE                               \
        };                                                       \
        SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);   \
        GPIO_Init(&init);                                        \
        AT25XE041B_CS_OFF();                                     \
    } while(0);
#elif defined APOS
    #define AT25XE041B_CS_PIN SPI_FLASH_CS_PIN

    #define AT25XE041B_CS_ON()   GPIO_ResetBits(SPI_FLASH_CS_PIN)
    #define AT25XE041B_CS_OFF()  GPIO_SetBits(SPI_FLASH_CS_PIN)

    #define AT25XE041B_CS_INIT() do {                            \
        GPIO_InitType init = {                                   \
            .GPIO_Pin = AT25XE041B_CS_PIN,                       \
            .GPIO_Mode = GPIO_Output,                            \
            .GPIO_Pull = ENABLE,                                 \
            .GPIO_HighPwr = ENABLE                               \
        };                                                       \
        SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);   \
        GPIO_Init(&init);                                        \
        AT25XE041B_CS_OFF();                                     \
    } while(0);
#else
    #define AT25XE041B_CS_PIN  (GPIO_PIN_9)
    #define AT25XE041B_CS_PORT (GPIOA)

    #define AT25XE041B_CS_INIT() do {             \
        GPIO_InitTypeDef init = {                 \
            .Pin = AT25XE041B_CS_PIN,             \
            .Mode = GPIO_MODE_OUTPUT_PP,          \
            .Pull = GPIO_NOPULL,                  \
            .Speed = GPIO_SPEED_FREQ_HIGH,        \
            .Alternate = 0                        \
        };                                        \
                                                  \
        HAL_GPIO_Init(AT25XE041B_CS_PORT, &init); \
    } while(0);                            

    #define AT25XE041B_CS_ON()   HAL_GPIO_WritePin(AT25XE041B_CS_PORT, AT25XE041B_CS_PIN, GPIO_PIN_RESET)
    #define AT25XE041B_CS_OFF()  HAL_GPIO_WritePin(AT25XE041B_CS_PORT, AT25XE041B_CS_PIN, GPIO_PIN_SET)
#endif

#if defined (SENSIBLE_2_0) || defined (USE_SENSI_ULE) || defined (USE_SENSI_NBIOT)
    #define AT25XE041B_ENTER_CRITICAL_SECTION()
    #define AT25XE041B_EXIT_CRITICAL_SECTION()
#elif defined APOS    
    #define AT25XE041B_ENTER_CRITICAL_SECTION()
    #define AT25XE041B_EXIT_CRITICAL_SECTION()
#else
    /*------------------------------IMPORTANT-------------------------------------*/
    //BLE module in SensiBLE_1.0 is connected to the same SPI, and it calls interrupt on pin A0.
    //When interrup is called during working with flash, errors might occur.
    //These macroses must be called to protect weak sections of code. 
    #define AT25XE041B_ENTER_CRITICAL_SECTION()  HAL_NVIC_DisableIRQ(SPI1_CMN_DEFAULT_EXTI_IRQn)
    #define AT25XE041B_EXIT_CRITICAL_SECTION()   HAL_NVIC_EnableIRQ(SPI1_CMN_DEFAULT_EXTI_IRQn)
    /*----------------------------------------------------------------------------*/
#endif

#define AT25XE041B_MANUFACTURER_ID   (0x1F)
#define AT25XE041B_DEVICE_ID_PART1   (0x44)
#define AT25XE041B_DEVICE_ID_PART2   (0x02)
#define AT25XE041B_EXNTENDED_DEV_INF (0x00)

#define AT25XE041B_BEGINADDR         (0x000000)
#define AT25XE041B_FINISHADDR        (AT25XE041B_FLASHSIZE - 1)
#define AT25XE041B_WAIT_TIMEOUT      (10000)

/* Private defines end -------------------------------------------------------*/

/* Private function prototypes begin -----------------------------------------*/
static AT25XE041B_StatusTypeDef AT25XE041B_ReadDeviceId(uint8_t* devId, uint8_t len);
static inline AT25XE041B_StatusTypeDef AT25XE041B_ReadStatusReg(uint8_t* statusReg, uint8_t len);
static AT25XE041B_StatusTypeDef AT25XE041B_ErasePage(uint32_t pageAddress);
static AT25XE041B_StatusTypeDef AT25XE041B_ReadPage(uint32_t pageAddress, uint8_t* data);
static AT25XE041B_StatusTypeDef AT25XE041B_Read(uint32_t address, uint8_t* data, uint32_t size);
static AT25XE041B_StatusTypeDef AT25XE041B_WritePage(uint32_t pageAddress, uint8_t* data);
static AT25XE041B_StatusTypeDef AT25XE041B_WritePartPage(uint32_t pageAddress, uint8_t* data, 
                                         uint16_t* len, uint16_t maxSize,
                                         uint8_t offset);

static AT25XE041B_StatusTypeDef AT25XE041B_WriteBytesWithoutErase(
        uint32_t address, uint8_t* data, uint16_t size);

static AT25XE041B_StatusTypeDef AT25XE041B_WaitForReady(uint32_t timeout);

static AT25XE041B_StatusTypeDef AT25XE041B_Write(uint32_t pageAddress, uint8_t* data, 
                                 uint16_t* len, uint16_t maxSize);

static AT25XE041B_StatusTypeDef AT25XE041B_WriteEnable(void);
static AT25XE041B_StatusTypeDef AT25XE041B_WriteDisable(void);

static uint32_t AT25XE041B_GetTick(void);
static void AT25XE041B_Wait(uint32_t time);
static AT25XE041B_StatusTypeDef AT25XE041B_ExitLowPowerMode(void);
static AT25XE041B_StatusTypeDef AT25XE041B_SPI_Init(void);
static AT25XE041B_StatusTypeDef AT25XE041B_SPI_FlushRxFifo(void);
static AT25XE041B_StatusTypeDef AT25XE041B_SPI_Receive(uint8_t *pData, uint16_t size, uint32_t timeout);
static AT25XE041B_StatusTypeDef AT25XE041B_SPI_Transmit(uint8_t *pData, uint16_t size, uint32_t timeout);
/* Private function prototypes end  ------------------------------------------*/

/* Private variables begin ---------------------------------------------------*/
enum AT25XE041B_Commands
{
    readArray = 0x0B,
    pageErase = 0x81,
    chipErase = 0x60,
    bytePageProgram = 0x02,
    writeEnable = 0x06,
    writeDisable = 0x04,
    protectSector = 0x36,
    unprotectSector = 0x39,
    readSectorProtectionRegister = 0x3C,
    readStatusRegister = 0x05,
    writeStatusRegisterByte1 = 0x01,
    writeStatusRegisterByte2 = 0x31,
    reset = 0xF0,
    readDeviceId = 0x9F,
    enterDeepPowerDown = 0xB9,
    exitDeepPowerDown = 0xAB,
    enterUltraDeepPowerDown = 0x79
};

struct AT25XE041B_StatusRegister
{
    //first byte
    uint16_t RDY_BSY  : 1;
    uint16_t WEL      : 1;
    uint16_t SWP      : 2;
    uint16_t WPP      : 1;
    uint16_t EPE      : 1;
    uint16_t SPM      : 1;
    uint16_t SPRL     : 1;
    //second byte
    uint16_t RDY_BS   : 1;
    uint16_t RES_13   : 3;
    uint16_t RSTE     : 1;
    uint16_t RES_57   : 3;
};

struct AT25XE041B_DeviceId
{
    uint8_t manufacturerId;
    uint8_t deviceIdPart1;
    uint8_t deviceIdPart2;
    uint8_t extendedDevInf;
};

typedef enum
{
    AT25XE041B_NORMAL_MODE                 = 0x00,
    AT25XE041B_DEEP_POWER_DOWN_MODE        = 0x01,
    AT25XE041B_ULTRA_DEEP_POWER_DOWN_MODE  = 0x02,
} AT25XE041B_POWER_MODE;

static volatile AT25XE041B_POWER_MODE currentMode = AT25XE041B_NORMAL_MODE;

/* Private variables end -----------------------------------------------------*/

/* Public functions realization begin ----------------------------------------*/

/**
* @brief   Initialize SPI peripheral and CS pin for driver.
*          Try to read spi flash device id.
* @param   none.
* @retval  AT25XE041B status
*/
AT25XE041B_StatusTypeDef AT25XE041B_Init(void)
{
    AT25XE041B_SPI_Init();
    struct AT25XE041B_DeviceId devId  = {0};
    AT25XE041B_StatusTypeDef status;
    
    AT25XE041B_CS_INIT();
    //need to protect weak section of code if another device uses the same spi
    AT25XE041B_ENTER_CRITICAL_SECTION();
    if(currentMode != AT25XE041B_NORMAL_MODE){
        status = AT25XE041B_ExitLowPowerMode();
        if(status != AT25XE041B_OK)
        {
            AT25XE041B_EXIT_CRITICAL_SECTION();
            return AT25XE041B_ERROR;
        }
    }
    status = AT25XE041B_ReadDeviceId((uint8_t*)&devId, sizeof(devId));
    if(status == AT25XE041B_OK){
        if((devId.manufacturerId == AT25XE041B_MANUFACTURER_ID) && 
           (devId.deviceIdPart1 == AT25XE041B_DEVICE_ID_PART1) &&
           (devId.deviceIdPart2 == AT25XE041B_DEVICE_ID_PART2) &&
           (devId.extendedDevInf == AT25XE041B_EXNTENDED_DEV_INF))
        {
            //need to protect weak section of code if another device uses the same spi
            AT25XE041B_EXIT_CRITICAL_SECTION();
            return AT25XE041B_OK;
        }
    }
    //need to protect weak section of code if another device uses the same spi
    AT25XE041B_EXIT_CRITICAL_SECTION();
    return AT25XE041B_ERROR;
}

/**
* @brief   Public function. Read a number of bytes from spi flash.
*          When the last byte (07FFFFh) of the memory array of spi flash 
*          has been read, the spi flash will continue reading back 
*          at the beginning of the array (000000h)
* @param   address: Specify the begin address of data that is needed to be read.
* @param   data:    Specify the pointer to an array, where to save data.
* @param   size: Specify the number of data to read.
* @retval  AT25XE041B status
*/
AT25XE041B_StatusTypeDef AT25XE041B_ReadByteArray(uint32_t address, uint8_t* data, uint32_t size)
{
    if((data == NULL) || (size == 0) || (address > AT25XE041B_FINISHADDR)){
        return AT25XE041B_ERROR;
    }
    AT25XE041B_StatusTypeDef status;
    //need to protect weak section of code if another device uses the same spi
    AT25XE041B_ENTER_CRITICAL_SECTION();
    if(currentMode != AT25XE041B_NORMAL_MODE){
        status = AT25XE041B_ExitLowPowerMode();
        if(status != AT25XE041B_OK)
        {
            //need to protect weak section of code if another device uses the same spi
            AT25XE041B_EXIT_CRITICAL_SECTION();
            return AT25XE041B_ERROR;
        }
    }
    status = AT25XE041B_Read(address, data, size);
    //need to protect weak section of code if another device uses the same spi
    AT25XE041B_EXIT_CRITICAL_SECTION();
    return status;
}

/**
* @brief   Public function. Write a number of bytes to spi flash.
* @param   address: Specify the begin address where to save data.
* @param   data:    Specify the pointer to an array, where are located data to be saved.
* @param   len:     Specify the pointer to a variable where to save the length 
*                   of saved data.
* @param   maxSize: Specify the maximum number of data to save.
* @retval  AT25XE041B status
*/
AT25XE041B_StatusTypeDef AT25XE041B_WriteByteArray(uint32_t address, uint8_t* data, 
                                          uint16_t* len, uint16_t maxSize)
{
    if((data == NULL) || (len == NULL) || (maxSize == 0)){
        return AT25XE041B_ERROR;
    }
    *len = 0;
    AT25XE041B_StatusTypeDef status;
    //need to protect weak section of code if another device uses the same spi
    AT25XE041B_ENTER_CRITICAL_SECTION();
    if(currentMode != AT25XE041B_NORMAL_MODE){
        status = AT25XE041B_ExitLowPowerMode();
        if(status != AT25XE041B_OK)
        {
            //need to protect weak section of code if another device uses the same spi
            AT25XE041B_EXIT_CRITICAL_SECTION();
            return AT25XE041B_ERROR;
        }
    }
    if((address % AT25XE041B_PAGESIZE) != 0){
        uint8_t offset = address % AT25XE041B_PAGESIZE;
        uint8_t size = AT25XE041B_PAGESIZE - offset;
        if(maxSize < size){
            size = maxSize;
        }
        status = AT25XE041B_WritePartPage((address - offset), data, len, size, offset);
        maxSize -= size;
        if(maxSize == 0 || status != AT25XE041B_OK){
            AT25XE041B_WriteDisable();
            //need to protect weak section of code if another device uses the same spi
            AT25XE041B_EXIT_CRITICAL_SECTION();
            return status;
        } else {
            address += size;
            data += size;
        }
    }
    //if received page begin address
    if(maxSize == AT25XE041B_PAGESIZE){
        status = AT25XE041B_Write(address, data, len, maxSize);
        AT25XE041B_WriteDisable();
        //need to protect weak section of code if another device uses the same spi
        AT25XE041B_EXIT_CRITICAL_SECTION();
        return status;
    } else if (maxSize > AT25XE041B_PAGESIZE){
        uint8_t numberOfPages = maxSize / AT25XE041B_PAGESIZE;
        uint8_t remainder = maxSize % AT25XE041B_PAGESIZE;
        
        for(uint8_t i = 0; i < numberOfPages; i++)
        {
            address += i * AT25XE041B_PAGESIZE;
            data += i * AT25XE041B_PAGESIZE;
            status = AT25XE041B_Write(address, data, len, AT25XE041B_PAGESIZE);
            if(status != AT25XE041B_OK){
                AT25XE041B_WriteDisable();
                //need to protect weak section of code if another device uses the same spi
                AT25XE041B_EXIT_CRITICAL_SECTION();
                return status;
            }
        }
        address += AT25XE041B_PAGESIZE;
        data += AT25XE041B_PAGESIZE;
        status = AT25XE041B_WritePartPage(address, data, len, remainder, 0);
        AT25XE041B_WriteDisable();
        //need to protect weak section of code if another device uses the same spi
        AT25XE041B_EXIT_CRITICAL_SECTION();
        return status;
    }else{
        //maxSize < AT25XE041B_PAGESIZE
        status = AT25XE041B_WritePartPage(address, data, len, maxSize, 0);
        AT25XE041B_WriteDisable();
        //need to protect weak section of code if another device uses the same spi
        AT25XE041B_EXIT_CRITICAL_SECTION();
        return status;
    }
}

/**
* @brief   Public function. Write a number of bytes to spi flash.
           All pages where to save data must be cleared before calling this function.
* @param   address: Specify the begin address where to save data.
* @param   data:    Specify the pointer to an array, where are located data to be saved.
* @param   len:     Specify the pointer to a variable where to save the length 
*                   of saved data.
* @param   maxSize: Specify the maximum number of data to save.
* @retval  AT25XE041B status
*/
AT25XE041B_StatusTypeDef AT25XE041B_WriteByteArrayWithoutErase(uint32_t address, uint8_t* data, uint16_t* len, uint16_t maxSize)
{
    AT25XE041B_StatusTypeDef status;
    if((data == NULL) || (len == NULL) || (maxSize == 0)){
        return AT25XE041B_ERROR;
    }
    
    *len = 0;
    
    //need to protect weak section of code if another device uses the same spi
    AT25XE041B_ENTER_CRITICAL_SECTION();
    if(currentMode != AT25XE041B_NORMAL_MODE){
        status = AT25XE041B_ExitLowPowerMode();
        if(status != AT25XE041B_OK)
        {
            goto fail;
        }
    }

    while(maxSize > 0){
        uint16_t bytesToWrite = 0;
        
        if((address % AT25XE041B_PAGESIZE) != 0){
            uint16_t offset = address % AT25XE041B_PAGESIZE;
            uint16_t leftPageSize = AT25XE041B_PAGESIZE - offset;
            if(maxSize > leftPageSize){
                bytesToWrite = maxSize - leftPageSize;
            } else {
                bytesToWrite = maxSize;
            }
        } else {
            if(maxSize > AT25XE041B_PAGESIZE){
                bytesToWrite = AT25XE041B_PAGESIZE;
            } else {
                bytesToWrite = maxSize;
            }
        }
        
        status = AT25XE041B_WriteEnable();
        if(status != AT25XE041B_OK){
            goto fail;
        }
        
        status = AT25XE041B_WriteBytesWithoutErase(address, data, bytesToWrite);
        if(status != AT25XE041B_OK){
            AT25XE041B_WriteDisable();
            goto fail;
        }
        maxSize -= bytesToWrite;
        *len += bytesToWrite;
        if(maxSize == 0){
            goto success;
        } else {
            address += bytesToWrite;
            data += bytesToWrite;
        }
    }

    success:
    AT25XE041B_WriteDisable();
    //need to protect weak section of code if another device uses the same spi
    AT25XE041B_EXIT_CRITICAL_SECTION();
    return AT25XE041B_OK;
    
    fail: 
    //need to protect weak section of code if another device uses the same spi
    AT25XE041B_EXIT_CRITICAL_SECTION();
    return status;
}

/**
* @brief   Erase the entire spi flash.
* @param   none.
* @retval  AT25XE041B status
*/
AT25XE041B_StatusTypeDef AT25XE041B_EraseAll(void)
{
    AT25XE041B_StatusTypeDef status;
    uint8_t command = chipErase;
    //need to protect weak section of code if another device uses the same spi
    AT25XE041B_ENTER_CRITICAL_SECTION();
    if(currentMode != AT25XE041B_NORMAL_MODE){
        status = AT25XE041B_ExitLowPowerMode();
        if(status != AT25XE041B_OK)
        {
            //need to protect weak section of code if another device uses the same spi
            AT25XE041B_EXIT_CRITICAL_SECTION();
            return AT25XE041B_ERROR;
        }
    }
    status = AT25XE041B_WriteEnable();
    if(status != AT25XE041B_OK){
        AT25XE041B_WriteDisable();
        //need to protect weak section of code if another device uses the same spi
        AT25XE041B_EXIT_CRITICAL_SECTION();
        return status;
    }
    
    AT25XE041B_CS_ON();
    //send erase command
    status = AT25XE041B_SPI_Transmit(&command, sizeof(command), AT25XE041B_WAIT_TIMEOUT);
    AT25XE041B_CS_OFF();
    if(status != AT25XE041B_OK){
        AT25XE041B_WriteDisable();
        //need to protect weak section of code if another device uses the same spi
        AT25XE041B_EXIT_CRITICAL_SECTION();
        return status;
    }
    status = AT25XE041B_WaitForReady(AT25XE041B_WAIT_TIMEOUT);
    AT25XE041B_WriteDisable();
    //need to protect weak section of code if another device uses the same spi
    AT25XE041B_EXIT_CRITICAL_SECTION();
    return status;
}

/**
* @brief   Erase a number of pages in spi flash.
* @param   pageAddress:   Specify the page address.
* @param   numberOfPages: Specify the number of pages to erase.
* @retval  AT25XE041B status
*/
AT25XE041B_StatusTypeDef AT25XE041B_ErasePages(uint32_t pageAddress, uint32_t numberOfPages)
{
    if(((pageAddress % AT25XE041B_PAGESIZE) != 0) || (numberOfPages == 0)
       || (numberOfPages > (AT25XE041B_FLASHSIZE - pageAddress) / AT25XE041B_PAGESIZE))
    {
        return AT25XE041B_ERROR;
    }
    AT25XE041B_StatusTypeDef status;
    //need to protect weak section of code if another device uses the same spi
    AT25XE041B_ENTER_CRITICAL_SECTION();
    if(currentMode != AT25XE041B_NORMAL_MODE){
        status = AT25XE041B_ExitLowPowerMode();
        if(status != AT25XE041B_OK)
        {
            //need to protect weak section of code if another device uses the same spi
            AT25XE041B_EXIT_CRITICAL_SECTION();
            return AT25XE041B_ERROR;
        }
    }
    for(uint32_t i = 0; i < numberOfPages; i++){
        status = AT25XE041B_WriteEnable();
        if(status != AT25XE041B_OK){
            AT25XE041B_WriteDisable();
            //need to protect weak section of code if another device uses the same spi
            AT25XE041B_EXIT_CRITICAL_SECTION();
            return status;
        }
        status = AT25XE041B_ErasePage(pageAddress);
        pageAddress += AT25XE041B_PAGESIZE;
        if(status != AT25XE041B_OK){
            AT25XE041B_WriteDisable();
            //need to protect weak section of code if another device uses the same spi
            AT25XE041B_EXIT_CRITICAL_SECTION();
            return status;
        }
    }
    AT25XE041B_WriteDisable();
    //need to protect weak section of code if another device uses the same spi
    AT25XE041B_EXIT_CRITICAL_SECTION();
    return status;
}

/**
* @brief   Go to deep power down mode.
* @param   none.
* @retval  AT25XE041B status
*/
AT25XE041B_StatusTypeDef AT25XE041B_EnterDeepPowerDown(void)
{
    AT25XE041B_StatusTypeDef status = AT25XE041B_ERROR;
    if (currentMode == AT25XE041B_NORMAL_MODE)
    {
        uint8_t command = enterDeepPowerDown;
        AT25XE041B_CS_ON();
        status = AT25XE041B_SPI_Transmit(&command, 1, AT25XE041B_WAIT_TIMEOUT);
        AT25XE041B_CS_OFF();
        currentMode = AT25XE041B_DEEP_POWER_DOWN_MODE;
    }
    return status;
}

/**
* @brief   Return from deep power down mode.
* @param   none.
* @retval  AT25XE041B status
*/
AT25XE041B_StatusTypeDef AT25XE041B_ExitDeepPowerDown(void)
{
    AT25XE041B_StatusTypeDef status = AT25XE041B_ERROR;
    if (currentMode == AT25XE041B_DEEP_POWER_DOWN_MODE)
    {
        uint8_t command = exitDeepPowerDown;
        AT25XE041B_CS_ON();
        status = AT25XE041B_SPI_Transmit(&command, 1, AT25XE041B_WAIT_TIMEOUT);
        AT25XE041B_CS_OFF();
        AT25XE041B_Wait(1);//max datasheet time is 8 microSec
        currentMode = AT25XE041B_NORMAL_MODE;
    }
    return status;
}

/**
* @brief   Go to ultra deep power down mode.
* @param   none.
* @retval  AT25XE041B status
*/
AT25XE041B_StatusTypeDef AT25XE041B_EnterUltraDeepPowerDown(void)
{
    AT25XE041B_StatusTypeDef status = AT25XE041B_ERROR;
    if (currentMode == AT25XE041B_NORMAL_MODE)
    {
        uint8_t command = enterUltraDeepPowerDown;
        AT25XE041B_CS_ON();
        status = AT25XE041B_SPI_Transmit(&command, 1, AT25XE041B_WAIT_TIMEOUT);
        AT25XE041B_CS_OFF();
        currentMode = AT25XE041B_ULTRA_DEEP_POWER_DOWN_MODE;
    }
    return status;
}

/**
* @brief   Return from ultra deep power down mode.
* @param   none.
* @retval  AT25XE041B status
*/
AT25XE041B_StatusTypeDef AT25XE041B_ExitUltraDeepPowerDown(void)
{
    if (currentMode == AT25XE041B_ULTRA_DEEP_POWER_DOWN_MODE)
    {
        AT25XE041B_CS_ON();
        AT25XE041B_Wait(1);//mim datasheet time is 70 microSec
        AT25XE041B_CS_OFF();
        currentMode = AT25XE041B_NORMAL_MODE;
        return AT25XE041B_OK;
    }
    return AT25XE041B_ERROR;
}
/* Public functions realization end ------------------------------------------*/

/* Private functions realization begin ---------------------------------------*/

/**
* @brief   Private function. Read device id from spi flash.
* @param   devId:   Specify the pointer to variable where to save device id.
* @param   len:     Specify the length of device id.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_ReadDeviceId(uint8_t* devId, uint8_t len)
{
    AT25XE041B_StatusTypeDef status;
    uint8_t command = readDeviceId;
    AT25XE041B_CS_ON();
    status = AT25XE041B_SPI_Transmit(&command, 1, AT25XE041B_WAIT_TIMEOUT);
    if(status != AT25XE041B_OK){
        AT25XE041B_CS_OFF();
        return status;
    }
    status = AT25XE041B_SPI_FlushRxFifo();
    if(status != AT25XE041B_OK){
        AT25XE041B_CS_OFF();
        return status;
    }
    status = AT25XE041B_SPI_Receive(devId, len, AT25XE041B_WAIT_TIMEOUT);
    AT25XE041B_CS_OFF();
    return status;
}

/**
* @brief   Private function. Read status register from spi flash.
* @param   statusReg: Specify the pointer to variable where to save data.
* @param   len:       Specify the length of status register.
* @retval  AT25XE041B status
*/
static inline AT25XE041B_StatusTypeDef AT25XE041B_ReadStatusReg(uint8_t* statusReg, uint8_t len)
{
    AT25XE041B_StatusTypeDef status;
    uint8_t command = readStatusRegister;
    AT25XE041B_CS_ON();
    status = AT25XE041B_SPI_Transmit(&command, 1, AT25XE041B_WAIT_TIMEOUT);
    if(status != AT25XE041B_OK){
        AT25XE041B_CS_OFF();
        return status;
    }
    status = AT25XE041B_SPI_FlushRxFifo();
    if(status != AT25XE041B_OK){
        AT25XE041B_CS_OFF();
        return status;
    }
    status = AT25XE041B_SPI_Receive(statusReg, len, AT25XE041B_WAIT_TIMEOUT);
    AT25XE041B_CS_OFF();
    return status;
}

/**
* @brief   Private function. Wait when spi flash will be 
*          ready for next operation.
* @param   timeout: Timeout duration.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_WaitForReady(uint32_t timeout)
{
    if(timeout == 0 || timeout == AT25XE041B_MAX_DELAY){
        return AT25XE041B_TIMEOUT;
    }
    
    AT25XE041B_StatusTypeDef status;
    uint32_t tickstart = AT25XE041B_GetTick();
    while((AT25XE041B_GetTick() - tickstart) <  timeout){
        struct AT25XE041B_StatusRegister reg = {0};
        status = AT25XE041B_ReadStatusReg((uint8_t*)&reg, sizeof(reg));
        
        if(status != AT25XE041B_OK){
            return status;
        }
        if(reg.RDY_BSY == 0){
            return AT25XE041B_OK;
        }
    }
    return AT25XE041B_TIMEOUT;
}

/**
* @brief   Erase one page in spi flash.
* @param   pageAddress: Specify the page address.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_ErasePage(uint32_t pageAddress)
{
    if((pageAddress % AT25XE041B_PAGESIZE) != 0){
        return AT25XE041B_ERROR;
    }
    AT25XE041B_StatusTypeDef status;
    uint8_t command[4] = {
        pageErase,
        (pageAddress & 0x070000) >> 16,
        (pageAddress & 0xFF00) >> 8,
        0
    };
    
    AT25XE041B_CS_ON();
    //send erase command
    status = AT25XE041B_SPI_Transmit(command, sizeof(command), AT25XE041B_WAIT_TIMEOUT);
    AT25XE041B_CS_OFF();
    
    if(status != AT25XE041B_OK){
        return status;
    }
    return AT25XE041B_WaitForReady(AT25XE041B_WAIT_TIMEOUT);
}

/**
* @brief   Public function. Read one page (AT25XE041B_PAGESIZE of bytes)
*          from spi flash.
* @param   pageAddress: Specify the page begin address.
* @param   data:        Specify the pointer to an array, where to save data.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_ReadPage(uint32_t pageAddress, uint8_t* data)
{
    if((data == NULL) || 
       (pageAddress > (AT25XE041B_FLASHSIZE - AT25XE041B_PAGESIZE)) ||
        ((pageAddress % AT25XE041B_PAGESIZE) != 0))
    {
        return AT25XE041B_ERROR;
    }
    return AT25XE041B_Read(pageAddress, data, AT25XE041B_PAGESIZE);
}

/**
* @brief   Private function. Write one page (AT25XE041B_PAGESIZE of bytes) 
*          to spi flash.
* @param   pageAddress: Specify the page begin address.
* @param   data:        Specify the pointer to an array, 
*                       where are located data to be saved.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_WritePage(uint32_t pageAddress, uint8_t* data)
{
    if(((pageAddress % AT25XE041B_PAGESIZE) != 0) || 
        (data == NULL) || (pageAddress >= AT25XE041B_FLASHSIZE))
    {
        return AT25XE041B_ERROR;
    }
    
    AT25XE041B_StatusTypeDef status;
    uint8_t command[4] = {
        bytePageProgram,
        (pageAddress & 0x070000) >> 16,
        (pageAddress & 0xFF00) >> 8,
        0
    };
    AT25XE041B_CS_ON();
    //send program command
    status = AT25XE041B_SPI_Transmit(command, sizeof(command), AT25XE041B_WAIT_TIMEOUT);
    if(status != AT25XE041B_OK){
        AT25XE041B_CS_OFF();
        return status;
    }
    //send data
    status = AT25XE041B_SPI_Transmit(data, AT25XE041B_PAGESIZE, AT25XE041B_WAIT_TIMEOUT);
    AT25XE041B_CS_OFF();
    if(status != AT25XE041B_OK){
        return status;
    }
    return AT25XE041B_WaitForReady(AT25XE041B_WAIT_TIMEOUT);
}

/**
* @brief   Private function. Write the number of bytes(max size == AT25XE041B_PAGESIZE) 
*          to spi flash.
* @param   address: Specify the begin address where to write data.
* @param   data:    Specify the pointer to an array, where are located data to be saved.
* @param   size:    Specife the number of bytes to be saved.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_WriteBytesWithoutErase(
        uint32_t address, uint8_t* data, uint16_t size)
{
    if((data == NULL) || ((address + size) > AT25XE041B_FLASHSIZE) || (size == 0))
    {
        return AT25XE041B_ERROR;
    }
    
    AT25XE041B_StatusTypeDef status;
    uint8_t command[4] = {
        bytePageProgram,
        (address & 0x070000) >> 16,
        (address & 0xFF00) >> 8,
        (address & 0xFF)
    };
    AT25XE041B_CS_ON();
    //send program command
    status = AT25XE041B_SPI_Transmit(command, sizeof(command), AT25XE041B_WAIT_TIMEOUT);
    if(status != AT25XE041B_OK){
        AT25XE041B_CS_OFF();
        return status;
    }
    //send data
    status = AT25XE041B_SPI_Transmit(data, size, AT25XE041B_WAIT_TIMEOUT);
    AT25XE041B_CS_OFF();
    if(status != AT25XE041B_OK){
        return status;
    }
    return AT25XE041B_WaitForReady(AT25XE041B_WAIT_TIMEOUT);
}

/**
* @brief   Private function. It is used to write data array to spi flash 
*          when it's length is less then AT25XE041B_PAGESIZE
* @param   pageAddress: Specify the page begin address.
* @param   data:        Specify the pointer to an array, where are located data 
*                       to be written to spi flash.
* @param   len:         Specify the pointer to a variable where to save  
*                       the length of saved data.
* @param   size:        Specify the number of data to save.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_WritePartPage(uint32_t pageAddress, uint8_t* data, 
                                         uint16_t* len, uint16_t size,
                                         uint8_t offset)
{
    uint8_t tmpArr[AT25XE041B_PAGESIZE] = {0};
    AT25XE041B_StatusTypeDef status;
    uint16_t tmpLen = 0;
    
    //read one page from spi flash
    status = AT25XE041B_ReadPage(pageAddress, tmpArr);
    if(status != AT25XE041B_OK){
        return status;
    }
    //copy data that is needed to be written to tmpArr
    //(modify read buffer, saving bytes that we don't want to modify)
    memcpy((tmpArr + offset), data, size);
    
    //write modified buffer to spi flash
    status = AT25XE041B_Write(pageAddress, tmpArr, &tmpLen, sizeof(tmpArr));
    if(status == AT25XE041B_OK){
        *len += size;
    }
    return status;
}

/**
* @brief   Private function. Erase one page and then write new data there.
* @param   address: Specify the begin address where to save data.
* @param   data:    Specify the pointer to an array, where are located data to be saved.
* @param   len:     Specify the pointer to a variable where to save the length 
*                   of saved data.
* @param   maxSize: Specify the maximum number of data to save.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_Write(uint32_t address, uint8_t* data, 
                                 uint16_t* len, uint16_t maxSize)
{
    if((maxSize == 0) || (data == NULL) || (len == NULL)){
        return AT25XE041B_ERROR;
    }
    if(((address % AT25XE041B_PAGESIZE) != 0) || (address >= AT25XE041B_FINISHADDR)){
       return AT25XE041B_ERROR;
    }
    AT25XE041B_StatusTypeDef status;
    status = AT25XE041B_WriteEnable();
    if(status != AT25XE041B_OK){
        return status;
    }
    status = AT25XE041B_ErasePage(address);
    if(status != AT25XE041B_OK){
        return status;
    }
    status = AT25XE041B_WriteEnable();
    if(status != AT25XE041B_OK){
        return status;
    }
    status = AT25XE041B_WritePage(address, data);
    if(status != AT25XE041B_OK){
        return status;
    }
    *len += maxSize;
    return status;
}

/**
* @brief   Private function. Read a number of bytes from spi flash.
*          When the last byte (07FFFFh) of the memory array of spi flash 
*          has been read, the spi flash will continue reading back 
*          at the beginning of the array (000000h)
* @param   address: Specify the begin address of data that is needed to be read.
* @param   data:    Specify the pointer to an array, where to save data.
* @param   size:    Specify the number of data to read.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_Read(uint32_t address, uint8_t* data, uint32_t size)
{
    if((data == NULL) || (size == 0) || (address > AT25XE041B_FINISHADDR)){
        return AT25XE041B_ERROR;
    }
    
    AT25XE041B_StatusTypeDef status;
    uint8_t command[5] = {
        readArray,
        (address & 0x00070000) >> 16,
        (address & 0x0000FF00) >> 8,
        address & 0x000000FF,
        0
    };
    AT25XE041B_CS_ON();
    status = AT25XE041B_SPI_Transmit(command, sizeof(command), AT25XE041B_WAIT_TIMEOUT);
    if(status != AT25XE041B_OK){
        AT25XE041B_CS_OFF();
        return status;
    }
    status = AT25XE041B_SPI_FlushRxFifo();
    if(status != AT25XE041B_OK){
        AT25XE041B_CS_OFF();
        return status;
    }
    status = AT25XE041B_SPI_Receive(data, size, AT25XE041B_WAIT_TIMEOUT);
    AT25XE041B_CS_OFF();
    return status;
}

/**
* @brief   Private function. Set the Write Enable Latch (WEL) bit in the 
*          Status Register to a logical "1" state.
* @param   none.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_WriteEnable(void)
{
    AT25XE041B_StatusTypeDef status;
    uint8_t cnt = 0;
    do{
        struct AT25XE041B_StatusRegister reg = {0};
        status = AT25XE041B_ReadStatusReg((uint8_t*)&reg, sizeof(reg));
        if(status != AT25XE041B_OK){
            return status;
        }
        //WEL == 0 - device is write disabled
        //WEL == 1 - device is write enabled
        if(reg.WEL != 1){
            uint8_t command = writeEnable;
            AT25XE041B_CS_ON();
            //send command
            status = AT25XE041B_SPI_Transmit(&command, 1, AT25XE041B_WAIT_TIMEOUT);
            AT25XE041B_CS_OFF();
            if(status != AT25XE041B_OK){
                return status;
            }
            continue;
        }
        //SWP == 0 - All sectors are software unprotected
        //SWP == 1 - Some sectors are software protected.
        //SWP == 3 - All sectors are software protected, default
        if(reg.SWP != 0){
            uint8_t command[2] = {
                writeStatusRegisterByte1, 
                // 0x00 - Global Unprotect
                // 0xFF - Global Protect
                0};
            AT25XE041B_CS_ON();
            //send command
            status = AT25XE041B_SPI_Transmit(command, sizeof(command), AT25XE041B_WAIT_TIMEOUT);
            AT25XE041B_CS_OFF();
            if(status != AT25XE041B_OK){
                return status;
            }
            continue;
        }
        return AT25XE041B_OK;
    }while(++cnt < 5);
    
    return AT25XE041B_ERROR;
}

/**
* @brief   Private function. Set the Write Enable Latch (WEL) bit in the 
*          Status Register to a logical "0" state.
* @param   none.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_WriteDisable(void)
{
    AT25XE041B_StatusTypeDef status;
    uint8_t command = writeDisable;
    AT25XE041B_CS_ON();
    //send command
    status = AT25XE041B_SPI_Transmit(&command, 1, AT25XE041B_WAIT_TIMEOUT);
    AT25XE041B_CS_OFF();
    return status;
}

/**
* @brief   Private function. Exit from low power mode.
* @param   none.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_ExitLowPowerMode(void)
{
    AT25XE041B_StatusTypeDef status;
    switch(currentMode)
    {
    case AT25XE041B_DEEP_POWER_DOWN_MODE:
        status = AT25XE041B_ExitDeepPowerDown();
        break;
    case AT25XE041B_ULTRA_DEEP_POWER_DOWN_MODE:
        status = AT25XE041B_EnterUltraDeepPowerDown();
        break;
    default:
        status = AT25XE041B_OK;
    }
    return status;
}

/**
* @brief   Private function. Return a tick value.
*          It is used as a wrapper for platform-dependent functions.
* @param   none.
* @retval  AT25XE041B status
*/
static uint32_t AT25XE041B_GetTick(void)
{
    #ifdef SENSIBLE_2_0
        return lSystickCounter;
    #elif defined APOS
        return APOS_GetTick();
    #elif defined USE_SENSI_NBIOT
        return osKernelSysTick();
    #else
        return HAL_GetTick();
    #endif
}

/**
* @brief   Private function. Wait a time ms.
*          It is used as a wrapper for platform-dependent functions.
* @param   none.
* @retval  AT25XE041B status
*/
static void AT25XE041B_Wait(uint32_t time)
{
    #if defined SENSIBLE_2_0
        uint32_t nWaitPeriod = ~lSystickCounter;
        if(nWaitPeriod < time)
        {
            while( lSystickCounter != 0xFFFFFFFF);
            nWaitPeriod = time - nWaitPeriod;
        }
        else
            nWaitPeriod = time + ~nWaitPeriod;
        while(lSystickCounter != nWaitPeriod) ;
    #elif defined USE_SENSI_NBIOT
        osDelay(time);
    #elif defined APOS
        APOS_DelayMs(time);
    #else
        HAL_Delay(time);
    #endif
}

/**
* @brief   Private function. Init spi periferal.
*          It is used as a wrapper for platform-dependent functions.
* @param   none.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_SPI_Init(void)
{
    #ifdef SENSIBLE_2_0
        BlueNRG1_SPI_Init();
    #elif defined APOS
        //BlueNRG1_SPI_Init();
        SdkEvalSpiInit(SPI_FREQUENCY);
    #else
        SPI_Global_Init();
    #endif
    return AT25XE041B_OK;
}

/**
* @brief   Private function. Flush the RX fifo.
*          It is used as a wrapper for platform-dependent functions.
* @param   none.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_SPI_FlushRxFifo(void)
{
    AT25XE041B_StatusTypeDef status;
    
    #ifdef SENSIBLE_2_0
        BlueNRG1_SPI_FlushRxFifo();
        status = AT25XE041B_OK;
    #elif defined APOS
        SPI_ClearTXFIFO();
        SPI_ClearRXFIFO();
        status = AT25XE041B_OK;
    #else
        HAL_StatusTypeDef halStatus;
        halStatus = HAL_SPIEx_FlushRxFifo(&SpiHandle);
        switch(halStatus){
        case HAL_OK:
            status = AT25XE041B_OK;
            break;
        default:
            status = AT25XE041B_ERROR;
        }
    #endif
    
    return status;
}

/**
* @brief   Private function. Receive an amount of data in blocking mode.
*          It is used as a wrapper for platform-dependent functions.
* @param   pData     Specify the pointer to data buffer.
* @param   size      Specify the amount of data to be received.
* @param   timeout   Specify the timeout duration.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_SPI_Receive(uint8_t *pData, uint16_t size, uint32_t timeout)
{
    AT25XE041B_StatusTypeDef status;
    
    #ifdef SENSIBLE_2_0
        BlueNRG1_SPI_StatusTypeDef blueNrgStatus;
        blueNrgStatus = BlueNRG1_SPI_Receive(pData, size, timeout);
        switch(blueNrgStatus){
        case BlueNRG1_SPI_OK:
            status = AT25XE041B_OK;
            break;
        case BlueNRG1_SPI_TIMEOUT:
            status = AT25XE041B_TIMEOUT;
        default:
            status = AT25XE041B_ERROR;
        }
    #elif defined APOS
        uint32_t tickstart = APOS_GetTick();
        for(uint16_t i = 0; i < size; i++)
        {
            while(RESET == SPI_GetFlagStatus(SPI_FLAG_TFE));
            SPI_SendData(0);
            while(RESET == SPI_GetFlagStatus(SPI_FLAG_RNE));
            pData[i] = SPI_ReceiveData() & 0xFF;
            while (SET == SPI_GetFlagStatus(SPI_FLAG_BSY));
            
            if((APOS_GetTick() - tickstart) >= timeout){
                status = AT25XE041B_TIMEOUT;
            }
        }
        status = AT25XE041B_OK;
    #else
        HAL_StatusTypeDef halStatus;
        halStatus = HAL_SPI_Receive(&SpiHandle, pData, size, timeout);
        switch(halStatus){
        case HAL_OK:
            status = AT25XE041B_OK;
            break;
        case HAL_TIMEOUT:
            status = AT25XE041B_TIMEOUT;
            break;
        default:
            status = AT25XE041B_ERROR;
        }
    #endif
    
    return status;
}

/**
* @brief   Private function. Transmit an amount of data in blocking mode.
*          It is used as a wrapper for platform-dependent functions.
* @param   pData     Specify the pointer to data buffer.
* @param   size      Specify the amount of data to be sent.
* @param   timeout   Specify the timeout duration.
* @retval  AT25XE041B status
*/
static AT25XE041B_StatusTypeDef AT25XE041B_SPI_Transmit(uint8_t *pData, uint16_t size, uint32_t timeout)
{    
    AT25XE041B_StatusTypeDef status;
    
    #ifdef SENSIBLE_2_0
        BlueNRG1_SPI_StatusTypeDef blueNrgStatus;
        blueNrgStatus = BlueNRG1_SPI_Transmit(pData, size, timeout);
        switch(blueNrgStatus){
        case BlueNRG1_SPI_OK:
            status = AT25XE041B_OK;
            break;
        case BlueNRG1_SPI_TIMEOUT:
            status = AT25XE041B_TIMEOUT;
        default:
            status = AT25XE041B_ERROR;
        }
    #elif defined APOS
        uint32_t tickstart = APOS_GetTick();
        for(uint16_t i = 0; i < size; i++)
        {
            while(RESET == SPI_GetFlagStatus(SPI_FLAG_TFE));
            SPI_SendData(pData[i]);
            while(RESET == SPI_GetFlagStatus(SPI_FLAG_RNE));
            SPI_ReceiveData();
            while (SET == SPI_GetFlagStatus(SPI_FLAG_BSY));
            
            if((APOS_GetTick() - tickstart) >= timeout){
                status = AT25XE041B_TIMEOUT;
            }
        }
        status = AT25XE041B_OK;
    #else
        HAL_StatusTypeDef halStatus;
        halStatus = HAL_SPI_Transmit(&SpiHandle, pData, size, timeout);
        switch(halStatus){
        case HAL_OK:
            status = AT25XE041B_OK;
            break;
        case HAL_TIMEOUT:
            status = AT25XE041B_TIMEOUT;
            break;
        default:
            status = AT25XE041B_ERROR;
        }
    #endif
    
    return status;
}
/* Private functions realization end -----------------------------------------*/

/************************ (C) COPYRIGHT SensiEDGE LTD *************************/
