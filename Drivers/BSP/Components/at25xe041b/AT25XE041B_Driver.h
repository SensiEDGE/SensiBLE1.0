/**
 ******************************************************************************
 * @file    AT25XE041B_Driver.h
 * @date    02-July-2019
 * @brief   AT25XE041B driver header file.
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

#ifndef __AT25XE041B_Driver_H
#define __AT25XE041B_Driver_H

#include <stdint.h>
#include <string.h>

#define AT25XE041B_FLASHSIZE   (0x080000)
#define AT25XE041B_PAGESIZE    (256)
#define AT25XE041B_PAGENUMBER  ((SPIFLASH_FLASHSIZE) / (SPIFLASH_PAGESIZE))

#define AT25XE041B_MAX_DELAY   (60000)

typedef enum
{
    AT25XE041B_OK       = 0x00,
    AT25XE041B_ERROR    = 0x01,
    AT25XE041B_TIMEOUT  = 0x02,
} AT25XE041B_StatusTypeDef;

AT25XE041B_StatusTypeDef AT25XE041B_Init(void);

AT25XE041B_StatusTypeDef AT25XE041B_ReadByteArray(uint32_t address, uint8_t* data, uint32_t size);
AT25XE041B_StatusTypeDef AT25XE041B_WriteByteArray(uint32_t address, uint8_t* data, uint16_t* len, uint16_t maxSize);
AT25XE041B_StatusTypeDef AT25XE041B_WriteByteArrayWithoutErase(uint32_t address, uint8_t* data, uint16_t* len, uint16_t maxSize);
AT25XE041B_StatusTypeDef AT25XE041B_EraseAll(void);
AT25XE041B_StatusTypeDef AT25XE041B_ErasePages(uint32_t firstPageAddress, uint32_t numberOfPages);

AT25XE041B_StatusTypeDef AT25XE041B_EnterDeepPowerDown(void);
AT25XE041B_StatusTypeDef AT25XE041B_ExitDeepPowerDown(void);
AT25XE041B_StatusTypeDef AT25XE041B_EnterUltraDeepPowerDown(void);
AT25XE041B_StatusTypeDef AT25XE041B_ExitUltraDeepPowerDown(void);

#endif //__AT25XE041B_Driver_H

/************************ (C) COPYRIGHT SensiEDGE LTD *************************/
