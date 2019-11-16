/**
  ******************************************************************************
  * @file    TargetPlatform.c
  * @author  Central LAB
  * @version V3.0.0
  * @date    12-May-2017
  * @brief   Initialization of the Target Platform
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
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"
#include "vbat.h"

#ifdef STM32_SENSORTILE
  #ifdef ALLMEMS1_ENABLE_PRINTF
    #include "usbd_core.h"
    #include "usbd_cdc.h"
    #include "usbd_cdc_interface.h"
  #endif /* ALLMEMS1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

/* Imported variables ---------------------------------------------------------*/
#ifdef STM32_SENSORTILE
  #ifdef ALLMEMS1_ENABLE_PRINTF
     extern USBD_DescriptorsTypeDef VCP_Desc;
  #endif /* ALLMEMS1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */
/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

#ifdef STM32_SENSORTILE
  #ifdef ALLMEMS1_ENABLE_PRINTF
    USBD_HandleTypeDef  USBD_Device;
  #endif /* ALLMEMS1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

volatile float RMS_Ch[AUDIO_CHANNELS];
float DBNOISE_Value_Old_Ch[AUDIO_CHANNELS];
uint16_t PCM_Buffer[AUDIO_CHANNELS*PCM_AUDIO_IN_SAMPLES];

#ifdef USE_STM32F4XX_NUCLEO
uint16_t PDM_Buffer[AUDIO_CHANNELS*PCM_AUDIO_IN_SAMPLES*64/8];
#endif /* USE_STM32F4XX_NUCLEO */

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/
static void Init_MEM1_Sensors(void);
static void Init_MEMS_Mics(uint32_t AudioFreq, uint32_t AudioVolume);


#ifdef USE_STM32L4XX_NUCLEO
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
#endif /* USE_STM32L4XX_NUCLEO */

/**
  * @brief  Initialize all the Target platform's Features
  * @param  TargetType_t BoardType Nucleo/BlueCoin/SensorTile
  * @retval None
  */
void InitTargetPlatform(TargetType_t BoardType)
{
  TargetBoardFeatures.BoardType = BoardType;
#ifdef STM32_NUCLEO
#ifdef ALLMEMS1_ENABLE_PRINTF
  /* UART Initialization */
  if(UART_Global_Init()!=HAL_OK) {
    Error_Handler();
  } else {
    ALLMEMS1_PRINTF("UART Initialized\r\n");
  }
#endif /* ALLMEMS1_ENABLE_PRINTF */

  /* I2C Initialization */
  if(I2C_Global_Init()!=HAL_OK) {
    Error_Handler();
  } else {
    ALLMEMS1_PRINTF("I2C  Initialized\r\n");
  }
  
  /* Initialize the BlueNRG SPI driver */
  if(SPI_Global_Init()!=HAL_OK) {
    Error_Handler();
  } else {
    ALLMEMS1_PRINTF("SPI  Initialized\r\n");
  }
  
  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
#elif STM32_SENSORTILE
  /* Configure and disable all the Chip Select pins */
  Sensor_IO_SPI_CS_Init_All();
  
  #ifdef ALLMEMS1_ENABLE_PRINTF
  /* enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();

  /* Configure the CDC */
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);
  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
  /* Add Interface callbacks for AUDIO and CDC Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
  /* Start Device Process */
  USBD_Start(&USBD_Device);
  /* 10 seconds ... for having time to open the Terminal
   * for looking the ALLMEMS1 Initialization phase */
  HAL_Delay(10000);
  #endif /* ALLMEMS1_ENABLE_PRINTF */
#endif /* STM32_NUCLEO */
  
  /* Initialize LED */
#ifdef STM32_NUCLEO  
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED1);
#elif STM32_SENSORTILE
  BSP_LED_Init( LED1 );  
#endif /* STM32_NUCLEO */

  ALLMEMS1_PRINTF("\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
#ifdef USE_STM32F4XX_NUCLEO
  #ifdef STM32_NUCLEO
    #ifdef STM32F401xE
        "\tSTM32F401RE-Nucleo board"
    #elif STM32F446xx
          "\tSTM32F446xx-Nucleo board"
    #endif /* STM32F401xE */
  #endif /* STM32_NUCLEO */
#elif USE_STM32L4XX_NUCLEO
  #ifdef STM32_SENSORTILE
        "\tSTM32476RG-SensorTile board"
  #elif STM32_NUCLEO
        "\tSTM32L476RG-Nucleo board"
  #endif /* STM32_SENSORTILE */
#endif /* USE_STM32L4XX_NUCLEO */
          "\r\n\n",
          ALLMEMS1_PACKAGENAME,
          ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  
  /* Discovery and Intialize all the MEMS Target's Features */
  Init_MEM1_Sensors();

#ifdef USE_SENSIBLE
  VBAT_Init();
#endif /* USE_SENSIBLE */
#ifdef STM32_SENSORTILE
  /* Inialize the Gas Gouge if the battery is present */
  if(BSP_GG_Init(&TargetBoardFeatures.HandleGGComponent) == COMPONENT_OK){
    ALLMEMS1_PRINTF("OK Gas Gouge Component\n\r");
  } else {
    ALLMEMS1_PRINTF("Battery not present\n\r");
  }
#endif /* STM32_SENSORTILE */
 Init_MEMS_Mics(AUDIO_SAMPLING_FREQUENCY, AUDIO_VOLUME_VALUE); //add2  
#ifdef USE_STM32F4XX_NUCLEO
  /* Initialize Mic */
  Init_MEMS_Mics(AUDIO_SAMPLING_FREQUENCY, AUDIO_VOLUME_VALUE); //add2  
#endif /* USE_STM32F4XX_NUCLEO */
  
  ALLMEMS1_PRINTF("\n\r");
  
#ifdef USE_STM32F4XX_NUCLEO
      //BSP_AUDIO_IN_Record(PDM_Buffer, 0);
#endif /* USE_STM32F4XX_NUCLEO */
  
  TargetBoardFeatures.bnrg_expansion_board = IDB04A1; /* IDB04A1 by default */
}

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{
#ifdef IKS01A1
  ALLMEMS1_PRINTF("Code compiled for X-NUCLEO-IKS01A1\r\n");
  TargetBoardFeatures.mems_expansion_board= _IKS01A1;
#elif IKS01A2
  ALLMEMS1_PRINTF("Code compiled for X-NUCLEO-IKS01A2\r\n");
  TargetBoardFeatures.mems_expansion_board= _IKS01A2;
#endif /* IKS01A1 */
  
  /* Accelero */

  if (BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &TargetBoardFeatures.HandleAccSensor ) == COMPONENT_OK){
    ALLMEMS1_PRINTF("OK Accelero Sensor\n\r");
  } else {
    ALLMEMS1_PRINTF("Error Accelero Sensor\n\r");
#warning "Removed hang up here!"
    // while(1);
  }
  
  /* DS3/DSM or DS0 */
#ifdef STM32_NUCLEO
  #ifdef IKS01A1
  {
    uint8_t WhoAmI;
    BSP_ACCELERO_Get_WhoAmI(TargetBoardFeatures.HandleAccSensor,&WhoAmI);
    if(LSM6DS3_ACC_GYRO_WHO_AM_I==WhoAmI) {
      ALLMEMS1_PRINTF("\tDS3 DIL24 Present\n\r");
      TargetBoardFeatures.HWAdvanceFeatures = 1;
    } else {
      TargetBoardFeatures.HWAdvanceFeatures = 0;
    }
  }
  #elif IKS01A2
    #ifndef DISABLE_LSM6DSL_IT
    TargetBoardFeatures.HWAdvanceFeatures = 1;
    #else
    TargetBoardFeatures.HWAdvanceFeatures = 0;
    #endif
  #endif /* IKS01A1 */ 
  
#else /* STM32_NUCLEO */
  TargetBoardFeatures.HWAdvanceFeatures = 1;
#endif /* STM32_NUCLEO */

  /* Gyro */
  if(BSP_GYRO_Init( GYRO_SENSORS_AUTO, &TargetBoardFeatures.HandleGyroSensor )==COMPONENT_OK){
    ALLMEMS1_PRINTF("OK Gyroscope Sensor\n\r");
  } else {
    ALLMEMS1_PRINTF("Error Gyroscope Sensor\n\r");
    #warning "Removed hang up here!"
    // while(1);
  }

  if(BSP_MAGNETO_Init( MAGNETO_SENSORS_AUTO, &TargetBoardFeatures.HandleMagSensor )==COMPONENT_OK){
    ALLMEMS1_PRINTF("OK Magneto Sensor\n\r");
  } else {
    ALLMEMS1_PRINTF("Error Magneto Sensor\n\r");
    #warning "Removed hang up here!"
    // while(1);
  }

  /* Humidity */  
  if(BSP_HUMIDITY_Init( HUMIDITY_SENSORS_AUTO, &TargetBoardFeatures.HandleHumSensor )==COMPONENT_OK){
    ALLMEMS1_PRINTF("OK Humidity Sensor\n\r");
  } else {
    ALLMEMS1_PRINTF("Error Humidity Sensor\n\r");
  }

  /* Temperature1 */
  if(BSP_TEMPERATURE_Init( TEMPERATURE_SENSORS_AUTO, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
     ALLMEMS1_PRINTF("OK Temperature Sensor1\n\r");
     TargetBoardFeatures.NumTempSensors++;
  } else {
    ALLMEMS1_PRINTF("Error Temperature Sensor1\n\r");
  }

  /* Temperature2 */
#warning "Temperature 2 was enabled here"
#ifndef USE_SENSIBLE // Disable second temperature sensor
#ifdef STM32_NUCLEO
  #ifdef IKS01A1
  if(BSP_TEMPERATURE_Init( LPS25HB_T_0, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
  #elif IKS01A2
  if(BSP_TEMPERATURE_Init( LPS22HB_T_0, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
  #endif /* IKS01A1 */
     ALLMEMS1_PRINTF("OK Temperature Sensor2\n\r");
     TargetBoardFeatures.NumTempSensors++;
  } else {
    ALLMEMS1_PRINTF("Error Temperature Sensor2\n\r");
  }
#elif STM32_SENSORTILE
  if(BSP_TEMPERATURE_Init( LPS22HB_T_0, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
     ALLMEMS1_PRINTF("OK Temperature Sensor2\n\r");
     TargetBoardFeatures.NumTempSensors++;
  } else {
    ALLMEMS1_PRINTF("Error Temperature Sensor2\n\r");
  }
#endif /* STM32_NUCLEO */
#endif /* USE_SENSIBLE */
  
  /* Pressure */
  if(BSP_PRESSURE_Init( PRESSURE_SENSORS_AUTO, &TargetBoardFeatures.HandlePressSensor )==COMPONENT_OK){
    ALLMEMS1_PRINTF("OK Pressure Sensor\n\r");
  } else {
    ALLMEMS1_PRINTF("Error Pressure Sensor\n\r");
  }

  /*  Enable all the sensors */
  if(TargetBoardFeatures.HandleAccSensor) {
    if(BSP_ACCELERO_Sensor_Enable( TargetBoardFeatures.HandleAccSensor )==COMPONENT_OK) {
      ALLMEMS1_PRINTF("Enabled Accelero Sensor\n\r");
    }
  }

  if(TargetBoardFeatures.HandleGyroSensor) {
    if(BSP_GYRO_Sensor_Enable( TargetBoardFeatures.HandleGyroSensor )==COMPONENT_OK) {
      ALLMEMS1_PRINTF("Enabled Gyroscope Sensor\n\r");
    }
  }

  if(TargetBoardFeatures.HandleMagSensor) {
    if(BSP_MAGNETO_Sensor_Enable( TargetBoardFeatures.HandleMagSensor )==COMPONENT_OK) {
      ALLMEMS1_PRINTF("Enabled Magneto Sensor\n\r");
    }
  }

  if(TargetBoardFeatures.HandleHumSensor) {
    if(BSP_HUMIDITY_Sensor_Enable( TargetBoardFeatures.HandleHumSensor)==COMPONENT_OK) {
      ALLMEMS1_PRINTF("Enabled Humidity Sensor\n\r");
    }
  }

  if(TargetBoardFeatures.HandleTempSensors[0]){
    if(BSP_TEMPERATURE_Sensor_Enable( TargetBoardFeatures.HandleTempSensors[0])==COMPONENT_OK) {
      ALLMEMS1_PRINTF("Enabled Temperature Sensor1\n\r");
    }
  }
  
  if(TargetBoardFeatures.HandleTempSensors[1]){
    if(BSP_TEMPERATURE_Sensor_Enable( TargetBoardFeatures.HandleTempSensors[1])==COMPONENT_OK) {
      ALLMEMS1_PRINTF("Enabled Temperature Sensor2\n\r");
    }
  }

  if(TargetBoardFeatures.HandlePressSensor) {
    if(BSP_PRESSURE_Sensor_Enable( TargetBoardFeatures.HandlePressSensor)==COMPONENT_OK) {
      ALLMEMS1_PRINTF("Enabled Pressure Sensor\n\r");
    }
  }
}





/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */

void Init_MEMS_Mics(uint32_t AudioFreq, uint32_t AudioVolume)
{
  uint8_t ret;
  
  ret= BSP_AUDIO_IN_Init(AudioFreq, 16, AUDIO_CHANNELS);
  
  if(ret != AUDIO_OK)
  {
    ALLMEMS1_PRINTF("\nError Audio Init\r\n");
    
    while(1) {
      #warning "Removed hang up here!"
      break;
    }
  }
  else
  {
    ALLMEMS1_PRINTF("\nOK Audio Init\t(Audio Freq.= %ld)\r\n", AudioFreq);
  }
  
  /* Set the volume level */
  ret= BSP_AUDIO_IN_SetVolume(AudioVolume);
  
  if(ret != AUDIO_OK)
  {
    ALLMEMS1_PRINTF("Error Audio Volume\r\n\n");
    
    while(1) {
      #warning "Removed hang up here!"
      break;
    }
  }
  else
  {
    ALLMEMS1_PRINTF("OK Audio Volume\t(Volume= %ld)\r\n", AudioVolume);
  } 

  /* Number of Microphones */
  TargetBoardFeatures.NumMicSensors=AUDIO_CHANNELS;
}

/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
void InitMics(uint32_t AudioFreq, uint32_t AudioVolume)
{       
  Init_MEMS_Mics(AudioFreq, AudioVolume); //add2
    
#ifdef USE_STM32F4XX_NUCLEO
  BSP_AUDIO_IN_Record(PDM_Buffer, 0);
#endif /* USE_STM32F4XX_NUCLEO */
  
#if defined(STM32_SENSORTILE) || defined(USE_STM32L4XX_NUCLEO)
  BSP_AUDIO_IN_Record(PCM_Buffer,0);
#endif /* STM32_SENSORTILE || USE_STM32L4XX_NUCLEO*/
}

/** @brief DeInitialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
void DeInitMics(void)
{
  BSP_AUDIO_IN_Stop();
  
#ifndef USE_SENSIBLE
#if defined(STM32_SENSORTILE) || defined(USE_STM32L4XX_NUCLEO)
  uint8_t ret;
  
  ret= BSP_AUDIO_IN_DeInit();

  if(ret != AUDIO_OK)
  {
    ALLMEMS1_PRINTF("Error Audio DeInit\r\n");
    
    while(1) {
      #warning "Removed hang up here!"
      break;
    }
  }
  else
  {
    ALLMEMS1_PRINTF("OK Audio DeInit\r\n");
  }
#endif /* STM32_SENSORTILE || USE_STM32L4XX_NUCLEO*/
#endif
}


/**
  * @brief  This function switches on the LED
  * @param  None
  * @retval None
  */
void LedOnTargetPlatform(void)
{
#ifdef STM32_NUCLEO
  BSP_LED_On(LED2);
#elif STM32_SENSORTILE
  BSP_LED_On( LED1 );  
#endif /* STM32_NUCLEO */
  
  TargetBoardFeatures.LedStatus=1;
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
#ifdef STM32_NUCLEO
  BSP_LED_Off(LED2);
#elif STM32_SENSORTILE
  BSP_LED_Off( LED1 );  
#endif /* STM32_NUCLEO */
  
  TargetBoardFeatures.LedStatus=0;
}

/** @brief  This function toggles the LED
  * @param  None
  * @retval None
  */
void LedToggleTargetPlatform(void)
{
#ifdef STM32_NUCLEO
  BSP_LED_Toggle(LED2);
#elif STM32_SENSORTILE
  BSP_LED_Toggle( LED1 );
#endif /* STM32_NUCLEO */
}

void Led1OnTargetPlatform(void)
{
  BSP_LED_On(LED1);
  TargetBoardFeatures.Led1Status = 1;
}
void Led1OffTargetPlatform(void)
{ 
  BSP_LED_Off(LED1);
  TargetBoardFeatures.Led1Status = 0;
}
void Led1ToggleTargetPlatform(void)
{
  BSP_LED_Toggle(LED1);
  if(TargetBoardFeatures.Led1Status == 0)
  {
    TargetBoardFeatures.Led1Status = 1;
  }
  else
  {
    TargetBoardFeatures.Led1Status = 0;
  }
}


#ifdef USE_STM32L4XX_NUCLEO
/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  
  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;
  
  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
  	/* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
  	/* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }
  
  return bank;
}

/**
 * @brief User function for Erasing the MDM on Flash
 * @param None
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForErasingFlash(void) {
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  uint32_t Success=1;

  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(MDM_FLASH_ADD);
  EraseInitStruct.Page        = GetPage(MDM_FLASH_ADD);
  EraseInitStruct.NbPages     = 2;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    /* Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    Success=0;
    Error_Handler();
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return Success;
}

/**
 * @brief User function for Saving the MDM  on the Flash
 * @param void *InitMetaDataVector Pointer to the MDM beginning
 * @param void *EndMetaDataVector Pointer to the MDM end
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForSavingFlash(void *InitMetaDataVector,void *EndMetaDataVector)
{
  uint32_t Success=1;

  /* Store in Flash Memory */
  uint32_t Address = MDM_FLASH_ADD;
  uint64_t *WriteIndex;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();
  for(WriteIndex =((uint64_t *) InitMetaDataVector); WriteIndex<((uint64_t *) EndMetaDataVector); WriteIndex++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address,*WriteIndex) == HAL_OK){
      Address = Address + 8;
    } else {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error
         FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
      Error_Handler();
      Success =0;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
   to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
 
  return Success;
}
#endif /* USE_STM32L4XX_NUCLEO */

#ifdef USE_STM32F4XX_NUCLEO
/**
 * @brief User function for Erasing the Flash data for MDM
 * @param None
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForErasingFlash(void) {
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  uint32_t Success=1;

  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_7;
  EraseInitStruct.NbSectors = 1;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    /* Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    Success=0;
    Error_Handler();
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return Success;
}

/**
 * @brief User function for Saving the MDM  on the Flash
 * @param void *InitMetaDataVector Pointer to the MDM beginning
 * @param void *EndMetaDataVector Pointer to the MDM end
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForSavingFlash(void *InitMetaDataVector,void *EndMetaDataVector)
{
  uint32_t Success=1;

  /* Store in Flash Memory */
  uint32_t Address = MDM_FLASH_ADD;
  uint32_t *WriteIndex;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  for(WriteIndex =((uint32_t *) InitMetaDataVector); WriteIndex<((uint32_t *) EndMetaDataVector); WriteIndex++) {
    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address,*WriteIndex) == HAL_OK){
      Address = Address + 4;
    } else {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error
         FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
      Error_Handler();
      Success=0;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
   to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
 
  return Success;
}
#endif /* USE_STM32F4XX_NUCLEO */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
