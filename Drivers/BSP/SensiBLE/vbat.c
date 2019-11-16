#include "vbat.h"
#include "stm32l4xx_hal.h"

BAT_Params_t BAT_Params;

static ADC_HandleTypeDef hadc1;

void VBAT_Init(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
    ADC_ChannelConfTypeDef sConfig;

    __HAL_RCC_ADC_CLK_ENABLE();
      
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    /**Common config */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    HAL_ADC_Init(&hadc1);

    /**Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLE_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Calibrate ADC
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}

void VBAT_Deinit(void)
{
  HAL_ADC_DeInit(&hadc1);
}

void VBAT_GetParams(BAT_Params_t * params)
{
  uint32_t adc_result;
    uint32_t adc_calibration;
    uint32_t voltage;
    
    // Do ADC
    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1, 1000) != HAL_OK){
        return;  
    }
    
    // Calculate volatage
    adc_result = HAL_ADC_GetValue(&hadc1);
    adc_calibration =  *((volatile uint16_t*)(0x1FFF75AA));
    voltage = (3000 * adc_calibration) / adc_result;
    
    // Fill params
    params->voltage = voltage;
    params->current = 0;
    params->status = SIM_BAT_STATUS_DISCHARGING;

    if (voltage >= 3000) params->level = 999;
    else if (voltage < 2000) params->level = 0;
    else params->level = voltage - 2000;
}
