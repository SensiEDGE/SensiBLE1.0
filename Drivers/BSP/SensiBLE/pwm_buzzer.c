#include "pwm_buzzer.h"
#include "stm32l4xx_hal.h"
#include "stm32_bluenrg_ble.h" // for SYSCLK_FREQ
#include "gp_timer.h"

// Variables
TIM_HandleTypeDef htim2;

typedef struct {
    uint8_t is_running;
    const PWM_Buzzer_Note_t *plist;
    uint16_t index;
    uint16_t length;
    struct timer timer;
} pwm_buzzer_state_t;

static pwm_buzzer_state_t state_machine;


// Prototypes
void StateMachine_Process(pwm_buzzer_state_t *sm);
void MX_TIM2_Init(uint32_t period, uint32_t pulse);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm);


void PWM_Buzzer_Init(void)
{
    state_machine.is_running = 0;
}

uint8_t PWM_Buzzer_IsRunning(void)
{
  return state_machine.is_running != 0;
}

void PWM_Buzzer_Output(uint32_t frequency)
{
    uint32_t period, pulse;

    if (frequency == 0)
    {
        HAL_TIM_PWM_DeInit(&htim2);
    }
    else
    {
        period = SYSCLK_FREQ / frequency;
        if(period > 0xFFFF) {
            // 0xFFFF is the maximum value
            period = 0xFFFF;
        }
        // pulse = period / 2 - 1;
        pulse = period / 10 - 1;    // reduce power consumption

        MX_TIM2_Init(period, pulse);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    }
}


void PWM_Buzzer_Play(const PWM_Buzzer_Note_t *list, uint16_t listSize)
{
    state_machine.is_running = 0;

    state_machine.plist = list;
    state_machine.index = 0;
    state_machine.length = listSize;

    StateMachine_Process(&state_machine);
    state_machine.is_running = 1;
}

void PWM_Buzzer_Beep(void)
{
    static const PWM_Buzzer_Note_t beeps[] = {
        { 1000, 50 }
    };

    PWM_Buzzer_Play(beeps, 1);
}

void PWM_Buzzer_BeepBeepBeep(void)
{
    static const PWM_Buzzer_Note_t beeps[] = {
        { 1000, 50 },
        { 0, 50 },
        { 1000, 50 },
        { 0, 50 },
        { 1000, 50 }
    };

    PWM_Buzzer_Play(beeps, 5);
}


void PWM_Buzzer_MainLoopHandler(void)
{
    if (state_machine.is_running) {

        if (Timer_Expired(&state_machine.timer)) {

            state_machine.index++;
            StateMachine_Process(&state_machine);
        }
    }
}


void StateMachine_Process(pwm_buzzer_state_t *sm)
{
    if (sm->index >= sm->length) {
        sm->is_running = 0;
        PWM_Buzzer_Output(0);
    }
    else {
        uint32_t frequency, interval;

        frequency = sm->plist[sm->index].Frequency;
        interval = sm->plist[sm->index].Interval;

        PWM_Buzzer_Output(frequency);
        Timer_Set(&sm->timer, interval);
    }
}


void MX_TIM2_Init(uint32_t period, uint32_t pulse)
{
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = period;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

    HAL_TIM_MspPostInit(&htim2);
}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{

  if(htim_pwm->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */

    /**TIM2 GPIO Configuration
    PB11     ------> TIM2_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }

}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{

  if(htim_pwm->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

      GPIO_InitTypeDef GPIO_InitStruct;

      GPIO_InitStruct.Pin = GPIO_PIN_11;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = 0;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */

}

