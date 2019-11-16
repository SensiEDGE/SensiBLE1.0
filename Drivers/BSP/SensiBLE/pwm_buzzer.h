#ifndef __PWM_BUZZER_H__
#define __PWM_BUZZER_H__

#include <stdint.h>

typedef struct
{
    uint32_t Frequency;
    uint32_t Interval;
} PWM_Buzzer_Note_t;

void PWM_Buzzer_Init(void);
void PWM_Buzzer_Output(uint32_t frequency);
void PWM_Buzzer_Play(const PWM_Buzzer_Note_t *list, uint16_t listSize);
void PWM_Buzzer_MainLoopHandler(void);
void PWM_Buzzer_Beep(void);
void PWM_Buzzer_BeepBeepBeep(void);
uint8_t PWM_Buzzer_IsRunning(void);

#endif //__PWM_BUZZER_H__
