#ifndef __VBAT_H__
#define __VBAT_H__

#include <stdint.h>
typedef struct {
    uint16_t level;
    uint16_t voltage;
    uint16_t current;
    uint8_t status;
} BAT_Params_t;

extern BAT_Params_t BAT_Params;

#define SIM_BAT_STATUS_LOW_BATTERY              0
#define SIM_BAT_STATUS_DISCHARGING              1
#define SIM_BAT_STATUS_PLUGGED_NOT_CHARGING     2
#define SIM_BAT_STATUS_CHARGING                 3
#define SIM_BAT_STATUS_ERROR                    0xFF

void VBAT_Init(void);
void VBAT_Deinit(void);
void VBAT_GetParams(BAT_Params_t * params);



#endif //__VBAT_H__
