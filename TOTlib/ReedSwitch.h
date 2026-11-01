#ifndef _REEDSWITCH_H
#define _REEDSWITCH_H

#include <stdint.h>
#include "TOTLib.h"

typedef uint8_t (*ReedSwitchReadState)(void);

typedef struct
{
    ReedSwitchReadState read_state;
    Debouncer_t debouncer;
    uint8_t previous_state;
    uint8_t armed;
} ReedSwitch;

void ReedSwitch_Init(ReedSwitch *sensor, ReedSwitchReadState read_state);
void ReedSwitch_Arm(ReedSwitch *sensor, uint32_t now);
void ReedSwitch_Disarm(ReedSwitch *sensor);
uint8_t ReedSwitch_RisingEdge(ReedSwitch *sensor, uint32_t now);

#endif