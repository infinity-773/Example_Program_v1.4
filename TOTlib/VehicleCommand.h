#ifndef _VEHICLECOMMAND_H
#define _VEHICLECOMMAND_H

#include <stdint.h>
#include "TOTLib.h"

typedef struct
{
    Debouncer_t debouncer;
    uint8_t previous_state;
} VehicleCommand;

/* initial_state 为 0 时，条件在装填时已成立也会触发 */
void VehicleCommand_Arm(VehicleCommand *command, uint8_t initial_state,
                        uint32_t debounce_ms, uint32_t now);
/* 消抖后条件出现上升沿时返回 1 */
uint8_t VehicleCommand_Triggered(VehicleCommand *command, uint8_t condition,
                                 uint32_t now);

#endif
