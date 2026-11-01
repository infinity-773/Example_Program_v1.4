#ifndef _VEHICLECONTROLLER_H
#define _VEHICLECONTROLLER_H

#include <stdint.h>
#include "Motor.h"
#include "ReedSwitch.h"

typedef enum
{
    WAITING_FOR_ENABLE = 0U,
    START_DELAY,
    DRIVING_UNARMED,
    MAGNET_ARMED,
    BRAKING,
    STOPPED
} VehicleState;

typedef struct
{
    Motor *motor;
    ReedSwitch *parking_sensor;
    VehicleState state;
    uint32_t state_started_at;
} VehicleController;

void VehicleController_Init(VehicleController *controller, Motor *motor,
                            ReedSwitch *parking_sensor);
void VehicleController_periodic(VehicleController *controller, uint32_t now,
                                int16_t drive_speed);

#endif