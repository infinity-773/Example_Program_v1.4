#ifndef _VEHICLECONTROLLER_H
#define _VEHICLECONTROLLER_H

#include <stdint.h>
#include "Motor.h"
#include "TOTLib.h"
#include "VehicleCommand.h"

typedef enum
{
    WAITING_FOR_ENABLE = 0U,
    START_DELAY,
    WAITING_FOR_START,
    DRIVING_UNARMED,
    PARKING_ARMED,
    BRAKING,
    STOPPED
} VehicleState;

typedef struct
{
    Motor motor;
    VehicleCommand start_command;
    VehicleCommand stop_command;
    VehicleState state;
    uint32_t state_started_at;
} VehicleController;

void VehicleController_Init(VehicleController *controller, BoardMotor_t motor_id,
                            MotorSpeedConsumer speed_consumer,
                            MotorEnableRead read_enable);
int16_t VehicleController_GetOutputSpeed(const VehicleController *controller);
void VehicleController_periodic(VehicleController *controller, uint32_t now,
                                int16_t drive_speed,
                                uint8_t start_condition,
                                uint8_t stop_condition);

#endif