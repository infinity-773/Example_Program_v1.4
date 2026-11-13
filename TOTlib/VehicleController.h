#ifndef _VEHICLECONTROLLER_H
#define _VEHICLECONTROLLER_H

#include <stdint.h>
#include "Motor.h"
#include "TOTLib.h"

typedef enum
{
    WAITING_FOR_ENABLE = 0U,
    START_DELAY,
    DRIVING_UNARMED,
    PARKING_ARMED,
    BRAKING,
    STOPPED
} VehicleState;

typedef struct
{
    Motor motor;
    Debouncer_t parking_debouncer;
    VehicleState state;
    uint32_t state_started_at;
    uint8_t previous_parking_state;
    uint8_t parking_sensor_armed;
} VehicleController;

void VehicleController_Init(VehicleController *controller, BoardMotor_t motor_id,
                            MotorSpeedConsumer speed_consumer,
                            MotorEnableRead read_enable);
int16_t VehicleController_GetOutputSpeed(const VehicleController *controller);
void VehicleController_periodic(VehicleController *controller, uint32_t now,
                                int16_t drive_speed,
                                uint8_t parking_condition);

#endif