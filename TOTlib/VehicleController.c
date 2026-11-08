#include "VehicleController.h"

#include <stddef.h>

#define VEHICLE_START_DELAY_MS      2000U
#define VEHICLE_MAGNET_ARM_DELAY_MS 5000U

void VehicleController_Init(VehicleController *controller, Motor *motor,
                            ReedSwitch *parking_sensor)
{
    if (controller == NULL)
    {
        return;
    }

    controller->motor = motor;
    controller->parking_sensor = parking_sensor;
    controller->state = WAITING_FOR_ENABLE;
    controller->state_started_at = 0U;
}

void VehicleController_periodic(VehicleController *controller, uint32_t now,
                                int16_t drive_speed)
{
    if (controller == NULL || controller->motor == NULL ||
        controller->parking_sensor == NULL)
    {
        return;
    }

    if (!Motor_IsEnabled(controller->motor))
    {
        controller->state = WAITING_FOR_ENABLE;
        controller->state_started_at = now;
        ReedSwitch_Disarm(controller->parking_sensor);
        Motor_periodic(controller->motor, now);
        return;
    }

    Motor_SetTargetSpeed(controller->motor, drive_speed);

    if (controller->state == WAITING_FOR_ENABLE)
    {
        controller->state = START_DELAY;
        controller->state_started_at = now;
    }

    switch (controller->state)
    {
    case START_DELAY:
        if ((uint32_t)(now - controller->state_started_at) >=
            VEHICLE_START_DELAY_MS)
        {
            Motor_Start(controller->motor);
            controller->state = DRIVING_UNARMED;
            controller->state_started_at = now;
        }
        break;

    case DRIVING_UNARMED:
        if ((uint32_t)(now - controller->state_started_at) >=
            VEHICLE_MAGNET_ARM_DELAY_MS)
        {
            ReedSwitch_Arm(controller->parking_sensor, now);
            controller->state = MAGNET_ARMED;
        }
        break;

    case MAGNET_ARMED:
        if (ReedSwitch_RisingEdge(controller->parking_sensor, now))
        {
            Motor_RequestBrake(controller->motor, now);
            controller->state = BRAKING;
        }
        break;

    case BRAKING:
    case STOPPED:
        break;

    case WAITING_FOR_ENABLE:
    default:
        controller->state = WAITING_FOR_ENABLE;
        Motor_Stop(controller->motor);
        break;
    }

    Motor_periodic(controller->motor, now);

    if (controller->state == BRAKING &&
        Motor_IsBraked(controller->motor))
    {
        controller->state = STOPPED;
    }
}