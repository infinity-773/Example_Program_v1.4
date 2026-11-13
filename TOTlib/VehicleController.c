#include "VehicleController.h"

#include <stddef.h>

#define VEHICLE_START_DELAY_MS      2000U
#define VEHICLE_PARKING_ARM_DELAY_MS 5000U
#define VEHICLE_PARKING_DEBOUNCE_MS 20U

void VehicleController_Init(VehicleController *controller, BoardMotor_t motor_id,
                            MotorSpeedConsumer speed_consumer,
                            MotorEnableRead read_enable)
{
    if (controller == NULL)
    {
        return;
    }

    Motor_Init(&controller->motor, motor_id, 0, speed_consumer, read_enable);
    controller->state = WAITING_FOR_ENABLE;
    controller->state_started_at = 0U;
    controller->previous_parking_state = 0U;
    controller->parking_sensor_armed = 0U;
}

int16_t VehicleController_GetOutputSpeed(const VehicleController *controller)
{
    return (controller != NULL) ? Motor_GetOutput(&controller->motor) : 0;
}

void VehicleController_periodic(VehicleController *controller, uint32_t now,
                                int16_t drive_speed,
                                uint8_t parking_condition)
{
    if (controller == NULL)
    {
        return;
    }

    parking_condition = (parking_condition != 0U) ? 1U : 0U;

    if (!Motor_IsEnabled(&controller->motor))
    {
        controller->state = WAITING_FOR_ENABLE;
        controller->state_started_at = now;
        controller->parking_sensor_armed = 0U;
        Motor_periodic(&controller->motor, now);
        return;
    }

    Motor_SetTargetSpeed(&controller->motor, drive_speed);

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
            Motor_Start(&controller->motor);
            controller->state = DRIVING_UNARMED;
            controller->state_started_at = now;
        }
        break;

    case DRIVING_UNARMED:
        if ((uint32_t)(now - controller->state_started_at) >=
            VEHICLE_PARKING_ARM_DELAY_MS)
        {
            Debouncer_Init(&controller->parking_debouncer,
                           parking_condition,
                           VEHICLE_PARKING_DEBOUNCE_MS,
                           DEBOUNCER_EDGE_BOTH, now);
            controller->previous_parking_state = parking_condition;
            controller->parking_sensor_armed = 1U;
            controller->state = PARKING_ARMED;
        }
        break;

    case PARKING_ARMED:
    {
        uint8_t stable_parking_state = Debouncer_calc(
            &controller->parking_debouncer, parking_condition, now);
        uint8_t parking_rising_edge =
            (!controller->previous_parking_state && stable_parking_state) ?
                1U : 0U;
        controller->previous_parking_state = stable_parking_state;

        if (controller->parking_sensor_armed && parking_rising_edge)
        {
            Motor_RequestBrake(&controller->motor, now);
            controller->state = BRAKING;
        }
        break;
    }

    case BRAKING:
    case STOPPED: // 停车后不恢复，需关闭使能再开启
        break;

    case WAITING_FOR_ENABLE:
    default:
        controller->state = WAITING_FOR_ENABLE;
        Motor_Stop(&controller->motor);
        break;
    }

    Motor_periodic(&controller->motor, now);

    if (controller->state == BRAKING &&
        Motor_IsBraked(&controller->motor))
    {
        controller->state = STOPPED;
    }
}