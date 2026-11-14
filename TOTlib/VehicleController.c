#include "VehicleController.h"

#include <stddef.h>

#define VEHICLE_START_DELAY_MS      2000U // 2s后启用开车条件判断
#define VEHICLE_PARKING_ARM_DELAY_MS 5000U // 5s后启用停车条件判断
#define VEHICLE_PARKING_DEBOUNCE_MS 10U // 停车条件消抖时间
#define VEHICLE_START_DEBOUNCE_MS   20U // 开车条件消抖时间

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
}

int16_t VehicleController_GetOutputSpeed(const VehicleController *controller)
{
    return (controller != NULL) ? Motor_GetOutput(&controller->motor) : 0;
}

void VehicleController_periodic(VehicleController *controller, uint32_t now,
                                int16_t drive_speed,
                                uint8_t start_condition,
                                uint8_t stop_condition)
{
    if (controller == NULL)
    {
        return;
    }

    if (!Motor_IsEnabled(&controller->motor))
    {
        controller->state = WAITING_FOR_ENABLE;
        controller->state_started_at = now;
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
            VehicleCommand_Arm(&controller->start_command, 0U,
                               VEHICLE_START_DEBOUNCE_MS, now);
            controller->state = WAITING_FOR_START;
        }
        break;

    case WAITING_FOR_START:
        if (VehicleCommand_Triggered(&controller->start_command,
                                     start_condition, now))
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
            VehicleCommand_Arm(&controller->stop_command, stop_condition,
                               VEHICLE_PARKING_DEBOUNCE_MS, now);
            controller->state = PARKING_ARMED;
        }
        break;

    case PARKING_ARMED:
        if (VehicleCommand_Triggered(&controller->stop_command,
                                     stop_condition, now))
        {
            Motor_RequestBrake(&controller->motor, now);
            controller->state = BRAKING;
        }
        break;

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