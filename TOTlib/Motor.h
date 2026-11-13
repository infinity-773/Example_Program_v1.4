#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>
#include "board.h"

typedef enum
{
    MOTOR_BRAKED = 0U,
    MOTOR_RUNNING,
    MOTOR_BRAKING
} MotorState;

typedef void (*MotorSpeedConsumer)(BoardMotor_t motor, int16_t speed);
typedef uint8_t (*MotorEnableRead)(void);

#define MOTOR_BRAKE_REVERSE_SPEED (-180)
#define MOTOR_BRAKE_DURATION_MS   200U

typedef struct
{
    MotorState state;
    BoardMotor_t id;
    int16_t speed;
    int16_t output;
    uint32_t breakAt;
    MotorSpeedConsumer speed_consumer;
    MotorEnableRead read_enable;
} Motor;

void Motor_Init(Motor *motor, BoardMotor_t id, int16_t speed,
                MotorSpeedConsumer speed_consumer,
                MotorEnableRead read_enable);
void Motor_SetTargetSpeed(Motor *motor, int16_t speed);
uint8_t Motor_IsEnabled(const Motor *motor);
uint8_t Motor_IsBraked(const Motor *motor);
int16_t Motor_GetOutput(const Motor *motor);
void Motor_periodic(Motor *motor, uint32_t now);
void Motor_RequestBrake(Motor *motor, uint32_t now);
void Motor_Start(Motor *motor);
void Motor_Stop(Motor *motor);

#endif