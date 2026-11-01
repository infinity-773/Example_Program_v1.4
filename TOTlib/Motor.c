#include "Motor.h"

#include <stddef.h>

static void motorApplySpeed(Motor *motor, int16_t speed)
{
	if (motor->speed_consumer != NULL)
	{
		motor->speed_consumer(motor->id, speed);
	}
}

void Motor_Init(Motor *motor, BoardMotor_t id, int16_t speed,
				MotorSpeedConsumer speed_consumer,
				MotorEnableRead read_enable)
{
	if (motor == NULL)
	{
		return;
	}

	motor->state = MOTOR_BRAKED;
	motor->id = id;
	motor->speed = speed;
	motor->breakAt = 0U;
	motor->speed_consumer = speed_consumer;
	motor->read_enable = read_enable;

	motorApplySpeed(motor, 0);
}

uint8_t Motor_IsEnabled(const Motor *motor)
{
	return (motor != NULL && motor->read_enable != NULL &&
			motor->read_enable()) ? 1U : 0U;
}

void Motor_SetTargetSpeed(Motor *motor, int16_t speed)
{
	if (motor != NULL)
	{
		motor->speed = speed;
	}
}

uint8_t Motor_IsBraked(const Motor *motor)
{
	return (motor != NULL && motor->state == MOTOR_BRAKED) ? 1U : 0U;
}

void Motor_periodic(Motor *motor, uint32_t now)
{
	if (motor == NULL)
	{
		return;
	}

	if (!Motor_IsEnabled(motor))
	{
		Motor_Stop(motor);
		return;
	}

	if (motor->state == MOTOR_BRAKING &&
		(uint32_t)(now - motor->breakAt) >= MOTOR_BRAKE_DURATION_MS)
	{
		motor->state = MOTOR_BRAKED;
	}

	switch (motor->state)
	{
	case MOTOR_BRAKED:
		motorApplySpeed(motor, 0);
		break;

	case MOTOR_RUNNING:
		motorApplySpeed(motor, motor->speed);
		break;

	case MOTOR_BRAKING:
		motorApplySpeed(motor, MOTOR_BRAKE_REVERSE_SPEED);
		break;

	default:
		motor->state = MOTOR_BRAKED;
		motorApplySpeed(motor, 0);
		break;
	}
}

void Motor_RequestBrake(Motor *motor, uint32_t now)
{
	if (motor != NULL && motor->state == MOTOR_RUNNING)
	{
		motor->state = MOTOR_BRAKING;
		motor->breakAt = now;
	}
}

void Motor_Start(Motor *motor)
{
	if (Motor_IsEnabled(motor) && motor->state == MOTOR_BRAKED)
	{
		motor->state = MOTOR_RUNNING;
	}
}

void Motor_Stop(Motor *motor)
{
	if (motor == NULL)
	{
		return;
	}

	motor->state = MOTOR_BRAKED;
	motorApplySpeed(motor, 0);
}
