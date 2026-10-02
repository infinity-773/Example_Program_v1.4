#ifndef __BOARD_H__
#define __BOARD_H__

#include "adc.h"
#include "PWM.h"
#include "main.h"

/*
 * ???????????? board.c ????????????? PIN_xxx ???????
 
 */

/* ADC??PA4 PC5 PA5 PC0 PC1 PC2 */
typedef enum
{
    PIN_PA4,
    PIN_PC5,
    PIN_PA5,
    PIN_PC0,
    PIN_PC1,
    PIN_PC2,
    PIN_ADC_COUNT
} BoardAdcPin_t;

/* PWM??A6 A7 B6 B7 B8 B9 B0 A0 A1 A2 A3????? PA11??LED1?? */
typedef enum
{
    PIN_PB0,
    PIN_PB6,
    PIN_PB7,
    PIN_PB8,
    PIN_PB9,
    PIN_PA0,
    PIN_PA1,
    PIN_PA2,
    PIN_PA3,
    PIN_PA6,
    PIN_PA7,
    PIN_PA11,
    PIN_PWM_COUNT
} BoardPwmPin_t;

/* ??? GPIO??B1 B4 B5 B10~B15, C6 C13, D2 */
typedef enum
{
    PIN_PB1,
    PIN_PB4,
    PIN_PB5,
    PIN_PB10,
    PIN_PB11,
    PIN_PB12,
    PIN_PB13,
    PIN_PB14,
    PIN_PB15,
    PIN_PC6,
    PIN_PC13,
    PIN_PD2,
    PIN_GPIO_COUNT
} BoardGpioPin_t;

typedef enum
{
    MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_COUNT
} BoardMotor_t;

uint16_t ADC_ReadPin(BoardAdcPin_t pin);
void PWM_SetPin(BoardPwmPin_t pin, int32_t duty);

void GPIO_WritePin(BoardGpioPin_t pin, GPIO_PinState state);
GPIO_PinState GPIO_ReadPin(BoardGpioPin_t pin);
void GPIO_TogglePin(BoardGpioPin_t pin);

void Motor_DriverEnable(uint8_t on);
void Motor_SetSpeed(BoardMotor_t motor, int16_t speed);

#endif /* __BOARD_H__ */
