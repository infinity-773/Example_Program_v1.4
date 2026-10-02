#include "board.h"
#include "Fun.h"

#define MOTOR_PWM_FULL_SCALE    (4800)

static const ADC_Channel_enum s_adc_map[PIN_ADC_COUNT] =
{
    [PIN_PA4] = ADC1_CH04_PA4,
    [PIN_PC5] = ADC1_CH15_PC5,
    [PIN_PA5] = ADC1_CH05_PA5,
    [PIN_PC0] = ADC1_CH10_PC0,
    [PIN_PC1] = ADC1_CH11_PC1,
    [PIN_PC2] = ADC1_CH12_PC2,
};

static uint32_t * const s_pwm_map[PIN_PWM_COUNT] =
{
    [PIN_PB0]  = PWM_TIM3_CH3_B0,
    [PIN_PB6]  = PWM_TIM4_CH1_B6,
    [PIN_PB7]  = PWM_TIM4_CH2_B7,
    [PIN_PB8]  = PWM_TIM4_CH3_B8,
    [PIN_PB9]  = PWM_TIM4_CH4_B9,
    [PIN_PA0]  = PWM_TIM2_CH1_A0,
    [PIN_PA1]  = PWM_TIM2_CH2_A1,
    [PIN_PA2]  = PWM_TIM5_CH3_A2,
    [PIN_PA3]  = PWM_TIM5_CH4_A3,
    [PIN_PA6]  = PWM_TIM3_CH1_A6,
    [PIN_PA7]  = PWM_TIM3_CH2_A7,
    [PIN_PA11] = PWM_TIM1_CH4_A11,
};

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} GpioHw_t;

static const GpioHw_t s_gpio[PIN_GPIO_COUNT] =
{
    [PIN_PB1]  = { GPIOB, GPIO_PIN_1  },
    [PIN_PB4]  = { GPIOB, GPIO_PIN_4  },
    [PIN_PB5]  = { GPIOB, GPIO_PIN_5  },
    [PIN_PB10] = { GPIOB, GPIO_PIN_10 },
    [PIN_PB11] = { GPIOB, GPIO_PIN_11 },
    [PIN_PB12] = { GPIOB, GPIO_PIN_12 },
    [PIN_PB13] = { GPIOB, GPIO_PIN_13 },
    [PIN_PB14] = { GPIOB, GPIO_PIN_14 },
    [PIN_PB15] = { GPIOB, GPIO_PIN_15 },
    [PIN_PC6]  = { GPIOC, GPIO_PIN_6  },
    [PIN_PC13] = { GPIOC, GPIO_PIN_13 },
    [PIN_PD2]  = { GPIOD, GPIO_PIN_2  },
};

typedef struct
{
    BoardPwmPin_t pwm_pin;
    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;
} MotorHw_t;

static const MotorHw_t s_motor[MOTOR_COUNT] =
{
    [MOTOR_LEFT]  = { PIN_PB7, Wheel_Left_IO_GPIO_Port,  Wheel_Left_IO_Pin  },
    [MOTOR_RIGHT] = { PIN_PB6, Wheel_Right_IO_GPIO_Port, Wheel_Right_IO_Pin },
};

uint16_t ADC_ReadPin(BoardAdcPin_t pin)
{
    if (pin >= PIN_ADC_COUNT)
    {
        return 0;
    }

    return ADC_GetValue_Channel(s_adc_map[pin]);
}

void PWM_SetPin(BoardPwmPin_t pin, int32_t duty)
{
    if (pin >= PIN_PWM_COUNT)
    {
        return;
    }

    PWM_SetDuty(s_pwm_map[pin], duty);
}

void GPIO_WritePin(BoardGpioPin_t pin, GPIO_PinState state)
{
    if (pin >= PIN_GPIO_COUNT)
    {
        return;
    }

    HAL_GPIO_WritePin(s_gpio[pin].port, s_gpio[pin].pin, state);
}

GPIO_PinState GPIO_ReadPin(BoardGpioPin_t pin)
{
    if (pin >= PIN_GPIO_COUNT)
    {
        return GPIO_PIN_RESET;
    }

    return HAL_GPIO_ReadPin(s_gpio[pin].port, s_gpio[pin].pin);
}

void GPIO_TogglePin(BoardGpioPin_t pin)
{
    if (pin >= PIN_GPIO_COUNT)
    {
        return;
    }

    HAL_GPIO_TogglePin(s_gpio[pin].port, s_gpio[pin].pin);
}

void Motor_DriverEnable(uint8_t on)
{
    HAL_GPIO_WritePin(Enable_IO_GPIO_Port, Enable_IO_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_SetSpeed(BoardMotor_t motor, int16_t speed)
{
    uint16_t pwm;
    GPIO_PinState dir;

    if (motor >= MOTOR_COUNT)
    {
        return;
    }

    speed = Data_Limit(speed, -4000, 4000);

    if (speed >= 0)
    {
        pwm = (uint16_t)speed;
        dir = GPIO_PIN_RESET;
    }
    else
    {
        pwm = (uint16_t)(MOTOR_PWM_FULL_SCALE + speed);
        dir = GPIO_PIN_SET;
    }

    PWM_SetPin(s_motor[motor].pwm_pin, pwm);
    HAL_GPIO_WritePin(s_motor[motor].dir_port, s_motor[motor].dir_pin, dir);
}
