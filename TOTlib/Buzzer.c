#include "Buzzer.h"

#include "board.h"
#include "tim.h"

#define BUZZER_VOLUME_MAX       100U
#define BUZZER_PWM_FULL_SCALE   4800U
#define BUZZER_TIM_TICK_HZ      500000U  /* TIM3 计数频率 (72 MHz / 144) */
#define BUZZER_DEFAULT_HZ       2500U

static BoardPwmPin_t s_pin;
static uint8_t s_volume = BUZZER_VOLUME_MAX;
static uint8_t s_enabled;
static uint8_t s_high;

static uint32_t buzzerDuty(void)
{
    return (uint32_t)s_volume * BUZZER_PWM_FULL_SCALE / BUZZER_VOLUME_MAX;
}

static void buzzerApplyVolume(void)
{
    if (s_enabled)
    {
        s_high = 1U;
        PWM_SetPin(s_pin, (int32_t)buzzerDuty());
        __HAL_TIM_SET_COUNTER(&htim3, 0U);
        __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    }
    else
    {
        __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
        PWM_SetPin(s_pin, 0);
    }
}

/* TIM3 每半个音频周期中断一次，把 TIM4 的 15 kHz 载波门控成方波 */
void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE))
    {
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

        s_high = (uint8_t)!s_high;
        PWM_SetPin(s_pin, s_high ? (int32_t)buzzerDuty() : 0);
    }
}

void Buzzer_Init(BoardPwmPin_t pwmPin)
{
    s_pin = pwmPin;
    s_volume = BUZZER_VOLUME_MAX;
    s_enabled = 0U;

    /* 关闭 CCR 预装载，门控沿不必等到下一个载波周期 */
    __HAL_TIM_DISABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_4);

    Buzzer_SetFrequency(BUZZER_DEFAULT_HZ);
    HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);  /* 立即装载 ARR */
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE(&htim3);

    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    buzzerApplyVolume();
}

void Buzzer_SetFrequency(uint16_t hz)
{
    if (hz == 0U)
    {
        return;
    }

    uint32_t ticks = (BUZZER_TIM_TICK_HZ + hz) / (2U * hz);
    if (ticks < 2U)
    {
        ticks = 2U;
    }
    else if (ticks > 0x10000U)
    {
        ticks = 0x10000U;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim3, ticks - 1U);
}

void Buzzer_SetVolume(uint8_t volume)
{
    if (volume > BUZZER_VOLUME_MAX)
    {
        volume = BUZZER_VOLUME_MAX;
    }

    s_volume = volume;
    s_enabled = (volume > 0U) ? 1U : 0U;
    buzzerApplyVolume();
}

void Buzzer_On(void)
{
    if (s_volume == 0U)
    {
        s_volume = BUZZER_VOLUME_MAX;
    }

    s_enabled = 1U;
    buzzerApplyVolume();
}

void Buzzer_Off(void)
{
    s_enabled = 0U;
    buzzerApplyVolume();
}

void Buzzer_Toggle(void)
{
    if (s_enabled)
    {
        Buzzer_Off();
    }
    else
    {
        Buzzer_On();
    }
}