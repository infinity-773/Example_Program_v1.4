#include "Buzzer.h"

#include "board.h"

#define BUZZER_VOLUME_MAX       100U
#define BUZZER_PWM_FULL_SCALE   4800U

static uint8_t s_volume = BUZZER_VOLUME_MAX;
static uint8_t s_enabled;

static void buzzerApplyVolume(void)
{
    uint32_t duty = 0U;

    if (s_enabled)
    {
        duty = (uint32_t)s_volume * BUZZER_PWM_FULL_SCALE /
               BUZZER_VOLUME_MAX;
    }

    PWM_SetPin(PIN_PB9, (int32_t)duty);
}

void Buzzer_Init(void)
{
    s_volume = BUZZER_VOLUME_MAX;
    s_enabled = 0U;
    buzzerApplyVolume();
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