#ifndef _BUZZER_H
#define _BUZZER_H

#include <stdint.h>

#include "board.h"

void Buzzer_Init(BoardPwmPin_t pwmPin);
void Buzzer_SetFrequency(uint16_t hz);
void Buzzer_SetVolume(uint8_t volume);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Toggle(void);

#endif