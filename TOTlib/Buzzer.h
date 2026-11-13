#ifndef _BUZZER_H
#define _BUZZER_H

#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_SetVolume(uint8_t volume);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Toggle(void);

#endif