

#ifndef __TOTLIB_H
#define __TOTLIB_H

#include <stdint.h>
#include <math.h>


float_t TOT_normalize(float_t input, float_t min, float_t max);

float TOT_cla(float leftnum,float rightnum);

typedef struct
{
	uint16_t samples[5];
	uint32_t sum;
	uint8_t next_index;
	uint8_t sample_count;
} TOT_MovingAverage5_t;

uint16_t TOT_AverageFilt(TOT_MovingAverage5_t *filter, uint16_t sample);



#endif