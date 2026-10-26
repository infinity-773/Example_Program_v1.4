

#ifndef __TOTLIB_H
#define __TOTLIB_H

#include <stdint.h>
#include <math.h>


float_t TOT_normalize(float_t input, float_t min, float_t max);

float TOT_DiffRatio(float leftnum,float rightnum);

typedef struct
{
	uint16_t samples[3];
	uint32_t sum;
	uint8_t next_index;
	uint8_t sample_count;
} TOT_MovingAverage3_t;

typedef struct{
	
	float Kp; 
	float Ki; 
	float Kd; 
	
	float pOut;
	float iOut;
	float dOut;

	float out;

	float pError;
	float iError;
	float dError[2];

	float state; //低通滤波器的状态变量

	float iOutMax;

	float outMaxABS; 

} PID_TypeDef;

typedef struct{
	
	float KpMin; 
	float KiMin; 
	float KdMin; 

	float KpMax; 
	float KiMax;
	float KdMax;	
} PID_Map;

uint16_t TOT_AverageFilt(TOT_MovingAverage3_t *filter, uint16_t sample);

void PID_LoadGains(PID_TypeDef *pid, float Kp, float Ki, float Kd);
float PID_Calculate(PID_TypeDef *pid, float pIn , float iIn, float dIn );

float LPF(float input, float *state, float alpha);

float lerp(float a, float b, float t);

void applyDeadZone(float* value , float Zone);

float clampf(float x , float low , float high);


#endif