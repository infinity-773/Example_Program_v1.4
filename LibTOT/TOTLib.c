#include "TOTLib.h"
#include <math.h>
#include <stddef.h>
#include "pid.h"
#include "stm32f1xx_hal.h"


//归一化
//返回值：浮点0~1
float_t TOT_normalize(float_t input, float_t min, float_t max){
    float_t output;
    output = (input - min) / (max - min);
    return output;
}

//差比和
//返回值：浮点-1~1
float TOT_DiffRatio(float leftnum,float rightnum){ //TODO :参数改为正数
    if ((leftnum + rightnum) == 0.0f)
    {
        return 0.0f; // 避免除以零的情况
    }
    float output = (leftnum - rightnum) / (rightnum + leftnum);
    return output;
}


//3点移动平均滤波
uint16_t TOT_AverageFilt(TOT_MovingAverage3_t *filter, uint16_t rawValue )
{
    if (filter == NULL)
    {
        return rawValue;
    }

    if (filter->sample_count < 3U)
    {
        filter->samples[filter->next_index] = rawValue;
        filter->sum += rawValue;
        filter->sample_count++;
    }
    else
    {
        filter->sum -= filter->samples[filter->next_index];
        filter->samples[filter->next_index] = rawValue;
        filter->sum += rawValue;
    }

    filter->next_index = (uint8_t)((filter->next_index + 1U) % 3U);

    return (uint16_t)(filter->sum / filter->sample_count);
}

//一阶低通滤波
float LPF(float input, float *state, float alpha)
{
    *state += alpha * (input - *state);
    return *state;
}

// 线性插值
float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}


float clampf(float x , float low , float high){
    if (x < low){return low;}
    if (x > high){return high;}
    return x;
}

//设置死区
void applyDeadZone( float* value,float zone ){
    if (fabs(*value) < zone){
        *value = 0.0;
    }
}

// 多输入源位置式PID
void PID_LoadGains(PID_TypeDef *pid, float_t Kp, float_t Ki, float_t Kd)
{
    if (pid == NULL)
    {
        return;
    }

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

float PID_Calculate(PID_TypeDef *pid, float pIn , float iIn, float dIn)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->pError = pIn;
    pid->iError = iIn;
    pid->dError[1] = pid->dError[0];
    pid->dError[0] = dIn;


    pid->pOut = pid->Kp * pid->pError;

    pid->iOut += pid->Ki * pid->iError ;

    // 对ADC原始值做微分->低通滤波->乘上Kd
    float dRaw = pid->dError[0] - pid->dError[1];
    pid->state = LPF(dRaw, &pid->state, 0.15);
    pid->dOut = pid->Kd * pid->state;


    if(pid->iOutMax != 0){
            pid->iOut = clampf(pid->iOut , -pid->iOutMax , pid->iOutMax);
    }

    pid->out = pid->pOut + pid->iOut + pid->dOut;


    if (fabs(pid->out) > pid->outMaxABS)
    {
        pid->out = (pid->out > 0) ? pid->outMaxABS : -pid->outMaxABS;
    }

    return pid->out;
}


