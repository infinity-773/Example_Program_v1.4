#include "TOTLib.h"
#include <math.h>
#include <stddef.h>
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
float TOT_cla(float leftnum,float rightnum){ //TODO :参数改为正数
    if ((leftnum + rightnum) == 0.0f)
    {
        return 0.0f; // 避免除以零的情况
    }
    float output = (leftnum - rightnum) / (rightnum + leftnum);
    return output;
}

uint16_t TOT_AverageFilt(TOT_MovingAverage5_t *filter, uint16_t rawValue)
{
    if (filter == NULL)
    {
        return rawValue;
    }

    if (filter->sample_count < 5U)
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

    filter->next_index = (uint8_t)((filter->next_index + 1U) % 5U);

    return (uint16_t)(filter->sum / filter->sample_count);
}

