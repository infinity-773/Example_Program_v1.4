#include "TOTLib.h"
#include <math.h>
#include <stddef.h>
#include "stm32f1xx_hal.h"


//归一化
//返回值：浮点0~1
float_t normalize(float_t input, float_t min, float_t max){
    float_t output;
    output = (input - min) / (max - min);
    return output;
}

//差比和
//返回值：浮点-1~1
float adc_cla(float leftnum,float rightnum,float input){
    float output;
    output = (input - leftnum) / (rightnum - leftnum);
    return output;
}

