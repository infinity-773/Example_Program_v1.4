/*************************************************
Copyright (C), 2016-2023, TYUT JBD TRoMaC
File name: RC.h
Author: Li_Jiang
Version: v0.1
Date: 2023-09-08
Description: Remote control receiver interface and PPM decoding state definition.
Others: This module parses the receiver PPM signal and converts it to channel values.
Function List:
    RC_Init
    RC_ReceiveCallback
History:
<author>    <time>          <version>       <desc>
**************************************************/
#ifndef __RC_h
#define __RC_h

#include "headfiles.h"

/* =========== 常量定义 ================ */
#define RC_ChannelIndexNum          8   // PPM 通道数（通常为 6~8 通道）


typedef enum
{
    RC_state_WaitingForSingnal  = 0,   // 等待有效信号
    RC_state_Receiving          ,       // 正在接收一帧信号
    RC_state_Ready              ,       // 一帧数据已准备好
} RC_state_enum;


typedef struct
{
    GPIO_TypeDef   *GPIO_Port;      // 外部中断对应 GPIO 端口
    uint16_t        GPIO_Pin;       // 外部中断对应 GPIO 引脚

    TIM_TypeDef    *TIM;            // 记录 PPM 脉冲计时的定时器

    RC_state_enum   State;          // 接收状态机状态
    uint8_t         SynchroFlag;    // 同步标志位，表示新一帧开始

    uint16_t        PPM_Duration_Min;   // PPM 脉冲最小有效持续时间
    uint16_t        PPM_Duration_Max;   // PPM 脉冲最大有效持续时间

    uint8_t         CurrentChannelIndex; // 当前处理到第几个通道
    uint16_t        ChannelData_Raw[RC_ChannelIndexNum];  // 原始脉冲宽度数据
    int16_t         ChannelData_Offset[RC_ChannelIndexNum]; // 通道偏移校正值
    int16_t         ChannelData_Remap[RC_ChannelIndexNum];  // 归一化后的通道值

} hRC_TypeDef;

/* =========== 全局变量声明 ========== */
extern hRC_TypeDef hRC;

/* =========== 对外函数声明 ============== */
void RC_Init(void);
void RC_ReceiveCallback(void);

#endif /* __RC_h */

