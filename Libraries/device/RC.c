/*************************************************
Copyright (C), 2016-2023, TYUT JBD TRoMaC
File name: RC.c
Author: Li_Jiang
Version: v0.1
Date: 2023-09-08
Description: PPM remote-control receiver decoding logic.
Others: The receiver sends a series of pulse widths; this module records each channel duration and converts it into normalized values.
Function List:
    RC_Init
    RC_ReceiveCallback
History:
<author>    <time>          <version>       <desc>
**************************************************/
#include "RC.h"

/* ============ 全局变量定义 ============= */
hRC_TypeDef hRC = {0};

/* ============ 局部变量定义 ============= */

/* ============ 内部函数定义 ============= */

/* ============ 对外函数 ================= */
void RC_Init(void)
{
    hRC.GPIO_Port       = PPM_GPIO_Port;        // 外部中断对应 GPIO 端口
    hRC.GPIO_Pin        = PPM_Pin;              // 外部中断对应 GPIO 引脚
    hRC.TIM             = htim_RC.Instance;     // 记录脉冲计时的定时器
    hRC.State           = RC_state_WaitingForSingnal;

    for (uint8_t i = 0; i < RC_ChannelIndexNum; i++)
    {
        hRC.ChannelData_Raw[i] = 0;
        hRC.ChannelData_Offset[i] = 0;
    }

    hRC.CurrentChannelIndex = 0;

    hRC.PPM_Duration_Max = 1600;
    hRC.PPM_Duration_Min = 1000;

    hRC.TIM->CR1 |= TIM_CR1_CEN;    // 使定时器开始计数
}

void RC_ReceiveCallback(void)
{
    uint16_t CurrentDuration = 0;    // 当前脉冲的持续时间

    // 状态切换：等待信号或上一帧已结束时，进入接收状态
    if (hRC.State == RC_state_WaitingForSingnal || hRC.State == RC_state_Ready)
    {
        hRC.State = RC_state_Receiving;
    }

    // 读取当前脉冲持续时间，并清零计数器，准备下一段测量
    CurrentDuration = hRC.TIM->CNT;
    hRC.TIM->CNT = 0;

    if (hRC.SynchroFlag == 1)
    {
        hRC.SynchroFlag = 0;
        hRC.CurrentChannelIndex = 0;
    }

    // 过滤无效时长，并在有效范围内记录通道值
    if (CurrentDuration < 4000 && hRC.CurrentChannelIndex < RC_ChannelIndexNum)
    {
        hRC.ChannelData_Raw[hRC.CurrentChannelIndex] = CurrentDuration;
        hRC.CurrentChannelIndex += 1;

        if (hRC.CurrentChannelIndex == RC_ChannelIndexNum)  // 一帧通道数据接收完成
        {
            for (uint8_t i = 0; i < RC_ChannelIndexNum; i++)
            {
                // 将脉冲宽度映射到 [-1000, 1000] 之间，便于后续控制算法使用
                hRC.ChannelData_Remap[i] = (hRC.ChannelData_Raw[i] + hRC.ChannelData_Offset[i] - hRC.PPM_Duration_Min) * 2000 /
                                           (hRC.PPM_Duration_Max - hRC.PPM_Duration_Min) - 1000;
            }

            hRC.State = RC_state_Ready;
        }
    }
    else
    {
        // 超出有效范围，说明到达同步间隔或信号错误，重新开始一帧
        hRC.CurrentChannelIndex = 0;
        hRC.SynchroFlag = 1;
    }
}

/* ==== PPM 原理示意图 ==== */
/* ===================================================================================================================== *
 *          |-t1--|-t2-----|-t3-|-t4----|-t5-------|-t6--|-Synchro Blank Time---------|-t1--|-t2-----|-t3--              *
 *           _     _        _    _       _          _     _                            _     _        _                  *
 * _________/ \___/ \______/ \__/ \_____/ \________/ \___/ \____________---___________/ \___/ \______/ \___ ---- CH_PPM  *
 *          |     |        |    |       |          |     |                            |     |        |                   *
 *          |_____|        |    |       |          |     |                            |_____|        |                   *
 * _________/     \________|____|_______|__________|_____|______________---___________/     \________|_____ ---- CH_1    *
 *                |        |    |       |          |     |                                  |        |                   *
 *                |________|    |       |          |     |                                  |________|                   *
 * _______________/        \____|_______|__________|_____|______________---_________________/        \_____ ---- CH_2    *
 *                         |    |       |          |     |                                           |                   *
 *                         |____|       |          |     |                                           |_____              *
 * ________________________/    \_______|__________|_____|______________---__________________________/      ---- CH_3    *
 *                              |       |          |     |                                                               *
 *                              |_______|          |     |                                                               *
 * _____________________________/       \__________|_____|______________---________________________________ ---- CH_4    *
 *                                      |          |     |                                                               *
 *                                      |__________|     |                                                               *
 * _____________________________________/          \_____|______________---________________________________ ---- CH_5    *
 *                                                 |     |                                                               *
 *                                                 |_____|                                                               *
 * ________________________________________________/     \______________---________________________________ ---- CH_6    *
 *                                                                                                                       *
 * ===================================================================================================================== */
