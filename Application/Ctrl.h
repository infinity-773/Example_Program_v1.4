/*************************************************
Copyright (C), 2016-2023, TYUT JBD TRoMaC
File name: Ctrl.h
Author: Li_Jiang
Version: v2.0
Date: 2023_09_16
Description:  Switch and LED control state definition.
Others:      This module manages the state of 4 switch inputs and the corresponding output/control data.
Function List:
    Switch_GetCode
    Switch_GetState_Index
History:
<author>    <time>          <version>       <desc>
**************************************************/
#ifndef __Ctrl_h
#define __Ctrl_h

#include "headfiles.h"

/* =========== 端口/极性定义 ================ */

#define Switch_Polarity                     GPIO_PIN_RESET                                  // 开关有效电平：低电平有效
#define Switch_GetState_Index(Index)        ((hCtrl.Switch.Code >> (2 * (Index - 1))) & 0x01U)      // 读取第 Index 个开关的状态位

#define Switch_EN_ON                        ((Switch_EN_GPIO_Port->IDR & Switch_EN_Pin) != 0x00U)
#define Switch_EN_OFF                       (!Switch_EN_ON)


typedef enum
{
    Switch_Index_1              = 1,
    Switch_Index_2              = 2,
    Switch_Index_3              = 3,
    Switch_Index_4              = 4,
} Switch_Index_enum;


typedef struct
{
    struct
    {
        uint8_t Code;   // 按键状态编码，按 2bit/开关 的方式打包
    } Switch;

    struct
    {
        uint8_t Code;   // LED 状态编码
    } LED;
} hCtrl_TypeDef;

/* =========== 全局变量定义 ========== */
extern hCtrl_TypeDef hCtrl;

/* =========== 对外函数声明 ============== */
uint8_t Switch_GetCode(void);

#endif /* __Ctrl_h */



