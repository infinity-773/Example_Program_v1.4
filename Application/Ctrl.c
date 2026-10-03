/*************************************************
Copyright (C), 2016-2023, TYUT JBD TRoMaC
File name: Ctrl.c
Author: Li_Jiang
Version: v2.0
Date: 2023_09_16
Description: Switch state acquisition and encoding module.
Others: NULL
Function List:
    Switch_GetCode
History:
<author>    <time>          <version>       <desc>
Li_Jiang    2023_09_10      v1.0            实现开关状态采样与简单编码
**************************************************/
#include "Ctrl.h"

/* ============ 全局变量定义 ============= */
hCtrl_TypeDef hCtrl;

/* ============ 局部变量定义 ============= */

/* ============ 内部函数定义 ============= */

/* ============ 对外函数 ================= */

/**
  * @name   Switch_GetCode
  * @brief  读取 4 个开关输入的状态，并按位打包存入 hCtrl.Switch.Code
  * @call   Internal or External
  * @param  NULL
  * @retval 8-bit packed switch state
  */
uint8_t Switch_GetCode(void)
{
    // 保存历史状态：把上一轮的有效位左移一位，保留 bit0/bit2/bit4/bit6 结构
    hCtrl.Switch.Code = (hCtrl.Switch.Code & 0x55U) << 1;

    // 读取当前开关状态，并写入每个开关对应的 2-bit 区域
    // Switch_1
    hCtrl.Switch.Code |= ((Switch_1_GPIO_Port->IDR & Switch_1_Pin) == Switch_Polarity ? 0x01U : 0x00U) << 2 * 0;
    // Switch_2
    hCtrl.Switch.Code |= ((Switch_2_GPIO_Port->IDR & Switch_2_Pin) == Switch_Polarity ? 0x01U : 0x00U) << 2 * 1;
    // Switch_3
    hCtrl.Switch.Code |= ((Switch_3_GPIO_Port->IDR & Switch_3_Pin) == Switch_Polarity ? 0x01U : 0x00U) << 2 * 2;
    // Switch_4
    hCtrl.Switch.Code |= ((Switch_4_GPIO_Port->IDR & Switch_4_Pin) == Switch_Polarity ? 0x01U : 0x00U) << 2 * 3;

    return hCtrl.Switch.Code & 0x55U;
} /* Switch_GetCode() */


