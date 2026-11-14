# 引脚与定时器状态表

MCU: STM32F103RCT6 (LQFP64)，SYSCLK 72 MHz，定时器时钟均为 72 MHz。
本文档由代码阅读整理，不随工程自动更新。工程已不再使用 CubeMX 重新生成，`Program.ioc` 与当前代码不一致，以代码为准。

依据：[Core/Src/tim.c](Core/Src/tim.c)、[Core/Src/gpio.c](Core/Src/gpio.c)、[Core/Src/adc.c](Core/Src/adc.c)、[Core/Src/usart.c](Core/Src/usart.c)、[Core/Src/main.c](Core/Src/main.c)、[Libraries/common/PWM.c](Libraries/common/PWM.c)、[Libraries/common/board.c](Libraries/common/board.c)。

原则：`Libraries/` 下的模板库不改，工程侧只配置实际使用的引脚。模板库 `PWM_Init` 仍会对未配置的通道调用 `HAL_TIM_PWM_Start`，这些通道没有映射到引脚，不产生输出。

## 1. 定时器

| 定时器 | 分频 / 重装 | 频率 | 通道 | 引脚 | 用途 |
|---|---|---|---|---|---|
| TIM1 | 0 / 4799 | 15 kHz | CH4 | PA11 | LED_1 呼吸灯 |
| TIM3 | 143 / 9999 | 2.5 kHz 音频节拍 (`Buzzer_Init` 改写重装值) | 无 | 无 | 蜂鸣器音频节拍 (`Buzzer.c`)，更新中断优先级 2 |
| TIM4 | 0 / 4799 | 15 kHz | CH2 | PB7 | 电机 PWM |
| TIM4 |  |  | CH4 | PB9 | 蜂鸣器 15 kHz 载波，由 TIM3 中断门控成音频 (`Buzzer.c`) |
| TIM5 | 143 / 10000 | 约 50 Hz | CH4 | PA3 | 转向舵机 |
| TIM6 | 71 / 65535 | 1 MHz 自由计数 | 无 | 无 | PPM 遥控脉宽测量 (`RC_Init` 置 CEN) |
| TIM7 | 71 / 999 | 1 kHz | 无 | 无 | HAL 时基，兼做车辆控制节拍 |
| TIM8 | 71 / 4999 | 200 Hz (5 ms) | 无 | 无 | 转向控制节拍，更新中断优先级 1 |

TIM2 仅保留基础初始化，没有通道和引脚，因为模板库 `PWM_Init` 引用 `htim2`。TIM7 不在 `tim.c` 中，它由 `Core/Src/stm32f1xx_hal_timebase_tim.c` 的 `HAL_InitTick` 配置并启动。

TIM8 更新中断 (`TIM8_UP_IRQHandler`)，每 5 ms：采样 ADC、跑 PID、更新转向 PWM。在 `main` 的 `USER CODE BEGIN 2` 中用 `HAL_TIM_Base_Start_IT` 启动。

TIM7 回调：每 10 ms 跑 `VehicleController_periodic`。

中断优先级 (数字越小越高)：EXTI3 (PPM) 0 > TIM8 (转向) 1 > TIM7 (时基、车辆控制) 15。

TIM3、TIM5 的计数单位是 2 us，转向中心值 747 约 1.49 ms。TIM4 满量程 4800，`Motor_SetSpeed` 限幅 ±4000。

## 2. 引脚

| 引脚 | 配置 | 用途 |
|---|---|---|
| PA3 | AF PP, TIM5_CH4 | 转向舵机 |
| PA4 | 模拟, ADC1_IN4 | `AD_Right` 右侧电感 |
| PA8 | 输入上拉 | Switch_1，低电平有效，选速度档并使能 LED_1 呼吸 |
| PA9 | AF PP, USART1_TX | VOFA 调试数据，115200 |
| PA10 | 输入, USART1_RX | 未见接收逻辑 |
| PA11 | AF PP, TIM1_CH4 | LED_1 呼吸灯 |
| PA12 | 输入无上拉 | Switch_EN，高电平使能电机 |
| PA13 / PA14 | SWD | 下载调试 |
| PA15 | 输出 | LED_2，跟随 Switch_2 |
| PB0 | 未使用 | 原 TIM3_CH3 舵机备用接口(判定损坏) |
| PB3 | 输出 | LED_4，跟随 Switch_4 |
| PB5 | 输出 | 电机方向，普通 GPIO (模板库通用 GPIO 表中的引脚) |
| PB7 | AF PP, TIM4_CH2 | 电机 PWM |
| PB9 | AF PP, TIM4_CH4 | 蜂鸣器 |
| PB15 | 输入上拉 | 红外传感器 (`IR_SENSOR_PIN`)，低电平触发停车 |
| PC3 | EXTI 下降沿, EXTI3 | 遥控 PPM 输入 |
| PC5 | 模拟, ADC1_IN15 | `AD_Left` 左侧电感 |
| PC6 | 输入下拉 | 干簧管(`MAGNET_SENSOR_PIN `)，在 `main` 的 `USER CODE BEGIN 2` 中配置|
| PC7 | 输入上拉 | Switch_4 |
| PC8 | 输入上拉 | Switch_2 |
| PC9 | 输入上拉 | Switch_3 |
| PC12 | 输出 | LED_3，跟随 Switch_3 |
| PC13 | 输出 | Enable_IO，电机驱动使能 |
| PD0 / PD1 | HSE | 外部晶振 |

红外传感器在 `main.c` 中由 `IR_SENSOR_PORT` 和 `IR_SENSOR_PIN` 配置，高电平触发。

以下引脚没有任何配置：PA0、PA1、PA2、PA5、PA6、PA7、PB1、PB2、PB4、PB6、PB8、PB10 到 PB14、PC0、PC1、PC2、PC4、PC10、PC11、PC14、PC15、PD2。

## 3. 其他外设

| 外设 | 配置 |
|---|---|
| ADC1 | 2 通道扫描 (PA4、PC5)，连续转换，239.5 周期，ADC 时钟 PCLK2/6；`ADC_Init` 校准后以 DMA1_Channel1 循环模式启动，缓冲区顺序 PA4、PC5 |
| USART1 | PA9/PA10，115200 8N1；`Usart_SendArr` 用阻塞发送；DMA1_Channel4 (USART1_TX) 已配置但未被使用 |
| EXTI3 | PC3，优先级 0，调用 `RC_ReceiveCallback` |
| TIM7 | 优先级 15，系统时基 |

USART3 已移除。

## 4. 与模板库定义不一致的地方

1. **右电机**：`board.c` 中的右电机已删除，`BoardMotor_t` 只剩 `MOTOR_MAIN` (PB7 PWM + PB5 方向)。PB6、PB8 空闲，PB9 用作蜂鸣器。
2. **ADC 枚举**：`adc.h` 里的 `ADC_Channel_enum` 仍保留 6 项以满足 `board.c`，但只有前两项 (PA4、PC5) 有数据，其余通道的 `ADC_GetValue_Channel` 返回 0。
3. **PA11 呼吸灯**：`PWM_BreathDuty` 最大 8000，而重装值只有 4799，超出部分表现为常亮。
4. **蜂鸣器频率**：TIM4 为 15 kHz，音量由占空比控制，适合有源蜂鸣器；无源蜂鸣器需要另设频率。
