/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  *....................................................................................................*
  *....................................................................................................*
  *....................................................................................................*
  *.............................................tttttttttttttttttttttt.................................*
  *............................................ttttttttttttttttttttttt.................................*
  *............................................tttttttttttttttttttttt..................................*
  *...........................................tttttttttttttttttttttt...................................*
  *..........................................ttttttttttttttttttttttt...................................*
  *..........................................tttttttttttttttttttttt....................................*
  *..........................................ttttttttttttttttttttt.....................................*
  *....................................................................................................*
  *....................ttttttttttttt.......tttttttttttttttttttttt...tttttttttt.........................*
  *...................tttttttttttt.....ttttttttttttttttttttttttt....tttttttttt.........................*
  *...................tttttttt......tttttttttttttttttttttttttttt...tttttttttt..........................*
  *..................ttttt......ttttttttttttttttttttttttttttttt...tttttttttt...........................*
  *.................ttt......ttttttttttttttttttttttttttttttttt....tttttttttt...........................*
  *................tt....ttttttttttttttttttttttttttttttttttttt...tttttttttt............................*
  *...................ttttttttttttttttttttttttttttttttttttttt....ttttttttt.............................*
  *................................ttttttttttttttttttttttttt...........................................*
  *...............................tttttttttttttttttttttttttt...........................................*
  *..............................tttttttttttttttttttttttttt............................................*
  *..............................ttttttttttttttttttttttttt.............................................*
  *.............................tttttttttttttttttttttttttt.............................................*
  *............................ttttttttttttttttttttttttttt........ttt..................................*
  *............................ttttttttttttttttttttttttttttttttttttt...................................*
  *...........................ttttttttttttttttttttttttttttttttttttt....................................*
  *...........................tttttttttttttttttttttttttttttttttttt.....................................*
  *...........................tttttttttttttttttttttttttttttttttttt.....................................*
  *...........................ttttttttttttttttttttttttttttttttttt......................................*
  *............................ttttttttttttttttttttttttttttttttt.......................................*
  *.............................ttttttttttttttttttttttttttttttt........................................*
  *..............................tttttttttttttttttttttttttttt..........................................*
  *.................................ttttttttttttttttttttt..............................................*
  *........................................tttttt......................................................*
  *....................................................................................................*
  *....................................................................................................*
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *                      
  */                                                                                                             
  
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "pid.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "headfiles.h"
#include "TOTLib.h"
#include <math.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Steer_PWM_Center        747
#define Steer_PWM_Limit_Lift    (Steer_PWM_Center - KR)//616
#define Steer_PWM_Limit_Right   (Steer_PWM_Center + KR)//872


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t Background_LastTick;
static TOT_MovingAverage3_t AD_Left_Filter;
static TOT_MovingAverage3_t AD_Right_Filter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void SteeringControlUpdate(void);
static void updateSwichState(int code);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*                          全局状态变量定义                                     */
volatile int16_t AD_Left, AD_Right;                         // 左右两路 ADC 采样值
static int16_t AD_Left_filted, AD_Right_filted;                    // 左右两路 ADC 滤波后值
static float AD_Left_Normalized , AD_Right_Normalized;      //左右两路归一化值
float currErr = 0;                                          //均值滤波->差比和
float rawErr = 0;                                           //原值->差比和
static float pidOut = 0;                                           // 当前误差值(-1~1)
float posErr = 0;                                           // 误差值转化为位置值(+-130)
int16_t Wheel_Left_Speed;                               // 电机目标速度
int16_t baseSpeed = 0;
int16_t addSpeed = 0;
static float kv  = 0;
uint16_t steerOutput = 747U;                                // 转向 PWM 输出值
static float kp = 0.0f; 
static float ki = 0.0f;  
static float kd = 0.0f;
float s =  0.0f;                                     //线性插值因子 && 速度因子
uint16_t swichState = 0;

/*                                     全局常量定义                                          */
static PID_HandleTypeDef PIDcontroller;                          // PID 控制器
static PID_InitTypeDef PIDInit;                  // PID 控制器初始化结构体
static PID_TypeDef PID = {
  .iOutMax = 0.0f,
  .outMaxABS = 1.0f
};
static const PID_Map PIDMap = {
    .KpMin = 0.77f,
    .KiMin = 0.0f,
    .KdMin = 0.23f,
    .KpMax = 1.53f,
    .KiMax = 0.0f,
    .KdMax = 0.5f
};
static const float_t KR = 131;          // PID 输出转化为位置值的比例系数 && 舵机最大PWM变化量
static const float deadZone = 0.03;

const uint16_t steerCenter = Steer_PWM_Center;

const float_t leftMin = 700.0f;                                 // 左路 ADC 最小值
const float_t leftMax = 4095.0f;                              // 左路 ADC 最大值 //TODO
const float_t rightMin = 700.0f;                                // 右路 ADC 最小值
const float_t rightMax = 4095.0f;                             // 右路 ADC 最大值  //TODO



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  PWM_Init();   // PWM 初始化
  RC_Init();    // SBUS 遥控接收初始化

  /* ADC 初始化 */
  ADC_Init();
  Background_LastTick = HAL_GetTick();

  //PID初始化
  PIDInit = (PID_InitTypeDef){
      .mode = PID_MODE_POSITION,
      .kp = kp,
      .ki = ki,
      .kd = kd,
      .iOutMax = 0.0f,
      .outMax = 1.0f
  };
  PID_init(&PIDcontroller, &PIDInit);
  
  //PID_LoadGains(&PID, kp, ki, kd);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
      uint32_t now = HAL_GetTick();
      if ((uint32_t)(now - Background_LastTick) < 10U)
      {
        __WFI();
        continue;
      }
      Background_LastTick = now;

      // 根据使能开关状态控制电机驱动和轮速
      if (Switch_EN_ON)
      {
          Motor_DriverEnable(1);
          //Wheel_Left_Speed = 420 + 100 * kv * kv;
          Wheel_Left_Speed = baseSpeed + addSpeed * kv * kv;
      }
      else
      {
          Motor_DriverEnable(0);
          Wheel_Left_Speed = 0;
      }

      // 设置左轮速度
      Motor_SetSpeed(MOTOR_LEFT, Wheel_Left_Speed);

      // 读取开关状态并更新编码值
      swichState = Switch_GetCode();

      updateSwichState(swichState);

      // 根据开关状态更新 LED 和 PWM 呼吸灯
      PWM_SetPin(PIN_PA11, Switch_GetState_Index(Switch_Index_1) * PWM_BreathDuty());
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, Switch_GetState_Index(Switch_Index_2));
      HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, Switch_GetState_Index(Switch_Index_3));
      HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, Switch_GetState_Index(Switch_Index_4));

      // 发送实时数据到串口调试界面
      VOFA_JustFloat(&huart_DBG);
      
    /* USER CODE END WHILE */

    
  }
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void SteeringControlUpdate(void)
{
  steerOutput = steerCenter + (int16_t)posErr;
  steerOutput = Data_Limit(steerOutput, Steer_PWM_Limit_Lift, Steer_PWM_Limit_Right);
}

static void updateADC(void){
  AD_Left = ADC_ReadPin(PIN_PC5);
  AD_Right = ADC_ReadPin(PIN_PA4);

  rawErr = TOT_DiffRatio(AD_Left, AD_Right); //-1~1

  AD_Left_filted = TOT_AverageFilt(&AD_Left_Filter, AD_Left);
  AD_Right_filted = TOT_AverageFilt(&AD_Right_Filter, AD_Right);
  AD_Left_Normalized = TOT_normalize(AD_Left_filted, leftMin, leftMax);
  AD_Right_Normalized = TOT_normalize(AD_Right_filted, rightMin, rightMax);
  currErr = TOT_DiffRatio(AD_Left_Normalized, AD_Right_Normalized); //-1~1
  currErr = clampf(currErr, -1 , 1);

  applyDeadZone(&currErr , deadZone);
  applyDeadZone(&rawErr , deadZone);
}

static void updatePID(void){
  pidOut = PID_calc_err(&PIDcontroller, -currErr);
  posErr = pidOut > 0 ? pidOut *  KR  : pidOut * (KR + 1);

}

/*currErr: 0------------0.06-----------0.5--------------1*/
/*s      : 0-------------0----0~1线性---1----------------1*/
static void updatePID_TOT(void){
  s = TOT_normalize(fabs(currErr) , 0.06 , 0.5);
  s = clampf(s , 0.0f , 1.0f);
  kp = lerp(PIDMap.KpMin , PIDMap.KpMax , s);
  kd = lerp(PIDMap.KdMin , PIDMap.KdMax , s);

  PID_LoadGains(&PID,kp,ki,kd);
  pidOut = PID_Calculate(&PID, -currErr, 0.0f, -rawErr);
  posErr = pidOut > 0 ? pidOut *  KR  : pidOut * (KR + 1);

}

static void updateSpeed(void){
    // set lower speed when rotating
  kv = 1 - s;
}

static void updateStaticSteer(void){
      steerOutput = 747U;

}

static void updateSwichState(int code){ 
      if (code & 0x01U) {
          // Switch_1 当前为有效状态
        baseSpeed = 390;
        addSpeed = 170;
      }
      if (code & 0x04U) {
          // Switch_2 当前为有效状态
      }
  
      if (code & 0x10U) {
          // Switch_3 当前为有效状态
      }
  
      if (code & 0x40U) {
          // Switch_4 当前为有效状态
      }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    static uint8_t sample_divider = 0U;
    static uint8_t control_divider = 0U;

    HAL_IncTick();
    sample_divider++;
    control_divider++;

    if (sample_divider >= 5U)
    {
      sample_divider = 0U;
      updateADC();
      updatePID_TOT();
      SteeringControlUpdate();
      PWM_SetPin(PIN_PB0, steerOutput);
    }

    if (control_divider >= 10U)
    {
      control_divider = 0U;
      //updatePID();
      //updateADC();
      //SteeringControlUpdate();
      updateSpeed();
      //PWM_SetPin(PIN_PB0, steerOutput);
    }
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
