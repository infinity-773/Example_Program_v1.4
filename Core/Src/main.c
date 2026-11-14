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
#include "board.h"
#include "dma.h"
#include "pid.h"
#include "stm32f103xe.h"
#include "stm32f1xx_hal_gpio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "headfiles.h"
#include "TOTLib.h"
#include "VehicleController.h"
#include "Buzzer.h"
#include <math.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//转向相关定义
#define Steer_PWM_Center        747U
#define Steer_PWM_Limit_Lift    (Steer_PWM_Center - 155)//616
#define Steer_PWM_Limit_Right   (Steer_PWM_Center + 155)//872
#define LOST_THRESHOLD_LEFT     400U //TODO
#define LOST_THRESHOLD_RIGHT    400U //TODO
#define LOST_DIFF_DEADBAND      20U
#define DEADZONE                0.03
#define ADCMIN                  0.0
#define ADCMAX                  4095.0
//干簧管相关定义
#define MAGNET_SENSOR_PORT      GPIOC
#define MAGNET_SENSOR_PIN       GPIO_PIN_6
#define MAGNET_ACTIVE_LEVEL     GPIO_PIN_SET
//红外传感器相关定义
#define IR_SENSOR_PORT          GPIOB
#define IR_SENSOR_PIN           GPIO_PIN_15
#define IR_ACTIVE_LEVEL         GPIO_PIN_RESET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t now;
static uint32_t Background_LastTick;
static VehicleController Vehicle;
static TOT_MovingAverage3_t AD_Left_Filter;
static TOT_MovingAverage3_t AD_Right_Filter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void SteeringControlUpdate(void);
static int16_t calculateDriveSpeed(void);
static uint8_t MotorEnableIsOn(void);
static uint8_t MagnetSensorIsActive(void);
static uint8_t InfraredSensorIsActive(void);
static uint8_t StartConditionIsOn(void);
static uint8_t StopConditionIsOn(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*                          全局状态变量定义                                     */
volatile int16_t AD_Left, AD_Right;                         // 左右两路 ADC 采样值
volatile int16_t Wheel_Left_Speed;
static int16_t AD_Left_filted, AD_Right_filted;                    // 左右两路 ADC 滤波后值
static float AD_Left_Normalized , AD_Right_Normalized;      //左右两路归一化值
float currErr = 0;                                          //均值滤波->差比和
float rawErr = 0;                                           //原值->差比和
static float pidOut = 0;                                           // 当前误差值(-1~1)
float posErr = 0;                                           // 误差值转化为位置值(+-130)
static float kv  = 0;
uint16_t steerOutput = 747U;                                // 转向 PWM 输出值
static float kp = 0.0f; 
static float ki = 0.0f;  
static float kd = 0.0f;
float s =  0.0f;                                     //线性插值因子 && 速度因子

/*                                     全局常量定义                                          */
static PID_TypeDef PID = {
  .iOutMax = 0.0f,
  .outMaxABS = 1000.0f
};
static const PID_Map PIDMap = {
    .KpMin = 4.0f,
    .KiMin = 0.0f,
    .KdMin = 0.0f,
    .KpMax = 250.0f,
    .KiMax = 0.0f,
    .KdMax = 0.0f
};       

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
  Motor_DriverEnable(1U);
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  GPIO_InitTypeDef sensorGPIO = {0};
  sensorGPIO.Pin = MAGNET_SENSOR_PIN;
  sensorGPIO.Mode = GPIO_MODE_INPUT;
  sensorGPIO.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MAGNET_SENSOR_PORT, &sensorGPIO);
  sensorGPIO.Pin = IR_SENSOR_PIN;
  sensorGPIO.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_SENSOR_PORT, &sensorGPIO);

  PWM_Init();   // PWM 初始化
  RC_Init();    // SBUS 遥控接收初始化
  Buzzer_Init();

  /* ADC 初始化 */
  ADC_Init();
  VehicleController_Init(&Vehicle, MOTOR_MAIN, Motor_SetSpeed, MotorEnableIsOn);
  Background_LastTick = HAL_GetTick();
  HAL_TIM_Base_Start_IT(&htim8);   // 转向控制节拍 200 Hz


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
      now = HAL_GetTick();
      if ((uint32_t)(now - Background_LastTick) < 10U)
      {
        __WFI();
        continue;
      }
      Background_LastTick = now;

      Switch_GetCode();
      //这个字段只用于log
      Wheel_Left_Speed = VehicleController_GetOutputSpeed(&Vehicle);

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
  uint8_t leftLost = AD_Left < LOST_THRESHOLD_LEFT;
  uint8_t rightLost = AD_Right < LOST_THRESHOLD_RIGHT;

  if (leftLost && !rightLost)
  {
    steerOutput = Steer_PWM_Limit_Right;
    return;
  }

  if (rightLost && !leftLost)
  {
    steerOutput = Steer_PWM_Limit_Lift;
    return;
  }

  if (leftLost && rightLost)
  {
    int32_t leftDeficit = (int32_t)LOST_THRESHOLD_LEFT - AD_Left;
    int32_t rightDeficit = (int32_t)LOST_THRESHOLD_RIGHT - AD_Right;

    if (leftDeficit > rightDeficit + LOST_DIFF_DEADBAND)
    {
      steerOutput = Steer_PWM_Limit_Right;
    }
    else if (rightDeficit > leftDeficit + LOST_DIFF_DEADBAND)
    {
      steerOutput = Steer_PWM_Limit_Lift;
    }
    return;
  }

  steerOutput = Steer_PWM_Center + (int16_t)posErr;
  steerOutput = Data_Limit(steerOutput, Steer_PWM_Limit_Lift, Steer_PWM_Limit_Right);
}

static void updateADC(void){
  AD_Left = ADC_ReadPin(PIN_PC5);
  AD_Right = ADC_ReadPin(PIN_PA4);

  rawErr = TOT_DiffRatio(
    TOT_normalize(AD_Left,ADCMIN,ADCMAX), 
    TOT_normalize(AD_Right,ADCMIN, ADCMAX)); //-1~1

  AD_Left_filted = TOT_AverageFilt(&AD_Left_Filter, AD_Left);
  AD_Right_filted = TOT_AverageFilt(&AD_Right_Filter, AD_Right);
  AD_Left_Normalized = TOT_normalize(AD_Left_filted,ADCMIN , ADCMAX);
  AD_Right_Normalized = TOT_normalize(AD_Right_filted,ADCMIN , ADCMAX);
  currErr = TOT_DiffRatio(AD_Left_Normalized, AD_Right_Normalized); //-1~1
  currErr = clampf(currErr, -1 , 1);

  applyDeadZone(&currErr , DEADZONE);
  applyDeadZone(&rawErr , DEADZONE);
}


/*currErr: 0------------0.06-----------0.5--------------1*/
/*s      : 0-------------0----0~1线性---1----------------1*/
static void updatePID_TOT(void){
  s = TOT_normalize(fabs(currErr) , 0.1 , 0.6);
  s = clampf(s , 0.0f , 1.0f);
  kp = lerp(PIDMap.KpMin , PIDMap.KpMax , s);
  kd = lerp(PIDMap.KdMin , PIDMap.KdMax , s);

  PID_LoadGains(&PID,kp,ki,kd);
  pidOut = PID_Calculate(&PID, -currErr, 0.0f, -rawErr);
  posErr = pidOut;

}

static void updateSpeed(void){
    // set lower speed when rotating
  kv = 1 - s;
}


static int16_t calculateDriveSpeed(void)
{
  if (Switch_GetState_Index(Switch_Index_1))
  {
    return 440 +420 * kv * kv ;
  }

  return 360 + 460 * kv * kv ;
}

static uint8_t MotorEnableIsOn(void)
{
  return Switch_EN_ON ? 1U : 0U;
}

//从硬件读取干簧管触发状态
static uint8_t MagnetSensorIsActive(void)
{
  return HAL_GPIO_ReadPin(MAGNET_SENSOR_PORT, MAGNET_SENSOR_PIN) ==
         MAGNET_ACTIVE_LEVEL;
}

//从硬件读取红外触发状态
static uint8_t InfraredSensorIsActive(void)
{
  return HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN) ==
         IR_ACTIVE_LEVEL;
}

//开车条件，暂恒为真（使能后延时结束即起步）
static uint8_t StartConditionIsOn(void)
{
  return 1U;
}

//停车条件：干簧管或红外任一触发
static uint8_t StopConditionIsOn(void)
{
  return (MagnetSensorIsActive() || InfraredSensorIsActive()) ? 1U : 0U;
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
  if (htim->Instance == TIM8)
  {
    updateADC();
    updatePID_TOT();
    SteeringControlUpdate();
    PWM_SetPin(PIN_PA3, steerOutput);
    PWM_SetPin(PIN_PB0, steerOutput);
  }
  else if (htim->Instance == TIM7)
  {
    static uint8_t control_divider = 0U;

    HAL_IncTick();
    control_divider++;

    if (control_divider >= 5U)
    {
      control_divider = 0U;
      //updatePID();
      //updateADC();
      //SteeringControlUpdate();
      updateSpeed();
      VehicleController_periodic(&Vehicle, HAL_GetTick(), calculateDriveSpeed(),
                 StartConditionIsOn(), StopConditionIsOn());
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
