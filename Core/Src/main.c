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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "headfiles.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Steer_PWM_Center        750
#define Steer_PWM_Limit_Lift    (Steer_PWM_Center - 60)
#define Steer_PWM_Limit_Right   (Steer_PWM_Center + 60)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int16_t     AD_Left, AD_Right;                              // ���ҵ�е�ADCֵ
float       Dir_Err;                                        // ����ƫ��
int16_t     Dir_Output;                                     // �������ֵ
uint16_t    SteerPWM = Steer_PWM_Center;                    // ���PWM���ֵ

// ��Ϊ˫���������ģ����һ������������������
int16_t     Wheel_Left_Speed;                               // ��������ٶ�����

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
  PWM_Init();   // PWM��ʼ��
  RC_Init();    // SBUSЭ����ճ�ʼ��
  
  /* ADC???? */
  ADC_Init();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // ��ȡ���ҵ��ֵ
      AD_Left    = ADC_ReadPin(PIN_PA4);
      AD_Right   = ADC_ReadPin(PIN_PC5);
	  
	  // ���㷽��ƫ��
      Dir_Err    = 0.5f * (AD_Left - AD_Right);
	  
	  // ���㷽�����ֵ
      Dir_Output = Steer_PWM_Center + Dir_Err;                // �˴�������������װ��ʽ�й�
	  
	  // !!!!!�������޷� ��ֹɾ��!!!!!
      SteerPWM   = Data_Limit(Dir_Output, Steer_PWM_Limit_Lift, Steer_PWM_Limit_Right);

	  
	  
	  // �����������
      if (Switch_EN_ON)
      {
          Motor_DriverEnable(1);
          Wheel_Left_Speed = 2800;	  
      }
      else
      {
          Motor_DriverEnable(0);
          Wheel_Left_Speed = 0;
      }

	  // ���
      PWM_SetPin(PIN_PB0, SteerPWM);
      Motor_SetSpeed(MOTOR_LEFT, Wheel_Left_Speed);

	  
	  // ��ȡ���뿪��״̬
      Switch_GetCode();

	  
	  // ������ư���LED
      PWM_SetPin(PIN_PA11, Switch_GetState_Index(Switch_Index_1) * PWM_BreathDuty());
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, Switch_GetState_Index(Switch_Index_2));
      HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, Switch_GetState_Index(Switch_Index_3));
      HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, Switch_GetState_Index(Switch_Index_4));
      
      // ���ڵ�����Ϣ����
      VOFA_JustFloat(&huart_DBG);
      
      
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    HAL_Delay(10);
  }
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
    HAL_IncTick();
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
