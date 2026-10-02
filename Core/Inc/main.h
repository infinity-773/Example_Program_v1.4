/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define huart_DBG huart1
#define huart_USER huart3
#define htim_RC htim6
#define Enable_IO_Pin GPIO_PIN_13
#define Enable_IO_GPIO_Port GPIOC
#define PPM_Pin GPIO_PIN_3
#define PPM_GPIO_Port GPIOC
#define PPM_EXTI_IRQn EXTI3_IRQn
#define ENC_CH1_A_Pin GPIO_PIN_0
#define ENC_CH1_A_GPIO_Port GPIOA
#define ENC_CH1_B_Pin GPIO_PIN_1
#define ENC_CH1_B_GPIO_Port GPIOA
#define Steer_A2_Pin GPIO_PIN_2
#define Steer_A2_GPIO_Port GPIOA
#define Steer_A3_Pin GPIO_PIN_3
#define Steer_A3_GPIO_Port GPIOA
#define ENC_CH2_A_Pin GPIO_PIN_6
#define ENC_CH2_A_GPIO_Port GPIOA
#define ENC_CH2_B_Pin GPIO_PIN_7
#define ENC_CH2_B_GPIO_Port GPIOA
#define Steer_B0_Pin GPIO_PIN_0
#define Steer_B0_GPIO_Port GPIOB
#define Switch_4_Pin GPIO_PIN_7
#define Switch_4_GPIO_Port GPIOC
#define Switch_2_Pin GPIO_PIN_8
#define Switch_2_GPIO_Port GPIOC
#define Switch_3_Pin GPIO_PIN_9
#define Switch_3_GPIO_Port GPIOC
#define Switch_1_Pin GPIO_PIN_8
#define Switch_1_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_11
#define LED_1_GPIO_Port GPIOA
#define Switch_EN_Pin GPIO_PIN_12
#define Switch_EN_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_15
#define LED_2_GPIO_Port GPIOA
#define LED_3_Pin GPIO_PIN_12
#define LED_3_GPIO_Port GPIOC
#define LED_4_Pin GPIO_PIN_3
#define LED_4_GPIO_Port GPIOB
#define Wheel_Right_PWM_Pin GPIO_PIN_6
#define Wheel_Right_PWM_GPIO_Port GPIOB
#define Wheel_Left_PWM_Pin GPIO_PIN_7
#define Wheel_Left_PWM_GPIO_Port GPIOB
#define Wheel_Left_IO_Pin GPIO_PIN_8
#define Wheel_Left_IO_GPIO_Port GPIOB
#define Wheel_Right_IO_Pin GPIO_PIN_9
#define Wheel_Right_IO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
