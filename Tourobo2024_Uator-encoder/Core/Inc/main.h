/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC2A_Pin GPIO_PIN_0
#define ENC2A_GPIO_Port GPIOC
#define ENC2A_EXTI_IRQn EXTI0_IRQn
#define ENC2B_Pin GPIO_PIN_1
#define ENC2B_GPIO_Port GPIOC
#define ENC2B_EXTI_IRQn EXTI1_IRQn
#define ENC3A_Pin GPIO_PIN_2
#define ENC3A_GPIO_Port GPIOC
#define ENC3A_EXTI_IRQn EXTI2_IRQn
#define ENC3B_Pin GPIO_PIN_3
#define ENC3B_GPIO_Port GPIOC
#define ENC3B_EXTI_IRQn EXTI3_IRQn
#define ENC4A_Pin GPIO_PIN_6
#define ENC4A_GPIO_Port GPIOA
#define ENC4A_EXTI_IRQn EXTI9_5_IRQn
#define ENC4B_Pin GPIO_PIN_7
#define ENC4B_GPIO_Port GPIOA
#define ENC4B_EXTI_IRQn EXTI9_5_IRQn
#define SOLV6_Pin GPIO_PIN_14
#define SOLV6_GPIO_Port GPIOB
#define SOLV5_Pin GPIO_PIN_15
#define SOLV5_GPIO_Port GPIOB
#define SOLV4_Pin GPIO_PIN_6
#define SOLV4_GPIO_Port GPIOC
#define SOLV3_Pin GPIO_PIN_7
#define SOLV3_GPIO_Port GPIOC
#define SOLV2_Pin GPIO_PIN_11
#define SOLV2_GPIO_Port GPIOA
#define SOLV1_Pin GPIO_PIN_12
#define SOLV1_GPIO_Port GPIOA
#define ENC1A_Pin GPIO_PIN_4
#define ENC1A_GPIO_Port GPIOB
#define ENC1A_EXTI_IRQn EXTI4_IRQn
#define ENC1B_Pin GPIO_PIN_5
#define ENC1B_GPIO_Port GPIOB
#define ENC1B_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
