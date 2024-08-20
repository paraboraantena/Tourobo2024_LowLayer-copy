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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Enc2A_Pin GPIO_PIN_0
#define Enc2A_GPIO_Port GPIOC
#define Enc2B_Pin GPIO_PIN_1
#define Enc2B_GPIO_Port GPIOC
#define Enc3A_Pin GPIO_PIN_2
#define Enc3A_GPIO_Port GPIOC
#define Enc3B_Pin GPIO_PIN_3
#define Enc3B_GPIO_Port GPIOC
#define Enc4A_Pin GPIO_PIN_6
#define Enc4A_GPIO_Port GPIOA
#define Enc4B_Pin GPIO_PIN_7
#define Enc4B_GPIO_Port GPIOA
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
#define Enc1A_Pin GPIO_PIN_4
#define Enc1A_GPIO_Port GPIOB
#define Enc1B_Pin GPIO_PIN_5
#define Enc1B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
