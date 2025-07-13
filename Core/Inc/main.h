/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define CS_Pin GPIO_PIN_0
#define CS_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_1
#define CE_GPIO_Port GPIOB
#define IRQ_Pin GPIO_PIN_2
#define IRQ_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_10
#define CS2_GPIO_Port GPIOB
#define CE2_Pin GPIO_PIN_12
#define CE2_GPIO_Port GPIOB
#define IRQ2_Pin GPIO_PIN_13
#define IRQ2_GPIO_Port GPIOB
#define BACK_Pin GPIO_PIN_15
#define BACK_GPIO_Port GPIOB
#define BACK_EXTI_IRQn EXTI15_10_IRQn
#define SELECT_Pin GPIO_PIN_8
#define SELECT_GPIO_Port GPIOA
#define SELECT_EXTI_IRQn EXTI9_5_IRQn
#define DOWN_Pin GPIO_PIN_9
#define DOWN_GPIO_Port GPIOA
#define DOWN_EXTI_IRQn EXTI9_5_IRQn
#define UP_Pin GPIO_PIN_10
#define UP_GPIO_Port GPIOA
#define UP_EXTI_IRQn EXTI15_10_IRQn
#define HOME_Pin GPIO_PIN_11
#define HOME_GPIO_Port GPIOA
#define HOME_EXTI_IRQn EXTI15_10_IRQn
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
