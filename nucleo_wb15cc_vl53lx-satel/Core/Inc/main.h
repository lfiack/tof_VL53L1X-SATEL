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
#include "stm32wbxx_hal.h"

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
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define INT_Pin GPIO_PIN_5
#define INT_GPIO_Port GPIOA
#define INT_EXTI_IRQn EXTI9_5_IRQn
#define STLINK_RX_Pin GPIO_PIN_9
#define STLINK_RX_GPIO_Port GPIOA
#define STLINK_TX_Pin GPIO_PIN_10
#define STLINK_TX_GPIO_Port GPIOA
#define JTMS_Pin GPIO_PIN_13
#define JTMS_GPIO_Port GPIOA
#define JTCK_Pin GPIO_PIN_14
#define JTCK_GPIO_Port GPIOA
#define JTDO_Pin GPIO_PIN_3
#define JTDO_GPIO_Port GPIOB
#define XSDN_Pin GPIO_PIN_4
#define XSDN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
