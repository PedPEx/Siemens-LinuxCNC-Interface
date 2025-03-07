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
#include "stm32l1xx_hal.h"

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
#define DebugLED_Pin GPIO_PIN_3
#define DebugLED_GPIO_Port GPIOA
#define Exp_Int5_Pin GPIO_PIN_12
#define Exp_Int5_GPIO_Port GPIOB
#define Exp_Int5_EXTI_IRQn EXTI15_10_IRQn
#define Exp_Int6_Pin GPIO_PIN_13
#define Exp_Int6_GPIO_Port GPIOB
#define Exp_Int6_EXTI_IRQn EXTI15_10_IRQn
#define Exp_Int7_Pin GPIO_PIN_14
#define Exp_Int7_GPIO_Port GPIOB
#define Exp_Int7_EXTI_IRQn EXTI15_10_IRQn
#define Exp_Int8_Pin GPIO_PIN_15
#define Exp_Int8_GPIO_Port GPIOB
#define Exp_Int8_EXTI_IRQn EXTI15_10_IRQn
#define Exp_Int1_Pin GPIO_PIN_4
#define Exp_Int1_GPIO_Port GPIOB
#define Exp_Int1_EXTI_IRQn EXTI4_IRQn
#define Exp_Int2_Pin GPIO_PIN_5
#define Exp_Int2_GPIO_Port GPIOB
#define Exp_Int2_EXTI_IRQn EXTI9_5_IRQn
#define Exp_Int3_Pin GPIO_PIN_8
#define Exp_Int3_GPIO_Port GPIOB
#define Exp_Int3_EXTI_IRQn EXTI9_5_IRQn
#define Exp_Int4_Pin GPIO_PIN_9
#define Exp_Int4_GPIO_Port GPIOB
#define Exp_Int4_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

void USB_CDC_RxHandler(uint8_t*, uint32_t);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
