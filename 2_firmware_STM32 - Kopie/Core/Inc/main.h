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
#define Int_Exp5_Pin GPIO_PIN_12
#define Int_Exp5_GPIO_Port GPIOB
#define Int_Exp6_Pin GPIO_PIN_13
#define Int_Exp6_GPIO_Port GPIOB
#define Int_Exp7_Pin GPIO_PIN_14
#define Int_Exp7_GPIO_Port GPIOB
#define Int_Exp8_Pin GPIO_PIN_15
#define Int_Exp8_GPIO_Port GPIOB
#define Int_Exp1_Pin GPIO_PIN_4
#define Int_Exp1_GPIO_Port GPIOB
#define Int_Exp2_Pin GPIO_PIN_5
#define Int_Exp2_GPIO_Port GPIOB
#define Int_Exp3_Pin GPIO_PIN_8
#define Int_Exp3_GPIO_Port GPIOB
#define Int_Exp4_Pin GPIO_PIN_9
#define Int_Exp4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
