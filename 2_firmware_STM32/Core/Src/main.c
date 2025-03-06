/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pca9555.h"
#include "pca9555.c"
#include <stdbool.h>

#define number_of_exp 8
#define number_of_input_exp 5

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PCA9555_HandleTypeDef exp[number_of_exp];

//Data multidimensional arrays
//[A,B,C,D,E,ENC1,ENC2,Key,Res][Inputs,Outputs][Bits]
bool data_ABC[3][2][16];
bool data_DEandMisc[6][2][8];

uint16_t raw_exp_inputdata[number_of_input_exp];
uint8_t input_expander[number_of_input_exp] = {1,2,3,5,6};

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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef HAL_status = 0, returnvalue = 0;
  init_expanders();
  init_dataregs();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3200;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DebugLED_GPIO_Port, DebugLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DebugLED_Pin */
  GPIO_InitStruct.Pin = DebugLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DebugLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Exp_Int4_Pin Exp_Int5_Pin Exp_Int6_Pin Exp_Int7_Pin
                           Exp_Int0_Pin Exp_Int1_Pin Exp_Int2_Pin Exp_Int3_Pin */
  GPIO_InitStruct.Pin = Exp_Int5_Pin|Exp_Int6_Pin|Exp_Int7_Pin|Exp_Int8_Pin
                          |Exp_Int1_Pin|Exp_Int2_Pin|Exp_Int3_Pin|Exp_Int4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}



/* USER CODE BEGIN 4 */

//init Data Registers
init_dataregs()
{
  //raw data register
  for(uint8_t i = 0; i < number_of_input_exp; i++)
  {
    pca9555_readRegister(&exp[input_expander[i]], PCA9555_CB_INPUTS_PORTS, &raw_exp_inputdata[i]);
    raw_exp_inputdata[i] = ~(raw_exp_inputdata[i]);
  }
}

//init Expanders
void init_expanders()
{
  //pause interrupts
  __disable_irq();
  uint16_t address_expander = 0x20;
  HAL_StatusTypeDef returnvalueinit = 0;
  
  //init Expanders on i2c busses
  returnvalueinit += pca9555_init(&exp[4], &hi2c2, address_expander++);
  returnvalueinit += pca9555_init(&exp[5], &hi2c2, address_expander++);
  returnvalueinit += pca9555_init(&exp[6], &hi2c2, address_expander++);
  returnvalueinit += pca9555_init(&exp[7], &hi2c2, address_expander++);
  returnvalueinit += pca9555_init(&exp[0], &hi2c1, address_expander++);
  returnvalueinit += pca9555_init(&exp[1], &hi2c1, address_expander++);
  returnvalueinit += pca9555_init(&exp[2], &hi2c1, address_expander++);
  returnvalueinit += pca9555_init(&exp[3], &hi2c1, address_expander);

  //init Expanders used as Outputs (EXP0, EXP3, EXP4 and EXP7)
  //init EXP0
  for(uint8_t i = 0; i <= 15; i++)
  {
    returnvalueinit += pca9555_pinMode(&exp[0], i, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    returnvalueinit += pca9555_DigitalWrite(&exp[0], i, PCA9555_BIT_SET);
  }
  //init EXP3 - First Byte Outputs, Second Byte: 4 LSBits Outputs, 4 MSBits Inputs
  for(uint8_t i = 0; i <= 7; i++)
  {
    returnvalueinit += pca9555_pinMode(&exp[3], i, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    returnvalueinit += pca9555_DigitalWrite(&exp[3], i, PCA9555_BIT_SET);
  }
  for(uint8_t i = 12; i <= 15; i++)
  {
    returnvalueinit += pca9555_pinMode(&exp[3], i, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    returnvalueinit += pca9555_DigitalWrite(&exp[3], i, PCA9555_BIT_SET);
  }
  //init EXP4
  for(uint8_t i = 0; i <= 15; i++)
  {
    returnvalueinit += pca9555_pinMode(&exp[4], i, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    returnvalueinit += pca9555_DigitalWrite(&exp[4], i, PCA9555_BIT_SET);
  }
  //init EXP7
  for(uint8_t i = 0; i <= 15; i++)
  {
    returnvalueinit += pca9555_pinMode(&exp[7], i, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
    returnvalueinit += pca9555_DigitalWrite(&exp[7], i, PCA9555_BIT_SET);
  }

  /*
  //init Expanders used as Inputs (EXP1, EXP2, EXP3, EXP5 and EXP6)
  //init EXP1
  for(uint8_t i = 0; i <= 15; i++)
  {
    returnvalueinit += pca9555_pinMode(&exp[1], i, PCA9555_PIN_INPUT_MODE, PCA9555_POLARITY_INVERTED);
  }
  //init EXP2
  for(uint8_t i = 0; i <= 15; i++)
  {
    returnvalueinit += pca9555_pinMode(&exp[2], i, PCA9555_PIN_INPUT_MODE, PCA9555_POLARITY_INVERTED);
  }
  //init EXP3
  for(uint8_t i = 8; i <= 11; i++) //just 4 LSBits in second Byte are Inputs
  {
    returnvalueinit += pca9555_pinMode(&exp[3], i, PCA9555_PIN_INPUT_MODE, PCA9555_POLARITY_INVERTED);
  }
  //init EXP5
  for(uint8_t i = 0; i <= 15; i++)
  {
    returnvalueinit += pca9555_pinMode(&exp[5], i, PCA9555_PIN_INPUT_MODE, PCA9555_POLARITY_INVERTED);
  }
  //init EXP6
  for(uint8_t i = 0; i <= 15; i++)
  {
    returnvalueinit += pca9555_pinMode(&exp[6], i, PCA9555_PIN_INPUT_MODE, PCA9555_POLARITY_INVERTED);
  } */



  //reenable interrupts
  __enable_irq();

  //Error handling
  /*if (returnvalueinit != HAL_OK)
  {
    Error_Handler();
  }*/
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  HAL_StatusTypeDef HALreturn = 0;
  //find out which expander got input change
  uint8_t exp_interrupt = 10;
  uint8_t exp_data = 0;

  switch (GPIO_Pin)
  {
    case GPIO_PIN_4: //exp_int1 - not used
      exp_interrupt = 0;
      exp_data = 0;
      break;
    case GPIO_PIN_5: //exp_int2
      exp_interrupt = 1;
      exp_data = 0;
      break;
    case GPIO_PIN_8: //exp_int3
      exp_interrupt = 2;
      exp_data = 1;
      break;
    case GPIO_PIN_9: //exp_int4
      exp_interrupt = 3;
      exp_data = 2;
      break;
    case GPIO_PIN_12: //exp_int5 - not used
      exp_interrupt = 4;
      exp_data = 0;
      break;
    case GPIO_PIN_13: //exp_int6
      exp_interrupt = 5;
      exp_data = 3;
      break;
    case GPIO_PIN_14: //exp_int7
      exp_interrupt = 6;
      exp_data = 4;
      break;
    case GPIO_PIN_15: //exp_int8 - not used
      exp_interrupt = 7;
      exp_data = 0;
      break;
    default:
      return HAL_ERROR;
      break;
  }

  //read corresponding expander inputs
  HALreturn = pca9555_readRegister(&exp[exp_interrupt], PCA9555_CB_INPUTS_PORTS, &raw_exp_inputdata[exp_data]);
  raw_exp_inputdata[exp_data] = ~(raw_exp_inputdata[exp_data]);

  //Error handling
  if (HALreturn != HAL_OK)
  {
    Error_Handler();
  }
}



/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
