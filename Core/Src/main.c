/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile _Bool Up;
volatile _Bool Down;
volatile _Bool Left;
volatile _Bool Right;
volatile _Bool Middle;
uint32_t ms;
uint8_t vel;
_Bool ledMatrix[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
};
volatile _Bool ms100;
volatile _Bool varTim;
volatile uint32_t mod;
_Bool recd;
uint8_t recduart[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void Configuration();
void displayHome();
void resetLED();
void Maze();
void Tennis();
void Tennis2();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	_Bool config = 1;
	ms = 0;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	if (config) {
		HAL_Delay(100);
		Configuration();
	}

	Up = 0;
	Down = 0;
	Left = 0;
	Right = 0;
	Middle = 0;
	resetLED();
	displayHome();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (Left) {
	  			Up = 0;
	  			Down = 0;
	  			Left = 0;
	  			Right = 0;
	  			Middle = 0;
	  			resetLED();
	  			Maze();
	  			Up = 0;
	  			Down = 0;
	  			Left = 0;
	  			Right = 0;
	  			Middle = 0;
	  			resetLED();
	  			displayHome();
	  		}
	  		else if (Middle) {
	  			Up = 0;
	  			Down = 0;
	  			Left = 0;
	  			Right = 0;
	  			Middle = 0;
	  			resetLED();
	  			Tennis();
	  			Up = 0;
	  			Down = 0;
	  			Left = 0;
	  			Right = 0;
	  			Middle = 0;
	  			resetLED();
	  			displayHome();
	  		}
	  		else if (Right) {
	  			Up = 0;
	  			Down = 0;
	  			Left = 0;
	  			Right = 0;
	  			Middle = 0;
	  			resetLED();
	  			Tennis2();
	  			Up = 0;
	  			Down = 0;
	  			Left = 0;
	  			Right = 0;
	  			Middle = 0;
	  			resetLED();
	  			displayHome();
	  		}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin PC1 PC2 */
  GPIO_InitStruct.Pin = B1_Pin|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 PA7 PA8 
                           PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB11 
                           PB12 PB13 PB14 PB15 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void Configuration()
{
	uint32_t now = 0;
	uint8_t i = 0;
	uint8_t posStr[] = "$1_______\n";

	HAL_UART_Transmit(&huart2,(uint8_t*) "$21573751\n", 10, 100);
	posStr[2] = '0';
	HAL_UART_Transmit(&huart2, posStr, 10, 100);
	now = ms;
	for (i = 0; i<8; i++){
		ledMatrix[8*i] = 1;
	}
	while((ms-now)<1000){
	}
	for (i = 0; i<8; i++){
		ledMatrix[8*i] = 0;
	}

	posStr[2] = '1';
	HAL_UART_Transmit(&huart2, posStr, 10, 100);
	now = ms;
	for (i = 0; i<8; i++){
		ledMatrix[8*i+1] = 1;
	}
	while((ms-now)<1000){
	}
	for (i = 0; i<8; i++){
		ledMatrix[8*i+1] = 0;
	}

	posStr[2] = '2';
	HAL_UART_Transmit(&huart2,posStr, 10, 100);
	now = ms;
	for (i = 0; i<8; i++){
		ledMatrix[8*i+2] = 1;
	}
	while((ms-now)<1000){
	}
	for (i = 0; i<8; i++){
		ledMatrix[8*i+2] = 0;
	}

	posStr[2] = '3';
	HAL_UART_Transmit(&huart2,posStr, 10, 100);
	now = ms;
	for (i = 0; i<8; i++){
		ledMatrix[8*i+3] = 1;
	}
	while((ms-now)<1000){
	}
	for (i = 0; i<8; i++){
		ledMatrix[8*i+3] = 0;
	}

	posStr[2] = '4';
	HAL_UART_Transmit(&huart2, posStr, 10, 100);
	now = ms;
	for (i = 0; i<8; i++){
		ledMatrix[8*i+4] = 1;
	}
	while((ms-now)<1000){
	}
	for (i = 0; i<8; i++){
		ledMatrix[8*i+4] = 0;
	}

	posStr[2] = '5';
	HAL_UART_Transmit(&huart2, posStr, 10, 100);
	now = ms;
	for (i = 0; i<8; i++){
		ledMatrix[8*i+5] = 1;
	}
	while((ms-now)<1000){
	}
	for (i = 0; i<8; i++){
		ledMatrix[8*i+5] = 0;
	}

	posStr[2] = '6';
	HAL_UART_Transmit(&huart2, posStr, 10, 100);
	now = ms;
	for (i = 0; i<8; i++){
		ledMatrix[8*i+6] = 1;
	}
	while((ms-now)<1000){
	}
	for (i = 0; i<8; i++){
		ledMatrix[8*i+6] = 0;
	}

	posStr[2] = '7';
	HAL_UART_Transmit(&huart2, posStr, 10, 100);
	now = ms;
	for (i = 0; i<8; i++){
		ledMatrix[8*i+7] = 1;
	}
	while((ms-now)<1000){
	}
	for (i = 0; i<8; i++){
		ledMatrix[8*i+7] = 0;
	}

	return;
}

void displayHome()
{
	ledMatrix[0] = 1;
	ledMatrix[7] = 1;
	ledMatrix[56] = 1;
	ledMatrix[63] = 1;

	return;
}

void resetLED(){
	uint8_t i = 0;
	uint8_t j = 0;

	for (i = 0; i<8; i++){
		for (j = 0; j<8; j++){
			ledMatrix[i+8*j] = 0;
		}
	}
}


void Maze()
{
	uint8_t xpos = 0;
	uint8_t ypos = 0;
	uint8_t endPosX = 7;
	uint8_t endPosY = 7;
	uint8_t posStr[] = "$300_____\n";
	_Bool gameEnd = 0;
	uint8_t maze1[] = {
			0, 0, 0, 0, 0, 0, 1, 0,
			1, 1, 0, 1, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0,
			0, 1, 1, 1, 1, 1, 0, 1,
			0, 0, 0, 0, 1, 0, 0, 0,
			1, 1, 0, 1, 1, 0, 1, 0,
			0, 0, 0, 1, 0, 0, 1, 0,
			0, 1, 0, 0, 0, 1, 1, 0,
	};
	uint8_t maze2[] = {
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 1, 0, 1, 1, 0,
			0, 0, 1, 0, 1, 0, 0, 0,
			0, 1, 0, 0, 0, 0, 1, 1,
			0, 0, 1, 1, 1, 1, 0, 0,
			1, 0, 1, 0, 0, 1, 1, 0,
			0, 0, 1, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 1, 0, 0, 0,
	};
	uint8_t maze3[] = {
			0, 0, 0, 1, 1, 0, 0, 0,
			0, 1, 0, 0, 1, 0, 1, 0,
			0, 0, 1, 0, 1, 0, 1, 0,
			1, 0, 1, 0, 0, 0, 1, 0,
			0, 0, 1, 0, 1, 1, 1, 0,
			0, 1, 0, 0, 1, 0, 0, 0,
			0, 1, 0, 1, 1, 0, 1, 1,
			0, 0, 0, 0, 1, 0, 0, 0,
	};
	uint8_t maze4[] = {
			0, 1, 0, 1, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 1, 1, 0,
			0, 1, 1, 1, 1, 0, 0, 0,
			0, 1, 0, 0, 1, 0, 1, 0,
			0, 1, 0, 1, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1, 0, 0,
			1, 1, 1, 1, 1, 1, 0, 1,
			0, 0, 0, 0, 0, 0, 0, 0,
	};
	uint8_t one[] = {
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
	};
	uint8_t two[] = {
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 1, 0, 0, 0,
			0, 0, 1, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 1, 1, 1, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
	};
	uint8_t three[] = {
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 1, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 1, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
	};
	uint8_t four[] = {
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 1, 0, 1, 0, 0,
			0, 0, 1, 1, 1, 1, 1, 0,
			0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
	};
	uint8_t i = 0;
	uint8_t j = 0;
	_Bool ledOn = 0;
	uint8_t mazeNumber = 1;
	_Bool changeMaze = 0;
	uint8_t i2c[2];
	int16_t imuX;
	int16_t imuY;

	Up = 0;
	Down = 0;
	Middle = 0;

	for (i = 0; i<8; i++){
		for (j = 0; j<8; j++){
			ledMatrix[8*i+j] = one[8*i+j];
		}
		HAL_GPIO_WritePin(LED1A, LED1, GPIO_PIN_SET);
	}
	while(!Middle){
		if(Up) {
			mazeNumber++;
			if (mazeNumber>4){
				mazeNumber = 4;
			}
			changeMaze = 1;
			Up = 0;
		}
		if(Down){
			mazeNumber--;
			if (mazeNumber < 1){
				mazeNumber = 1;
			}
			changeMaze = 1;
			Down = 0;
		}
		if (changeMaze) {
			HAL_GPIO_WritePin(LED1A, LED1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2A, LED2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3A, LED3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED4A, LED4, GPIO_PIN_RESET);
			switch (mazeNumber){
			case 1:
				for (i = 0; i<8; i++){
					for (j = 0; j<8; j++){
						ledMatrix[8*i+j] = one[8*i+j];
					}
				}
				HAL_GPIO_WritePin(LED1A, LED1, GPIO_PIN_SET);
				break;
			case 2:
				for (i = 0; i<8; i++){
					for (j = 0; j<8; j++){
						ledMatrix[8*i+j] = two[8*i+j];
					}
				}
				HAL_GPIO_WritePin(LED2A, LED2, GPIO_PIN_SET);
				break;
			case 3:
				for (i = 0; i<8; i++){
					for (j = 0; j<8; j++){
						ledMatrix[8*i+j] = three[8*i+j];
					}
				}
				HAL_GPIO_WritePin(LED3A, LED3, GPIO_PIN_SET);
				break;
			case 4:
				for (i = 0; i<8; i++){
					for (j = 0; j<8; j++){
						ledMatrix[8*i+j] = four[8*i+j];
					}
				}
				HAL_GPIO_WritePin(LED4A, LED4, GPIO_PIN_SET);
				break;
			}
			changeMaze = 0;
		}
	}

	Middle = 0;
	switch (mazeNumber){
	case 1:
		for (i = 0; i<8; i++){
			for (j = 0; j<8; j++){
				ledMatrix[8*i+j] = maze1[8*i+j];
			}
		}
		endPosX = 7;
		endPosY = 7;
		break;
	case 2:
		for (i = 0; i<8; i++){
			for (j = 0; j<8; j++){
				ledMatrix[8*i+j] = maze2[8*i+j];
			}
		}
		endPosX = 3;
		endPosY = 3;
		break;
	case 3:
		for (i = 0; i<8; i++){
			for (j = 0; j<8; j++){
				ledMatrix[8*i+j] = maze3[8*i+j];
			}
		}
		endPosX = 7;
		endPosY = 7;
		break;
	case 4:
		for (i = 0; i<8; i++){
			for (j = 0; j<8; j++){
				ledMatrix[8*i+j] = maze4[8*i+j];
			}
		}
		endPosX = 2;
		endPosY = 0;
		break;
	}
	vel = 9;
	mod = 0;
	ms100 = 0;
	varTim = 0;

	while(!varTim){
	}
	while ((!Middle)&&(!gameEnd))
	{
		if(ms100)
		{
			ms100 = 0;
			ledMatrix[endPosX + 8*endPosY] = 1 - ledMatrix[endPosX + 8*endPosY];
		}
		if(varTim)
		{
			ledOn = 1 - ledOn;
			varTim = 0;
			if(Left && xpos>0 && ((ledMatrix[xpos-1 + 8*ypos] == 0)||((xpos == endPosX+1) && (ypos == endPosY))))
			{
				ledMatrix[xpos+8*ypos] = 0;
				xpos -= 1;
			}
			else if(Right && xpos<7  && ((ledMatrix[xpos+1 + 8*ypos] == 0)||((xpos == endPosX-1) && (ypos == endPosY))))
			{
				ledMatrix[xpos+8*ypos] = 0;
				xpos += 1;
			}
			Left = 0;
			Right = 0;
			if(Up && ypos>0 && ((ledMatrix[xpos + 8*(ypos-1)] == 0)||((xpos == endPosX) && (ypos == endPosY+1))))
			{
				ledMatrix[xpos+8*ypos] = 0;
				ypos -= 1;
			}
			else if(Down && ypos<7  && ((ledMatrix[xpos + 8*(ypos+1)] == 0)||((xpos == endPosX) && (ypos == endPosY-1))))
			{
				ledMatrix[xpos+8*ypos] = 0;
				ypos += 1;
			}
			Up = 0;
			Down = 0;

			posStr[6] = 'N';
			i2c[0] = 0x37;
			HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x20, 1, (uint8_t*) i2c, 1, 100);
			i2c[0] = 0x08;
			HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x23, 1, (uint8_t*) i2c, 1, 100);
			HAL_I2C_Mem_Read(&hi2c1, 0x33, 0xA8, 1, i2c, 2, 100);
			imuX = (i2c[1]<<4)+(i2c[0]>>4);
			if ((imuX>>11) == 1){
				imuX = (0xF<<12) + imuX;
			}
			if (imuX>512) {
				if (xpos>0 && ((ledMatrix[xpos-1 + 8*ypos] == 0)||((xpos == endPosX+1) && (ypos == endPosY)))) {
					ledMatrix[xpos+8*ypos] = 0;
					xpos--;
				}
				posStr[6] = 'L';
			}
			else if (imuX<-512) {
				if (xpos<7  && ((ledMatrix[xpos+1 + 8*ypos] == 0)||((xpos == endPosX-1) && (ypos == endPosY)))) {
					ledMatrix[xpos+8*ypos] = 0;
					xpos++;
				}
				posStr[6] = 'R';
			}
			HAL_I2C_Mem_Read(&hi2c1, 0x33, 0xAA, 1, i2c, 2, 100);
			imuY = (i2c[1]<<4)+(i2c[0]>>4);
			if ((imuY>>11) == 1){
				imuY = (0xF<<12) + imuY;
			}
			if (imuY>512) {
				if (ypos<7  && ((ledMatrix[xpos + 8*(ypos+1)] == 0)||((xpos == endPosX) && (ypos == endPosY-1)))) {
					ledMatrix[xpos+8*ypos] = 0;
					ypos++;
				}
				posStr[6] = 'D';
			}
			else if (imuY<-512) {
				if (ypos>0 && ((ledMatrix[xpos + 8*(ypos-1)] == 0)||((xpos == endPosX) && (ypos == endPosY+1)))) {
					ledMatrix[xpos+8*ypos] = 0;
					ypos--;
				}
				posStr[6] = 'U';
			}

			if (ledOn){
				ledMatrix[xpos+8*ypos] = 1;
			}
			else {
				ledMatrix[xpos+8*ypos] = 0;
			}
			posStr[2] = xpos+48;
			posStr[3] = ypos+48;
			if (ledMatrix[xpos+8*ypos]) {
				posStr[4] = 49;
			}
			else {
				posStr[4] = 48;
			}
			if (ledMatrix[63]) {
				posStr[5] = 49;
			}
			else {
				posStr[5] = 48;
			}
			HAL_UART_Transmit(&huart2, posStr, 10, 100);
			if ((xpos == endPosX) && (ypos == endPosY))
			{
				gameEnd = 1;
			}
		}
	}
	HAL_GPIO_WritePin(LED1A, LED1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2A, LED2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3A, LED3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED4A, LED4, GPIO_PIN_RESET);
	return;
}

void Tennis()
{
	uint8_t xpos = 0;
	uint8_t ypos = 4;
	uint8_t xpospre = 0;
	uint8_t ypospre = 0;
	uint8_t posStr[] = "$200_____\n";
	uint8_t ballX = 7;
	uint8_t ballY = 4;
	uint8_t ballXpre = 0;
	uint8_t ballYpre = 0;
	uint8_t dir = 0;
	uint8_t hitCtr = 0;
	_Bool gameEnd = 0;
	uint8_t i2c[2];
	int16_t imuX;
	int16_t imuY;

	vel = 1;
	mod = 0;
	varTim = 0;
	while(!varTim){
	}
	varTim = 0;
	ms100 = 0;
	while(!ms100){
	}
	ms100 = 0;

	posStr[8] = 'N';
	i2c[0] = 0x37;
	HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x20, 1, (uint8_t*) i2c, 1, 100);
	i2c[0] = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x23, 1, (uint8_t*) i2c, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x33, 0xA8, 1, i2c, 2, 100);
	imuX = (i2c[1]<<4)+(i2c[0]>>4);
	if ((imuX>>11) == 1){
		imuX = (0xF<<12) + imuX;
	}
	if (imuX>512) {
		if (xpos>0) {
			xpos--;
		}
		posStr[8] = 'L';
	}
	else if (imuX<-512) {
		if (xpos<7) {
			xpos++;
		}
		posStr[8] = 'R';
	}
	HAL_I2C_Mem_Read(&hi2c1, 0x33, 0xAA, 1, i2c, 2, 100);
	imuY = (i2c[1]<<4)+(i2c[0]>>4);
	if ((imuY>>11) == 1){
		imuY = (0xF<<12) + imuY;
	}
	if (imuY>512) {
		if (ypos<6) {
			ypos++;
		}
		posStr[8] = 'D';
	}
	else if (imuY<-512) {
		if (ypos>0) {
			ypos--;
		}
		posStr[8] = 'U';
	}
	ledMatrix[xpos+8*ypos] = 1;
	ledMatrix[xpos+8*(ypos+1)] = 1;
	ledMatrix[ballX + 8*ballY] = 1;
	posStr[2] = ballX+48;
	posStr[3] = ballY+48;
	posStr[4] = vel+48;
	posStr[5] = dir+48;
	posStr[6] = xpos+48;
	posStr[7] = ypos+48;
	HAL_UART_Transmit(&huart2, posStr, 10, 100);
	Up = 0;
	Down = 0;
	Left = 0;
	Right = 0;

	while((!Middle)&&(!gameEnd)){
		if(ms100){
			ms100 = 0;
			xpospre = xpos;
			ypospre = ypos;
			if(Up) {
				if (ypos>0){
					ypos--;
				}
			}
			else if(Down) {
				if (ypos<6){
					ypos++;
				}
			}
			if(Right) {
				if (xpos<7){
					xpos++;
				}
			}
			else if(Left) {
				if (xpos>0){
					xpos--;
				}
			}
			Up = 0;
			Down = 0;
			Left = 0;
			Right = 0;

			posStr[8] = 'N';
			i2c[0] = 0x37;
			HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x20, 1, (uint8_t*) i2c, 1, 100);
			i2c[0] = 0x08;
			HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x23, 1, (uint8_t*) i2c, 1, 100);
			HAL_I2C_Mem_Read(&hi2c1, 0x33, 0xA8, 1, i2c, 2, 100);
			imuX = (i2c[1]<<4)+(i2c[0]>>4);
			if ((imuX>>11) == 1){
				imuX = (0xF<<12) + imuX;
			}
			if (imuX>512) {
				if (xpos>0) {
					xpos--;
				}
				posStr[8] = 'L';
			}
			else if (imuX<-512) {
				if (xpos<7) {
					xpos++;
				}
				posStr[8] = 'R';
			}
			HAL_I2C_Mem_Read(&hi2c1, 0x33, 0xAA, 1, i2c, 2, 100);
			imuY = (i2c[1]<<4)+(i2c[0]>>4);
			if ((imuY>>11) == 1){
				imuY = (0xF<<12) + imuY;
			}
			if (imuY>512) {
				if (ypos<6) {
					ypos++;
				}
				posStr[8] = 'D';
			}
			else if (imuY<-512) {
				if (ypos>0) {
					ypos--;
				}
				posStr[8] = 'U';
			}

			ledMatrix[xpospre+8*ypospre] = 0;
			ledMatrix[xpospre+8*(ypospre+1)] = 0;
			ledMatrix[xpos+8*ypos] = 1;
			ledMatrix[xpos+8*(ypos+1)] = 1;
			ledMatrix[ballX + 8*ballY] = 1;
			posStr[2] = ballX+48;
			posStr[3] = ballY+48;
			posStr[4] = vel+48;
			posStr[5] = dir+48;
			posStr[6] = xpos+48;
			posStr[7] = ypos+48;
			HAL_UART_Transmit(&huart2, posStr, 10, 100);
		}
		if (varTim)
		{
			varTim = 0;
			ballXpre = ballX;
			ballYpre = ballY;
			switch(dir) {
			case 0:
				if (ledMatrix[ballX-1 + 8*ballY])
				{
					if(ballY == ypos)
					{
						ballX++;
						ballY--;
						dir = 4;
						hitCtr++;
					}
					else if(ballY == (ypos+1))
					{
						ballX++;
						ballY++;
						dir = 2;
						hitCtr++;
					}
				}
				else {
					ballX--;
				}
				break;
			case 1:
				if ((ballX+1) > 7)
				{
					ballX--;
					dir = 0;
				}
				else {
					ballX++;
				}
				break;
			case 2:
				if (((ballX+1) > 7) && ((ballY+1) > 7))
				{
					ballX--;
					ballY--;
					dir = 3;
				}
				else if ((ballX+1) > 7)
				{
					ballX--;
					ballY++;
					dir = 5;
				}
				else if ((ballY+1) > 7)
				{
					ballX++;
					ballY--;
					dir = 4;
				}
				else {
					ballX++;
					ballY++;
				}
				break;
			case 3:
				if ((ballY-1) < 0)
				{
					if (ledMatrix[ballX-1 + 8*(ballY+1)]){
						ballX++;
						ballY++;
						dir = 2;
						hitCtr++;
					}
					else {
						ballX--;
						ballY++;
						dir = 5;
					}
				}
				else if (ledMatrix[ballX-1 + 8*(ballY-1)])
				{
					if((ballY-1) == ypos)
					{
						ballX++;
						ballY--;
						dir = 4;
						hitCtr++;
					}
					else if((ballY-1) == (ypos+1))
					{
						ballX++;
						ballY++;
						dir = 2;
						hitCtr++;
					}
				}
				else{
					ballX--;
					ballY--;
				}
				break;
			case 4:
				if (((ballX+1) > 7) && ((ballY-1) < 0))
				{
					ballX--;
					ballY++;
					dir = 5;
				}
				else if ((ballX+1) > 7)
				{
					ballX--;
					ballY--;
					dir = 3;
				}
				else if ((ballY-1) < 0)
				{
					ballX++;
					ballY++;
					dir = 2;
				}
				else{
					ballX++;
					ballY--;
				}
				break;
			case 5:
				if ((ballY+1) > 7)
				{
					if (ledMatrix[ballX-1 + 8*(ballY-1)]){
						ballX++;
						ballY--;
						dir = 4;
						hitCtr++;
					}
					else {
						ballX--;
						ballY--;
						dir = 3;
					}
				}
				else if (ledMatrix[ballX-1 + 8*(ballY+1)])
				{
					if((ballY+1) == ypos)
					{
						ballX++;
						ballY--;
						dir = 4;
						hitCtr++;
					}
					else if((ballY+1) == (ypos+1))
					{
						ballX++;
						ballY++;
						dir = 2;
						hitCtr++;
					}
				}
				else {
					ballX--;
					ballY++;
				}
				break;
			}
			ledMatrix[ballXpre + 8*ballYpre] = 0;
			ledMatrix[ballX + 8*ballY] = 1;

			if (ballX == 0)
			{
				while (!ms100){
				}
				posStr[2] = ballX+48;
				posStr[3] = ballY+48;
				posStr[4] = vel+48;
				posStr[5] = dir+48;
				posStr[6] = xpos+48;
				posStr[7] = ypos+48;
				HAL_UART_Transmit(&huart2, posStr, 10, 100);
				while (!varTim){
				}
				varTim = 0;
				gameEnd = 1;
			}
			else if((hitCtr==3)&&(vel<10))
			{
				vel++;
				mod = ms%(750-vel*50);
				hitCtr=0;
			}
		}
	}
	return;
}

void Tennis2()
{
	uint8_t xpos = 0;
	uint8_t ypos = 4;
	uint8_t xpospre = 0;
	uint8_t ypospre = 0;
	uint8_t posStr[] = "$2_______\n";
	uint8_t posStrEnd[] = "$2_______\n";
	uint8_t ballX = 7;
	uint8_t ballY = 4;
	uint8_t ballXpre = 0;
	uint8_t ballYpre = 0;
	uint8_t dir = 0;
	uint8_t hitCtr = 0;
	_Bool gameEnd = 0;
	_Bool gameEndO = 0;
	uint8_t i2c[2];
	int16_t imuX;
	int16_t imuY;
	_Bool rec = 0;

	HAL_UART_Transmit(&huart3, posStrEnd, 10, 100);
	vel = 1;
	mod = 0;
	varTim = 0;
	while(!varTim){
	}
	varTim = 0;
	ms100 = 0;
	while(!ms100){
	}
	ms100 = 0;

	posStr[8] = 'N';
	i2c[0] = 0x37;
	HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x20, 1, (uint8_t*) i2c, 1, 100);
	i2c[0] = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x23, 1, (uint8_t*) i2c, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x33, 0xA8, 1, i2c, 2, 100);
	imuX = (i2c[1]<<4)+(i2c[0]>>4);
	if ((imuX>>11) == 1){
		imuX = (0xF<<12) + imuX;
	}
	if (imuX>512) {
		if (xpos>0) {
			xpos--;
		}
		posStr[8] = 'L';
	}
	else if (imuX<-512) {
		if (xpos<7) {
			xpos++;
		}
		posStr[8] = 'R';
	}
	HAL_I2C_Mem_Read(&hi2c1, 0x33, 0xAA, 1, i2c, 2, 100);
	imuY = (i2c[1]<<4)+(i2c[0]>>4);
	if ((imuY>>11) == 1){
		imuY = (0xF<<12) + imuY;
	}
	if (imuY>512) {
		if (ypos<6) {
			ypos++;
		}
		posStr[8] = 'D';
	}
	else if (imuY<-512) {
		if (ypos>0) {
			ypos--;
		}
		posStr[8] = 'U';
	}
	ledMatrix[xpos+8*ypos] = 1;
	ledMatrix[xpos+8*(ypos+1)] = 1;
	ledMatrix[ballX + 8*ballY] = 1;
	posStr[2] = ballX+48;
	posStr[3] = ballY+48;
	posStr[4] = vel+48;
	posStr[5] = dir+48;
	posStr[6] = xpos+48;
	posStr[7] = ypos+48;
	HAL_UART_Transmit(&huart2, posStr, 10, 100);
	Up = 0;
	Down = 0;
	Left = 0;
	Right = 0;
	HAL_UART_Receive_IT(&huart3, posStr, 10);
	recd = 0;

	while((!Middle)&&(!gameEnd)){
		if(ms100){
			ms100 = 0;
			xpospre = xpos;
			ypospre = ypos;
			if(Up) {
				if (ypos>0){
					ypos--;
				}
			}
			else if(Down) {
				if (ypos<6){
					ypos++;
				}
			}
			if(Right) {
				if (xpos<7){
					xpos++;
				}
			}
			else if(Left) {
				if (xpos>0){
					xpos--;
				}
			}
			Up = 0;
			Down = 0;
			Left = 0;
			Right = 0;

			posStr[8] = 'N';
			i2c[0] = 0x37;
			HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x20, 1, (uint8_t*) i2c, 1, 100);
			i2c[0] = 0x08;
			HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x23, 1, (uint8_t*) i2c, 1, 100);
			HAL_I2C_Mem_Read(&hi2c1, 0x33, 0xA8, 1, i2c, 2, 100);
			imuX = (i2c[1]<<4)+(i2c[0]>>4);
			if ((imuX>>11) == 1){
				imuX = (0xF<<12) + imuX;
			}
			if (imuX>512) {
				if (xpos>0) {
					xpos--;
				}
				posStr[8] = 'L';
			}
			else if (imuX<-512) {
				if (xpos<7) {
					xpos++;
				}
				posStr[8] = 'R';
			}
			HAL_I2C_Mem_Read(&hi2c1, 0x33, 0xAA, 1, i2c, 2, 100);
			imuY = (i2c[1]<<4)+(i2c[0]>>4);
			if ((imuY>>11) == 1){
				imuY = (0xF<<12) + imuY;
			}
			if (imuY>512) {
				if (ypos<6) {
					ypos++;
				}
				posStr[8] = 'D';
			}
			else if (imuY<-512) {
				if (ypos>0) {
					ypos--;
				}
				posStr[8] = 'U';
			}
			if(!rec) {
				ledMatrix[ballX + 8*ballY] = 1;
				posStr[2] = ballX+48;
				posStr[3] = ballY+48;
			}
			else {
				posStr[2] = ':';
				posStr[3] = ')';
			}
			ledMatrix[xpospre+8*ypospre] = 0;
			ledMatrix[xpospre+8*(ypospre+1)] = 0;
			ledMatrix[xpos+8*ypos] = 1;
			ledMatrix[xpos+8*(ypos+1)] = 1;
			posStr[4] = vel+48;
			posStr[5] = dir+48;
			posStr[6] = xpos+48;
			posStr[7] = ypos+48;
			HAL_UART_Transmit(&huart2, posStr, 10, 100);
			if(recd){
				if (recduart[5] == '_'){
					gameEnd = 1;
					gameEndO = 1;
				}
				recd = 0;
			}
		}

		if (varTim && rec)
		{
			varTim = 0;
			ledMatrix[ballX + 8*ballY] = 0;
			if (recd){
				recd = 0;
				rec = 0;
				dir = recduart[5]-48;
				if (recduart[5] == '_'){
					gameEnd = 1;
					gameEndO = 1;
				}
				else {
					ballX = 7;
					if (dir == 3) {
						ballY = recduart[3] - 48 - 1;
						if (ballY < 0){
							dir = 5;
							ballY = 1;
						}
					}
					else if (dir == 5) {
						ballY = recduart[3] - 48 - 1;
						if (ballY < 7){
							dir = 3;
							ballY = 6;
						}
					}
					else {
						dir = 0;
						ballY = recduart[3] - 48;
					}
					ledMatrix[ballX + 8*ballY] = 1;
				}
			}
		}

		if (varTim && !rec)
		{
			varTim = 0;
			ballXpre = ballX;
			ballYpre = ballY;
			switch(dir) {
			case 0:
				if (ledMatrix[ballX-1 + 8*ballY])
				{
					if(ballY == ypos)
					{
						ballX++;
						ballY--;
						dir = 4;
						hitCtr++;
					}
					else if(ballY == (ypos+1))
					{
						ballX++;
						ballY++;
						dir = 2;
						hitCtr++;
					}
				}
				else {
					ballX--;
				}
				break;
			case 1:
				if ((ballX+1) > 7)
				{
					ballX--;
					dir = 0;
				}
				else {
					ballX++;
				}
				break;
			case 2:
				if (((ballX+1) > 7) && ((ballY+1) > 7))
				{
					ballX--;
					ballY--;
					dir = 3;
				}
				else if ((ballX+1) > 7)
				{
					ballX--;
					ballY++;
					dir = 5;
				}
				else if ((ballY+1) > 7)
				{
					ballX++;
					ballY--;
					dir = 4;
				}
				else {
					ballX++;
					ballY++;
				}
				break;
			case 3:
				if ((ballY-1) < 0)
				{
					if (ledMatrix[ballX-1 + 8*(ballY+1)]){
						ballX++;
						ballY++;
						dir = 2;
						hitCtr++;
					}
					else {
						ballX--;
						ballY++;
						dir = 5;
					}
				}
				else if (ledMatrix[ballX-1 + 8*(ballY-1)])
				{
					if((ballY-1) == ypos)
					{
						ballX++;
						ballY--;
						dir = 4;
						hitCtr++;
					}
					else if((ballY-1) == (ypos+1))
					{
						ballX++;
						ballY++;
						dir = 2;
						hitCtr++;
					}
				}
				else{
					ballX--;
					ballY--;
				}
				break;
			case 4:
				if (((ballX+1) > 7) && ((ballY-1) < 0))
				{
					ballX--;
					ballY++;
					dir = 5;
				}
				else if ((ballX+1) > 7)
				{
					ballX--;
					ballY--;
					dir = 3;
				}
				else if ((ballY-1) < 0)
				{
					ballX++;
					ballY++;
					dir = 2;
				}
				else{
					ballX++;
					ballY--;
				}
				break;
			case 5:
				if ((ballY+1) > 7)
				{
					if (ledMatrix[ballX-1 + 8*(ballY-1)]){
						ballX++;
						ballY--;
						dir = 4;
						hitCtr++;
					}
					else {
						ballX--;
						ballY--;
						dir = 3;
					}
				}
				else if (ledMatrix[ballX-1 + 8*(ballY+1)])
				{
					if((ballY+1) == ypos)
					{
						ballX++;
						ballY--;
						dir = 4;
						hitCtr++;
					}
					else if((ballY+1) == (ypos+1))
					{
						ballX++;
						ballY++;
						dir = 2;
						hitCtr++;
					}
				}
				else {
					ballX--;
					ballY++;
				}
				break;
			}
			ledMatrix[ballXpre + 8*ballYpre] = 0;
			ledMatrix[ballX + 8*ballY] = 1;
			if ((ballX+1) > 7) {
				rec = 1;
			}
			if (ballX == 0)
			{
				while (!ms100){
				}
				posStr[2] = ballX+48;
				posStr[3] = ballY+48;
				posStr[4] = vel+48;
				posStr[5] = dir+48;
				posStr[6] = xpos+48;
				posStr[7] = ypos+48;
				HAL_UART_Transmit(&huart2, posStr, 10, 100);
				while (!varTim){
				}
				varTim = 0;
				gameEnd = 1;
			}
		}

	}

	if (!gameEndO) {
		HAL_UART_Transmit(&huart3, posStrEnd, 10, 100);
	}

	return;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  recd = 1;
  HAL_UART_Receive_IT(&huart3, recduart, 10);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
