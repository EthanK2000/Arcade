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
#define C0 GPIO_PIN_8
#define C0A GPIOC
#define C1 GPIO_PIN_6
#define C1A GPIOC
#define C2 GPIO_PIN_5
#define C2A GPIOC
#define C3 GPIO_PIN_12
#define C3A GPIOA
#define C4 GPIO_PIN_11
#define C4A GPIOA
#define C5 GPIO_PIN_12
#define C5A GPIOB
#define C6 GPIO_PIN_11
#define C6A GPIOB
#define C7 GPIO_PIN_2
#define C7A GPIOB
#define R0 GPIO_PIN_1
#define R0A GPIOB
#define R1 GPIO_PIN_15
#define R1A GPIOB
#define R2 GPIO_PIN_14
#define R2A GPIOB
#define R3 GPIO_PIN_13
#define R3A GPIOB
#define R4 GPIO_PIN_4
#define R4A GPIOC
#define R5 GPIO_PIN_10
#define R5A GPIOB
#define R6 GPIO_PIN_8
#define R6A GPIOA
#define R7 GPIO_PIN_10
#define R7A GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
_Bool Up;
_Bool Down;
_Bool Left;
_Bool Right;
_Bool Middle;
uint32_t ms;
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void displayMove(_Bool* matrix);
void displayHome();
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
	uint16_t adcVal = 0;
	_Bool config = 1;
	ms = 0;
	uint32_t now = 0;
	uint8_t xpos = 4;
	uint8_t ypos = 4;
	uint8_t xpospre = 4;
	uint8_t ypospre = 4;
	_Bool gamePlay = 0;
	uint8_t posStr[] = "$300_____\n";
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
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  if (config) {
	  HAL_Delay(100);
	  now = ms;
 	  HAL_GPIO_WritePin(R0A, R0, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(R1A, R1, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(R2A, R2, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(R3A, R3, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(R4A, R4, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(R5A, R5, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(R6A, R6, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(R7A, R7, GPIO_PIN_SET);

  	  HAL_UART_Transmit(&huart2,(uint8_t*) "S21573751\n", 10, 100);

  	  HAL_UART_Transmit(&huart2,(uint8_t*) "S10______\n", 10, 100);
 	  HAL_GPIO_WritePin(C0A, C0, GPIO_PIN_SET);
  	  while((ms-now)<1000){
  	  }
  	  HAL_GPIO_WritePin(C0A, C0, GPIO_PIN_RESET);

  	  while((ms-now)<2000){
  	  }

  	  HAL_UART_Transmit(&huart2,(uint8_t*) "S11______\n", 10, 100);
  	  HAL_GPIO_WritePin(C1A, C1, GPIO_PIN_SET);
  	  while((ms-now)<3000){
  	  }
  	  HAL_GPIO_WritePin(C1A, C1, GPIO_PIN_RESET);

  	  while((ms-now)<4000){
  	  }

  	  HAL_UART_Transmit(&huart2,(uint8_t*) "S12______\n", 10, 100);
 	  HAL_GPIO_WritePin(C2A, C2, GPIO_PIN_SET);
  	  while((ms-now)<5000){
  	  }
  	  HAL_GPIO_WritePin(C2A, C2, GPIO_PIN_RESET);

  	  while((ms-now)<6000){
  	  }

  	  HAL_UART_Transmit(&huart2,(uint8_t*) "S13______\n", 10, 100);
  	  HAL_GPIO_WritePin(C3A, C3, GPIO_PIN_SET);
  	  while((ms-now)<7000){
  	  }
  	  HAL_GPIO_WritePin(C3A, C3, GPIO_PIN_RESET);

  	  while((ms-now)<8000){
  	  }

  	  HAL_UART_Transmit(&huart2,(uint8_t*) "S14______\n", 10, 100);
  	  HAL_GPIO_WritePin(C4A, C4, GPIO_PIN_SET);
  	  while((ms-now)<9000){
  	  }
  	  HAL_GPIO_WritePin(C4A, C4, GPIO_PIN_RESET);

  	  while((ms-now)<10000){
  	  }

  	  HAL_UART_Transmit(&huart2,(uint8_t*) "S15______\n", 10, 100);
  	  HAL_GPIO_WritePin(C5A, C5, GPIO_PIN_SET);
  	  while((ms-now)<11000){
  	  }
  	  HAL_GPIO_WritePin(C5A, C5, GPIO_PIN_RESET);

  	  while((ms-now)<12000){
  	  }

  	  HAL_UART_Transmit(&huart2,(uint8_t*) "S16______\n", 10, 100);
  	  HAL_GPIO_WritePin(C6A, C6, GPIO_PIN_SET);
  	  while((ms-now)<13000){
  	  }
  	  HAL_GPIO_WritePin(C6A, C6, GPIO_PIN_RESET);

  	  while((ms-now)<14000){
  	  }

  	  HAL_UART_Transmit(&huart2,(uint8_t*) "S17______\n", 10, 100);
  	  HAL_GPIO_WritePin(C7A, C7, GPIO_PIN_SET);
  	  while((ms-now)<15000){
  	  }
  	  HAL_GPIO_WritePin(C7A, C7, GPIO_PIN_RESET);

  	  HAL_GPIO_WritePin(R0A, R0, GPIO_PIN_RESET);
  	  HAL_GPIO_WritePin(R1A, R1, GPIO_PIN_RESET);
  	  HAL_GPIO_WritePin(R2A, R2, GPIO_PIN_RESET);
  	  HAL_GPIO_WritePin(R3A, R3, GPIO_PIN_RESET);
  	  HAL_GPIO_WritePin(R4A, R4, GPIO_PIN_RESET);
  	  HAL_GPIO_WritePin(R5A, R5, GPIO_PIN_RESET);
  	  HAL_GPIO_WritePin(R6A, R6, GPIO_PIN_RESET);
  	  HAL_GPIO_WritePin(R7A, R7, GPIO_PIN_RESET);

  	  while((ms-now)<16000){
  	  }

  	  displayHome();
  	  Up = 0;
  	  Down = 0;
  	  Left = 0;
  	  Right = 0;
  	  Middle = 0;
    }
    else {
  	  displayHome();
  	  Up = 0;
  	  Down = 0;
  	  Left = 0;
  	  Right = 0;
  	  Middle = 0;
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (Middle) {
		  now = ms;
		  if (gamePlay) {
			  gamePlay = 0;
			  displayHome();
		  }
		  else
		  {
			  gamePlay = 1;
		  }
		  Middle = 0;
	  }
	  if (gamePlay)
	  {
		  now = ms;
		  displayMove(ledMatrix);
		  posStr[2] = xpos+48;
		  posStr[3] = ypos+48;
		  HAL_UART_Transmit_IT(&huart2, posStr, 10);
		  ypospre = ypos;
		  xpospre = xpos;
		  HAL_ADC_Start(&hadc2);
		  if(HAL_ADC_PollForConversion(&hadc2, 100)==HAL_OK){
			  adcVal = HAL_ADC_GetValue(&hadc2);
			  ypos = 7-(adcVal/512)%8;
			  HAL_ADC_Stop(&hadc2);
		  }
		  if(Left && xpos>0)
		  {
			  xpos -= 1;
		  }
		  else if(Right && xpos<7)
		  {
			  xpos += 1;
		  }
		  Left = 0;
		  Right = 0;
		  ledMatrix[xpospre*8+ypospre] = 0;
		  ledMatrix[xpos*8+ypos] = 1;
		  while(ms-now<100){
		  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB11 
                           PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void displayHome()
{
	HAL_GPIO_WritePin(R0A, R0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R1A, R1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R2A, R2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R3A, R3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R4A, R4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R5A, R5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R6A, R6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R7A, R7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C0A, C0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C1A, C1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C2A, C2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C3A, C3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C4A, C4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C5A, C5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C6A, C6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C7A, C7, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(C0A, C0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C7A, C7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R0A, R0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R7A, R7, GPIO_PIN_SET);

	return;
}

void displayMove(_Bool* matrix)
{
	uint8_t i = 0;
	uint8_t j = 0;

	HAL_GPIO_WritePin(R0A, R0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R1A, R1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R2A, R2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R3A, R3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R4A, R4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R5A, R5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R6A, R6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R7A, R7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C0A, C0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C1A, C1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C2A, C2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C3A, C3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C4A, C4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C5A, C5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C6A, C6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C7A, C7, GPIO_PIN_RESET);

	for(i=0;i<8;i++){
		for(j=0;j<8;j++){
			if(matrix[8*i+j]==1)
			{
				switch (i)
				{
					case 0:
						HAL_GPIO_WritePin(C0A, C0, GPIO_PIN_SET);
						break;
					case 1:
						HAL_GPIO_WritePin(C1A, C1, GPIO_PIN_SET);
						break;
					case 2:
						HAL_GPIO_WritePin(C2A, C2, GPIO_PIN_SET);
						break;
					case 3:
						HAL_GPIO_WritePin(C3A, C3, GPIO_PIN_SET);
						break;
					case 4:
						HAL_GPIO_WritePin(C4A, C4, GPIO_PIN_SET);
						break;
					case 5:
						HAL_GPIO_WritePin(C5A, C5, GPIO_PIN_SET);
						break;
					case 6:
						HAL_GPIO_WritePin(C6A, C6, GPIO_PIN_SET);
						break;
					case 7:
						HAL_GPIO_WritePin(C7A, C7, GPIO_PIN_SET);
						break;
				}
				switch (j)
				{
					case 0:
						HAL_GPIO_WritePin(R0A, R0, GPIO_PIN_SET);
						break;
					case 1:
						HAL_GPIO_WritePin(R1A, R1, GPIO_PIN_SET);
						break;
					case 2:
						HAL_GPIO_WritePin(R2A, R2, GPIO_PIN_SET);
						break;
					case 3:
						HAL_GPIO_WritePin(R3A, R3, GPIO_PIN_SET);
						break;
					case 4:
						HAL_GPIO_WritePin(R4A, R4, GPIO_PIN_SET);
						break;
					case 5:
						HAL_GPIO_WritePin(R5A, R5, GPIO_PIN_SET);
						break;
					case 6:
						HAL_GPIO_WritePin(R6A, R6, GPIO_PIN_SET);
						break;
					case 7:
						HAL_GPIO_WritePin(R7A, R7, GPIO_PIN_SET);
						break;
				}
			}
		}
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
