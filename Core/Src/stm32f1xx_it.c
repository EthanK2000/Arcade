/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile _Bool Up;
extern volatile _Bool Down;
extern volatile _Bool Left;
extern volatile _Bool Right;
extern volatile _Bool Middle;
extern uint32_t ms;
extern volatile uint32_t mod;
extern _Bool ledMatrix[];
extern volatile _Bool ms100;
extern volatile _Bool varTim;
extern uint8_t vel;
uint32_t bounceDelay = 125;
uint32_t bounceLast = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void displayLED(uint8_t row);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void displayLED(uint8_t row)
{
	uint8_t i = 0;

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

	switch (row)
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
	for (i = 0; i<8; i++)
	{
		if(ledMatrix[8*row+i]==1)
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
		}
	}
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	ms++;
	if((ms%100)==89){
		ms100 = 1;
	}
	if((ms%(750-vel*50))==mod){
		varTim = 1;
	}
	displayLED(ms%8);
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	if ((!Down) && ((ms-bounceLast)>bounceDelay)) {
		Down = 1;
		bounceLast = ms;
	}
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	if ((!Middle) && ((ms-bounceLast)>bounceDelay)) {
		Middle = 1;
		bounceLast = ms;
	}
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	if ((!Up) && ((ms-bounceLast)>bounceDelay)) {
		Up = 1;
		bounceLast = ms;
	}
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	if ((!Left) && ((ms-bounceLast)>bounceDelay)) {
		Left = 1;
		bounceLast = ms;
	}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	if ((!Right) && ((ms-bounceLast)>bounceDelay)) {
		Right = 1;
		bounceLast = ms;
	}
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
