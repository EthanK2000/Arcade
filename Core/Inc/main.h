/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
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

#define LED1 GPIO_PIN_8
#define LED1A GPIOB
#define LED2 GPIO_PIN_9
#define LED2A GPIOB
#define LED3 GPIO_PIN_6
#define LED3A GPIOA
#define LED4 GPIO_PIN_7
#define LED4A GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
