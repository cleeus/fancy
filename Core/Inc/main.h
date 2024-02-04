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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_PC13_Pin GPIO_PIN_13
#define LED_PC13_GPIO_Port GPIOC
#define RELAY_K1_Pin GPIO_PIN_0
#define RELAY_K1_GPIO_Port GPIOA
#define RELAY_K2_Pin GPIO_PIN_1
#define RELAY_K2_GPIO_Port GPIOA
#define RELAY_K3_Pin GPIO_PIN_2
#define RELAY_K3_GPIO_Port GPIOA
#define RELAY_K4_Pin GPIO_PIN_3
#define RELAY_K4_GPIO_Port GPIOA
#define RELAY_K5_Pin GPIO_PIN_4
#define RELAY_K5_GPIO_Port GPIOA
#define RELAY_K6_Pin GPIO_PIN_5
#define RELAY_K6_GPIO_Port GPIOA
#define RELAY_K7_Pin GPIO_PIN_6
#define RELAY_K7_GPIO_Port GPIOA
#define RELAY_K8_Pin GPIO_PIN_7
#define RELAY_K8_GPIO_Port GPIOA
#define RELAY_OE_Pin GPIO_PIN_0
#define RELAY_OE_GPIO_Port GPIOB
#define NTC_IN_Pin GPIO_PIN_1
#define NTC_IN_GPIO_Port GPIOB
#define DHT11_DATA_Pin GPIO_PIN_11
#define DHT11_DATA_GPIO_Port GPIOB
#define DHT11_DATA_EXTI_IRQn EXTI15_10_IRQn
#define CH7_MOSFET_12V_Pin GPIO_PIN_12
#define CH7_MOSFET_12V_GPIO_Port GPIOB
#define CH7_MOSFET_ALTVOLT_Pin GPIO_PIN_13
#define CH7_MOSFET_ALTVOLT_GPIO_Port GPIOB
#define ALTVOLT_MOSFET_2_Pin GPIO_PIN_9
#define ALTVOLT_MOSFET_2_GPIO_Port GPIOA
#define ALTVOLT_MOSFET_1_Pin GPIO_PIN_10
#define ALTVOLT_MOSFET_1_GPIO_Port GPIOA
#define TM1637_NCLK_Pin GPIO_PIN_3
#define TM1637_NCLK_GPIO_Port GPIOB
#define TM1637_NDIO_Pin GPIO_PIN_4
#define TM1637_NDIO_GPIO_Port GPIOB
#define BUZZER_TIM_Pin GPIO_PIN_8
#define BUZZER_TIM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
