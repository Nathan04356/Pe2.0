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
#include "stm32l0xx_hal.h"

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
#define OSC_Pin GPIO_PIN_0
#define OSC_GPIO_Port GPIOH
#define STATUS_Pin GPIO_PIN_1
#define STATUS_GPIO_Port GPIOH
#define ADC0_Pin GPIO_PIN_0
#define ADC0_GPIO_Port GPIOA
#define ADC1_Pin GPIO_PIN_1
#define ADC1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_2
#define ADC2_GPIO_Port GPIOA
#define ADC3_Pin GPIO_PIN_3
#define ADC3_GPIO_Port GPIOA
#define ADC4_Pin GPIO_PIN_4
#define ADC4_GPIO_Port GPIOA
#define ADC5_Pin GPIO_PIN_5
#define ADC5_GPIO_Port GPIOA
#define LEVEL_Pin GPIO_PIN_6
#define LEVEL_GPIO_Port GPIOA
#define IR1_Pin GPIO_PIN_7
#define IR1_GPIO_Port GPIOA
#define DRUK5_Pin GPIO_PIN_0
#define DRUK5_GPIO_Port GPIOB
#define DRUK4_Pin GPIO_PIN_1
#define DRUK4_GPIO_Port GPIOB
#define DRUK3_Pin GPIO_PIN_2
#define DRUK3_GPIO_Port GPIOB
#define CH3_Pin GPIO_PIN_10
#define CH3_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_11
#define BIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_12
#define BIN1_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_13
#define STBY_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_14
#define AIN2_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_15
#define AIN1_GPIO_Port GPIOB
#define IR2_Pin GPIO_PIN_8
#define IR2_GPIO_Port GPIOA
#define IR3_Pin GPIO_PIN_9
#define IR3_GPIO_Port GPIOA
#define IR4_Pin GPIO_PIN_10
#define IR4_GPIO_Port GPIOA
#define IR5_Pin GPIO_PIN_11
#define IR5_GPIO_Port GPIOA
#define IR0_Pin GPIO_PIN_12
#define IR0_GPIO_Port GPIOA
#define CH1_Pin GPIO_PIN_15
#define CH1_GPIO_Port GPIOA
#define DRUK2_Pin GPIO_PIN_3
#define DRUK2_GPIO_Port GPIOB
#define DRUK1_Pin GPIO_PIN_4
#define DRUK1_GPIO_Port GPIOB
#define DRUK0_Pin GPIO_PIN_5
#define DRUK0_GPIO_Port GPIOB
#define IR_Pin GPIO_PIN_8
#define IR_GPIO_Port GPIOB
#define IR_EXTI_IRQn EXTI4_15_IRQn
#define STOF_Pin GPIO_PIN_9
#define STOF_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
