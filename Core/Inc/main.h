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
#include "stm32h5xx_hal.h"

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
#define SWITCH_Pin GPIO_PIN_13
#define SWITCH_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_14
#define LED0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOH
#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOH
#define LED_FL_Pin GPIO_PIN_0
#define LED_FL_GPIO_Port GPIOA
#define LED_SL_Pin GPIO_PIN_1
#define LED_SL_GPIO_Port GPIOA
#define LED_SR_Pin GPIO_PIN_2
#define LED_SR_GPIO_Port GPIOA
#define LED_FR_Pin GPIO_PIN_3
#define LED_FR_GPIO_Port GPIOA
#define TOF_EN0_Pin GPIO_PIN_5
#define TOF_EN0_GPIO_Port GPIOC
#define TOF_EN1_Pin GPIO_PIN_0
#define TOF_EN1_GPIO_Port GPIOB
#define TOF_EN2_Pin GPIO_PIN_1
#define TOF_EN2_GPIO_Port GPIOB
#define TOF_EN3_Pin GPIO_PIN_2
#define TOF_EN3_GPIO_Port GPIOB
#define SPI2_CS0_Pin GPIO_PIN_12
#define SPI2_CS0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
