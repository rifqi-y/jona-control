/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define user_led_Pin GPIO_PIN_3
#define user_led_GPIO_Port GPIOE
#define S34_12_PUL_1_Pin GPIO_PIN_5
#define S34_12_PUL_1_GPIO_Port GPIOD
#define S34_12_DIR_1_Pin GPIO_PIN_6
#define S34_12_DIR_1_GPIO_Port GPIOD
#define S34_12_PUL_Pin GPIO_PIN_7
#define S34_12_PUL_GPIO_Port GPIOD
#define S34_12_DIR_Pin GPIO_PIN_3
#define S34_12_DIR_GPIO_Port GPIOB
#define S34_85_PUL_Pin GPIO_PIN_4
#define S34_85_PUL_GPIO_Port GPIOB
#define S34_85_DIR_Pin GPIO_PIN_5
#define S34_85_DIR_GPIO_Port GPIOB
#define S23_PUL_Pin GPIO_PIN_6
#define S23_PUL_GPIO_Port GPIOB
#define S23_DIR_Pin GPIO_PIN_7
#define S23_DIR_GPIO_Port GPIOB
#define S17_12_PUL_Pin GPIO_PIN_8
#define S17_12_PUL_GPIO_Port GPIOB
#define S17_12_DIR_Pin GPIO_PIN_9
#define S17_12_DIR_GPIO_Port GPIOB
#define S17_07_PUL_Pin GPIO_PIN_0
#define S17_07_PUL_GPIO_Port GPIOE
#define S17_07_DIR_Pin GPIO_PIN_1
#define S17_07_DIR_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
