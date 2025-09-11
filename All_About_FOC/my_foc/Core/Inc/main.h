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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "foc_interface.h"
//#include "foc_callbacks.h"
//#include "utils_math.h"

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
#define A_CURR_Pin GPIO_PIN_0
#define A_CURR_GPIO_Port GPIOC
#define B_CURR_Pin GPIO_PIN_1
#define B_CURR_GPIO_Port GPIOC
#define C_CURR_Pin GPIO_PIN_2
#define C_CURR_GPIO_Port GPIOC
#define VIN_SENS_Pin GPIO_PIN_3
#define VIN_SENS_GPIO_Port GPIOC
#define VOLTAGE_1_Pin GPIO_PIN_0
#define VOLTAGE_1_GPIO_Port GPIOA
#define VOLTAGE_2_Pin GPIO_PIN_1
#define VOLTAGE_2_GPIO_Port GPIOA
#define VOLTAGE_3_Pin GPIO_PIN_2
#define VOLTAGE_3_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOB
#define INL_A_Pin GPIO_PIN_13
#define INL_A_GPIO_Port GPIOB
#define INL_B_Pin GPIO_PIN_14
#define INL_B_GPIO_Port GPIOB
#define INL_C_Pin GPIO_PIN_15
#define INL_C_GPIO_Port GPIOB
#define ENC_1_Pin GPIO_PIN_6
#define ENC_1_GPIO_Port GPIOC
#define ENC_2_Pin GPIO_PIN_7
#define ENC_2_GPIO_Port GPIOC
#define ENC_3_Pin GPIO_PIN_8
#define ENC_3_GPIO_Port GPIOC
#define DRV_CS_Pin GPIO_PIN_9
#define DRV_CS_GPIO_Port GPIOC
#define INH_A_Pin GPIO_PIN_8
#define INH_A_GPIO_Port GPIOA
#define INH_B_Pin GPIO_PIN_9
#define INH_B_GPIO_Port GPIOA
#define INH_C_Pin GPIO_PIN_10
#define INH_C_GPIO_Port GPIOA
#define DRV_SCK_Pin GPIO_PIN_10
#define DRV_SCK_GPIO_Port GPIOC
#define DRV_MISO_Pin GPIO_PIN_3
#define DRV_MISO_GPIO_Port GPIOB
#define DRV_MOSI_Pin GPIO_PIN_4
#define DRV_MOSI_GPIO_Port GPIOB
#define DRV_EN_Pin GPIO_PIN_5
#define DRV_EN_GPIO_Port GPIOB
#define DRV_nFault_Pin GPIO_PIN_7
#define DRV_nFault_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
