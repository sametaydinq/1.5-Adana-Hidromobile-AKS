/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Gaz_Sensor_Pin GPIO_PIN_0
#define Gaz_Sensor_GPIO_Port GPIOA
#define Voltage_EYS_Pin GPIO_PIN_1
#define Voltage_EYS_GPIO_Port GPIOA
#define Voltage_Engine_Pin GPIO_PIN_2
#define Voltage_Engine_GPIO_Port GPIOA
#define SIGN_EYS_Pin GPIO_PIN_3
#define SIGN_EYS_GPIO_Port GPIOA
#define Voltage_Telemetry_Pin GPIO_PIN_4
#define Voltage_Telemetry_GPIO_Port GPIOA
#define RELAY_ENGINE_12V_Pin GPIO_PIN_5
#define RELAY_ENGINE_12V_GPIO_Port GPIOA
#define Voltage_SMPS_Pin GPIO_PIN_6
#define Voltage_SMPS_GPIO_Port GPIOA
#define SIGN_Telemetry_Pin GPIO_PIN_7
#define SIGN_Telemetry_GPIO_Port GPIOA
#define Voltage_BMS_Pin GPIO_PIN_0
#define Voltage_BMS_GPIO_Port GPIOB
#define SIGN_BMS_Pin GPIO_PIN_8
#define SIGN_BMS_GPIO_Port GPIOA
#define SIGN_SMPS_Pin GPIO_PIN_9
#define SIGN_SMPS_GPIO_Port GPIOA
#define RELAY_SMPS_BAT_Pin GPIO_PIN_12
#define RELAY_SMPS_BAT_GPIO_Port GPIOA
#define RELAY_AC_IN_Pin GPIO_PIN_15
#define RELAY_AC_IN_GPIO_Port GPIOA
#define IN_AC_DETECT_Pin GPIO_PIN_3
#define IN_AC_DETECT_GPIO_Port GPIOB
#define RELAY_ENGINE_96V_Pin GPIO_PIN_4
#define RELAY_ENGINE_96V_GPIO_Port GPIOB
#define RELAY_MAIN_Pin GPIO_PIN_5
#define RELAY_MAIN_GPIO_Port GPIOB
#define SELENOID_VALF_Pin GPIO_PIN_6
#define SELENOID_VALF_GPIO_Port GPIOB
#define LIGHT_Pin GPIO_PIN_7
#define LIGHT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
