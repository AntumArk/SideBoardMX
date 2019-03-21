/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#define CONTROL_BAUD 19200
#define DEBUG_BAUD 115200
#define PWM_FREQ 16000
#define DEAD_TIME 32
#define LEFT_DC_CUR_PIN_Pin GPIO_PIN_0
#define LEFT_DC_CUR_PIN_GPIO_Port GPIOC
#define RIGHT_DC_CUR_PIN_Pin GPIO_PIN_1
#define RIGHT_DC_CUR_PIN_GPIO_Port GPIOC
#define DCLINK_PIN_Pin GPIO_PIN_2
#define DCLINK_PIN_GPIO_Port GPIOC
#define LEFT_V_CUR_PIN_Pin GPIO_PIN_3
#define LEFT_V_CUR_PIN_GPIO_Port GPIOC
#define LEFT_U_CUR_PIN_Pin GPIO_PIN_0
#define LEFT_U_CUR_PIN_GPIO_Port GPIOA
#define BUTTON_PIN_Pin GPIO_PIN_1
#define BUTTON_PIN_GPIO_Port GPIOA
#define BUZZER_PIN_Pin GPIO_PIN_4
#define BUZZER_PIN_GPIO_Port GPIOA
#define OFF_PIN_Pin GPIO_PIN_5
#define OFF_PIN_GPIO_Port GPIOA
#define LEFT_TIM_UL_PIN_Pin GPIO_PIN_7
#define LEFT_TIM_UL_PIN_GPIO_Port GPIOA
#define RIGHT_V_CUR_PIN_Pin GPIO_PIN_5
#define RIGHT_V_CUR_PIN_GPIO_Port GPIOC
#define LEFT_TIM_VL_PIN_Pin GPIO_PIN_0
#define LEFT_TIM_VL_PIN_GPIO_Port GPIOB
#define LEFT_TIM_WL_PIN_Pin GPIO_PIN_1
#define LEFT_TIM_WL_PIN_GPIO_Port GPIOB
#define LED_PIN_Pin GPIO_PIN_2
#define LED_PIN_GPIO_Port GPIOB
#define RIGHT_TIM_UL_PIN_Pin GPIO_PIN_13
#define RIGHT_TIM_UL_PIN_GPIO_Port GPIOB
#define RIGHT_TIM_VL_PIN_Pin GPIO_PIN_14
#define RIGHT_TIM_VL_PIN_GPIO_Port GPIOB
#define RIGHT_TIM_WL_PIN_Pin GPIO_PIN_15
#define RIGHT_TIM_WL_PIN_GPIO_Port GPIOB
#define LEFT_TIM_UH_PIN_Pin GPIO_PIN_6
#define LEFT_TIM_UH_PIN_GPIO_Port GPIOC
#define LEFT_TIM_VH_PIN_Pin GPIO_PIN_7
#define LEFT_TIM_VH_PIN_GPIO_Port GPIOC
#define LEFT_TIM_WH_PIN_Pin GPIO_PIN_8
#define LEFT_TIM_WH_PIN_GPIO_Port GPIOC
#define RIGHT_TIM_UH_PIN_Pin GPIO_PIN_8
#define RIGHT_TIM_UH_PIN_GPIO_Port GPIOA
#define RIGHT_TIM_VH_PIN_Pin GPIO_PIN_9
#define RIGHT_TIM_VH_PIN_GPIO_Port GPIOA
#define RIGHT_TIM_WH_PIN_Pin GPIO_PIN_10
#define RIGHT_TIM_WH_PIN_GPIO_Port GPIOA
#define CHARGER_PIN_Pin GPIO_PIN_12
#define CHARGER_PIN_GPIO_Port GPIOA
#define RIGHT_HALL_V_PIN_Pin GPIO_PIN_11
#define RIGHT_HALL_V_PIN_GPIO_Port GPIOC
#define RIGHT_HALL_W_PIN_Pin GPIO_PIN_12
#define RIGHT_HALL_W_PIN_GPIO_Port GPIOC
#define LEFT_HALL_V_PIN_Pin GPIO_PIN_4
#define LEFT_HALL_V_PIN_GPIO_Port GPIOB
#define LEFT_HALL_U_PIN_Pin GPIO_PIN_5
#define LEFT_HALL_U_PIN_GPIO_Port GPIOB
#define LEFT_HALL_W_PIN_Pin GPIO_PIN_7
#define LEFT_HALL_W_PIN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
