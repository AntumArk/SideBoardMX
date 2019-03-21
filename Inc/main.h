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
#include "config.h"
#include "bldc.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
  uint16_t rr1;
  uint16_t rr2;
  uint16_t rl1;
  uint16_t rl2;
  uint16_t dcr;
  uint16_t dcl;
  uint16_t batt1;
  uint16_t l_tx2;
  uint16_t temp;
  uint16_t l_rx2;
} adc_buf_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define NO 0
#define YES 1
#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0) : (((x) < (-lowhigh)) ? (-1.0) : (0.0)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0) : (((x) < (low)) ? (-1.0) : (0.0)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0)
#define RAD(a) ((a)*180.0 / M_PI)
#define SIGN(a) (((a) < 0.0) ? (-1.0) : (((a) > 0.0) ? (1.0) : (0.0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0), 1.0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))
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
#define RIGHT_U_CUR_PIN_Pin GPIO_PIN_4
#define RIGHT_U_CUR_PIN_GPIO_Port GPIOC
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
#define RIGHT_HALL_U_PIN_Pin GPIO_PIN_10
#define RIGHT_HALL_U_PIN_GPIO_Port GPIOC
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
#define DELAY_TIM_FREQUENCY_US 1000000

#define MOTOR_AMP_CONV_DC_AMP 0.02  // A per bit (12) on ADC.

#define MILLI_R (R * 1000)
#define MILLI_PSI (PSI * 1000)
#define MILLI_V (V * 1000)

//Timer defines
#define LEFT_TIM TIM8
#define RIGHT_TIM TIM1
#define LEFT_TIM_U CCR1
#define LEFT_TIM_V CCR2
#define LEFT_TIM_W CCR3
#define RIGHT_TIM_U CCR1
#define RIGHT_TIM_V CCR2
#define RIGHT_TIM_W CCR3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
