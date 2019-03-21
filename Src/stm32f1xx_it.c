/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
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
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
 DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  if(offsetcount < 1000) {  // calibrate ADC offsets
    offsetcount++;
    offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
    offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
    offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
    offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
    batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
  }

  //disable PWM when current limit is reached (current chopping)
  if(ABS((adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP) > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
    //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
  }

  if(ABS((adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP)  > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  int ul, vl, wl;
  int ur, vr, wr;

  //determine next position based on hall sensors
  uint8_t hall_ul = !(LEFT_HALL_U_PIN_GPIO_Port->IDR & LEFT_HALL_U_PIN_Pin);
  uint8_t hall_vl = !(LEFT_HALL_V_PIN_GPIO_Port->IDR & LEFT_HALL_V_PIN_Pin);
  uint8_t hall_wl = !(LEFT_HALL_W_PIN_GPIO_Port->IDR & LEFT_HALL_W_PIN_Pin);

  uint8_t hall_ur = !(RIGHT_HALL_U_PIN_GPIO_Port->IDR & RIGHT_HALL_U_PIN_Pin);
  uint8_t hall_vr = !(RIGHT_HALL_V_PIN_GPIO_Port->IDR & RIGHT_HALL_V_PIN_Pin);
  uint8_t hall_wr = !(RIGHT_HALL_W_PIN_GPIO_Port->IDR & RIGHT_HALL_W_PIN_Pin);

  uint8_t halll = hall_ul * 1 + hall_vl * 2 + hall_wl * 4;
  posl          = hall_to_pos[halll];
  posl += 2;
  posl %= 6;

  uint8_t hallr = hall_ur * 1 + hall_vr * 2 + hall_wr * 4;
  posr          = hall_to_pos[hallr];
  posr += 2;
  posr %= 6;

  blockPhaseCurrent(posl, adc_buffer.rl1 - offsetrl1, adc_buffer.rl2 - offsetrl2, &curl);

  //setScopeChannel(2, (adc_buffer.rl1 - offsetrl1) / 8);
  //setScopeChannel(3, (adc_buffer.rl2 - offsetrl2) / 8);


  // uint8_t buzz(uint16_t *notes, uint32_t len){
    // static uint32_t counter = 0;
    // static uint32_t timer = 0;
    // if(len == 0){
        // return(0);
    // }
    
    // struct {
        // uint16_t freq : 4;
        // uint16_t volume : 4;
        // uint16_t time : 8;
    // } note = notes[counter];
    
    // if(timer / 500 == note.time){
        // timer = 0;
        // counter++;
    // }
    
    // if(counter == len){
        // counter = 0;
    // }

    // timer++;
    // return(note.freq);
  // }


  //create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerTimer % buzzerFreq == 0) {
      HAL_GPIO_TogglePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin);
    }
  } else {
      HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 0);
  }

  //update PWM channels based on position
  blockPWM(pwml, posl, &ul, &vl, &wl);
  blockPWM(pwmr, posr, &ur, &vr, &wr);

  int weakul, weakvl, weakwl;
  if (pwml > 0) {
    blockPWM(weakl, (posl+5) % 6, &weakul, &weakvl, &weakwl);
  } else {
    blockPWM(-weakl, (posl+1) % 6, &weakul, &weakvl, &weakwl);
  }
  ul += weakul;
  vl += weakvl;
  wl += weakwl;

  int weakur, weakvr, weakwr;
  if (pwmr > 0) {
    blockPWM(weakr, (posr+5) % 6, &weakur, &weakvr, &weakwr);
  } else {
    blockPWM(-weakr, (posr+1) % 6, &weakur, &weakvr, &weakwr);
  }
  ur += weakur;
  vr += weakvr;
  wr += weakwr;

  LEFT_TIM->LEFT_TIM_U = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_V = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_W = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);

  RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */
//TODO insert comms code
  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
