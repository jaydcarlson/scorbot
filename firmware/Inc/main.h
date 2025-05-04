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
#define INA3_Pin GPIO_PIN_2
#define INA3_GPIO_Port GPIOE
#define INB3_Pin GPIO_PIN_3
#define INB3_GPIO_Port GPIOE
#define INA8_Pin GPIO_PIN_4
#define INA8_GPIO_Port GPIOE
#define PWM3_Pin GPIO_PIN_5
#define PWM3_GPIO_Port GPIOE
#define PWM4_Pin GPIO_PIN_6
#define PWM4_GPIO_Port GPIOE
#define INA7_Pin GPIO_PIN_13
#define INA7_GPIO_Port GPIOC
#define INB7_Pin GPIO_PIN_14
#define INB7_GPIO_Port GPIOC
#define MS7_Pin GPIO_PIN_15
#define MS7_GPIO_Port GPIOC
#define CS8_Pin GPIO_PIN_0
#define CS8_GPIO_Port GPIOC
#define CS3_Pin GPIO_PIN_2
#define CS3_GPIO_Port GPIOC
#define CS5_Pin GPIO_PIN_3
#define CS5_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_0
#define PWM1_GPIO_Port GPIOA
#define PWM6_Pin GPIO_PIN_3
#define PWM6_GPIO_Port GPIOA
#define CS6_Pin GPIO_PIN_4
#define CS6_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_5
#define CS1_GPIO_Port GPIOA
#define CS4_Pin GPIO_PIN_6
#define CS4_GPIO_Port GPIOA
#define CS2_Pin GPIO_PIN_0
#define CS2_GPIO_Port GPIOB
#define CS7_Pin GPIO_PIN_1
#define CS7_GPIO_Port GPIOB
#define INA6_Pin GPIO_PIN_2
#define INA6_GPIO_Port GPIOB
#define RF_WIP_Pin GPIO_PIN_7
#define RF_WIP_GPIO_Port GPIOE
#define INB8_Pin GPIO_PIN_8
#define INB8_GPIO_Port GPIOE
#define ENC_A_M7_Pin GPIO_PIN_9
#define ENC_A_M7_GPIO_Port GPIOE
#define INA2_Pin GPIO_PIN_10
#define INA2_GPIO_Port GPIOE
#define ENC_B_M7_Pin GPIO_PIN_11
#define ENC_B_M7_GPIO_Port GPIOE
#define MS8_Pin GPIO_PIN_12
#define MS8_GPIO_Port GPIOE
#define MS3_Pin GPIO_PIN_15
#define MS3_GPIO_Port GPIOE
#define MS4_Pin GPIO_PIN_10
#define MS4_GPIO_Port GPIOB
#define PWM5_Pin GPIO_PIN_14
#define PWM5_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_15
#define PWM2_GPIO_Port GPIOB
#define INA4_Pin GPIO_PIN_8
#define INA4_GPIO_Port GPIOD
#define INB4_Pin GPIO_PIN_9
#define INB4_GPIO_Port GPIOD
#define MS2_Pin GPIO_PIN_10
#define MS2_GPIO_Port GPIOD
#define MS1_Pin GPIO_PIN_11
#define MS1_GPIO_Port GPIOD
#define ENC_A_M3_Pin GPIO_PIN_12
#define ENC_A_M3_GPIO_Port GPIOD
#define ENC_A_M3_EXTI_IRQn EXTI15_10_IRQn
#define ENC_A_M5_Pin GPIO_PIN_13
#define ENC_A_M5_GPIO_Port GPIOD
#define ENC_A_M5_EXTI_IRQn EXTI15_10_IRQn
#define MS5_Pin GPIO_PIN_14
#define MS5_GPIO_Port GPIOD
#define MS6_Pin GPIO_PIN_15
#define MS6_GPIO_Port GPIOD
#define ENC_B_M3_Pin GPIO_PIN_6
#define ENC_B_M3_GPIO_Port GPIOC
#define ENC_B_M5_Pin GPIO_PIN_7
#define ENC_B_M5_GPIO_Port GPIOC
#define ENC_A_M6_Pin GPIO_PIN_8
#define ENC_A_M6_GPIO_Port GPIOC
#define ENC_A_M6_EXTI_IRQn EXTI9_5_IRQn
#define ENC_B_M2_Pin GPIO_PIN_9
#define ENC_B_M2_GPIO_Port GPIOC
#define ENC_B_M4_Pin GPIO_PIN_9
#define ENC_B_M4_GPIO_Port GPIOA
#define ENC_B_M4_EXTI_IRQn EXTI9_5_IRQn
#define ENC_A_M1_Pin GPIO_PIN_10
#define ENC_A_M1_GPIO_Port GPIOA
#define ENC_A_M1_EXTI_IRQn EXTI15_10_IRQn
#define ENC_A_M2_Pin GPIO_PIN_15
#define ENC_A_M2_GPIO_Port GPIOA
#define ENC_A_M2_EXTI_IRQn EXTI15_10_IRQn
#define INA1_Pin GPIO_PIN_2
#define INA1_GPIO_Port GPIOD
#define INB1_Pin GPIO_PIN_3
#define INB1_GPIO_Port GPIOD
#define INB6_Pin GPIO_PIN_4
#define INB6_GPIO_Port GPIOD
#define DEBUG_TX_Pin GPIO_PIN_5
#define DEBUG_TX_GPIO_Port GPIOD
#define DEBUG_RX_Pin GPIO_PIN_6
#define DEBUG_RX_GPIO_Port GPIOD
#define INB5_Pin GPIO_PIN_7
#define INB5_GPIO_Port GPIOD
#define ENC_B_M6_Pin GPIO_PIN_3
#define ENC_B_M6_GPIO_Port GPIOB
#define ENC_A_M4_Pin GPIO_PIN_4
#define ENC_A_M4_GPIO_Port GPIOB
#define ENC_A_M4_EXTI_IRQn EXTI4_IRQn
#define ENC_B_M1_Pin GPIO_PIN_5
#define ENC_B_M1_GPIO_Port GPIOB
#define PWM7_Pin GPIO_PIN_8
#define PWM7_GPIO_Port GPIOB
#define PWM8_Pin GPIO_PIN_9
#define PWM8_GPIO_Port GPIOB
#define INA5_Pin GPIO_PIN_0
#define INA5_GPIO_Port GPIOE
#define INB2_Pin GPIO_PIN_1
#define INB2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
