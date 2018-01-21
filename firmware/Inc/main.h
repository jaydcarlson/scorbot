/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

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

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
