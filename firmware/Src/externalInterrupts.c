/*
 * softwareEncoder.c
 *
 *  Created on: Jan 13, 2018
 *      Author: jay
 */

#include "softwareEncoder.h"
#include "stm32f4xx_hal.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case ENC_A_M1_INT:
	case ENC_B_M1_INT:
		if(ENC_A_M1_GPIO_Port->IDR & ENC_A_M1_Pin)
		{
			if(ENC_B_M1_GPIO_Port->IDR & ENC_B_M1_Pin)
			{
				// backward
				motor1_encoder--;
			} else {
				// forward
				motor1_encoder++;
			}
		} else {
			if(ENC_B_M1_GPIO_Port->IDR & ENC_B_M1_Pin)
			{
				// forward
				motor1_encoder++;
			} else {
				// backward
				motor1_encoder--;
			}
		}
		break;

	case ENC_A_M6_INT:
		if(ENC_A_M6_GPIO_Port->IDR & ENC_A_M6_Pin)
		{
			if(ENC_B_M6_GPIO_Port->IDR & ENC_B_M6_Pin)
			{
				// backward
				motor6_encoder--;
			} else {
				// forward
				motor6_encoder++;
			}
		} else {
			if(ENC_B_M6_GPIO_Port->IDR & ENC_B_M6_Pin)
			{
				// forward
				motor6_encoder++;
			} else {
				// backward
				motor6_encoder--;
			}
		}
		break;

	case ENC_A_M8_INT:
	case ENC_B_M8_INT:
			if(ENC_A_M8_GPIO_Port->IDR & ENC_A_M8_Pin)
			{
				if(ENC_B_M8_GPIO_Port->IDR & ENC_B_M8_Pin)
				{
					// backward
					motor8_encoder--;
				} else {
					// forward
					motor8_encoder++;
				}
			} else {
				if(ENC_B_M8_GPIO_Port->IDR & ENC_B_M8_Pin)
				{
					// forward
					motor8_encoder++;
				} else {
					// backward
					motor8_encoder--;
				}
			}
			break;
	}
}
