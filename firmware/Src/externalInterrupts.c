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

	case ENC_A_M2_INT:
		if(ENC_A_M2_GPIO_Port->IDR & ENC_A_M2_Pin)
		{
			if(ENC_B_M2_GPIO_Port->IDR & ENC_B_M2_Pin)
			{
				// backward
				motor2_encoder--;
			} else {
				// forward
				motor2_encoder++;
			}
		} else {
			if(ENC_B_M2_GPIO_Port->IDR & ENC_B_M2_Pin)
			{
				// forward
				motor2_encoder++;
			} else {
				// backward
				motor2_encoder--;
			}
		}
		break;

	case ENC_A_M3_INT:
		if(ENC_A_M3_GPIO_Port->IDR & ENC_A_M3_Pin)
		{
			if(ENC_B_M3_GPIO_Port->IDR & ENC_B_M3_Pin)
			{
				// backward
				motor3_encoder--;
			} else {
				// forward
				motor3_encoder++;
			}
		} else {
			if(ENC_B_M3_GPIO_Port->IDR & ENC_B_M3_Pin)
			{
				// forward
				motor3_encoder++;
			} else {
				// backward
				motor3_encoder--;
			}
		}
		break;

	case ENC_A_M4_INT:
		if(ENC_A_M4_GPIO_Port->IDR & ENC_A_M4_Pin)
		{
			if(ENC_B_M4_GPIO_Port->IDR & ENC_B_M4_Pin)
			{
				// backward
				motor4_encoder--;
			} else {
				// forward
				motor4_encoder++;
			}
		} else {
			if(ENC_B_M4_GPIO_Port->IDR & ENC_B_M4_Pin)
			{
				// forward
				motor4_encoder++;
			} else {
				// backward
				motor4_encoder--;
			}
		}
		break;

	case ENC_A_M5_INT:
		if(ENC_A_M5_GPIO_Port->IDR & ENC_A_M5_Pin)
		{
			if(ENC_B_M5_GPIO_Port->IDR & ENC_B_M5_Pin)
			{
				// backward
				motor5_encoder--;
			} else {
				// forward
				motor5_encoder++;
			}
		} else {
			if(ENC_B_M5_GPIO_Port->IDR & ENC_B_M5_Pin)
			{
				// forward
				motor5_encoder++;
			} else {
				// backward
				motor5_encoder--;
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
	}
}
