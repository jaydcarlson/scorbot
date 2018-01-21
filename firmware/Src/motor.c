/*
 * motor.c
 *
 *  Created on: Jan 12, 2018
 *      Author: jay
 */

#include "motor.h"
#include "main.h"
#include "softwareEncoder.h"
#include "stm32f4xx_hal.h"

typedef struct motor {
	volatile uint32_t* ccr;
	volatile GPIO_TypeDef* a_port;
	volatile GPIO_TypeDef* b_port;
	volatile GPIO_TypeDef* ms_port;
	volatile int16_t* current_position;
	int32_t setpoint;
	uint16_t a_pin;
	uint16_t b_pin;
	uint16_t ms_pin;
	motor_mode_t mode;
	uint8_t invert_direction;
	uint8_t invert_homing_direction;
	float k_p;
} motor_t;

// Motor PWM timer / counter
#define MOTOR1_PWM_TIM 		TIM5
#define MOTOR1_PWM_CHANNEL	TIM_CHANNEL_1
#define MOTOR1_PWM_CCR		CCR1

#define MOTOR2_PWM_TIM 		TIM12
#define MOTOR2_PWM_CHANNEL	TIM_CHANNEL_2
#define MOTOR2_PWM_CCR		CCR2

#define MOTOR3_PWM_TIM 		TIM9
#define MOTOR3_PWM_CHANNEL	TIM_CHANNEL_1
#define MOTOR3_PWM_CCR		CCR1

#define MOTOR4_PWM_TIM 		TIM9
#define MOTOR4_PWM_CHANNEL	TIM_CHANNEL_2
#define MOTOR4_PWM_CCR		CCR2

#define MOTOR5_PWM_TIM 		TIM12
#define MOTOR5_PWM_CHANNEL	TIM_CHANNEL_1
#define MOTOR5_PWM_CCR		CCR1

#define MOTOR6_PWM_TIM 		TIM5
#define MOTOR6_PWM_CHANNEL	TIM_CHANNEL_4
#define MOTOR6_PWM_CCR		CCR4

#define MOTOR7_PWM_TIM 		TIM10
#define MOTOR7_PWM_CHANNEL	TIM_CHANNEL_1
#define MOTOR7_PWM_CCR		CCR1

//#define MOTOR8_PWM_TIM 		TIM11
//#define MOTOR8_PWM_CHANNEL	TIM_CHANNEL_1
//#define MOTOR8_PWM_CCR		CCR1

// encoder position counters (hardware or software-based)
//#define MOTOR2_POSITION_TIM	TIM2
//#define MOTOR3_POSITION_TIM	TIM4
//#define MOTOR4_POSITION_TIM	TIM3
//#define MOTOR5_POSITION_TIM	TIM8
#define MOTOR7_POSITION_TIM		TIM1

#define MOTOR1_POSITION	&motor1_encoder
#define MOTOR2_POSITION &motor2_encoder
#define MOTOR3_POSITION &motor3_encoder
#define MOTOR4_POSITION &motor4_encoder
#define MOTOR5_POSITION &motor5_encoder
#define MOTOR6_POSITION &motor6_encoder
#define MOTOR7_POSITION (int16_t*)&(MOTOR7_POSITION_TIM->CNT)

//#define MOTOR8_POSITION &motor8_encoder

extern TIM_HandleTypeDef htim6;

motor_t motors[7] = {
	// motor 1
	{
		.current_position=MOTOR1_POSITION,
		.ccr=&MOTOR1_PWM_TIM->MOTOR1_PWM_CCR,
		.a_port=INA1_GPIO_Port,
		.b_port=INB1_GPIO_Port,
		.a_pin=INA1_Pin,
		.b_pin=INB1_Pin,
		.ms_port=MS1_GPIO_Port,
		.ms_pin=MS1_Pin,
		.setpoint=0,
		.mode = MOTOR_MODE_RUNNING,
		.k_p = 20,
		.invert_direction = 0,
		.invert_homing_direction = 1
	},

	// motor 2
	{
		.current_position=MOTOR2_POSITION,
		.ccr=&MOTOR2_PWM_TIM->MOTOR2_PWM_CCR,
		.a_port=INA2_GPIO_Port,
		.b_port=INB2_GPIO_Port,
		.a_pin=INA2_Pin,
		.b_pin=INB2_Pin,
		.ms_port=MS2_GPIO_Port,
		.ms_pin=MS2_Pin,
		.setpoint=0,
		.mode = MOTOR_MODE_RUNNING,
		.k_p = 20,
		.invert_direction = 0,
		.invert_homing_direction = 0
	},

	// motor 3
	{
		.current_position=MOTOR3_POSITION,
		.ccr=&MOTOR3_PWM_TIM->MOTOR3_PWM_CCR,
		.a_port=INA3_GPIO_Port,
		.b_port=INB3_GPIO_Port,
		.a_pin=INA3_Pin,
		.b_pin=INB3_Pin,
		.ms_port=MS3_GPIO_Port,
		.ms_pin=MS3_Pin,
		.setpoint=0,
		.mode = MOTOR_MODE_RUNNING,
		.k_p = 20,
		.invert_direction = 0,
		.invert_homing_direction = 1
	},

	// motor 4
	{
		.current_position=MOTOR4_POSITION,
		.ccr=&MOTOR4_PWM_TIM->MOTOR4_PWM_CCR,
		.a_port=INA4_GPIO_Port,
		.b_port=INB4_GPIO_Port,
		.a_pin=INA4_Pin,
		.b_pin=INB4_Pin,
		.ms_port=MS4_GPIO_Port,
		.ms_pin=MS4_Pin,
		.setpoint=0,
		.mode = MOTOR_MODE_RUNNING,
		.k_p = 20,
		.invert_direction = 0,
		.invert_homing_direction = 1
	},

	// motor 5
	{
		.current_position=MOTOR5_POSITION,
		.ccr=&MOTOR5_PWM_TIM->MOTOR5_PWM_CCR,
		.a_port=INA5_GPIO_Port,
		.b_port=INB5_GPIO_Port,
		.a_pin=INA5_Pin,
		.b_pin=INB5_Pin,
		.ms_port=MS5_GPIO_Port,
		.ms_pin=MS5_Pin,
		.setpoint=0,
		.mode = MOTOR_MODE_RUNNING,
		.k_p = 20,
		.invert_direction = 0,
		.invert_homing_direction = 1
	},

	// motor 6
	{
		.current_position=MOTOR6_POSITION,
		.ccr=&MOTOR6_PWM_TIM->MOTOR6_PWM_CCR,
		.a_port=INA6_GPIO_Port,
		.b_port=INB6_GPIO_Port,
		.a_pin=INA6_Pin,
		.b_pin=INB6_Pin,
		.ms_port=MS6_GPIO_Port,
		.ms_pin=MS6_Pin,
		.setpoint=0,
		.mode = MOTOR_MODE_RUNNING,
		.k_p = 20,
		.invert_direction = 0,
		.invert_homing_direction = 1
	},

	// motor 7
	{
		.current_position=MOTOR7_POSITION,
		.ccr=&MOTOR7_PWM_TIM->MOTOR7_PWM_CCR,
		.a_port=INA7_GPIO_Port,
		.b_port=INB7_GPIO_Port,
		.a_pin=INA7_Pin,
		.b_pin=INB7_Pin,
		.ms_port=MS7_GPIO_Port,
		.ms_pin=MS7_Pin,
		.setpoint=0,
		.mode = MOTOR_MODE_RUNNING,
		.k_p = 20,
		.invert_direction = 0,
		.invert_homing_direction = 1
	}//,
//
//	// motor 8
//	{
//		.current_position=MOTOR8_POSITION,
//		.ccr=&MOTOR8_PWM_TIM->MOTOR8_PWM_CCR,
//		.a_port=INA8_GPIO_Port,
//		.b_port=INB8_GPIO_Port,
//		.a_pin=INA8_Pin,
//		.b_pin=INB8_Pin,
//		.ms_port=MS8_GPIO_Port,
//		.ms_pin=MS8_Pin,
//		.setpoint=0,
//		.mode = MOTOR_MODE_RUNNING,
//		.k_p = 20,
//		.invert_direction = 1,
//		.invert_homing_direction = 1
//	}

};

void motor_init()
{
	///// ENABLE ENCODERS
	// all motors are now software encoded except 7
//	// motor 1 -> software
//	// motor 2
//	TIM_CCxChannelCmd(MOTOR2_POSITION_TIM, TIM_CHANNEL_1, TIM_CCx_ENABLE);
//	MOTOR2_POSITION_TIM->CR1 |= TIM_CR1_CEN;
//
//	// motor 3
//	TIM_CCxChannelCmd(MOTOR3_POSITION_TIM, TIM_CHANNEL_1, TIM_CCx_ENABLE);
//	MOTOR3_POSITION_TIM->CR1 |= TIM_CR1_CEN;
//
//	// motor 4
//	TIM_CCxChannelCmd(MOTOR4_POSITION_TIM, TIM_CHANNEL_1, TIM_CCx_ENABLE);
//	MOTOR4_POSITION_TIM->CR1 |= TIM_CR1_CEN;
//
//	// motor 5
//	TIM_CCxChannelCmd(MOTOR5_POSITION_TIM, TIM_CHANNEL_1, TIM_CCx_ENABLE);
//	MOTOR5_POSITION_TIM->CR1 |= TIM_CR1_CEN;
//
//	// motor 6 --> software
//
//	// motor 7
	TIM_CCxChannelCmd(MOTOR7_POSITION_TIM, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	MOTOR7_POSITION_TIM->CR1 |= TIM_CR1_CEN;
//
//	// motor 8 --> software



	//// ENABLE PWMs
	// motor 1
	TIM_CCxChannelCmd(MOTOR1_PWM_TIM, MOTOR1_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR1_PWM_TIM->CR1 |= TIM_CR1_CEN;

	// motor 2
	TIM_CCxChannelCmd(MOTOR2_PWM_TIM, MOTOR2_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR2_PWM_TIM->CR1 |= TIM_CR1_CEN;

	// motor 3
	TIM_CCxChannelCmd(MOTOR3_PWM_TIM, MOTOR3_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR3_PWM_TIM->CR1 |= TIM_CR1_CEN;

	// motor 4
	TIM_CCxChannelCmd(MOTOR4_PWM_TIM, MOTOR4_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR4_PWM_TIM->CR1 |= TIM_CR1_CEN;

	// motor 5
	TIM_CCxChannelCmd(MOTOR5_PWM_TIM, MOTOR5_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR5_PWM_TIM->CR1 |= TIM_CR1_CEN;

	// motor 6
	TIM_CCxChannelCmd(MOTOR6_PWM_TIM, MOTOR6_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR6_PWM_TIM->CR1 |= TIM_CR1_CEN;

	// motor 7
	TIM_CCxChannelCmd(MOTOR7_PWM_TIM, MOTOR7_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR7_PWM_TIM->CR1 |= TIM_CR1_CEN;

	// motor 8
//	TIM_CCxChannelCmd(MOTOR8_PWM_TIM, MOTOR8_PWM_CHANNEL, TIM_CCx_ENABLE);
//	MOTOR8_PWM_TIM->CR1 |= TIM_CR1_CEN;

	HAL_TIM_Base_Start_IT(&htim6); // motor interrupt loop
}

void motor_control_loop()
{
	for(int i = 0; i < 7; i++)
	{
		motor_t* motor = &motors[i];
		if(motor->mode != MOTOR_MODE_RUNNING)
			return;

		float error = motor->setpoint - (int16_t)*(motor->current_position);

		if(motor->invert_direction)
			error = -error;

		// update direction
		if(error > 0)
		{
			// forward
			motor->a_port->ODR |= motor->a_pin;
			motor->b_port->ODR &= ~(motor->b_pin);
		} else {
			error = -error;
			// reverse
			motor->a_port->ODR &= ~(motor->a_pin);
			motor->b_port->ODR |= motor->b_pin;
		}

	//	 update speed
		float speed_update = error * motor->k_p;
		if(speed_update > 12000)
			speed_update = 12000;
		*(motor->ccr) = (uint16_t)speed_update;
	}
}

void motor_set_position(int number, int position)
{
	motors[number].setpoint = position;
}

void motor_home(int number)
{
	motor_t* motor = &motors[number];

	motor->mode = MOTOR_MODE_HOMING;

	motor->setpoint = 0;

	if(motor->invert_homing_direction)
	{
		// forward
		motor->a_port->ODR |= motor->a_pin;
		motor->b_port->ODR &= ~(motor->b_pin);
	} else {
		// reverse
		motor->a_port->ODR &= ~(motor->a_pin);
		motor->b_port->ODR |= motor->b_pin;
	}

	*(motor->ccr) = 4000;
	while(motor->ms_port->IDR & motor->ms_pin);
	*(motor->ccr) = 0;
	*(motor->current_position) = 0;

	printf("homing complete\r\n");
	motor->mode = MOTOR_MODE_RUNNING;

}
