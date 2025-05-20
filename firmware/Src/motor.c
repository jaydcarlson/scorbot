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
#include <math.h>



#define MAX_PWM_VAL			4199

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

motor_t motors[NUM_MOTORS] = {
	// motor 0 -> base rotate
	{
		.current_position=MOTOR1_POSITION,
		.ccr=&MOTOR1_PWM_TIM->MOTOR1_PWM_CCR,
		.a_port=INA1_GPIO_Port,
		.b_port=INB1_GPIO_Port,
		.a_pin=INA1_Pin,
		.b_pin=INB1_Pin,
		.control_mode = MOTOR_CONTROL_MODE_POSITION,
		.k_p = 0.1,
		.homing_speed = 8000,
		.invert_direction_pins = 0,
		.invert_counter = 0
	},

	// motor 1 -> lower link
	{
		.current_position=MOTOR2_POSITION,
		.ccr=&MOTOR2_PWM_TIM->MOTOR2_PWM_CCR,
		.a_port=INA2_GPIO_Port,
		.b_port=INB2_GPIO_Port,
		.a_pin=INA2_Pin,
		.b_pin=INB2_Pin,
		.homing_speed = 4000,
		.control_mode = MOTOR_CONTROL_MODE_POSITION,
		.k_p = 0.1,
		.invert_direction_pins = 0,
		.invert_counter = 0
	},

	// motor 2 -> middle link
	{
		.current_position=MOTOR3_POSITION,
		.ccr=&MOTOR3_PWM_TIM->MOTOR3_PWM_CCR,
		.a_port=INA3_GPIO_Port,
		.b_port=INB3_GPIO_Port,
		.a_pin=INA3_Pin,
		.b_pin=INB3_Pin,
		.homing_speed = 4000,
		.control_mode = MOTOR_CONTROL_MODE_POSITION,
		.k_p = 0.1,
		.invert_direction_pins = 1,
		.invert_counter = 1
	},

	// motor 3 -> wrist #1
	{
		.current_position=MOTOR4_POSITION,
		.ccr=&MOTOR4_PWM_TIM->MOTOR4_PWM_CCR,
		.a_port=INA4_GPIO_Port,
		.b_port=INB4_GPIO_Port,
		.a_pin=INA4_Pin,
		.b_pin=INB4_Pin,
		.homing_speed = 4000,
		.control_mode = MOTOR_CONTROL_MODE_POSITION,
		.k_p = 0.1,
		.invert_direction_pins = 0,
		.invert_counter = 0
	},

	// motor 4 -> wrist #2
	{
		.current_position=MOTOR5_POSITION,
		.ccr=&MOTOR5_PWM_TIM->MOTOR5_PWM_CCR,
		.a_port=INA5_GPIO_Port,
		.b_port=INB5_GPIO_Port,
		.a_pin=INA5_Pin,
		.b_pin=INB5_Pin,
		.homing_speed = 4000,
		.control_mode = MOTOR_CONTROL_MODE_POSITION,
		.k_p = 0.1,
		.invert_direction_pins = 0,
		.invert_counter = 0
	},

	// motor 5 -> open/close
	{
		.current_position=MOTOR6_POSITION,
		.ccr=&MOTOR6_PWM_TIM->MOTOR6_PWM_CCR,
		.a_port=INA6_GPIO_Port,
		.b_port=INB6_GPIO_Port,
		.a_pin=INA6_Pin,
		.b_pin=INB6_Pin,
		.homing_speed = 4000,
		.control_mode = MOTOR_CONTROL_MODE_POSITION,
		.k_p = 0.1,
		.invert_direction_pins = 1,
		.invert_counter = 1
	},

	// motor 6 -> linear rail
	{
		.current_position=MOTOR7_POSITION,
		.ccr=&MOTOR7_PWM_TIM->MOTOR7_PWM_CCR,
		.a_port=INA7_GPIO_Port,
		.b_port=INB7_GPIO_Port,
		.a_pin=INA7_Pin,
		.b_pin=INB7_Pin,
		.homing_speed = 4000,
		.control_mode = MOTOR_CONTROL_MODE_POSITION,
		.k_p = 0.10,
		.invert_direction_pins = 1,
		.invert_counter = 1
	}
};

inline void motor_forward(motor_t* motor)
{
	if(motor->invert_direction_pins)
	{
		motor->a_port->ODR &= ~(motor->a_pin);
		motor->b_port->ODR |= motor->b_pin;
	} else {
		motor->a_port->ODR |= motor->a_pin;
		motor->b_port->ODR &= ~(motor->b_pin);
	}
}

inline void motor_reverse(motor_t* motor)
{
	if(motor->invert_direction_pins)
	{
		motor->a_port->ODR |= motor->a_pin;
		motor->b_port->ODR &= ~(motor->b_pin);
	} else {
		motor->a_port->ODR &= ~(motor->a_pin);
		motor->b_port->ODR |= motor->b_pin;
	}
}

int16_t motor_get_current_position(motor_t* motor)
{
	int16_t position = *(motor->current_position);
	if(motor->invert_counter)
		position = -position;

	return position;
}

float motor_get_current_velocity(motor_t* motor)
{
	return motor->current_velocity;
}

float motor_get_current_torque(motor_t* motor)
{
	return motor->current_torque;
}

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
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		motor_t* motor = &motors[i];
		float error = motor->position_setpoint - motor_get_current_position(motor);

		motor->current_velocity = error / 1000.0f;

		if(motor->control_mode != MOTOR_CONTROL_MODE_POSITION)
			return; // if we're homing or whatever, don't run the loop

		motor_set_pwm(motor, error * motor->k_p);



	}
}

void motor_set_position(motor_t* motor, int position)
{
	motor->position_setpoint = position;
}

void motor_set_encoder_value(motor_t* motor, int16_t new_position)
{
	*(motor->current_position) = new_position;
}

void motor_set_pwm(motor_t* motor, float pwm)
{
	if(pwm > 0)
	{
		motor_forward(motor);
	} else {
		motor_reverse(motor);
	}

	pwm = fmin(fabs(pwm), 1.0f);
	*(motor->ccr) = (uint16_t)(pwm * MAX_PWM_VAL);
}

void motor_set_control_mode(motor_t* motor, motor_control_mode_t control_mode)
{
	motor->control_mode = control_mode;
}


