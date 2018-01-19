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


motor_t motors[8] = {
	// motor 1
	{
		.current_position=&motor1_encoder,
		.setpoint=0,
		.mode = MOTOR_MODE_OFF
	},

	// motor 2
	{
		.current_position=(int32_t*)&(TIM2->CNT),
		.setpoint=0,
		.mode = MOTOR_MODE_OFF
	},

	// motor 3
	{
		.current_position=(int32_t*)&(TIM4->CNT),
		.setpoint=0,
		.mode = MOTOR_MODE_OFF
	},

	// motor 4
	{
		.current_position=(int32_t*)&(TIM3->CNT),
		.setpoint=0,
		.mode = MOTOR_MODE_OFF
	},

	// motor 5
	{
		.current_position=(int32_t*)&(TIM8->CNT),
		.setpoint=0,
		.mode = MOTOR_MODE_OFF
	},

	// motor 6
	{
		.current_position=&motor6_encoder,
		.setpoint=0,
		.mode = MOTOR_MODE_OFF
	},

	// motor 7
	{
		.current_position=(int32_t*)&(TIM1->CNT),
		.ccr=&TIM10->CCR1,
		.a_port=INA7_GPIO_Port,
		.b_port=INB7_GPIO_Port,
		.a_pin=INA7_Pin,
		.b_pin=INB7_Pin,
		.ms_port=MS7_GPIO_Port,
		.ms_pin=MS7_Pin,
		.setpoint=0,
		.mode = MOTOR_MODE_RUNNING,
		.k_p = 20,
		.invert_direction = 1,
		.invert_homing_direction = 1
	},

	// motor 8
	{
		.current_position=&motor8_encoder,
		.setpoint=0,
		.mode = MOTOR_MODE_OFF
	}

};

void motor_control_loop()
{
	motor_t* motor = &motors[6];
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
