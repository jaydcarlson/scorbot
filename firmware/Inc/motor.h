/*
 * motor.h
 *
 *  Created on: Jan 12, 2018
 *      Author: jay
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef enum motor_mode {
	MOTOR_MODE_OFF,
	MOTOR_MODE_BUSY,
	MOTOR_MODE_RUNNING,
	MOTOR_MODE_HOMING
} motor_mode_t;

typedef struct motor {
	volatile uint32_t* ccr;
	volatile GPIO_TypeDef* a_port;
	volatile GPIO_TypeDef* b_port;
	volatile int32_t* current_position;
	int32_t setpoint;
	uint16_t a_pin;
	uint16_t b_pin;
	motor_mode_t mode;
	int invert_direction;
	float k_p;
} motor_t;


void motor_control_loop();
void motor_set_position(motor_t* position);

#endif /* MOTOR_H_ */
