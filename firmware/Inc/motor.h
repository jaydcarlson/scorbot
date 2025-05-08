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
	MOTOR_CONTROL_MODE_OFF,
	MOTOR_CONTROL_MODE_BUSY,
	MOTOR_CONTROL_MODE_PWM,
	MOTOR_CONTROL_MODE_POSITION,
} motor_control_mode_t;


typedef struct motor {
	volatile uint32_t* ccr;
	volatile GPIO_TypeDef* a_port;
	volatile GPIO_TypeDef* b_port;
	volatile int16_t* current_position;
	int32_t position_setpoint;
	uint16_t a_pin;
	uint16_t b_pin;
	uint16_t homing_speed;
	motor_control_mode_t control_mode;
	uint8_t invert_direction_pins;
	uint8_t invert_counter;
	float k_p;
} motor_t;

#define NUM_MOTORS 7

extern motor_t motors[NUM_MOTORS];

void motor_init();
void motor_control_loop();
void motor_set_position(motor_t* motor, int position);
void motor_set_pwm(motor_t* motor, float pwm);
void motor_set_control_mode(motor_t* motor, motor_control_mode_t control_mode);
int16_t motor_get_current_position(motor_t* motor);
void motor_set_encoder_value(motor_t* motor, int16_t new_position);
#endif /* MOTOR_H_ */
