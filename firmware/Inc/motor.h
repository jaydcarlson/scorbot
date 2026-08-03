/*
 * motor.h
 *
 *  Created on: Jan 12, 2018
 *      Author: jay
 *
 * Per-motor state and the closed-loop controller that runs in the TIM6
 * interrupt at MOTOR_LOOP_HZ. Everything here works in encoder counts; the
 * joint layer above converts to and from degrees.
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

/*
 * TIM6 is fed from an 84 MHz APB1 timer clock with a prescaler of 8400 and a
 * period of 2 counts, so the control loop runs at 5 kHz.
 */
#define MOTOR_LOOP_HZ 5000.0f
#define MOTOR_LOOP_DT (1.0f / MOTOR_LOOP_HZ)

typedef enum motor_mode {
	MOTOR_CONTROL_MODE_OFF,       /* coasting, no drive                     */
	MOTOR_CONTROL_MODE_BUSY,      /* reserved                               */
	MOTOR_CONTROL_MODE_PWM,       /* open loop duty, used by homing         */
	MOTOR_CONTROL_MODE_POSITION,  /* closed loop on position_setpoint       */
} motor_control_mode_t;

typedef struct motor {
	volatile uint32_t* ccr;
	volatile GPIO_TypeDef* a_port;
	volatile GPIO_TypeDef* b_port;

	/*
	 * Raw hardware counter. The software quadrature counters and TIM1 are both
	 * 16-bit, so this wraps; position below accumulates the deltas into 32 bits
	 * so travel is not limited to +/-32767 counts. At 20000 counts per turn on
	 * the shoulder the raw counter would otherwise wrap inside two revolutions.
	 */
	volatile int16_t* raw_counter;
	int16_t last_raw;
	int32_t position;        /* accumulated counts, wrap-free               */

	int32_t position_setpoint;
	float current_velocity;  /* counts per second, from a real derivative   */
	float current_effort;    /* last commanded duty, -1.0 .. 1.0            */

	uint16_t a_pin;
	uint16_t b_pin;
	motor_control_mode_t control_mode;
	uint8_t invert_direction_pins;
	uint8_t invert_counter;
	uint8_t saturated;

	/* PID, all in the counts domain. The joint layer scales from degrees. */
	float k_p;
	float k_d;
	float k_i;
	float i_accum;
	float i_clamp;
	float out_clamp;
	float deadband;          /* counts of error below which output is zero  */
	float max_vel;           /* counts per second setpoint slew limit       */
	float ramped_setpoint;   /* slew-limited setpoint actually tracked      */

	float pwm_command;       /* open loop duty when in PWM mode             */
} motor_t;

#define NUM_MOTORS 7

extern motor_t motors[NUM_MOTORS];

void motor_init();
void motor_control_loop();
void motor_set_position(motor_t* motor, int32_t position);
void motor_set_pwm(motor_t* motor, float pwm);
void motor_set_control_mode(motor_t* motor, motor_control_mode_t control_mode);
int32_t motor_get_current_position(motor_t* motor);
void motor_set_encoder_value(motor_t* motor, int32_t new_position);

float motor_get_current_velocity(motor_t* motor);
float motor_get_current_effort(motor_t* motor);

#endif /* MOTOR_H_ */
