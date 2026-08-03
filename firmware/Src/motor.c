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

/*
 * Velocity is differentiated from encoder counts every 200 us, which is far too
 * noisy to feed a derivative term directly. This first-order filter has roughly
 * a 40 Hz corner at the 5 kHz loop rate.
 */
#define VELOCITY_FILTER_ALPHA 0.05f

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

#define MOTOR7_POSITION_TIM		TIM1

#define MOTOR1_POSITION	&motor1_encoder
#define MOTOR2_POSITION &motor2_encoder
#define MOTOR3_POSITION &motor3_encoder
#define MOTOR4_POSITION &motor4_encoder
#define MOTOR5_POSITION &motor5_encoder
#define MOTOR6_POSITION &motor6_encoder
#define MOTOR7_POSITION (int16_t*)&(MOTOR7_POSITION_TIM->CNT)

extern TIM_HandleTypeDef htim6;

/*
 * Default gains reproduce the original proportional-only behaviour: 0.1 duty
 * per count of error, so ten counts commands full scale. The integral and
 * derivative terms start disabled and are tuned from the host.
 */
#define DEFAULT_KP 0.1f
#define DEFAULT_KI 0.0f
#define DEFAULT_KD 0.0f
#define DEFAULT_I_CLAMP 0.25f
#define DEFAULT_OUT_CLAMP 1.0f
#define DEFAULT_DEADBAND 2.0f
#define DEFAULT_MAX_VEL 0.0f   /* zero disables setpoint slew limiting */

#define MOTOR_DEFAULTS \
	.k_p = DEFAULT_KP, \
	.k_i = DEFAULT_KI, \
	.k_d = DEFAULT_KD, \
	.i_clamp = DEFAULT_I_CLAMP, \
	.out_clamp = DEFAULT_OUT_CLAMP, \
	.deadband = DEFAULT_DEADBAND, \
	.max_vel = DEFAULT_MAX_VEL

motor_t motors[NUM_MOTORS] = {
	// motor 0 -> base rotate
	{
		.raw_counter=MOTOR1_POSITION,
		.ccr=&MOTOR1_PWM_TIM->MOTOR1_PWM_CCR,
		.a_port=INA1_GPIO_Port,
		.b_port=INB1_GPIO_Port,
		.a_pin=INA1_Pin,
		.b_pin=INB1_Pin,
		.control_mode = MOTOR_CONTROL_MODE_OFF,
		.invert_direction_pins = 0,
		.invert_counter = 0,
		MOTOR_DEFAULTS
	},

	// motor 1 -> lower link
	{
		.raw_counter=MOTOR2_POSITION,
		.ccr=&MOTOR2_PWM_TIM->MOTOR2_PWM_CCR,
		.a_port=INA2_GPIO_Port,
		.b_port=INB2_GPIO_Port,
		.a_pin=INA2_Pin,
		.b_pin=INB2_Pin,
		.control_mode = MOTOR_CONTROL_MODE_OFF,
		.invert_direction_pins = 0,
		.invert_counter = 0,
		MOTOR_DEFAULTS
	},

	// motor 2 -> middle link
	{
		.raw_counter=MOTOR3_POSITION,
		.ccr=&MOTOR3_PWM_TIM->MOTOR3_PWM_CCR,
		.a_port=INA3_GPIO_Port,
		.b_port=INB3_GPIO_Port,
		.a_pin=INA3_Pin,
		.b_pin=INB3_Pin,
		.control_mode = MOTOR_CONTROL_MODE_OFF,
		.invert_direction_pins = 1,
		.invert_counter = 1,
		MOTOR_DEFAULTS
	},

	// motor 3 -> wrist #1
	{
		.raw_counter=MOTOR4_POSITION,
		.ccr=&MOTOR4_PWM_TIM->MOTOR4_PWM_CCR,
		.a_port=INA4_GPIO_Port,
		.b_port=INB4_GPIO_Port,
		.a_pin=INA4_Pin,
		.b_pin=INB4_Pin,
		.control_mode = MOTOR_CONTROL_MODE_OFF,
		.invert_direction_pins = 0,
		.invert_counter = 0,
		MOTOR_DEFAULTS
	},

	// motor 4 -> wrist #2
	{
		.raw_counter=MOTOR5_POSITION,
		.ccr=&MOTOR5_PWM_TIM->MOTOR5_PWM_CCR,
		.a_port=INA5_GPIO_Port,
		.b_port=INB5_GPIO_Port,
		.a_pin=INA5_Pin,
		.b_pin=INB5_Pin,
		.control_mode = MOTOR_CONTROL_MODE_OFF,
		.invert_direction_pins = 0,
		.invert_counter = 0,
		MOTOR_DEFAULTS
	},

	// motor 5 -> open/close
	{
		.raw_counter=MOTOR6_POSITION,
		.ccr=&MOTOR6_PWM_TIM->MOTOR6_PWM_CCR,
		.a_port=INA6_GPIO_Port,
		.b_port=INB6_GPIO_Port,
		.a_pin=INA6_Pin,
		.b_pin=INB6_Pin,
		.control_mode = MOTOR_CONTROL_MODE_OFF,
		.invert_direction_pins = 1,
		.invert_counter = 1,
		MOTOR_DEFAULTS
	},

	// motor 6 -> linear rail
	{
		.raw_counter=MOTOR7_POSITION,
		.ccr=&MOTOR7_PWM_TIM->MOTOR7_PWM_CCR,
		.a_port=INA7_GPIO_Port,
		.b_port=INB7_GPIO_Port,
		.a_pin=INA7_Pin,
		.b_pin=INB7_Pin,
		.control_mode = MOTOR_CONTROL_MODE_OFF,
		.invert_direction_pins = 1,
		.invert_counter = 1,
		MOTOR_DEFAULTS
	}
};

static inline void motor_forward(motor_t* motor)
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

static inline void motor_reverse(motor_t* motor)
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

/*
 * Sign conventions, which are easy to get wrong and expensive when you do.
 *
 * motor->position is the RAW accumulated hardware count. Everything else -
 * setpoints, PWM commands, velocity, and the whole control loop - works in the
 * LOGICAL frame, where increasing values always mean increasing joint angle and
 * a positive duty always drives that way.
 *
 * invert_counter therefore describes one physical fact only: that this motor's
 * encoder counts down when the motor is driven forward. It is applied here,
 * where raw hardware is read, and nowhere else. Applying it to the drive as
 * well cancels it out, which silently reverses the three motors that have it
 * set and sends homing into a hard stop instead of toward the limit switch.
 */
static inline int32_t motor_logical_position(motor_t* motor)
{
	return motor->invert_counter ? -motor->position : motor->position;
}

int32_t motor_get_current_position(motor_t* motor)
{
	return motor_logical_position(motor);
}

float motor_get_current_velocity(motor_t* motor)
{
	return motor->current_velocity;  /* already logical */
}

float motor_get_current_effort(motor_t* motor)
{
	return motor->current_effort;
}

void motor_set_encoder_value(motor_t* motor, int32_t new_position)
{
	/*
	 * Re-reference the accumulator rather than writing the hardware counter,
	 * which keeps the wrap arithmetic below consistent and works for TIM1 too.
	 */
	motor->position = motor->invert_counter ? -new_position : new_position;
	motor->last_raw = *(motor->raw_counter);
	motor->ramped_setpoint = (float)new_position;
	motor->position_setpoint = new_position;
	motor->i_accum = 0.0f;
	motor->current_velocity = 0.0f;
}

/*
 * Fold the 16-bit hardware counter into the 32-bit accumulator. The subtraction
 * is done in int16_t so it wraps correctly; at 5 kHz no motor can move more
 * than half a counter period between samples.
 */
static inline void motor_update_encoder(motor_t* motor)
{
	int16_t raw = *(motor->raw_counter);
	int16_t delta = (int16_t)(raw - motor->last_raw);
	motor->last_raw = raw;
	motor->position += (int32_t)delta;

	const float logical_delta = motor->invert_counter ? -(float)delta : (float)delta;
	const float instantaneous = logical_delta * MOTOR_LOOP_HZ;
	motor->current_velocity +=
		VELOCITY_FILTER_ALPHA * (instantaneous - motor->current_velocity);
}

static inline void motor_apply_duty(motor_t* motor, float duty)
{
	if(duty > 0.0f)
	{
		motor_forward(motor);
	} else {
		motor_reverse(motor);
	}

	float magnitude = fabsf(duty);
	if(magnitude > 1.0f) {
		magnitude = 1.0f;
	}
	motor->current_effort = duty;
	*(motor->ccr) = (uint16_t)(magnitude * MAX_PWM_VAL);
}

static float motor_position_pid(motor_t* motor)
{
	/* Slew limit the setpoint so a large step does not command a lurch. */
	const float target = (float)motor->position_setpoint;
	if(motor->max_vel > 0.0f) {
		const float step = motor->max_vel * MOTOR_LOOP_DT;
		float delta = target - motor->ramped_setpoint;
		if(delta > step) {
			delta = step;
		} else if(delta < -step) {
			delta = -step;
		}
		motor->ramped_setpoint += delta;
	} else {
		motor->ramped_setpoint = target;
	}

	const float error = motor->ramped_setpoint - (float)motor_logical_position(motor);
	if(fabsf(error) < motor->deadband) {
		motor->i_accum = 0.0f;
		motor->saturated = 0;
		return 0.0f;
	}

	const float integral_step = motor->k_i * error * MOTOR_LOOP_DT;
	motor->i_accum += integral_step;
	if(motor->i_accum > motor->i_clamp) {
		motor->i_accum = motor->i_clamp;
	} else if(motor->i_accum < -motor->i_clamp) {
		motor->i_accum = -motor->i_clamp;
	}

	/*
	 * Derivative on the measurement rather than on the error: differentiating
	 * the error would kick hard every time the host moves the setpoint.
	 */
	const float derivative = -motor->k_d * motor->current_velocity;

	float out = (motor->k_p * error) + motor->i_accum + derivative;

	if(out > motor->out_clamp) {
		out = motor->out_clamp;
		motor->saturated = 1;
		/* Stop winding up while the output is pinned. */
		motor->i_accum -= integral_step;
	} else if(out < -motor->out_clamp) {
		out = -motor->out_clamp;
		motor->saturated = 1;
		motor->i_accum -= integral_step;
	} else {
		motor->saturated = 0;
	}

	return out;
}

void motor_init()
{
	// all motors are now software encoded except 7
	TIM_CCxChannelCmd(MOTOR7_POSITION_TIM, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	MOTOR7_POSITION_TIM->CR1 |= TIM_CR1_CEN;

	//// ENABLE PWMs
	TIM_CCxChannelCmd(MOTOR1_PWM_TIM, MOTOR1_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR1_PWM_TIM->CR1 |= TIM_CR1_CEN;

	TIM_CCxChannelCmd(MOTOR2_PWM_TIM, MOTOR2_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR2_PWM_TIM->CR1 |= TIM_CR1_CEN;

	TIM_CCxChannelCmd(MOTOR3_PWM_TIM, MOTOR3_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR3_PWM_TIM->CR1 |= TIM_CR1_CEN;

	TIM_CCxChannelCmd(MOTOR4_PWM_TIM, MOTOR4_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR4_PWM_TIM->CR1 |= TIM_CR1_CEN;

	TIM_CCxChannelCmd(MOTOR5_PWM_TIM, MOTOR5_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR5_PWM_TIM->CR1 |= TIM_CR1_CEN;

	TIM_CCxChannelCmd(MOTOR6_PWM_TIM, MOTOR6_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR6_PWM_TIM->CR1 |= TIM_CR1_CEN;

	TIM_CCxChannelCmd(MOTOR7_PWM_TIM, MOTOR7_PWM_CHANNEL, TIM_CCx_ENABLE);
	MOTOR7_PWM_TIM->CR1 |= TIM_CR1_CEN;

	/* Seed the encoder accumulators and leave every motor coasting. */
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		motors[i].last_raw = *(motors[i].raw_counter);
		motors[i].position = 0;
		motors[i].position_setpoint = 0;
		motors[i].ramped_setpoint = 0.0f;
		motors[i].control_mode = MOTOR_CONTROL_MODE_OFF;
		motor_apply_duty(&motors[i], 0.0f);
	}

	HAL_TIM_Base_Start_IT(&htim6); // motor interrupt loop
}

void motor_control_loop()
{
	/* Diagnostic only: counts raw edges on every encoder pin at loop rate. */
	encoder_debug_sample();

	for(int i = 0; i < NUM_MOTORS; i++)
	{
		motor_t* motor = &motors[i];

		/*
		 * The encoder is folded on every pass regardless of mode. Position must
		 * stay truthful even for a coasting or back-driven motor, otherwise the
		 * reported angle silently drifts away from reality.
		 */
		motor_update_encoder(motor);

		switch(motor->control_mode)
		{
		case MOTOR_CONTROL_MODE_POSITION:
			motor_apply_duty(motor, motor_position_pid(motor));
			break;

		case MOTOR_CONTROL_MODE_PWM:
			/* Open loop. Keep the loop's own state tracking reality so that
			 * switching back to position control does not jump. */
			motor->i_accum = 0.0f;
			motor->ramped_setpoint = (float)motor_logical_position(motor);
			motor->position_setpoint = motor_logical_position(motor);
			motor_apply_duty(motor, motor->pwm_command);
			break;

		case MOTOR_CONTROL_MODE_OFF:
		case MOTOR_CONTROL_MODE_BUSY:
		default:
			motor->i_accum = 0.0f;
			motor->ramped_setpoint = (float)motor_logical_position(motor);
			motor->position_setpoint = motor_logical_position(motor);
			motor->saturated = 0;
			motor_apply_duty(motor, 0.0f);
			break;
		}
	}
}

/* Both take logical-frame values; see the sign-convention note above. */
void motor_set_position(motor_t* motor, int32_t position)
{
	motor->position_setpoint = position;
}

void motor_set_pwm(motor_t* motor, float pwm)
{
	motor->pwm_command = pwm;
}

void motor_set_control_mode(motor_t* motor, motor_control_mode_t control_mode)
{
	if(motor->control_mode == control_mode) {
		return;
	}
	/* Entering position control from anywhere else starts from where we are. */
	if(control_mode == MOTOR_CONTROL_MODE_POSITION) {
		motor->ramped_setpoint = (float)motor_logical_position(motor);
		motor->i_accum = 0.0f;
	}
	if(control_mode != MOTOR_CONTROL_MODE_PWM) {
		motor->pwm_command = 0.0f;
	}
	motor->control_mode = control_mode;
}
