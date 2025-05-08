#pragma once

#include "motor.h"

typedef struct {
    motor_t* primary_motor;
    motor_t* coupled_motor;
    float coupling_factor;
    float gear_ratio;
    float angle_setpoint;
    float angle_actual;
	uint16_t ms_pin;
	volatile GPIO_TypeDef* ms_port;
} joint_t;

void joint_home(joint_t* joint);
void joint_set_angle(joint_t* joint, float angle);

#define NUM_JOINTS 6

extern joint_t joints[NUM_JOINTS];