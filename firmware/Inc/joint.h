#pragma once

#include "motor.h"

typedef enum {
    JOINT_CMD_NONE,
    JOINT_CMD_SET_PASSIVE,
    JOINT_CMD_HOME,
    JOINT_CMD_SET_ANGLE,
    JOINT_CMD_SET_VELOCITY,
    JOINT_CMD_SET_TORQUE
} joint_cmd_opcode_t;

typedef struct __attribute__((packed)) {
    joint_cmd_opcode_t opcode;
    float value;
} joint_cmd_t;

typedef struct __attribute__((packed)) {
    float angle;
    float velocity;
    float torque;
} joint_status_t;

typedef struct {
    char name[16];
    motor_t* primary_motor;
    motor_t* coupled_motor;
    float coupling_factor;
    int32_t coupling_offset;
    float gear_ratio;
    float angle_setpoint;
    float angle_actual;
    float max_angle;
    float min_angle;
	uint16_t ms_pin;
	volatile GPIO_TypeDef* ms_port;
} joint_t;

void joint_home(joint_t* joint);
void joint_set_angle(joint_t* joint, float angle);
void joint_set_velocity(joint_t* joint, float velocity);
void joint_set_torque(joint_t* joint, float torque);
void joint_set_passive(joint_t* joint);

void joint_execute_cmd(joint_t* joint, joint_cmd_t* cmd);

void joint_get_status(joint_t* joint, joint_status_t* status);

#define NUM_JOINTS 6

extern joint_t joints[NUM_JOINTS];