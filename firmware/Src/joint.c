#include "joint.h"
#include "main.h"
#include <math.h>

#define HOMING_SPEED (1.0f)

joint_t joints[NUM_JOINTS] = {
    {
        .primary_motor = &motors[0],
        .ms_port = MS1_GPIO_Port,
        .ms_pin = MS1_Pin,
        .gear_ratio = (6500.0f / 90.0f),
        .max_angle = 160.0f,
        .min_angle = -130.0f
    },
    {
        .primary_motor = &motors[1],
        .ms_port = MS2_GPIO_Port,
        .ms_pin = MS2_Pin,
        .gear_ratio = (20000.0f / 360.0f),
        .max_angle = 90.0f,
        .min_angle = 0.0f
    },
    {
        .primary_motor = &motors[2],
        .ms_port = MS3_GPIO_Port,
        .ms_pin = MS3_Pin,
        .gear_ratio = (5000.0f / 90.0f),
        .max_angle = 160.0f,
        .min_angle = -130.0f
    },
    {
        .primary_motor = &motors[4],
        .coupled_motor = &motors[3],
        .coupling_factor = -1.0f,
        .ms_port = MS4_GPIO_Port,
        .ms_pin = MS4_Pin,
        .gear_ratio = (1200.0f / 90.0f),
        .max_angle = INFINITY,
        .min_angle = -INFINITY
    },
    {
        .primary_motor = &motors[3],
        .coupled_motor = &motors[4],
        .coupling_factor = 1.0f,
        .ms_port = MS5_GPIO_Port,
        .ms_pin = MS5_Pin,
        .gear_ratio = (1200.0f / 90.0f),
        .max_angle = INFINITY,
        .min_angle = -INFINITY
    },
    {
        .primary_motor = &motors[5],
        .ms_port = MS6_GPIO_Port,
        .ms_pin = MS6_Pin,
        .gear_ratio = 1.0f,
        .max_angle = INFINITY,
        .min_angle = -INFINITY
    }
};

void joint_home(joint_t* joint)
{
    // TODO: we *really* should be doing this by walking the position setpoints backwards until we hit the limit switch
    // since for coupled motors, we're going to get drift since our "speed controller" is just a PWM signal
    // save previous control mode
    motor_control_mode_t previous_control_mode = joint->primary_motor->control_mode;
    motor_control_mode_t previous_coupled_control_mode = joint->coupled_motor->control_mode;

    motor_set_control_mode(joint->primary_motor, MOTOR_CONTROL_MODE_PWM);
    motor_set_pwm(joint->primary_motor, -HOMING_SPEED);
    if(joint->coupled_motor != NULL) {
        motor_set_control_mode(joint->coupled_motor, MOTOR_CONTROL_MODE_PWM);
        motor_set_pwm(joint->coupled_motor, -HOMING_SPEED * joint->coupling_factor);
    }
    
    // wait for limit switch to be hit
    while(joint->ms_port->IDR & joint->ms_pin);

    motor_set_pwm(joint->primary_motor, 0.0f);
    motor_set_encoder_value(joint->primary_motor, 0);
    joint_set_angle(joint, 0.0f);
    motor_set_control_mode(joint->primary_motor, previous_control_mode);
    if(joint->coupled_motor != NULL) {
        motor_set_pwm(joint->coupled_motor, 0.0f);
        motor_set_control_mode(joint->coupled_motor, previous_coupled_control_mode);
    // we want to drive both the motors but we don't want to home the coupled motor.
    //     motor_set_encoder_value(joint->coupled_motor, 0);
    // instead, we want to save whatever position the coupled motor ended up at as the coupling offset
    joint->coupling_offset = motor_get_current_position(joint->coupled_motor);
    }

    // this shouldn't move the motor, but hold its current position
    joint_set_angle(joint, 0);
}

void joint_set_angle(joint_t* joint, float angle)
{
    // clip angle to max and min
    angle = fmin(angle, joint->max_angle);
    angle = fmax(angle, joint->min_angle);

    motor_set_position(joint->primary_motor, angle * joint->gear_ratio);
    if(joint->coupled_motor != NULL) {
        motor_set_position(joint->coupled_motor, angle * joint->gear_ratio * joint->coupling_factor + joint->coupling_offset);
    }
}

