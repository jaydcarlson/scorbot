#include "joint.h"
#include "main.h"


#define HOMING_SPEED (1.0f)

joint_t joints[NUM_JOINTS] = {
    {
        .primary_motor = &motors[0],
        .ms_port = MS1_GPIO_Port,
        .ms_pin = MS1_Pin,
        .gear_ratio = (6500.0f / 90.0f)
    },
    {
        .primary_motor = &motors[1],
        .ms_port = MS2_GPIO_Port,
        .ms_pin = MS2_Pin,
        .gear_ratio = (20000.0f / 360.0f)
    },
    {
        .primary_motor = &motors[2],
        .ms_port = MS3_GPIO_Port,
        .ms_pin = MS3_Pin,
        .gear_ratio = 1.0f
    },
    {
        .primary_motor = &motors[4],
        .coupled_motor = &motors[3],
        .coupling_factor = -1.0f,
        .ms_port = MS4_GPIO_Port,
        .ms_pin = MS4_Pin,
        .gear_ratio = 1.0f
    },
    {
        .primary_motor = &motors[3],
        .coupled_motor = &motors[4],
        .coupling_factor = 1.0f,
        .ms_port = MS5_GPIO_Port,
        .ms_pin = MS5_Pin,
        .gear_ratio = 1.0f
    },
    {
        .primary_motor = &motors[5],
        .ms_port = MS6_GPIO_Port,
        .ms_pin = MS6_Pin,
        .gear_ratio = 1.0f
    }
};

void joint_home(joint_t* joint)
{
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
        motor_set_encoder_value(joint->coupled_motor, 0);
        motor_set_control_mode(joint->coupled_motor, previous_coupled_control_mode);
    }
}

void joint_set_angle(joint_t* joint, float angle)
{
    motor_set_position(joint->primary_motor, angle * joint->gear_ratio);
    if(joint->coupled_motor != NULL) {
        motor_set_position(joint->coupled_motor, angle * joint->gear_ratio * joint->coupling_factor);
    }
}

