#pragma once

#include "motor.h"
#include "scorbot_proto.h"

/*
 * A joint is the unit the protocol talks about: an angle in degrees, its travel
 * limits, its limit switch, and one or two motors underneath. The wrist is two
 * mechanically coupled motors driven together, which is why coupling_factor and
 * coupling_offset exist.
 */
typedef struct {
    char name[SCORBOT_NAME_LEN];
    motor_t* primary_motor;
    motor_t* coupled_motor;
    float coupling_factor;
    int32_t coupling_offset;
    float gear_ratio;        /* encoder counts per degree                   */
    float max_angle;
    float min_angle;
    uint16_t ms_pin;
    volatile GPIO_TypeDef* ms_port;

    uint8_t mode;            /* enum scorbot_joint_mode                     */
    uint8_t homed;
    uint8_t faulted;
    float setpoint;          /* degrees, degrees/second, or duty, per mode  */
    float max_speed_dps;     /* full-duty speed, used to scale velocity mode */

    scorbot_gains_t gains;         /* as presented to the host, in degrees  */
    scorbot_homing_cfg_t homing;

    /* Runaway protection; see joint_safety_tick. */
    uint32_t straining_since_ms;
    uint8_t straining;
} joint_t;

#define NUM_JOINTS 7

/*
 * The wrist is one differential mechanism, not two independent joints: motors 3
 * and 4 are driven opposed to roll and in unison to pitch. Neither axis can be
 * commanded on its own, so these two indices are always resolved together.
 */
#define JOINT_WRIST_ROLL 3
#define JOINT_WRIST_PITCH 4

extern joint_t joints[NUM_JOINTS];

void joint_init(void);

void joint_set_mode(joint_t* joint, uint8_t mode);
void joint_set_setpoint(joint_t* joint, float value);

/* Pushes the joint's mode and setpoint down onto its motor or motors. */
void joint_apply(joint_t* joint);

/*
 * Pushes every joint down to the motors at once.
 *
 * Always prefer this over looping joint_apply yourself. Applying the two wrist
 * joints independently makes them fight over motors 3 and 4, and whichever is
 * applied last silently wins: commanding roll while pitch sits idle drives the
 * motors and then immediately stops them again, which looks exactly like a dead
 * motor. This resolves the pair as a superposition instead.
 */
void joint_apply_all(void);

/*
 * Runaway protection. Call periodically with a millisecond tick.
 *
 * A joint whose encoder has failed reports an error that never shrinks, so the
 * position loop winds up, saturates, and drives into whatever it is jammed
 * against for as long as the setpoint stands. This faults any joint that is
 * pushing hard and demonstrably not moving, which covers a dead encoder, a
 * seized joint and an unplugged motor alike.
 *
 * It deliberately does not fire on a joint holding station under gravity: that
 * needs effort at zero velocity too, and is distinguished by its position error
 * being small.
 */
void joint_safety_tick(uint32_t now_ms);

/* Clears a joint fault latched by joint_safety_tick. */
void joint_clear_fault(joint_t* joint);

float joint_get_angle(joint_t* joint);
float joint_get_velocity(joint_t* joint);
float joint_get_effort(joint_t* joint);

/* Declares the joint's present position to be the given angle. */
void joint_zero(joint_t* joint, float angle);

void joint_set_gains(joint_t* joint, const scorbot_gains_t* gains);
void joint_get_gains(joint_t* joint, scorbot_gains_t* out);

/* Limit switches are active low with a pull-up, so a closed switch reads 0. */
static inline uint8_t joint_limit_active(joint_t* joint)
{
    if(joint->ms_port == 0) {
        return 0;
    }
    return (joint->ms_port->IDR & joint->ms_pin) ? 0u : 1u;
}

/* Drives both motors of the joint open loop, for the homing state machine. */
void joint_drive_duty(joint_t* joint, float duty);
void joint_stop(joint_t* joint);
