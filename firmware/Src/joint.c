#include "joint.h"
#include "main.h"
#include <math.h>
#include <string.h>

/*
 * Runaway thresholds. A joint has to be pushing meaningfully hard, be visibly
 * not moving, and still be far from its target, continuously for the whole
 * window, before it is faulted. All three together are what separate a genuine
 * runaway from a joint legitimately holding its position under load.
 */
#define STALL_EFFORT_MIN 0.30f      /* duty considered "pushing hard"       */
#define STALL_VELOCITY_MAX 0.75f    /* deg/s considered "not moving"        */
#define STALL_ERROR_MIN 2.0f        /* degrees still to go                  */
#define STALL_WINDOW_MS 1500u

/*
 * Homing order: gripper jaw, wrist roll, wrist pitch, elbow, shoulder lift,
 * shoulder pan, then the slide. Joints sharing an order value home together.
 *
 * The two wrist joints deliberately do NOT share an order. They are one
 * differential mechanism: both are driven by motors 3 and 4, opposed for roll
 * and in unison for pitch. Homing them concurrently would have each issuing
 * contradictory commands to the same two motors, so they go one after the other.
 *
 * Every switch is a zero index rather than an end of travel: the angle latched
 * when it closes is 0, and min_angle/max_angle describe the travel either side
 * of it. All joints seek in the negative direction to find their switch.
 */
#define HOME_ORDER_GRIPPER     0
#define HOME_ORDER_WRIST_ROLL  1
#define HOME_ORDER_WRIST_PITCH 2
#define HOME_ORDER_ELBOW       3
#define HOME_ORDER_LIFT        4
#define HOME_ORDER_PAN         5
#define HOME_ORDER_SLIDE       6

#define HOMING_DEFAULTS(order_value, home_at) \
    .homing = { \
        .direction = -1, \
        .order = (order_value), \
        .enabled = 1, \
        .method = SCORBOT_HOME_METHOD_SWITCH, \
        .seek_duty = 0.55f, \
        /* Measured on the wrist: 0.15 is below stiction and the joint does not
         * move at all, while 0.30 does. Creep wants to be as slow as possible
         * for repeatability, but slower than the joint can actually move just
         * looks like a stall. Tune per joint with SET_HOMING_CFG. */ \
        .creep_duty = 0.30f, \
        .backoff_deg = 3.0f, \
        .home_offset_deg = (home_at), \
        .timeout_ms = 25000, \
        /* Deliberately lenient for switch homing, where a stall means failure:
         * a false trip aborts a legitimate home, while a missed one is still
         * bounded by timeout_ms, so err toward patience. */ \
        .stall_eps_deg = 0.20f, \
        .stall_window_ms = 1500 \
    }

/*
 * The gripper jaw has no limit switch, so it references off its own closed
 * stop. Detection is tuned the opposite way from switch homing: a short window
 * and a coarse threshold, because here a stall is the expected outcome and
 * every millisecond spent detecting it is a millisecond of pushing on the stop.
 * Measured travel is around 11 units/s, far above the 1.25 units/s threshold.
 */
#define HOMING_STALL_DEFAULTS(order_value, home_at) \
    .homing = { \
        .direction = -1, \
        .order = (order_value), \
        .enabled = 1, \
        .method = SCORBOT_HOME_METHOD_STALL, \
        .seek_duty = 0.35f, \
        .creep_duty = 0.25f, \
        .backoff_deg = 2.0f, \
        .home_offset_deg = (home_at), \
        .timeout_ms = 15000, \
        .stall_eps_deg = 0.50f, \
        .stall_window_ms = 400 \
    }

#define GAIN_DEFAULTS(kp_deg) \
    .gains = { \
        .kp = (kp_deg), \
        .ki = 0.0f, \
        .kd = 0.0f, \
        .i_clamp = 0.25f, \
        .out_clamp = 1.0f, \
        .deadband_deg = 0.05f, \
        .max_vel_dps = 60.0f \
    }

joint_t joints[NUM_JOINTS] = {
    {
        .name = "shoulder_pan",
        .primary_motor = &motors[0],
        .ms_port = MS1_GPIO_Port,
        .ms_pin = MS1_Pin,
        .gear_ratio = (6500.0f / 90.0f),
        .max_angle = 160.0f,
        .min_angle = -130.0f,
        .max_speed_dps = 60.0f,
        GAIN_DEFAULTS(7.0f),
        HOMING_DEFAULTS(HOME_ORDER_PAN, 0.0f)
    },
    {
        .name = "shoulder_lift",
        .primary_motor = &motors[1],
        .ms_port = MS2_GPIO_Port,
        .ms_pin = MS2_Pin,
        .gear_ratio = (20000.0f / 360.0f),
        .max_angle = 90.0f,
        .min_angle = 0.0f,
        .max_speed_dps = 40.0f,
        GAIN_DEFAULTS(5.5f),
        HOMING_DEFAULTS(HOME_ORDER_LIFT, 0.0f)
    },
    {
        .name = "elbow",
        .primary_motor = &motors[2],
        .ms_port = MS3_GPIO_Port,
        .ms_pin = MS3_Pin,
        .gear_ratio = (5000.0f / 90.0f),
        .max_angle = 160.0f,
        .min_angle = -130.0f,
        .max_speed_dps = 60.0f,
        GAIN_DEFAULTS(5.5f),
        HOMING_DEFAULTS(HOME_ORDER_ELBOW, 0.0f)
    },
    {
        /* Motors 3 and 4 opposed: this is the continuous gripper-rotate axis.
         * It has no end of travel, so the range is simply one turn either way
         * from the index, widened with SET_LIMITS if more is wanted. */
        .name = "wrist_roll",
        .primary_motor = &motors[4],
        .coupled_motor = &motors[3],
        .coupling_factor = -1.0f,
        .ms_port = MS4_GPIO_Port,
        .ms_pin = MS4_Pin,
        .gear_ratio = (1200.0f / 90.0f),
        .max_angle = 360.0f,
        .min_angle = -360.0f,
        .max_speed_dps = 90.0f,
        GAIN_DEFAULTS(1.4f),
        HOMING_DEFAULTS(HOME_ORDER_WRIST_ROLL, 0.0f)
    },
    {
        /* Motors 3 and 4 in unison: wrist pitch. */
        .name = "wrist_pitch",
        .primary_motor = &motors[3],
        .coupled_motor = &motors[4],
        .coupling_factor = 1.0f,
        .ms_port = MS5_GPIO_Port,
        .ms_pin = MS5_Pin,
        .gear_ratio = (1200.0f / 90.0f),
        .max_angle = 180.0f,
        .min_angle = -180.0f,
        .max_speed_dps = 90.0f,
        GAIN_DEFAULTS(1.4f),
        HOMING_DEFAULTS(HOME_ORDER_WRIST_PITCH, 0.0f)
    },
    {
        .name = "gripper",
        .primary_motor = &motors[5],
        .ms_port = MS6_GPIO_Port,
        .ms_pin = MS6_Pin,
        .gear_ratio = 100.0f,
        .max_angle = 60.0f,
        .min_angle = 0.0f,
        .max_speed_dps = 90.0f,
        GAIN_DEFAULTS(0.1f),
        HOMING_STALL_DEFAULTS(HOME_ORDER_GRIPPER, 0.0f)
    },
    {
        /* The linear slide. Its units are millimetres rather than degrees, but
         * it moves through the same machinery, so the protocol carries it in
         * the same float field. */
        .name = "slide",
        .primary_motor = &motors[6],
        .ms_port = MS7_GPIO_Port,
        .ms_pin = MS7_Pin,
        .gear_ratio = 50.0f,
        .max_angle = 300.0f,
        .min_angle = 0.0f,
        .max_speed_dps = 100.0f,
        GAIN_DEFAULTS(0.2f),
        HOMING_DEFAULTS(HOME_ORDER_SLIDE, 0.0f)
    }
};

/*
 * The linear slide is not currently connected to the arm, so it is left out of
 * the homing sequence. Re-enable it from the host with SET_HOMING_CFG, or flip
 * this back, once the hardware is present and its switch situation is known.
 */
#define SLIDE_JOINT_INDEX 6

/*
 * The protocol expresses gains per degree because that is what a human tuning
 * the arm can reason about. The interrupt-rate loop works in encoder counts, so
 * the conversion happens once here rather than on every pass.
 */
static void joint_push_gains(joint_t* joint)
{
    const float ratio = joint->gear_ratio;
    if(ratio <= 0.0f) {
        return;
    }

    motor_t* targets[2] = { joint->primary_motor, joint->coupled_motor };
    for(int i = 0; i < 2; i++) {
        motor_t* m = targets[i];
        if(m == NULL) {
            continue;
        }
        m->k_p = joint->gains.kp / ratio;
        m->k_i = joint->gains.ki / ratio;
        m->k_d = joint->gains.kd / ratio;
        m->i_clamp = joint->gains.i_clamp;
        m->out_clamp = joint->gains.out_clamp;
        m->deadband = joint->gains.deadband_deg * ratio;
        m->max_vel = joint->gains.max_vel_dps * ratio;
    }
}

void joint_init(void)
{
    for(int i = 0; i < NUM_JOINTS; i++) {
        joints[i].mode = SCORBOT_MODE_IDLE;
        joints[i].homed = 0;
        joints[i].faulted = 0;
        joints[i].setpoint = 0.0f;
        joints[i].gains.joint = (uint8_t)i;
        joints[i].homing.joint = (uint8_t)i;
        joint_push_gains(&joints[i]);
    }
    joints[SLIDE_JOINT_INDEX].homing.enabled = 0;
}

void joint_set_gains(joint_t* joint, const scorbot_gains_t* gains)
{
    const uint8_t index = joint->gains.joint;
    joint->gains = *gains;
    joint->gains.joint = index;
    joint_push_gains(joint);
}

void joint_get_gains(joint_t* joint, scorbot_gains_t* out)
{
    *out = joint->gains;
}

void joint_set_mode(joint_t* joint, uint8_t mode)
{
    if(joint->mode == mode) {
        return;
    }
    joint->mode = mode;

    if(mode == SCORBOT_MODE_HOLD) {
        /* Hold means stay exactly where we are right now. */
        joint->setpoint = joint_get_angle(joint);
    }
    joint_apply(joint);
}

void joint_set_setpoint(joint_t* joint, float value)
{
    joint->setpoint = value;
}

void joint_apply(joint_t* joint)
{
    motor_t* primary = joint->primary_motor;
    motor_t* coupled = joint->coupled_motor;
    if(primary == NULL) {
        return;
    }

    switch(joint->mode) {
    case SCORBOT_MODE_POSITION:
    case SCORBOT_MODE_HOLD: {
        float angle = joint->setpoint;
        if(angle > joint->max_angle) {
            angle = joint->max_angle;
        }
        if(angle < joint->min_angle) {
            angle = joint->min_angle;
        }
        motor_set_control_mode(primary, MOTOR_CONTROL_MODE_POSITION);
        motor_set_position(primary, (int32_t)(angle * joint->gear_ratio));
        if(coupled != NULL) {
            motor_set_control_mode(coupled, MOTOR_CONTROL_MODE_POSITION);
            motor_set_position(coupled,
                (int32_t)(angle * joint->gear_ratio * joint->coupling_factor)
                    + joint->coupling_offset);
        }
        break;
    }

    case SCORBOT_MODE_VELOCITY: {
        const float duty = (joint->max_speed_dps > 0.0f)
            ? (joint->setpoint / joint->max_speed_dps)
            : 0.0f;
        joint_drive_duty(joint, duty);
        break;
    }

    case SCORBOT_MODE_PWM:
        joint_drive_duty(joint, joint->setpoint);
        break;

    case SCORBOT_MODE_IDLE:
    default:
        joint_stop(joint);
        break;
    }
}

/* Returns the open-loop duty an axis is asking for, or 0 if it is not driving. */
static float joint_requested_duty(const joint_t* joint)
{
    switch(joint->mode) {
    case SCORBOT_MODE_PWM:
        return joint->setpoint;
    case SCORBOT_MODE_VELOCITY:
        return (joint->max_speed_dps > 0.0f) ? (joint->setpoint / joint->max_speed_dps) : 0.0f;
    default:
        return 0.0f;
    }
}

static float clamp_unit(float v)
{
    if(v > 1.0f) {
        return 1.0f;
    }
    if(v < -1.0f) {
        return -1.0f;
    }
    return v;
}

/*
 * Resolve the differential wrist.
 *
 * Roll drives motors 3 and 4 opposed, pitch drives them in unison, so the two
 * axes superpose onto the same pair rather than owning a motor each:
 *
 *     motor3 = pitch - roll        motor4 = pitch + roll
 *
 * which is the inverse of how joint_get_angle recovers the two angles. An axis
 * that is idle contributes nothing to a duty command and holds its present
 * angle in a position command, so leaving one axis alone no longer cancels the
 * other.
 */
static void joint_apply_wrist_pair(void)
{
    joint_t* roll = &joints[JOINT_WRIST_ROLL];
    joint_t* pitch = &joints[JOINT_WRIST_PITCH];
    motor_t* m3 = pitch->primary_motor;  /* motors[3] */
    motor_t* m4 = roll->primary_motor;   /* motors[4] */

    if(roll->mode == SCORBOT_MODE_IDLE && pitch->mode == SCORBOT_MODE_IDLE) {
        motor_set_pwm(m3, 0.0f);
        motor_set_pwm(m4, 0.0f);
        motor_set_control_mode(m3, MOTOR_CONTROL_MODE_OFF);
        motor_set_control_mode(m4, MOTOR_CONTROL_MODE_OFF);
        return;
    }

    const uint8_t roll_open = (roll->mode == SCORBOT_MODE_PWM ||
                               roll->mode == SCORBOT_MODE_VELOCITY);
    const uint8_t pitch_open = (pitch->mode == SCORBOT_MODE_PWM ||
                                pitch->mode == SCORBOT_MODE_VELOCITY);

    if(roll_open || pitch_open) {
        const float dr = joint_requested_duty(roll);
        const float dp = joint_requested_duty(pitch);
        motor_set_control_mode(m3, MOTOR_CONTROL_MODE_PWM);
        motor_set_control_mode(m4, MOTOR_CONTROL_MODE_PWM);
        motor_set_pwm(m3, clamp_unit(dp - dr));
        motor_set_pwm(m4, clamp_unit(dp + dr));
        return;
    }

    /* Both axes are holding or tracking a position. */
    float roll_angle = roll->setpoint;
    float pitch_angle = pitch->setpoint;
    if(roll->mode == SCORBOT_MODE_IDLE) {
        roll_angle = joint_get_angle(roll);
    }
    if(pitch->mode == SCORBOT_MODE_IDLE) {
        pitch_angle = joint_get_angle(pitch);
    }

    if(roll_angle > roll->max_angle) {
        roll_angle = roll->max_angle;
    }
    if(roll_angle < roll->min_angle) {
        roll_angle = roll->min_angle;
    }
    if(pitch_angle > pitch->max_angle) {
        pitch_angle = pitch->max_angle;
    }
    if(pitch_angle < pitch->min_angle) {
        pitch_angle = pitch->min_angle;
    }

    const float ratio = pitch->gear_ratio;
    motor_set_control_mode(m3, MOTOR_CONTROL_MODE_POSITION);
    motor_set_control_mode(m4, MOTOR_CONTROL_MODE_POSITION);
    motor_set_position(m3, (int32_t)((pitch_angle - roll_angle) * ratio));
    motor_set_position(m4, (int32_t)((pitch_angle + roll_angle) * ratio));
}

void joint_apply_all(void)
{
    for(int i = 0; i < NUM_JOINTS; i++) {
        if(i == JOINT_WRIST_ROLL || i == JOINT_WRIST_PITCH) {
            continue;
        }
        joint_apply(&joints[i]);
    }
    joint_apply_wrist_pair();
}

void joint_clear_fault(joint_t* joint)
{
    joint->faulted = 0;
    joint->straining = 0;
    joint->straining_since_ms = 0;
}

void joint_safety_tick(uint32_t now_ms)
{
    for(int i = 0; i < NUM_JOINTS; i++) {
        joint_t* joint = &joints[i];

        /* Idle joints are not being asked to do anything, and homing runs its
         * own stall detection with parameters suited to seeking a switch. */
        if(joint->mode == SCORBOT_MODE_IDLE || joint->faulted) {
            joint->straining = 0;
            continue;
        }

        const float effort = fabsf(joint_get_effort(joint));
        const float velocity = fabsf(joint_get_velocity(joint));

        uint8_t suspicious = (effort >= STALL_EFFORT_MIN) && (velocity <= STALL_VELOCITY_MAX);

        /* Holding station under gravity looks identical to a runaway except
         * that the joint is already where it was asked to be. */
        if(suspicious && (joint->mode == SCORBOT_MODE_POSITION ||
                          joint->mode == SCORBOT_MODE_HOLD)) {
            if(fabsf(joint->setpoint - joint_get_angle(joint)) < STALL_ERROR_MIN) {
                suspicious = 0;
            }
        }

        if(!suspicious) {
            joint->straining = 0;
            continue;
        }

        if(!joint->straining) {
            joint->straining = 1;
            joint->straining_since_ms = now_ms;
            continue;
        }

        if((now_ms - joint->straining_since_ms) >= STALL_WINDOW_MS) {
            joint->faulted = 1;
            joint->straining = 0;
            joint->mode = SCORBOT_MODE_IDLE;
            joint_stop(joint);
            printf("scorbot: %s faulted - driving at %.2f but not moving\r\n",
                   joint->name, (double)effort);
        }
    }
}

/*
 * Correction applied to the wrist axis that is not being driven, in duty per
 * degree of drift, and the most it is allowed to contribute.
 */
/*
 * Proportional only, so the residual drift is the rate mismatch divided by this
 * gain. At 0.05 the other axis still walked 5 degrees during a homing pass;
 * 0.25 brings that down to sub-degree without the windup risk an integral term
 * would add to a loop that only runs in bursts.
 */
#define WRIST_HOLD_GAIN 0.25f
#define WRIST_HOLD_MAX 0.50f

static float wrist_hold_target;
static uint8_t wrist_hold_valid;

/*
 * Drive one wrist axis open loop while holding the other one still.
 *
 * Driving both shared motors at the same duty and calling it pure pitch only
 * works if they move at the same rate, and they do not: different friction and
 * load make them diverge, which shows up as the other axis drifting. Homing
 * pitch this way walked roll 169 degrees off a reference it had just
 * established. So the idle axis gets regulated back to where it started rather
 * than being left to the mercy of whichever motor happens to be freer.
 */
static void joint_drive_wrist_duty(joint_t* seeking, float duty)
{
    joint_t* roll = &joints[JOINT_WRIST_ROLL];
    joint_t* pitch = &joints[JOINT_WRIST_PITCH];
    const uint8_t seeking_roll = (seeking == roll);
    joint_t* partner = seeking_roll ? pitch : roll;

    if(!wrist_hold_valid) {
        wrist_hold_target = joint_get_angle(partner);
        wrist_hold_valid = 1;
    }

    float correction = WRIST_HOLD_GAIN * (wrist_hold_target - joint_get_angle(partner));
    if(correction > WRIST_HOLD_MAX) {
        correction = WRIST_HOLD_MAX;
    } else if(correction < -WRIST_HOLD_MAX) {
        correction = -WRIST_HOLD_MAX;
    }

    /* pitch = (m3 + m4) / 2 and roll = (m4 - m3) / 2, so a pitch correction
     * adds equally to both motors and a roll correction adds oppositely. */
    float m3, m4;
    if(seeking_roll) {
        m3 = -duty + correction;   /* correction holds pitch */
        m4 = duty + correction;
    } else {
        m3 = duty - correction;    /* correction holds roll  */
        m4 = duty + correction;
    }

    motor_set_control_mode(pitch->primary_motor, MOTOR_CONTROL_MODE_PWM);
    motor_set_control_mode(roll->primary_motor, MOTOR_CONTROL_MODE_PWM);
    motor_set_pwm(pitch->primary_motor, clamp_unit(m3));   /* motors[3] */
    motor_set_pwm(roll->primary_motor, clamp_unit(m4));    /* motors[4] */
}

void joint_drive_duty(joint_t* joint, float duty)
{
    duty = clamp_unit(duty);

    if(joint == &joints[JOINT_WRIST_ROLL] || joint == &joints[JOINT_WRIST_PITCH]) {
        joint_drive_wrist_duty(joint, duty);
        return;
    }

    motor_set_control_mode(joint->primary_motor, MOTOR_CONTROL_MODE_PWM);
    motor_set_pwm(joint->primary_motor, duty);
    if(joint->coupled_motor != NULL) {
        motor_set_control_mode(joint->coupled_motor, MOTOR_CONTROL_MODE_PWM);
        motor_set_pwm(joint->coupled_motor, duty * joint->coupling_factor);
    }
}

void joint_stop(joint_t* joint)
{
    if(joint == &joints[JOINT_WRIST_ROLL] || joint == &joints[JOINT_WRIST_PITCH]) {
        /* Re-latch the hold reference next time either wrist axis is driven. */
        wrist_hold_valid = 0;
    }
    motor_set_pwm(joint->primary_motor, 0.0f);
    motor_set_control_mode(joint->primary_motor, MOTOR_CONTROL_MODE_OFF);
    if(joint->coupled_motor != NULL) {
        motor_set_pwm(joint->coupled_motor, 0.0f);
        motor_set_control_mode(joint->coupled_motor, MOTOR_CONTROL_MODE_OFF);
    }
}

float joint_get_angle(joint_t* joint)
{
    float angle = (float)motor_get_current_position(joint->primary_motor) / joint->gear_ratio;
    if(joint->coupled_motor != NULL) {
        angle += (float)(motor_get_current_position(joint->coupled_motor) - joint->coupling_offset)
                 / (joint->gear_ratio * joint->coupling_factor);
        angle *= 0.5f;  /* both motors report the same joint; average them */
    }
    return angle;
}

float joint_get_velocity(joint_t* joint)
{
    float velocity = motor_get_current_velocity(joint->primary_motor) / joint->gear_ratio;
    if(joint->coupled_motor != NULL) {
        velocity += motor_get_current_velocity(joint->coupled_motor)
                    / (joint->gear_ratio * joint->coupling_factor);
        velocity *= 0.5f;
    }
    return velocity;
}

float joint_get_effort(joint_t* joint)
{
    /* No current sensing on this board yet, so report the commanded duty. */
    return motor_get_current_effort(joint->primary_motor);
}

void joint_zero(joint_t* joint, float angle)
{
    motor_set_encoder_value(joint->primary_motor, (int32_t)(angle * joint->gear_ratio));
    if(joint->coupled_motor != NULL) {
        /*
         * The coupled motor is not zeroed. Its offset from the primary is the
         * mechanical reality of the differential wrist, so record it and factor
         * it out of every later conversion instead.
         */
        joint->coupling_offset = motor_get_current_position(joint->coupled_motor)
                                 - (int32_t)(angle * joint->gear_ratio * joint->coupling_factor);
    }
    joint->setpoint = angle;
    joint->homed = 1;
}
