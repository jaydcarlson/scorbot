#include "homing.h"

#include <math.h>
#include <string.h>

/* Per-joint progress through its own two-pass approach. */
typedef enum {
    JOINT_HOME_IDLE = 0,
    JOINT_HOME_SEEK,
    JOINT_HOME_BACKOFF,
    JOINT_HOME_CREEP,
    JOINT_HOME_LATCHED
} joint_home_phase_t;

typedef struct {
    joint_home_phase_t phase;
    uint32_t phase_started_ms;
    uint32_t stall_checked_ms;
    float stall_reference_deg;
    float backoff_target_deg;
} joint_home_state_t;

static joint_home_state_t states[NUM_JOINTS];
static uint8_t sequence_state = SCORBOT_HOMING_IDLE;
static uint8_t sequence_mask;
static uint8_t current_order;
static uint8_t current_joint = 0xFF;
static uint8_t fault_code = SCORBOT_FAULT_NONE;
static uint32_t started_ms;

void homing_init(void)
{
    memset(states, 0, sizeof(states));
    sequence_state = SCORBOT_HOMING_IDLE;
    sequence_mask = 0;
    current_joint = 0xFF;
    fault_code = SCORBOT_FAULT_NONE;
}

uint8_t homing_state(void) { return sequence_state; }
uint8_t homing_current_joint(void) { return current_joint; }

uint8_t homing_active(void)
{
    return (sequence_state != SCORBOT_HOMING_IDLE &&
            sequence_state != SCORBOT_HOMING_DONE &&
            sequence_state != SCORBOT_HOMING_FAULT) ? 1u : 0u;
}

void homing_get_status(scorbot_home_status_t* out)
{
    memset(out, 0, sizeof(*out));
    out->state = sequence_state;
    out->joint = current_joint;
    out->order = current_order;
    out->error = fault_code;
    for(int i = 0; i < NUM_JOINTS && i < SCORBOT_MAX_JOINTS; i++) {
        if(joints[i].homed) {
            out->homed_mask |= (uint8_t)(1u << i);
        }
    }
    out->elapsed_ms = (sequence_state == SCORBOT_HOMING_IDLE) ? 0u : (HAL_GetTick() - started_ms);
}

static uint8_t joint_selected(int index)
{
    return (index < NUM_JOINTS && (sequence_mask & (1u << index)) != 0u &&
            joints[index].homing.enabled != 0u) ? 1u : 0u;
}

void homing_abort(void)
{
    for(int i = 0; i < NUM_JOINTS; i++) {
        joint_stop(&joints[i]);
        joints[i].mode = SCORBOT_MODE_IDLE;
        states[i].phase = JOINT_HOME_IDLE;
    }
    sequence_state = SCORBOT_HOMING_IDLE;
    sequence_mask = 0;
    current_joint = 0xFF;
}

static void homing_fail(int index, uint8_t code)
{
    fault_code = code;
    sequence_state = SCORBOT_HOMING_FAULT;
    if(index >= 0 && index < NUM_JOINTS) {
        joints[index].faulted = 1;
        current_joint = (uint8_t)index;
    }
    for(int i = 0; i < NUM_JOINTS; i++) {
        joint_stop(&joints[i]);
        joints[i].mode = SCORBOT_MODE_IDLE;
    }
}

uint16_t homing_start(uint8_t joint_mask)
{
    if(homing_active()) {
        return SCORBOT_ERR_BUSY;
    }

    sequence_mask = joint_mask;
    fault_code = SCORBOT_FAULT_NONE;
    started_ms = HAL_GetTick();
    current_joint = 0xFF;

    for(int i = 0; i < NUM_JOINTS; i++) {
        states[i].phase = JOINT_HOME_IDLE;
        joints[i].faulted = 0;
        if(joint_selected(i)) {
            joints[i].homed = 0;
        }
    }

    /* Quiesce everything before anything starts moving. */
    for(int i = 0; i < NUM_JOINTS; i++) {
        joint_stop(&joints[i]);
        joints[i].mode = SCORBOT_MODE_IDLE;
    }
    sequence_state = SCORBOT_HOMING_PARK;
    return SCORBOT_OK;
}

/* Puts every selected joint of one order group into its fast seek. */
static void homing_begin_group(uint8_t order, uint32_t now_ms)
{
    current_order = order;
    current_joint = 0xFF;

    for(int i = 0; i < NUM_JOINTS; i++) {
        if(!joint_selected(i) || joints[i].homing.order != order) {
            continue;
        }
        joint_t* joint = &joints[i];
        joint_home_state_t* st = &states[i];

        st->phase = JOINT_HOME_SEEK;
        st->phase_started_ms = now_ms;
        st->stall_checked_ms = now_ms;
        st->stall_reference_deg = joint_get_angle(joint);

        /*
         * Hard-stop homing approaches at the gentle creep duty from the start.
         * There is no switch to catch it, so the stop itself takes the impact
         * and the arrival is only as violent as the approach.
         */
        const float duty = (joint->homing.method == SCORBOT_HOME_METHOD_STALL)
            ? joint->homing.creep_duty
            : joint->homing.seek_duty;

        joint->mode = SCORBOT_MODE_PWM;
        joint_drive_duty(joint, (float)joint->homing.direction * duty);

        if(current_joint == 0xFF) {
            current_joint = (uint8_t)i;
        }
    }
    sequence_state = SCORBOT_HOMING_SEEK;
}

/* Lowest order group that still has an unhomed selected joint, or -1. */
static int homing_next_order(int above)
{
    int best = -1;
    for(int i = 0; i < NUM_JOINTS; i++) {
        if(!joint_selected(i) || joints[i].homed) {
            continue;
        }
        const int order = (int)joints[i].homing.order;
        if(above >= 0 && order <= above) {
            continue;
        }
        if(best < 0 || order < best) {
            best = order;
        }
    }
    return best;
}

/*
 * A joint that is being commanded but is not moving has hit something, or its
 * switch never closed. Either way, keep driving into it and the gearbox loses.
 */
static uint8_t homing_stalled(joint_t* joint, joint_home_state_t* st, uint32_t now_ms)
{
    if(joint->homing.stall_window_ms == 0) {
        return 0;
    }
    if((now_ms - st->stall_checked_ms) < joint->homing.stall_window_ms) {
        return 0;
    }
    const float angle = joint_get_angle(joint);
    const uint8_t stalled =
        (fabsf(angle - st->stall_reference_deg) < joint->homing.stall_eps_deg) ? 1u : 0u;
    st->stall_checked_ms = now_ms;
    st->stall_reference_deg = angle;
    return stalled;
}

static void homing_step_joint(int index, uint32_t now_ms)
{
    joint_t* joint = &joints[index];
    joint_home_state_t* st = &states[index];

    if((now_ms - st->phase_started_ms) > joint->homing.timeout_ms) {
        homing_fail(index, SCORBOT_FAULT_HOMING);
        return;
    }

    if(joint->homing.method == SCORBOT_HOME_METHOD_STALL) {
        /*
         * No switch to wait for: the stop is the reference. The same stall
         * detector that means failure for a switch-homed joint means success
         * here, because arriving and staying put is exactly what should happen.
         */
        if(st->phase == JOINT_HOME_SEEK && homing_stalled(joint, st, now_ms)) {
            joint_stop(joint);
            joint_zero(joint, joint->homing.home_offset_deg);
            /* Back away from the stop so the joint does not sit leaning on it. */
            joint->mode = SCORBOT_MODE_HOLD;
            joint->setpoint = joint->homing.home_offset_deg
                - ((float)joint->homing.direction * joint->homing.backoff_deg);
            joint_apply(joint);
            st->phase = JOINT_HOME_LATCHED;
        }
        return;
    }

    switch(st->phase) {
    case JOINT_HOME_SEEK:
        if(joint_limit_active(joint)) {
            /* Retreat far enough that the switch definitely releases, so the
             * slow pass always starts from a known-open state. */
            st->phase = JOINT_HOME_BACKOFF;
            st->phase_started_ms = now_ms;
            st->stall_checked_ms = now_ms;
            st->stall_reference_deg = joint_get_angle(joint);
            st->backoff_target_deg = joint_get_angle(joint)
                - ((float)joint->homing.direction * joint->homing.backoff_deg);
            joint_drive_duty(joint,
                -(float)joint->homing.direction * joint->homing.creep_duty * 1.5f);
        } else if(homing_stalled(joint, st, now_ms)) {
            homing_fail(index, SCORBOT_FAULT_HOMING);
        }
        break;

    case JOINT_HOME_BACKOFF: {
        const float angle = joint_get_angle(joint);
        const float remaining = (float)joint->homing.direction * (angle - st->backoff_target_deg);
        if(!joint_limit_active(joint) && remaining <= 0.0f) {
            st->phase = JOINT_HOME_CREEP;
            st->phase_started_ms = now_ms;
            st->stall_checked_ms = now_ms;
            st->stall_reference_deg = angle;
            joint_drive_duty(joint,
                (float)joint->homing.direction * joint->homing.creep_duty);
        }
        break;
    }

    case JOINT_HOME_CREEP:
        if(joint_limit_active(joint)) {
            joint_stop(joint);
            joint_zero(joint, joint->homing.home_offset_deg);
            joint->mode = SCORBOT_MODE_HOLD;
            joint->setpoint = joint->homing.home_offset_deg;
            joint_apply(joint);
            st->phase = JOINT_HOME_LATCHED;
        } else if(homing_stalled(joint, st, now_ms)) {
            homing_fail(index, SCORBOT_FAULT_HOMING);
        }
        break;

    case JOINT_HOME_IDLE:
    case JOINT_HOME_LATCHED:
    default:
        break;
    }
}

void homing_tick(uint32_t now_ms)
{
    if(!homing_active()) {
        return;
    }

    if(sequence_state == SCORBOT_HOMING_PARK) {
        const int first = homing_next_order(-1);
        if(first < 0) {
            sequence_state = SCORBOT_HOMING_DONE;
            current_joint = 0xFF;
            return;
        }
        homing_begin_group((uint8_t)first, now_ms);
        return;
    }

    uint8_t group_complete = 1;
    uint8_t reported = 0;

    for(int i = 0; i < NUM_JOINTS; i++) {
        if(!joint_selected(i) || joints[i].homing.order != current_order) {
            continue;
        }
        if(states[i].phase == JOINT_HOME_LATCHED) {
            continue;
        }

        homing_step_joint(i, now_ms);
        if(sequence_state == SCORBOT_HOMING_FAULT) {
            return;
        }
        if(states[i].phase != JOINT_HOME_LATCHED) {
            group_complete = 0;
            if(!reported) {
                current_joint = (uint8_t)i;
                /* Surface whichever phase the leading joint of the group is in. */
                switch(states[i].phase) {
                case JOINT_HOME_SEEK:    sequence_state = SCORBOT_HOMING_SEEK; break;
                case JOINT_HOME_BACKOFF: sequence_state = SCORBOT_HOMING_BACKOFF; break;
                case JOINT_HOME_CREEP:   sequence_state = SCORBOT_HOMING_CREEP; break;
                default: break;
                }
                reported = 1;
            }
        }
    }

    if(!group_complete) {
        return;
    }

    const int next = homing_next_order((int)current_order);
    if(next < 0) {
        sequence_state = SCORBOT_HOMING_DONE;
        current_joint = 0xFF;
        return;
    }
    homing_begin_group((uint8_t)next, now_ms);
}
