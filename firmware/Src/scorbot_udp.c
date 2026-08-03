#include "scorbot_udp.h"

#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "homing.h"
#include "joint.h"
#include "lwip/api.h"
#include "task.h"

scorbot_session_t scorbot_session;

static struct netconn* data_conn;
static uint32_t last_pose_ms;
static uint32_t tick_seq;
static uint32_t echo_seq;
static uint64_t t_echo_ns;
static uint8_t estop_latched;
static uint8_t fault_code;
static uint8_t watchdog_tripped;
static uint16_t missed_deadlines;

/* Serialises the two tasks' access to the shared echo and session fields. */
static osMutexId state_mutex;

void scorbot_set_estop(uint8_t engaged) { estop_latched = engaged; }
uint8_t scorbot_get_estop(void) { return estop_latched; }
void scorbot_set_fault(uint8_t code) { fault_code = code; }
uint8_t scorbot_get_fault(void) { return fault_code; }
uint8_t scorbot_watchdog_tripped(void) { return watchdog_tripped; }

void scorbot_session_reset(void)
{
    /*
     * Clearing the echo fields matters: carrying a previous session's transmit
     * timestamp into a new one makes the host compute a round trip spanning
     * both, which shows up as an absurd multi-second latency sample.
     */
    echo_seq = 0;
    t_echo_ns = 0;
    tick_seq = 0;
    watchdog_tripped = 0;
    missed_deadlines = 0;
    last_pose_ms = HAL_GetTick();
}

void scorbot_udp_init(void)
{
    memset(&scorbot_session, 0, sizeof(scorbot_session));
    scorbot_session.state_period_us = 1000;
    scorbot_session.watchdog_ms = 100;

    osMutexDef(scorbot_state_mutex);
    state_mutex = osMutexCreate(osMutex(scorbot_state_mutex));

    data_conn = netconn_new(NETCONN_UDP);
    if(data_conn == NULL) {
        return;
    }
    if(netconn_bind(data_conn, NULL, SCORBOT_DATA_PORT) != ERR_OK) {
        netconn_delete(data_conn);
        data_conn = NULL;
    }
}

static void scorbot_fill_state(scorbot_state_t* out)
{
    memset(out, 0, sizeof(*out));
    out->magic = SCORBOT_MAGIC_STATE;
    out->version = SCORBOT_PROTO_VERSION;
    out->session_id = scorbot_session.id;
    out->seq = ++tick_seq;
    out->t_echo_ns = t_echo_ns;
    out->echo_seq = echo_seq;
    out->t_fw_us = HAL_GetTick() * 1000u;
    out->homing_state = homing_state();
    out->homing_joint = homing_current_joint();
    out->fault = fault_code;
    out->missed_deadlines = missed_deadlines;

    if(scorbot_session.streaming) {
        out->flags |= SCORBOT_STATE_FLAG_STREAMING;
    }
    if(watchdog_tripped) {
        out->flags |= SCORBOT_STATE_FLAG_WATCHDOG;
    }
    if(estop_latched) {
        out->flags |= SCORBOT_STATE_FLAG_ESTOP;
    }
    if(homing_active()) {
        out->flags |= SCORBOT_STATE_FLAG_HOMING;
    }

    for(int i = 0; i < NUM_JOINTS && i < SCORBOT_MAX_JOINTS; i++) {
        joint_t* joint = &joints[i];
        out->mode[i] = joint->mode;
        out->position[i] = joint_get_angle(joint);
        out->velocity[i] = joint_get_velocity(joint);
        out->effort[i] = joint_get_effort(joint);
        out->encoder[i] = motor_get_current_position(joint->primary_motor);

        uint8_t flags = 0;
        if(joint->homed) {
            flags |= SCORBOT_JFLAG_HOMED;
        }
        if(joint_limit_active(joint)) {
            flags |= SCORBOT_JFLAG_AT_LIMIT;
            out->limit_mask |= (uint16_t)(1u << i);
        }
        if(joint->primary_motor->saturated) {
            flags |= SCORBOT_JFLAG_SATURATED;
        }
        if(joint->faulted) {
            flags |= SCORBOT_JFLAG_FAULT;
        }
        out->jflags[i] = flags;
    }
}

static void scorbot_send_state(void)
{
    if(data_conn == NULL || !scorbot_session.streaming || scorbot_session.host_port == 0) {
        return;
    }

    static scorbot_state_t state;
    scorbot_fill_state(&state);

    ip_addr_t dst;
    IP4_ADDR(&dst,
             (scorbot_session.host_ip >> 24) & 0xFF,
             (scorbot_session.host_ip >> 16) & 0xFF,
             (scorbot_session.host_ip >> 8) & 0xFF,
             scorbot_session.host_ip & 0xFF);

    struct netbuf* buf = netbuf_new();
    if(buf == NULL) {
        return;
    }
    /* Reference the static buffer rather than copying; it is only in flight
     * until netconn_sendto returns. */
    if(netbuf_ref(buf, &state, sizeof(state)) == ERR_OK) {
        (void)netconn_sendto(data_conn, buf, &dst, scorbot_session.host_port);
    }
    netbuf_delete(buf);
}

/* Everything to a safe state without losing the homed references. */
static void scorbot_enter_safe_state(void)
{
    for(int i = 0; i < NUM_JOINTS; i++) {
        if(joints[i].mode == SCORBOT_MODE_IDLE) {
            continue;
        }
        joints[i].mode = SCORBOT_MODE_HOLD;
        joints[i].setpoint = joint_get_angle(&joints[i]);
        joint_apply(&joints[i]);
    }
}

static void scorbot_handle_pose(const scorbot_pose_t* pose)
{
    if(pose->magic != SCORBOT_MAGIC_POSE || pose->version != SCORBOT_PROTO_VERSION) {
        return;
    }
    if(scorbot_session.id == 0 || pose->session_id != scorbot_session.id) {
        return;
    }

    last_pose_ms = HAL_GetTick();
    if(watchdog_tripped) {
        watchdog_tripped = 0;
        if(fault_code == SCORBOT_FAULT_WATCHDOG) {
            fault_code = SCORBOT_FAULT_NONE;
        }
    }
    echo_seq = pose->seq;
    t_echo_ns = pose->t_tx_ns;

    if((pose->flags & SCORBOT_POSE_FLAG_ESTOP) != 0u) {
        estop_latched = 1;
        fault_code = SCORBOT_FAULT_ESTOP;
        homing_abort();
        for(int i = 0; i < NUM_JOINTS; i++) {
            joints[i].mode = SCORBOT_MODE_IDLE;
            joint_stop(&joints[i]);
        }
        return;
    }

    /*
     * Setpoints are ignored while homing runs or an e-stop is latched. Homing
     * owns the motors for the duration, and letting a stream fight it would be
     * both confusing and dangerous.
     */
    const uint8_t keepalive = (pose->flags & SCORBOT_POSE_FLAG_KEEPALIVE) != 0u;
    if(!keepalive && !estop_latched && !homing_active()) {
        /*
         * Take in the whole pose first, then push it down in one pass. Applying
         * each joint as it is parsed lets the two wrist joints overwrite each
         * other on the motors they share.
         */
        for(int i = 0; i < NUM_JOINTS && i < SCORBOT_MAX_JOINTS; i++) {
            joint_t* joint = &joints[i];
            /* A latched fault stays latched. Re-commanding a joint that has
             * already proved it cannot move would just restart the runaway. */
            if(joint->faulted) {
                joint->mode = SCORBOT_MODE_IDLE;
                continue;
            }
            const uint8_t want = pose->mode[i];
            if(want == SCORBOT_MODE_HOLD && joint->mode != SCORBOT_MODE_HOLD) {
                joint->setpoint = joint_get_angle(joint);
            } else if(want != SCORBOT_MODE_HOLD) {
                joint->setpoint = pose->setpoint[i];
            }
            joint->mode = want;
        }
        joint_apply_all();
    }

    if((pose->flags & SCORBOT_POSE_FLAG_REPLY_NOW) != 0u) {
        scorbot_send_state();
    }
}

void scorbot_udp_rx_task(void const* arg)
{
    (void)arg;
    struct netbuf* buf;

    if(data_conn == NULL) {
        for(;;) {
            osDelay(1000);
        }
    }


    /* Blocks indefinitely: this task exists only to service arriving poses, so
     * sleeping until one shows up is exactly the behaviour we want. */
    for(;;) {
        if(netconn_recv(data_conn, &buf) != ERR_OK) {
            /*
             * Must yield here. A bare continue turns any receive error into a
             * hot loop, and this task sits at or above the LwIP TCP/IP thread,
             * so spinning starves the stack itself: DHCP never runs and the
             * board silently never gets an address.
             */
            osDelay(1);
            continue;
        }

        void* payload = NULL;
        u16_t len = 0;
        if(netbuf_data(buf, &payload, &len) == ERR_OK && len == sizeof(scorbot_pose_t)) {
            /*
             * Copy into an aligned local rather than casting the pbuf payload.
             * The firmware is built with -mno-unaligned-access, so a misaligned
             * struct read would fault rather than merely being slow.
             */
            scorbot_pose_t pose;
            memcpy(&pose, payload, sizeof(pose));
            osMutexWait(state_mutex, osWaitForever);
            scorbot_handle_pose(&pose);
            osMutexRelease(state_mutex);
        }
        netbuf_delete(buf);
    }
}

void scorbot_udp_tx_task(void const* arg)
{
    (void)arg;
    uint32_t next_wake = osKernelSysTick();


    for(;;) {
        uint32_t period_ms = scorbot_session.state_period_us / 1000u;
        if(period_ms == 0u) {
            period_ms = 1u;
        }

        const uint32_t now = HAL_GetTick();

        osMutexWait(state_mutex, osWaitForever);

        homing_tick(now);
        if(!homing_active()) {
            /* Homing runs its own stall detection tuned for seeking a switch. */
            joint_safety_tick(now);
        }

        if(scorbot_session.streaming) {
            /*
             * The watchdog is suspended while homing runs. Homing is driven
             * entirely by the firmware and takes seconds, during which the host
             * has nothing to stream; faulting out mid-sequence would abandon
             * the arm sitting on a limit switch.
             */
            if(homing_active()) {
                last_pose_ms = now;
            } else if(!watchdog_tripped &&
                      (now - last_pose_ms) > scorbot_session.watchdog_ms) {
                watchdog_tripped = 1;
                fault_code = SCORBOT_FAULT_WATCHDOG;
                scorbot_enter_safe_state();
            }
            scorbot_send_state();
        }

        osMutexRelease(state_mutex);

        /*
         * Always yields at least one tick. osDelayUntil returns immediately
         * when its deadline has already passed, which would turn an overrun
         * into a spin that starves the TCP/IP thread, so the remaining time is
         * computed here and an overrun resynchronises instead of trying to
         * catch up in a burst.
         */
        next_wake += period_ms;
        const uint32_t tick_now = osKernelSysTick();
        const int32_t remaining = (int32_t)(next_wake - tick_now);
        if(remaining > 0) {
            osDelay((uint32_t)remaining);
        } else {
            missed_deadlines++;
            next_wake = tick_now;
            osDelay(1);
        }
    }
}
