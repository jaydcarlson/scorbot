#include "scorbot_ctrl.h"

#include <string.h>

#include "cmsis_os.h"
#include "homing.h"
#include "joint.h"
#include "lwip.h"
#include "lwip/api.h"
#include "lwip/netif.h"
#include "scorbot_proto.h"
#include "scorbot_udp.h"

#define SCORBOT_FW_VERSION 0x00050000u

static struct netconn* ctrl_conn;
static uint32_t next_session_id = 1;

/*
 * One shared reply buffer; the control plane is strictly one request at a time.
 * It doubles as a one-deep reply cache so a retried request can be answered
 * without executing it twice - see the duplicate check in ctrl_handle.
 */
static uint8_t reply_buf[SCORBOT_CTRL_MAX_DATAGRAM];
static uint16_t reply_len;
static uint32_t last_xid;
static uint16_t last_opcode;
static uint32_t last_from_ip;
static uint16_t last_from_port;
static uint8_t have_last_reply;

static void ctrl_reply(struct netbuf* request, uint16_t opcode, uint32_t xid, uint16_t status,
                       const void* payload, uint16_t len)
{
    scorbot_ctrl_hdr_t* hdr = (scorbot_ctrl_hdr_t*)reply_buf;
    memset(hdr, 0, sizeof(*hdr));
    hdr->magic = SCORBOT_MAGIC_CTRL;
    hdr->version = SCORBOT_PROTO_VERSION;
    hdr->opcode = opcode;
    hdr->xid = xid;
    hdr->session_id = scorbot_session.id;
    hdr->status = status;
    hdr->payload_len = len;
    if(payload != NULL && len > 0) {
        memcpy(reply_buf + sizeof(*hdr), payload, len);
    }

    reply_len = (uint16_t)(sizeof(*hdr) + len);
    last_xid = xid;
    last_opcode = opcode;
    last_from_ip = lwip_ntohl(netbuf_fromaddr(request)->addr);
    last_from_port = netbuf_fromport(request);
    have_last_reply = 1;

    struct netbuf* out = netbuf_new();
    if(out == NULL) {
        return;
    }
    if(netbuf_ref(out, reply_buf, reply_len) == ERR_OK) {
        (void)netconn_sendto(ctrl_conn, out, netbuf_fromaddr(request),
                             netbuf_fromport(request));
    }
    netbuf_delete(out);
}

/* Replays the cached reply for a request we have already executed. */
static void ctrl_resend_last(struct netbuf* request)
{
    struct netbuf* out = netbuf_new();
    if(out == NULL) {
        return;
    }
    if(netbuf_ref(out, reply_buf, reply_len) == ERR_OK) {
        (void)netconn_sendto(ctrl_conn, out, netbuf_fromaddr(request),
                             netbuf_fromport(request));
    }
    netbuf_delete(out);
}

static uint32_t local_ip_host_order(void)
{
    return lwip_ntohl(netif_ip4_addr(&gnetif)->addr);
}

static void ctrl_fill_hello(scorbot_hello_reply_t* out)
{
    memset(out, 0, sizeof(*out));
    out->fw_version = SCORBOT_FW_VERSION;
    out->proto_version = SCORBOT_PROTO_VERSION;
    out->uptime_ms = HAL_GetTick();
    out->active_session = scorbot_session.id;
    out->owner_ip = scorbot_session.owner_ip;
    memcpy(out->robot_mac, gnetif.hwaddr, 6);
    out->joint_count = NUM_JOINTS;

    for(int i = 0; i < NUM_JOINTS && i < SCORBOT_MAX_JOINTS; i++) {
        strncpy(out->joint_name[i], joints[i].name, SCORBOT_NAME_LEN - 1);
        out->min_angle[i] = joints[i].min_angle;
        out->max_angle[i] = joints[i].max_angle;
    }
}

static void ctrl_handle(struct netbuf* request, const uint8_t* data, uint16_t len)
{
    if(len < sizeof(scorbot_ctrl_hdr_t)) {
        return;
    }

    scorbot_ctrl_hdr_t hdr;
    memcpy(&hdr, data, sizeof(hdr));
    if(hdr.magic != SCORBOT_MAGIC_CTRL) {
        return;
    }
    if(hdr.version != SCORBOT_PROTO_VERSION) {
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_VERSION, NULL, 0);
        return;
    }

    /*
     * Retry deduplication. The host resends on timeout, and several operations
     * are emphatically not idempotent: a repeated HOME lands while the first is
     * still running and comes back BUSY, so a merely slow reply looks like a
     * refusal. Replaying the stored reply for a request already executed is
     * what actually makes the retry safe, rather than just asserting it is.
     */
    if(have_last_reply && hdr.xid == last_xid && hdr.opcode == last_opcode &&
       lwip_ntohl(netbuf_fromaddr(request)->addr) == last_from_ip &&
       netbuf_fromport(request) == last_from_port) {
        ctrl_resend_last(request);
        return;
    }

    const uint8_t* payload = data + sizeof(hdr);
    uint16_t payload_len = hdr.payload_len;
    if((uint32_t)payload_len + sizeof(hdr) > len) {
        payload_len = (uint16_t)(len - sizeof(hdr));
    }

    /* Only discovery and session establishment work without holding the session. */
    if(hdr.opcode != SCORBOT_OP_HELLO && hdr.opcode != SCORBOT_OP_OPEN_SESSION) {
        if(scorbot_session.id == 0 || hdr.session_id != scorbot_session.id) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_SESSION, NULL, 0);
            return;
        }
    }

    switch(hdr.opcode) {
    case SCORBOT_OP_HELLO: {
        static scorbot_hello_reply_t hello;
        ctrl_fill_hello(&hello);
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, &hello, sizeof(hello));
        return;
    }

    case SCORBOT_OP_OPEN_SESSION: {
        if(payload_len != sizeof(scorbot_open_req_t)) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_LENGTH, NULL, 0);
            return;
        }
        scorbot_open_req_t req;
        memcpy(&req, payload, sizeof(req));

        const uint32_t from_ip = lwip_ntohl(netbuf_fromaddr(request)->addr);
        if(scorbot_session.id != 0 && req.force == 0u && scorbot_session.owner_ip != from_ip) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BUSY, NULL, 0);
            return;
        }

        scorbot_session.id = next_session_id++;
        scorbot_session.owner_ip = from_ip;
        scorbot_session.host_ip = req.host_ip;
        scorbot_session.host_port = req.host_port;
        memcpy(scorbot_session.host_mac, req.host_mac, 6);
        scorbot_session.watchdog_ms = req.watchdog_ms ? req.watchdog_ms : 100u;
        scorbot_session.streaming = 0;
        scorbot_session_reset();
        scorbot_set_estop(0);
        scorbot_set_fault(SCORBOT_FAULT_NONE);
        /* A new session starts clean; latched joint faults are the previous
         * operator's problem and they have had their chance to look at them. */
        for(int i = 0; i < NUM_JOINTS; i++) {
            joint_clear_fault(&joints[i]);
        }

        scorbot_open_reply_t reply;
        memset(&reply, 0, sizeof(reply));
        reply.session_id = scorbot_session.id;
        memcpy(reply.robot_mac, gnetif.hwaddr, 6);
        reply.robot_port = SCORBOT_DATA_PORT;
        reply.robot_ip = local_ip_host_order();
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, &reply, sizeof(reply));
        return;
    }

    case SCORBOT_OP_CLOSE_SESSION:
        scorbot_session.id = 0;
        scorbot_session.streaming = 0;
        /*
         * Abort homing too. Idling the joints without stopping the state
         * machine leaves it driving motors that the disconnect just switched
         * off, so the two fight and the arm ends up somewhere unintended with
         * nobody watching.
         */
        homing_abort();
        for(int i = 0; i < NUM_JOINTS; i++) {
            joints[i].mode = SCORBOT_MODE_IDLE;
            joint_stop(&joints[i]);
        }
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, NULL, 0);
        return;

    case SCORBOT_OP_SET_MODE: {
        if(payload_len != sizeof(scorbot_set_mode_req_t)) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_LENGTH, NULL, 0);
            return;
        }
        scorbot_set_mode_req_t req;
        memcpy(&req, payload, sizeof(req));
        for(int i = 0; i < NUM_JOINTS && i < SCORBOT_MAX_JOINTS; i++) {
            if((req.joint_mask & (1u << i)) == 0u) {
                continue;
            }
            joint_t* joint = &joints[i];
            if(req.mode[i] == SCORBOT_MODE_HOLD && joint->mode != SCORBOT_MODE_HOLD) {
                joint->setpoint = joint_get_angle(joint);
            }
            joint->mode = req.mode[i];
        }
        joint_apply_all();
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, NULL, 0);
        return;
    }

    case SCORBOT_OP_SET_GAINS: {
        if(payload_len != sizeof(scorbot_gains_t)) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_LENGTH, NULL, 0);
            return;
        }
        scorbot_gains_t gains;
        memcpy(&gains, payload, sizeof(gains));
        if(gains.joint >= NUM_JOINTS) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_JOINT, NULL, 0);
            return;
        }
        joint_set_gains(&joints[gains.joint], &gains);
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, NULL, 0);
        return;
    }

    case SCORBOT_OP_GET_GAINS: {
        if(payload_len != sizeof(scorbot_gains_t)) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_LENGTH, NULL, 0);
            return;
        }
        scorbot_gains_t req;
        memcpy(&req, payload, sizeof(req));
        if(req.joint >= NUM_JOINTS) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_JOINT, NULL, 0);
            return;
        }
        scorbot_gains_t out;
        joint_get_gains(&joints[req.joint], &out);
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, &out, sizeof(out));
        return;
    }

    case SCORBOT_OP_SET_HOMING_CFG: {
        if(payload_len != sizeof(scorbot_homing_cfg_t)) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_LENGTH, NULL, 0);
            return;
        }
        scorbot_homing_cfg_t cfg;
        memcpy(&cfg, payload, sizeof(cfg));
        if(cfg.joint >= NUM_JOINTS) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_JOINT, NULL, 0);
            return;
        }
        if(homing_active()) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BUSY, NULL, 0);
            return;
        }
        joints[cfg.joint].homing = cfg;
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, NULL, 0);
        return;
    }

    case SCORBOT_OP_GET_HOMING_CFG: {
        if(payload_len != sizeof(scorbot_homing_cfg_t)) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_LENGTH, NULL, 0);
            return;
        }
        scorbot_homing_cfg_t req;
        memcpy(&req, payload, sizeof(req));
        if(req.joint >= NUM_JOINTS) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_JOINT, NULL, 0);
            return;
        }
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, &joints[req.joint].homing,
                   sizeof(scorbot_homing_cfg_t));
        return;
    }

    case SCORBOT_OP_SET_LIMITS: {
        if(payload_len != sizeof(scorbot_limits_t)) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_LENGTH, NULL, 0);
            return;
        }
        scorbot_limits_t limits;
        memcpy(&limits, payload, sizeof(limits));
        if(limits.joint >= NUM_JOINTS) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_JOINT, NULL, 0);
            return;
        }
        if(limits.min_angle > limits.max_angle) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_PARAM, NULL, 0);
            return;
        }
        joints[limits.joint].min_angle = limits.min_angle;
        joints[limits.joint].max_angle = limits.max_angle;
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, NULL, 0);
        return;
    }

    case SCORBOT_OP_ZERO: {
        if(payload_len != sizeof(scorbot_zero_req_t)) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_LENGTH, NULL, 0);
            return;
        }
        scorbot_zero_req_t req;
        memcpy(&req, payload, sizeof(req));
        if(req.joint >= NUM_JOINTS) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_JOINT, NULL, 0);
            return;
        }
        joint_zero(&joints[req.joint], req.angle);
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, NULL, 0);
        return;
    }

    case SCORBOT_OP_HOME: {
        if(payload_len != sizeof(scorbot_home_req_t)) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_LENGTH, NULL, 0);
            return;
        }
        scorbot_home_req_t req;
        memcpy(&req, payload, sizeof(req));
        if(req.confirm != SCORBOT_HOME_CONFIRM) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_NOT_CONFIRMED, NULL, 0);
            return;
        }
        if(scorbot_get_estop()) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_FAULTED, NULL, 0);
            return;
        }
        const uint16_t status = homing_start(req.joint_mask);
        ctrl_reply(request, hdr.opcode, hdr.xid, status, NULL, 0);
        return;
    }

    case SCORBOT_OP_HOME_STATUS: {
        scorbot_home_status_t status;
        homing_get_status(&status);
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, &status, sizeof(status));
        return;
    }

    case SCORBOT_OP_STREAM_START: {
        if(payload_len != sizeof(scorbot_stream_req_t)) {
            ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_LENGTH, NULL, 0);
            return;
        }
        scorbot_stream_req_t req;
        memcpy(&req, payload, sizeof(req));
        scorbot_session.state_period_us = req.state_period_us ? req.state_period_us : 1000u;
        scorbot_session.watchdog_ms = req.watchdog_ms ? req.watchdog_ms : 100u;
        scorbot_session_reset();
        scorbot_session.streaming = 1;
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, NULL, 0);
        return;
    }

    case SCORBOT_OP_STREAM_STOP:
        scorbot_session.streaming = 0;
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, NULL, 0);
        return;

    case SCORBOT_OP_ABORT:
        scorbot_set_estop(1);
        scorbot_set_fault(SCORBOT_FAULT_ESTOP);
        homing_abort();
        for(int i = 0; i < NUM_JOINTS; i++) {
            joints[i].mode = SCORBOT_MODE_IDLE;
            joint_stop(&joints[i]);
        }
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_OK, NULL, 0);
        return;

    default:
        ctrl_reply(request, hdr.opcode, hdr.xid, SCORBOT_ERR_BAD_OPCODE, NULL, 0);
        return;
    }
}

void scorbot_ctrl_init(void)
{
    ctrl_conn = netconn_new(NETCONN_UDP);
    if(ctrl_conn == NULL) {
        return;
    }
    if(netconn_bind(ctrl_conn, NULL, SCORBOT_CTRL_PORT) != ERR_OK) {
        netconn_delete(ctrl_conn);
        ctrl_conn = NULL;
    }
}

void scorbot_ctrl_task(void const* arg)
{
    (void)arg;
    struct netbuf* buf;

    if(ctrl_conn == NULL) {
        for(;;) {
            osDelay(1000);
        }
    }


    for(;;) {
        if(netconn_recv(ctrl_conn, &buf) != ERR_OK) {
            /* Yield: a bare continue would spin on any receive error. */
            osDelay(1);
            continue;
        }
        void* payload = NULL;
        u16_t len = 0;
        if(netbuf_data(buf, &payload, &len) == ERR_OK) {
            static uint8_t request[SCORBOT_CTRL_MAX_DATAGRAM];
            if(len <= sizeof(request)) {
                memcpy(request, payload, len);
                ctrl_handle(buf, request, len);
            }
        }
        netbuf_delete(buf);
    }
}
