#pragma once

#include <stdint.h>
#include "scorbot_proto.h"

/*
 * Streaming data plane. Two tasks: one blocks on incoming poses and applies
 * them, the other emits telemetry on a periodic tick. They are deliberately
 * separate so a pose carrying SCORBOT_POSE_FLAG_REPLY_NOW can be answered the
 * instant it lands rather than waiting for the next tick.
 */

typedef struct {
    uint32_t id;              /* 0 when no host holds the data plane        */
    uint32_t owner_ip;
    uint32_t host_ip;
    uint16_t host_port;
    uint8_t host_mac[6];
    uint32_t watchdog_ms;
    uint32_t state_period_us;
    uint8_t streaming;
} scorbot_session_t;

extern scorbot_session_t scorbot_session;

void scorbot_udp_init(void);
void scorbot_udp_rx_task(void const* arg);
void scorbot_udp_tx_task(void const* arg);

/* Called by the control plane when a session opens, closes or is stolen. */
void scorbot_session_reset(void);

void scorbot_set_estop(uint8_t engaged);
uint8_t scorbot_get_estop(void);
void scorbot_set_fault(uint8_t code);
uint8_t scorbot_get_fault(void);
uint8_t scorbot_watchdog_tripped(void);
