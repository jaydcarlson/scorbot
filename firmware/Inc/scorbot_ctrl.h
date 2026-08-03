#pragma once

#include <stdint.h>

/*
 * Control plane server: strict call/response on SCORBOT_CTRL_PORT. Carries
 * discovery, session setup, motor modes, PID gains, homing and abort. Runs in
 * its own task because these operations are rare and may block briefly, and
 * must not interfere with the streaming data plane.
 */

void scorbot_ctrl_init(void);
void scorbot_ctrl_task(void const* arg);
