#pragma once

#include "joint.h"
#include "scorbot_proto.h"

/*
 * Non-blocking homing sequence.
 *
 * The original implementation spun on the limit switch inside joint_home(),
 * which blocked whichever task called it and left no way to abort. This is a
 * state machine instead: homing_start() arms it and homing_tick() advances it,
 * so the control loop, telemetry and the control plane all keep running while
 * the arm is moving.
 *
 * Each joint makes two passes at its switch. A fast seek finds it, the joint
 * backs off until the switch releases, then a slow creep re-approaches. The
 * second, slow trip is what makes the reference repeatable, and backing off
 * first is what fixes the coupled-wrist drift the old code warned about.
 */

void homing_init(void);

/* Arms the sequence for every joint whose bit is set. Returns an scorbot_status. */
uint16_t homing_start(uint8_t joint_mask);

/* Advances the state machine. Call periodically; now_ms is a millisecond tick. */
void homing_tick(uint32_t now_ms);

/* Stops immediately and leaves every joint idle. */
void homing_abort(void);

void homing_get_status(scorbot_home_status_t* out);

uint8_t homing_state(void);
uint8_t homing_active(void);
uint8_t homing_current_joint(void);
