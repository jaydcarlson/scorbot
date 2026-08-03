/*
 * server.h
 *
 *  Created on: Jan 17, 2018
 *      Author: jay
 */

#ifndef SCORBOT_H_
#define SCORBOT_H_

#include "motor.h"
#include "joint.h"

/*
 * The fixed six-joint WebSocket command and status structs that used to live
 * here are gone. scorbot_pose_t and scorbot_state_t in scorbot_proto.h replace
 * them: eight joint slots, per-joint modes, sequence numbers and echo fields,
 * and one definition shared with the host rather than two that could drift.
 */

void Scorbot_MainTask();

#endif /* SCORBOT_H_ */
