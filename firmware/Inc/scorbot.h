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

typedef struct __attribute__((packed)) {
    joint_cmd_t shoulder_pan;
    joint_cmd_t shoulder_lift;
    joint_cmd_t elbow;
    joint_cmd_t wrist_1;
    joint_cmd_t wrist_2;
    joint_cmd_t gripper;
} scorbot_cmd_t;

typedef struct __attribute__((packed)) {
    joint_status_t shoulder_pan;
    joint_status_t shoulder_lift;
    joint_status_t elbow;
    joint_status_t wrist_1;
    joint_status_t wrist_2;
    joint_status_t gripper;
} scorbot_status_t;

void Scorbot_MainTask();

#endif /* SCORBOT_H_ */
