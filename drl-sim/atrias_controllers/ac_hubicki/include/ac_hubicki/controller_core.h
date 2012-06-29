/*
 * controller_core.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_CORE_H_
#define CONTROLLER_CORE_H_

#include <ac_hubicki/controller_common.h>
#include <atrias_control/core_library.h>
#include <drl_library/drl_math.h>
#include <math.h>

#define ESTIMATED_SPRING_STIFFNESS  0.
#define ESTIMATED_GEAR_RATIO        20

bool in_flight;
bool after_mid_stance;
bool after_mid_flight;

float eclapsedTime;

float peak_ht;
float last_leg_len;
float last_stance_hip_angle;

float time_lo;
float time_td;

float time_of_last_stance;
float last_hip_angle;

void hubicki_flight_controller(robot_state* state, InputData* id, ControllerStatus* cs, ControllerOutput* output);
void hubicki_stance_controller(robot_state* state, InputData* id, ControllerStatus* cs, ControllerOutput* output);

#endif /* CONTROLLER_CORE_H_ */
