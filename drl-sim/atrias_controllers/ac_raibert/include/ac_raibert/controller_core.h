/*
 * controller_core.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_CORE_H_
#define CONTROLLER_CORE_H_

#include <ac_raibert/controller_common.h>
#include <atrias_control/core_library.h>
#include <drl_library/drl_math.h>
#include <math.h>

bool in_flight;

float peak_ht;
float stance_time;

float spring_def_sumA;
float spring_def_sumB;

void flight_controller(robot_state* state, InputData* id, ControllerOutput* output);
void stance_controller(robot_state* state, InputData* id, ControllerOutput* output);

#endif /* CONTROLLER_CORE_H_ */
