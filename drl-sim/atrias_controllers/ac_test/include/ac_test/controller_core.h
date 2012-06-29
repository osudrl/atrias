/*
 * controller_core.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_CORE_H_
#define CONTROLLER_CORE_H_

#include <ac_test/controller_common.h>
#include <atrias_control/core_library.h>
#include <drl_library/drl_math.h>
#include <math.h>

#define MAX_TORQUE 120

bool  in_flight;
int   stance_time;
float p_gain;
float d_gain;
float des_mtr_angA;
float des_mtr_angB;
float des_leg_angle;

void flight_controller(robot_state& state, InputData* id, ControllerOutput* output);
void stance_controller(robot_state& state, InputData* id, ControllerOutput* output);

float calcLegAngleTorque(robot_state state);
float calcLegLengthTorque(robot_state state);

#endif /* CONTROLLER_CORE_H_ */
