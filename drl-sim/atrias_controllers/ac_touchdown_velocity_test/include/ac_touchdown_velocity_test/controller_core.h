/*
 * controller_core.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_CORE_H_
#define CONTROLLER_CORE_H_

#include <ac_touchdown_velocity_test/controller_common.h>
#include <atrias_control/core_library.h>
#include <drl_library/drl_math.h>
#include <math.h>

#define AVG_SAMPLES  100
#define           G 9.81

bool  in_flight;
int   stance_time;
float smoothed_hor_vel;
float smoothed_vert_vel;
float hor_vel_samples[AVG_SAMPLES];
float vert_vel_samples[AVG_SAMPLES];
int   cur_sample_index;
float accum_hor_samples;
float accum_vert_samples;
float rem_air_time;
float p_gain;
float d_gain;
float des_mtr_angA;
float des_mtr_angB;
float des_mtr_velA;
float des_mtr_velB;
float des_leg_angle;
float des_td_ang_vel;

void flight_controller(robot_state& state, InputData* id, ControllerOutput* output);
void stance_controller(robot_state& state, InputData* id, ControllerOutput* output);

void calc_averages();
void reset_averages();
void calc_rem_air_time();

#endif /* CONTROLLER_CORE_H_ */
