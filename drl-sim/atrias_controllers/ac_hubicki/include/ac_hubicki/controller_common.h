/*
 * controller_common.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_COMMON_H_
#define CONTROLLER_COMMON_H_

#include <atrias_control/controller.h>
#include <atrias_control/ucontroller.h>

typedef struct {
    float des_hor_vel;
    float des_hop_ht;
    float hor_vel_gain;
    float hop_ht_gain;
    float leg_ang_gain;
    float stance_p_gain;
    float stance_d_gain;
    float stance_spring_threshold;
    float preferred_leg_len;
    float flight_p_gain;
    float flight_d_gain;
    float flight_spring_threshold;
    float stance_hip_p_gain;
    float stance_hip_d_gain;
    float flight_hip_p_gain;
    float flight_hip_d_gain;
} InputData;

typedef struct {
    bool in_flight;
} ControllerStatus;

#endif /* CONTROLLER_COMMON_H_ */
