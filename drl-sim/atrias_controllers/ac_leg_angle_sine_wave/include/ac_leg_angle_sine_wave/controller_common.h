/*
 * controller_common.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_COMMON_H_
#define CONTROLLER_COMMON_H_

#include <atrias_control/controller.h>

typedef struct {
    // Inputs
    float leg_ang_frq;
    float leg_ang_amp;
    float leg_len_frq;
    float leg_len_amp;
    float p_gain;
    float d_gain;
} InputData;

#endif /* CONTROLLER_COMMON_H_ */
