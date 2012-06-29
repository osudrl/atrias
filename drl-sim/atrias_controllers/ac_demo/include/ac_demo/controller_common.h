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
    uint8_t commandedState;
    uint8_t commandedDemo;
    float amplitude;
    float p_gain;
    float d_gain;
    float hip_p_gain;
    float hip_d_gain;
} InputData;

#endif /* CONTROLLER_COMMON_H_ */
