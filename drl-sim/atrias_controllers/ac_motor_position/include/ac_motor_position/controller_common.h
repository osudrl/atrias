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
    float mtr_angA;
    float mtr_angB;
    float p_gain;
    float d_gain;
} InputData;

#endif /* CONTROLLER_COMMON_H_ */
