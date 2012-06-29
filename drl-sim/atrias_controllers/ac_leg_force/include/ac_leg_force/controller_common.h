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
    float p_gainA;
    float d_gainA;
    float i_gainA;
    float p_gainB;
    float d_gainB;
    float i_gainB;
    float spring_deflection;
} InputData;

#endif /* CONTROLLER_COMMON_H_ */
