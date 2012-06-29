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
    float stanceP;
    float stanceD;
    float flightP;
    float flightD;
    float hipP;
    float hipD;

    float desiredLegLength;
    float desiredLegAngle;
    float takeoff_height;
    float touchdown_height;
} InputData;

#endif /* CONTROLLER_COMMON_H_ */
