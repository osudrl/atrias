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
    float mtr_trqA;
    float mtr_trqB;
    float adjustment_speed;
} InputData;

typedef struct {
    uint16_t loopTime;
    double progress;
} ControllerStatus;

#endif /* CONTROLLER_COMMON_H_ */
