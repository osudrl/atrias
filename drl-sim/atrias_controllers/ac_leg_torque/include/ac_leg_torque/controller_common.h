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
    float leg_ang_trq;
    float leg_len_trq;
} InputData;

#endif /* CONTROLLER_COMMON_H_ */
