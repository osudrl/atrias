/*
 * controller_core.cpp
 *
 * Motor Position Controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <ac_motor_position/controller_core.h>

ControllerInitResult controllerInit() {
	ControllerInitResult cir;
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    cir.error = false;
    return cir;
}

void controllerUpdate(robot_state state, ByteArray input, ControllerOutput *output, ByteArray &status) {
    InputData *id = BYTE_ARRAY_TO_STRUCT(input, InputData*);

    output->motor_torqueA = id->p_gain * (id->mtr_angA - state.motor_angleA)
            - id->d_gain * state.motor_velocityA;

    output->motor_torqueB = id->p_gain * (id->mtr_angB - state.motor_angleB)
            - id->d_gain * state.motor_velocityB;
}

void controllerTakedown() {

}
