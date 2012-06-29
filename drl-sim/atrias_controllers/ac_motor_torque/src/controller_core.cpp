/*
 * controller_core.cpp
 *
 * Motor Torque Controller
 *
 *  Created on: May 2, 2012
 *      Author: Michael Anderson
 */

#include <ac_motor_torque/controller_core.h>

ControllerInitResult controllerInit() {
	ControllerInitResult cir;
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    cir.error = false;
    return cir;
}

void controllerUpdate(robot_state state, ByteArray input, ControllerOutput *output, ByteArray &status) {
    InputData *id = BYTE_ARRAY_TO_STRUCT(input, InputData*);

    output->motor_torqueA = id->mtr_trqA;
    output->motor_torqueB = id->mtr_trqB;
    output->motor_torque_hip = id->mtr_trq_hip;
}

void controllerTakedown() {

}
