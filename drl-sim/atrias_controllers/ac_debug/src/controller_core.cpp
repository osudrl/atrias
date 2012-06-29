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
    cir.controllerStatusSize = sizeof(ControllerStatus);
    cir.error = false;
    return cir;
}

void controllerUpdate(robot_state state, ByteArray input, ControllerOutput *output, ByteArray &status) {
    InputData *id = BYTE_ARRAY_TO_STRUCT(input, InputData*);
    ControllerStatus cs;

    output->motor_torqueA = id->mtr_trqA;
    output->motor_torqueB = id->mtr_trqB;

    if (goingUp) {
    	progress += id->adjustment_speed / 500.;
    	if (progress >= 1.) {
    		progress = 1.;
    		goingUp = false;
    	}
    }
    else {
    	progress -= id->adjustment_speed / 500.;
    	if (progress <= 0.) {
    		progress = 0.;
    		goingUp = true;
    	}
    }
    cs.progress = progress;
    cs.loopTime = state.loopTime;

    structToByteArray(cs, status);
}

void controllerTakedown() {

}
