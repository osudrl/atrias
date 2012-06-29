/*
 * controller_core.cpp
 *
 * Leg Torque Controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <ac_leg_torque/controller_core.h>

ControllerInitResult controllerInit() {
    ControllerInitResult cir;
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    cir.error = false;
    return cir;
}

void controllerUpdate(robot_state state, ByteArray input, ControllerOutput *output, ByteArray &status) {
    InputData *id = BYTE_ARRAY_TO_STRUCT(input, InputData*);

    output->motor_torqueA = id->leg_len_trq - id->leg_ang_trq;
    output->motor_torqueB = id->leg_len_trq + id->leg_ang_trq;
}

void controllerTakedown() {

}
