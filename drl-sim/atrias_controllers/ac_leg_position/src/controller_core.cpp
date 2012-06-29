/*
 * controller_core.cpp
 *
 * Leg Position Controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <ac_leg_position/controller_core.h>

ControllerInitResult controllerInit() {
	ControllerInitResult cir;
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    cir.error = false;
    return cir;
}

void controllerUpdate(robot_state state, ByteArray input, ControllerOutput *output, ByteArray &status) {
    InputData *id = BYTE_ARRAY_TO_STRUCT(input, InputData*);

    float des_mtr_angA = id->leg_ang - PI + acos(id->leg_len);
    float des_mtr_angB = id->leg_ang + PI - acos(id->leg_len);
    float des_hip_ang = 0.99366 * state.body_angle + 0.03705;

    des_hip_ang = CLAMP(des_hip_ang, -0.2007, 0.148);

    output->motor_torqueA = id->p_gain * (des_mtr_angA - state.motor_angleA) - id->d_gain * state.motor_velocityA;
    output->motor_torqueB = id->p_gain * (des_mtr_angB - state.motor_angleB) - id->d_gain * state.motor_velocityB;
    output->motor_torque_hip = id->hip_p_gain * (des_hip_ang - state.motor_angle_hip) - id->hip_d_gain * state.motor_velocity_hip;
}

void controllerTakedown() {

}
