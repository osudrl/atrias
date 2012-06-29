/*
 * controller_core.cpp
 *
 * Leg Angle Sine Wave Controller
 *
 *  Created on: May 6, 2012
 *      Author: Michael Anderson
 */

#include <ac_leg_angle_sine_wave/controller_core.h>

ControllerInitResult controllerInit() {
    accumulator_length = 0.0;
    accumulator_angle  = 0.0;

    ControllerInitResult cir;
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    cir.error = false;
    return cir;
}

void controllerUpdate(robot_state state, ByteArray input, ControllerOutput *output, ByteArray &status) {
    InputData *id = BYTE_ARRAY_TO_STRUCT(input, InputData*);

    // We use a constant timestep of 1ms because this is the rate that this function is actually being called at.
    accumulator_length += 0.002 * PI * id->leg_len_frq;
    accumulator_angle  += 0.002 * PI * id->leg_ang_frq;

    if (accumulator_length >= 2.0*PI) accumulator_length -= 2.0*PI;
    if (accumulator_angle  >= 2.0*PI) accumulator_angle  -= 2.0*PI;

    float des_leg_ang = PI/2. + id->leg_ang_amp * sin(accumulator_angle);
    float des_leg_ang_vel = 2. * PI * id->leg_ang_frq * id->leg_ang_amp * cos(accumulator_angle);
    float des_leg_len = 0.85 + id->leg_len_amp * sin(accumulator_length);
    float des_leg_len_vel = 2. * PI * id->leg_len_frq * id->leg_len_amp * cos(accumulator_length);

    float des_mtr_angA = des_leg_ang - PI + acos(des_leg_len);
    float des_mtr_angB = des_leg_ang + PI - acos(des_leg_len);

    float des_mtr_ang_velA = -des_leg_len_vel / 2. / sin(des_mtr_angA - des_mtr_angB) - des_leg_ang_vel / 2.;
    float des_mtr_ang_velB =  des_leg_len_vel / 2. / sin(des_mtr_angA - des_mtr_angB) - des_leg_ang_vel / 2.;

    output->motor_torqueA = id->p_gain * (des_mtr_angA - state.motor_angleA) + id->d_gain * (des_mtr_ang_velA - state.motor_velocityA);
    output->motor_torqueB = id->p_gain * (des_mtr_angB - state.motor_angleB) + id->d_gain * (des_mtr_ang_velB - state.motor_velocityB);
}

void controllerTakedown() {

}
