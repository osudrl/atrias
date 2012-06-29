/*
 * controller_core.cpp
 *
 * Test Controller
 *
 *  Created on: June 20, 2012
 */

#include <ac_test/controller_core.h>

#include "hacky_controller_logger.c"

ControllerInitResult controllerInit() {
	in_flight = true;
	stance_time = 0;
	p_gain             =  0.0;
	d_gain             =  0.0;
	des_mtr_angA       = -1.0;
	des_mtr_angB       =  4.0;
	des_leg_angle      = PI/2;
	
	init_logger();
	
	ControllerInitResult cir;
	cir.controllerInputSize = sizeof(InputData);
	cir.controllerStatusSize = 0;
	cir.error = false;
	return cir;
}

float calcLegAngleTorque(robot_state state) {
	return (p_gain * (des_mtr_angA - state.motor_angleA + des_mtr_angB - state.motor_angleB) -
	        d_gain * (state.motor_velocityA + state.motor_velocityB)) / 2.0;
}

float calcLegLengthTorque(robot_state state) {
	return (p_gain * (-des_mtr_angA + state.motor_angleA + des_mtr_angB - state.motor_angleB) -
	        d_gain * (-state.motor_velocityA + state.motor_velocityB)) / 2.0;
}

void controllerUpdate(robot_state state, ByteArray input, ControllerOutput *output, ByteArray &status) {
	InputData *id = BYTE_ARRAY_TO_STRUCT(input, InputData*);
	
	if(state.command == CMD_DISABLE) {
		send_log();
		reset_log();
	}

	if (in_flight) {
		flight_controller(state, id, output);
	} else {
		stance_controller(state, id, output);
	}
	
	des_mtr_angA  = des_leg_angle - PI + acos(id->desiredLegLength);
	des_mtr_angB  = des_leg_angle + PI - acos(id->desiredLegLength);

	float des_hip_ang = 0.99366 * state.body_angle + 0.03705;
	des_hip_ang = CLAMP( des_hip_ang, -0.2007, 0.148 );
	
	output->motor_torque_hip = id->hipP * (des_hip_ang - state.motor_angle_hip)
		- id->hipD * state.motor_velocity_hip;
	
	float legAngTorque = CLAMP(calcLegAngleTorque(state), -MAX_TORQUE, MAX_TORQUE);
	float legLenTorque = CLAMP(calcLegLengthTorque(state), fabsf(legAngTorque) - MAX_TORQUE, MAX_TORQUE - fabsf(legAngTorque));
	
	output->motor_torqueA = legAngTorque - legLenTorque;
	output->motor_torqueB = legAngTorque + legLenTorque;

	// Log debug data.
    output->in_flight           = in_flight;
    output->des_leg_angle       = des_leg_angle;
    output->des_hip_angle       = des_hip_ang;
    output->stance_time         = stance_time;

    // For Ryan's hacky_controller_logger.
    log_write(state, id, output, in_flight, des_hip_ang, (float) 0.0, (float) 0.0, des_mtr_angA, des_mtr_angB, stance_time);
}

void flight_controller(robot_state& state, InputData* id, ControllerOutput* output) {
	p_gain        = id->flightP;
	d_gain        = id->flightD;
	des_leg_angle = id->desiredLegAngle;

	// Check to see if ground contact has occured.
	if ((state.zPosition <= id->touchdown_height) && (state.toe_switch == 1)) {
		in_flight = false;
		stance_time = 0;
	}
}

void stance_controller(robot_state& state, InputData* id, ControllerOutput* output) {
	p_gain = id->stanceP;
	d_gain = id->stanceD;
	
	des_leg_angle = (state.leg_angleA + state.leg_angleB) / 2.0;

	// Check to see if lift off has occured.
	if ((stance_time > 50) && (state.zPosition >= id->takeoff_height) && (state.toe_switch == 0)) {
		in_flight = true;
	}
	stance_time += 1;
}

void controllerTakedown() {

}
