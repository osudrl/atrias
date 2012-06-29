/*
 * controller_core.cpp
 *
 * Touchdown Velocity Test Controller
 *
 *  Created on: June 20, 2012
 */

#include <ac_touchdown_velocity_test/controller_core.h>

ControllerInitResult controllerInit() {
	in_flight = true;
	stance_time = 0;
	cur_sample_index = 0;
	for (int i = 0; i < AVG_SAMPLES; i++) {
		hor_vel_samples[i]  = 0.0;
		vert_vel_samples[i] = 0.0;
	}
	accum_hor_samples  =  0.0;
	accum_vert_samples =  0.0;
	smoothed_hor_vel   =  0.0;
	smoothed_vert_vel  =  0.0;
	rem_air_time       =  0.0;
	p_gain             =  0.0;
	d_gain             =  0.0;
	des_mtr_angA       = -1.0;
	des_mtr_angB       =  4.0;
	des_mtr_velA       =  0.0;
	des_mtr_velB       =  0.0;
	des_leg_angle      = PI/2;
	des_td_ang_vel     =  0.0;
	
	ControllerInitResult cir;
	cir.controllerInputSize = sizeof(InputData);
	cir.controllerStatusSize = 0;
	cir.error = false;
	return cir;
}

void reset_averages(robot_state& state) {
	cur_sample_index = 0;
	for (int i = 0; i < AVG_SAMPLES; i++) {
		hor_vel_samples[i]  = 0.0;
		// We have to make it looks as if it's been flying for a little while.
		vert_vel_samples[i] = state.zVelocity + (AVG_SAMPLES - i - 1) * .001 * G;
	}
	accum_hor_samples  = 0.0;
	accum_vert_samples = 0.0;
	smoothed_hor_vel   = 0.0;
	smoothed_vert_vel  = state.zVelocity;
}

void calc_averages(robot_state& state) {
	accum_hor_samples  += state.xVelocity;
	accum_vert_samples += state.zVelocity - cur_sample_index * 0.001 * G;
	
	smoothed_hor_vel  += (state.xVelocity -  hor_vel_samples[cur_sample_index]) / ((float) AVG_SAMPLES);
	// No need to compensate for acceleration here as the corrections cancel out.
	smoothed_vert_vel += (state.zVelocity - vert_vel_samples[cur_sample_index]) / ((float) AVG_SAMPLES);
	
	hor_vel_samples[cur_sample_index]  = state.xVelocity;
	vert_vel_samples[cur_sample_index] = state.zVelocity;

	cur_sample_index++;
	// Reset our average, to avoid (theoretical?) numerical stability issues.
	if (cur_sample_index >= AVG_SAMPLES) {
		cur_sample_index   = 0;
		smoothed_hor_vel   = accum_hor_samples  / ((float) AVG_SAMPLES);
		smoothed_vert_vel  = accum_vert_samples / ((float) AVG_SAMPLES);
		accum_hor_samples  = 0.0;
		accum_vert_samples = 0.0;
	}
}

// Calculates the time until touchdown -- needs updated averages
void calc_rem_air_time(robot_state& state, InputData* id) {
	// Implements the quadratic formula -- basic projectile physics
	float discriminant = powf(smoothed_vert_vel, 2.0) + 2 * G * (state.zPosition - id->desiredLegLength * sinf(id->desiredTDAngle));
	if (discriminant < 0.0) discriminant = 0.0;
	rem_air_time = (smoothed_vert_vel + sqrtf(discriminant)) / G;
}

// This calculates the desired angular velocity at touchdown -- may be changed without touching other code.
void calc_des_td_ang_vel(robot_state& state, InputData* id) {
	des_td_ang_vel = smoothed_hor_vel / (id->desiredLegLength * sinf(id->desiredTDAngle));
}

void controllerUpdate(robot_state state, ByteArray input, ControllerOutput *output, ByteArray &status) {
	InputData *id = BYTE_ARRAY_TO_STRUCT(input, InputData*);

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
	
	output->motor_torqueA = p_gain * (des_mtr_angA - state.motor_angleA)
		+ d_gain * (des_mtr_velA - state.motor_velocityA);
	output->motor_torqueB = p_gain * (des_mtr_angB - state.motor_angleB)
		+ d_gain * (des_mtr_velB - state.motor_velocityB);

	// log robot status
	output->in_flight = in_flight;
}

void flight_controller(robot_state& state, InputData* id, ControllerOutput* output) {
	calc_averages(state);
	calc_rem_air_time(state, id);
	calc_des_td_ang_vel(state, id);
	
	if (rem_air_time > id->desiredAccelTime) {
		p_gain        = id->flightP;
		d_gain        = id->flightD;
		des_leg_angle = id->desiredTDAngle - 0.5 * des_td_ang_vel * id->desiredAccelTime;
		des_mtr_velA  = 0.0;
		des_mtr_velB  = 0.0;
	} else {
		p_gain = id->accelP;
		d_gain = id->accelD;
		des_mtr_velA = des_mtr_velB = des_td_ang_vel * (1.0 - rem_air_time / id->desiredAccelTime);
		des_leg_angle = id->desiredTDAngle - 0.5 * rem_air_time * (des_td_ang_vel + des_mtr_velA);
	}

	// Check to see if ground contact has occured.
	if ((state.zPosition <= id->touchdown_height) && (state.toe_switch == 1)) {
		in_flight = false;
		stance_time = 0;
	}
}

void stance_controller(robot_state& state, InputData* id, ControllerOutput* output) {
	p_gain = id->stanceP;
	d_gain = id->stanceD;
	
	des_mtr_velA = 0.0;
	des_mtr_velB = 0.0;
	
	des_leg_angle = (state.leg_angleA + state.leg_angleB) / 2.0;

	// Check to see if lift off has occured.
	if ((stance_time > 50) && (state.zPosition >= id->takeoff_height) && (state.toe_switch == 0)) {
		in_flight = true;
		reset_averages(state);
	}
	stance_time += 1;
}

void controllerTakedown() {

}
