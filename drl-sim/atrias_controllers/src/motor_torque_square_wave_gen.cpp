// Devin Koepl

#include <atrias_controllers/leg_angle_sin_wave.h>

#define MID_MOT_ANG_A	 -1.0254
#define MID_MOT_ANG_B		4.167
#define F_SLOW .1

typedef struct
{
	float time;
} SineData;

void initialize_leg_angle_sin_wave(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	SIN_WAVE_CONTROLLER_STATE(state)->time = 0.;

	output->motor_torqueA = output->motor_torqueB = 0.;
}


void update_leg_angle_sin_wave(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	// We use a constant timestep of 1ms because this is the rate that this function is actually being called at.
	SIN_WAVE_CONTROLLER_STATE(state)->time += 0.001;
/*
	float des_leg_ang = PI/2. + SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_amp 
		* sin(2. * PI * SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_frq * SIN_WAVE_CONTROLLER_STATE(state)->time)
		* pow(sin(2.*PI * F_SLOW * SIN_WAVE_CONTROLLER_STATE(state)->time),4.0);
	float des_leg_ang_vel = 2. * PI * SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_frq * SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_amp 
		* cos(2. * PI * SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_frq * SIN_WAVE_CONTROLLER_STATE(state)->time) * pow(sin(2.*PI*F_SLOW*SIN_WAVE_CONTROLLER_STATE(state)->time),4.0)
		+ 8. * PI * F_SLOW * SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_amp
 		* sin(2. * PI * SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_frq * SIN_WAVE_CONTROLLER_STATE(state)->time) * cos(2.*PI*F_SLOW*SIN_WAVE_CONTROLLER_STATE(state)->time) * pow(sin(2*PI*F_SLOW*SIN_WAVE_CONTROLLER_STATE(state)->time),3.0);
*/

	// *****************************************************
	// HUBICKI DEBUG:  ATTEMPTING TO REPLICATE "TORQUE CUT-OUT" PROBLEM
	// *****************************************************
	float constant_current_A = 7.5;
	float constant_current_B = 15.;
	int repos_period = 5*1000;

	int signal_period = (int) (1/SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_frq*1000);
	int time_ms = (int)(SIN_WAVE_CONTROLLER_STATE(state)->time * 1000);
	

	output->motor_torqueA = 0;
	output->motor_torqueB = 0;

	float reset_position_A = -PI/4;
	float reset_position_B = 5*PI/4;

	if((time_ms % repos_period) < 1000)
	{
		// Reposition Motor
		output->motor_torqueA = SIN_WAVE_CONTROLLER_DATA(data)->p_gain*(reset_position_A - input->motor_angleA) + SIN_WAVE_CONTROLLER_DATA(data)->d_gain*(-1*(input->motor_velocityA));
		output->motor_torqueB = SIN_WAVE_CONTROLLER_DATA(data)->p_gain*(reset_position_B - input->motor_angleB) + SIN_WAVE_CONTROLLER_DATA(data)->d_gain*(-1*(input->motor_velocityB));
	}
	else if((time_ms % signal_period) < (signal_period / 4))
	{
		// Motor positive
		output->motor_torqueA = constant_current_A;
		output->motor_torqueB = constant_current_B;
	}
	else if((time_ms % signal_period) < (signal_period / 2))
	{
		// Motor off
		output->motor_torqueA = 0.;
		output->motor_torqueB = 0.;

	}
	else if((time_ms % signal_period) < 3*(signal_period / 4))
	{
		// Motor negative
		output->motor_torqueA = -1*constant_current_A;
		output->motor_torqueB = -1*constant_current_B;

	}
	else
	{
		// Motor off
		output->motor_torqueA = 0.;
		output->motor_torqueB = 0.;

	}
	



/*

	float des_leg_ang = PI/2. + SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_amp 
                * sin(2. * PI * SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_frq * SIN_WAVE_CONTROLLER_STATE(state)->time);
        float des_leg_ang_vel = 2. * PI * SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_frq * SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_amp 
                * cos(2. * PI * SIN_WAVE_CONTROLLER_DATA(data)->leg_ang_frq * SIN_WAVE_CONTROLLER_STATE(state)->time);


	float des_leg_len = 0.85 + SIN_WAVE_CONTROLLER_DATA(data)->leg_len_amp 
		* sin(2. * PI * SIN_WAVE_CONTROLLER_DATA(data)->leg_len_frq * SIN_WAVE_CONTROLLER_STATE(state)->time);
	float des_leg_len_vel = 2. * PI * SIN_WAVE_CONTROLLER_DATA(data)->leg_len_frq * SIN_WAVE_CONTROLLER_DATA(data)->leg_len_amp 
		* cos(2. * PI * SIN_WAVE_CONTROLLER_DATA(data)->leg_len_frq * SIN_WAVE_CONTROLLER_STATE(state)->time);

	float des_mtr_angA = des_leg_ang - PI + acos(des_leg_len);
	float des_mtr_angB = des_leg_ang + PI - acos(des_leg_len); 

	float des_mtr_ang_velA = -des_leg_len_vel / 2. / sin(des_mtr_angA - des_mtr_angB) - des_leg_ang_vel / 2.;
	float des_mtr_ang_velB =	des_leg_len_vel / 2. / sin(des_mtr_angA - des_mtr_angB) - des_leg_ang_vel / 2.;

	output->motor_torqueA = SIN_WAVE_CONTROLLER_DATA(data)->p_gain * (des_mtr_angA - input->motor_angleA) 
		+ SIN_WAVE_CONTROLLER_DATA(data)->d_gain * (des_mtr_ang_velA - input->motor_velocityA);
	output->motor_torqueB = SIN_WAVE_CONTROLLER_DATA(data)->p_gain * (des_mtr_angB - input->motor_angleB) 
		+ SIN_WAVE_CONTROLLER_DATA(data)->d_gain * (des_mtr_ang_velB - input->motor_velocityB);
*/
}


void takedown_leg_angle_sin_wave(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
}
