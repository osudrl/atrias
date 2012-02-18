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
}


void takedown_leg_angle_sin_wave(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
}
