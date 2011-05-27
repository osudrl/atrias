// Devin Koepl

#include <atrias_controllers/controller.h>

extern void initialize_motor_position_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
}


extern void update_motor_position_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA = MTR_POS_CONTROLLER_DATA(data)->p_gain * (MTR_POS_CONTROLLER_DATA(data)->mtr_angA - input->motor_angleA) 
		- MTR_POS_CONTROLLER_DATA(data)->d_gain * input->motor_velocityA;
	output->motor_torqueB = MTR_POS_CONTROLLER_DATA(data)->p_gain * (MTR_POS_CONTROLLER_DATA(data)->mtr_angB - input->motor_angleB) 
		- MTR_POS_CONTROLLER_DATA(data)->d_gain * input->motor_velocityB;
}


extern void takedown_motor_position_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
}
