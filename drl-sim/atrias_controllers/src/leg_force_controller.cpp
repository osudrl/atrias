#include <atrias_controllers/controller.h>

extern void initialize_leg_force_controller(ControllerInput* input, ControllerOutput* output, ControllerState* state, 
	ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
}



extern void update_leg_force_controller(ControllerInput* input, ControllerOutput* output, ControllerState* state, ControllerData *data)
{
	output->motor_torqueA = FORCE_CONTROLLER_DATA(data)->p_gain * (((-1.0)*FORCE_CONTROLLER_DATA(data)->spring_deflection) - (input->motor_angleA - input->leg_angleA)) 
		- FORCE_CONTROLLER_DATA(data)->d_gain * (input->motor_velocityA);
	output->motor_torqueB = FORCE_CONTROLLER_DATA(data)->p_gain * ((FORCE_CONTROLLER_DATA(data)->spring_deflection) - (input->motor_angleB - input->leg_angleB)) 
		- FORCE_CONTROLLER_DATA(data)->d_gain * (input->motor_velocityB);
}


extern void takedown_leg_force_controller(ControllerInput* input, ControllerOutput* output, ControllerState* state, ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
}
