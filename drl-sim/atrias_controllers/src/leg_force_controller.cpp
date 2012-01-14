#include <atrias_controllers/controller.h>

extern void initialize_leg_force_controller(ControllerInput* input, ControllerOutput* output, ControllerState* state, 
	ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
	FORCE_CONTROLLER_STATE(state)->ringBufferLocation = 0;
}



extern void update_leg_force_controller(ControllerInput* input, ControllerOutput* output, ControllerState* state, ControllerData *data)
{	
	float springASum = 0;
	float springBSum = 0;
	int i;

	// Put the current spring deflections into the ring buffers
	FORCE_CONTROLLER_STATE(state)->ringBufferLocation = FORCE_CONTROLLER_STATE(state)->ringBufferLocation % 250;
	FORCE_CONTROLLER_STATE(state)->springDeflectionA[FORCE_CONTROLLER_STATE(state)->ringBufferLocation] = ((FORCE_CONTROLLER_DATA(data)->spring_deflection) - (input->motor_angleA - input->leg_angleA));
	FORCE_CONTROLLER_STATE(state)->springDeflectionB[FORCE_CONTROLLER_STATE(state)->ringBufferLocation++] = ((FORCE_CONTROLLER_DATA(data)->spring_deflection) - (input->motor_angleB - input->leg_angleB));

	// Generate sun
	for (i = 0; i < 250; i++) {
		springASum += FORCE_CONTROLLER_STATE(state)->springDeflectionA[i];
		springBSum += FORCE_CONTROLLER_STATE(state)->springDeflectionB[i];
	}

	output->motor_torqueA = FORCE_CONTROLLER_DATA(data)->p_gainA * (((-1.0)*FORCE_CONTROLLER_DATA(data)->spring_deflection) - (input->motor_angleA - input->leg_angleA)) 
		- FORCE_CONTROLLER_DATA(data)->d_gainA * (input->motor_velocityA) + (FORCE_CONTROLLER_DATA(data)->i_gainA * (springASum/250.0));
	output->motor_torqueB = FORCE_CONTROLLER_DATA(data)->p_gainB * ((FORCE_CONTROLLER_DATA(data)->spring_deflection) - (input->motor_angleB - input->leg_angleB)) 
		- FORCE_CONTROLLER_DATA(data)->d_gainB * (input->motor_velocityB) + (FORCE_CONTROLLER_DATA(data)->i_gainB * (springBSum/250.0));
}


extern void takedown_leg_force_controller(ControllerInput* input, ControllerOutput* output, ControllerState* state, ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
}
