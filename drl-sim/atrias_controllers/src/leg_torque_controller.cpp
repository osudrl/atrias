// Devin Koepl

#include <atrias_controllers/leg_torque_controller.h>

void initialize_leg_torque_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
}


void update_leg_torque_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA = LEG_TRQ_CONTROLLER_DATA(data)->leg_len_trq - LEG_TRQ_CONTROLLER_DATA(data)->leg_ang_trq;
	output->motor_torqueB = LEG_TRQ_CONTROLLER_DATA(data)->leg_len_trq + LEG_TRQ_CONTROLLER_DATA(data)->leg_ang_trq;
}


void takedown_leg_torque_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
}
