// Devin Koepl

#include <atrias_controllers/controller.h>

extern void initialize_motor_torque_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
}


extern void update_motor_torque_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA = MTR_TRQ_CONTROLLER_DATA(data)->mtr_trqA;
	output->motor_torqueB = MTR_TRQ_CONTROLLER_DATA(data)->mtr_trqB;
	output->motor_torque_hip = MTR_TRQ_CONTROLLER_DATA(data)->mtr_trq_hip;
}


extern void takedown_motor_torque_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
}
