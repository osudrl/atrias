#include <atrias_controllers/controller.h>

extern void initialize_leg_position_controller(ControllerInput* input, ControllerOutput* output, ControllerState* state, 
	ControllerData *data)
{
	output->motor_torqueA = 0.;
	output->motor_torqueB = 0.;
	output->motor_torque_hip = 0.;
}


extern void update_leg_position_controller(ControllerInput* input, ControllerOutput* output, ControllerState* state, ControllerData *data)
{
	float des_mtr_angA = LEG_POS_CONTROLLER_DATA(data)->leg_ang - PI + acos( LEG_POS_CONTROLLER_DATA(data)->leg_len );
	float des_mtr_angB = LEG_POS_CONTROLLER_DATA(data)->leg_ang + PI - acos( LEG_POS_CONTROLLER_DATA(data)->leg_len );

	output->motor_torqueA = LEG_POS_CONTROLLER_DATA(data)->p_gain * (des_mtr_angA - input->motor_angleA) 
		- LEG_POS_CONTROLLER_DATA(data)->d_gain * input->motor_velocityA;
	//printk("P: %d\n", (int) LEG_POS_CONTROLLER_DATA(data)->p_gain);
	//printk("D: %d\n", (int) LEG_POS_CONTROLLER_DATA(data)->d_gain);
	//printk("V: %d\n", (int) input->motor_velocityA);
	output->motor_torqueB = LEG_POS_CONTROLLER_DATA(data)->p_gain * (des_mtr_angB - input->motor_angleB) 
		- LEG_POS_CONTROLLER_DATA(data)->d_gain * input->motor_velocityB;

	output->motor_torque_hip = LEG_POS_CONTROLLER_DATA(data)->hip_p_gain * (LEG_POS_CONTROLLER_DATA(data)->hip_ang - input->hip_angle)
		- ((int)(LEG_POS_CONTROLLER_DATA(data)->hip_d_gain) * (int)input->hip_angle_vel);
	//printk("H: %d\n", (int) output->motor_torque_hip);
}


extern void takedown_leg_position_controller(ControllerInput* input, ControllerOutput* output, ControllerState* state, ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = 0.;
}
