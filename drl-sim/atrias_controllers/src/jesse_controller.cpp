#include <atrias_controllers/controller.h>

#define HUBICKI_ESTIMATED_SPRING_STIFFNESS  0.
#define HUBICKI_ESTIMATED_GEAR_RATIO        20

void hubicki_flight_controller(ControllerInput *, ControllerOutput *, ControllerState *, ControllerData *);
void hubicki_stance_controller(ControllerInput *, ControllerOutput *, ControllerState *, ControllerData *);

extern void initialize_hubicki_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	HUBICKI_CONTROLLER_STATE(state)->in_flight = true;
	HUBICKI_CONTROLLER_STATE(state)->after_mid_stance = false;
	HUBICKI_CONTROLLER_STATE(state)->stance_time = 0.0;
	HUBICKI_CONTROLLER_STATE(state)->peak_ht = 1.0;	//peak height

	output->motor_torqueA    = 0.;
	output->motor_torqueB    = 0.;
	output->motor_torque_hip = 0.;

	PRINT_MSG("Hubicki Controller Initialized.\n");

	HUBICKI_CONTROLLER_STATE(state)->time_of_last_stance = HUBICKI_CONTROLLER_STATE(state)->time;
}


extern void update_hubicki_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	//HUBICKI_CONTROLLER_STATE(state)->in_flight = false;

	if ( HUBICKI_CONTROLLER_STATE(state)->in_flight )
	{
		hubicki_flight_controller(input, output, state, data);
	}
	else
	{
		hubicki_stance_controller(input, output, state, data);
	}	
	

	// Regardless of if we are in stance or flight we control the hip the same
	// Do that now.
	float des_hip_ang = 0.99366*input->body_angle + 0.03705;

	//  Added Hubicki
	des_hip_ang = CLAMP( des_hip_ang, -0.2007, 0.148 );
	// End Hubicki	

	output->motor_torque_hip = HUBICKI_CONTROLLER_DATA(data)->stance_hip_p_gain * (des_hip_ang - input->hip_angle)
                - HUBICKI_CONTROLLER_DATA(data)->stance_hip_d_gain * input->hip_angle_vel;

	HUBICKI_CONTROLLER_STATE(state)->last_leg_len = cos( ( 2.*PI + input->leg_angleA - input->leg_angleB ) / 2. );

}


extern void takedown_hubicki_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA 	 = 0.;
	output->motor_torqueB    = 0.;
	output->motor_torque_hip = 0.;
}

void hubicki_flight_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	float RAIBERT_CONTROLLER_STATE(state)->stance_time = 0.0;
	float stance_trigger_height = 0.910; 
    float des_mtr_angA = LEG_POS_CONTROLLER_DATA(data)->leg_ang - PI + acos( LEG_POS_CONTROLLER_DATA(data)->leg_len );
	float des_mtr_angB = LEG_POS_CONTROLLER_DATA(data)->leg_ang + PI - acos( LEG_POS_CONTROLLER_DATA(data)->leg_len );
	float des_hip_ang = 0.99366*input->body_angle + 0.03705;

	//	if ((des_hip_ang < -0.2007) || (des_hip_ang > 0.148))
	des_hip_ang = CLAMP(des_hip_ang,-0.2007,0.148);

	output->motor_torqueA = LEG_POS_CONTROLLER_DATA(data)->flight_p_gain * (des_mtr_angA - input->motor_angleA) 
		- LEG_POS_CONTROLLER_DATA(data)->flight_d_gain * input->motor_velocityA;
	output->motor_torqueB = LEG_POS_CONTROLLER_DATA(data)->flight_p_gain * (des_mtr_angB - input->motor_angleB) 
		- LEG_POS_CONTROLLER_DATA(data)->flight_d_gain * input->motor_velocityB;

	output->motor_torque_hip = LEG_POS_CONTROLLER_DATA(data)->hip_p_gain * (des_hip_ang - input->hip_angle)
		- LEG_POS_CONTROLLER_DATA(data)->hip_d_gain * input->hip_angle_vel;
//	printk("                                                                                              %d\n",(int)((des_hip_ang)*1000));
	if ( input->zPosition <= stance_trigger_height) && (input->toe_switch == 1)
	{
		// Check to see if ground contact has occured.
		PRINT_MSG("TD!\n");

		HUBICKI_CONTROLLER_STATE(state)->in_flight = false;
	}

}

void hubicki_stance_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	float des_mtr_angA = LEG_POS_CONTROLLER_DATA(data)->leg_ang - PI + acos( LEG_POS_CONTROLLER_DATA(data)->leg_len );
	float des_mtr_angB = LEG_POS_CONTROLLER_DATA(data)->leg_ang + PI - acos( LEG_POS_CONTROLLER_DATA(data)->leg_len );
	float des_hip_ang = 0.99366*input->body_angle + 0.03705;

//	if ((des_hip_ang < -0.2007) || (des_hip_ang > 0.148))
	des_hip_ang = CLAMP(des_hip_ang,-0.2007,0.148);

	output->motor_torqueA = LEG_POS_CONTROLLER_DATA(data)->stance_p_gain * (des_mtr_angA - input->motor_angleA) 
		- LEG_POS_CONTROLLER_DATA(data)->stance_d_gain * input->motor_velocityA;
	output->motor_torqueB = LEG_POS_CONTROLLER_DATA(data)->stance_p_gain * (des_mtr_angB - input->motor_angleB) 
		- LEG_POS_CONTROLLER_DATA(data)->stance_d_gain * input->motor_velocityB;

	output->motor_torque_hip = LEG_POS_CONTROLLER_DATA(data)->hip_p_gain * (des_hip_ang - input->hip_angle)
		- LEG_POS_CONTROLLER_DATA(data)->hip_d_gain * input->hip_angle_vel;
//	printk("                                                                                              %d\n",(int)((des_hip_ang)*1000));
	if (RAIBERT_CONTROLLER_STATE(state)->stance_time > 10) && (input->zPosition >= stance_trigger_height-0.01) && (input->toe_switch == 0)
	{
		// Check to see if lift off has occured.

		PRINT_MSG("LO!\n");

		HUBICKI_CONTROLLER_STATE(state)->in_flight = true;


	}	
	RAIBERT_CONTROLLER_STATE(state)->stance_time += 1.0;

}

