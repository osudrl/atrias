#include <atrias_controllers/controller.h>

#define RAIBERT_ESTIMATED_SPRING_STIFFNESS  0.
#define RAIBERT_ESTIMATED_GEAR_RATIO        20

void RAIBERT_flight_controller(ControllerInput *, ControllerOutput *, ControllerState *, ControllerData *);
void RAIBERT_stance_controller(ControllerInput *, ControllerOutput *, ControllerState *, ControllerData *);

float movavg2,movavg1,movavg0,tapex,yapex;

extern void initialize_raibert_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	
	RAIBERT_CONTROLLER_STATE(state)->in_flight = true;
	RAIBERT_CONTROLLER_STATE(state)->after_mid_stance = false;
	RAIBERT_CONTROLLER_STATE(state)->stance_time = 0.0;
	RAIBERT_CONTROLLER_STATE(state)->peak_ht = 1.0;	//peak height

	movavg2=input->zVelocity;
	movavg1=input->zVelocity;
	movavg=input->zVelocity;

	output->motor_torqueA    = 0.;
	output->motor_torqueB    = 0.;
	output->motor_torque_hip = 0.;

	PRINT_MSG("RAIBERT Controller Initialized.\n");

//	RAIBERT_CONTROLLER_STATE(state)->time_of_last_stance = RAIBERT_CONTROLLER_STATE(state)->time;
}


extern void update_raibert_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	//RAIBERT_CONTROLLER_STATE(state)->in_flight = false;

	if ( RAIBERT_CONTROLLER_STATE(state)->in_flight )
	{
		RAIBERT_flight_controller(input, output, state, data);
	}
	else
	{
		RAIBERT_stance_controller(input, output, state, data);
	}	
	

	// Regardless of if we are in stance or flight we control the hip the same
	// Do that now.
	float des_hip_ang = 0.99366*input->body_angle + 0.03705;

	//  Added RAIBERT
	des_hip_ang = CLAMP( des_hip_ang, -0.2007, 0.148 );
	// End RAIBERT	

	output->motor_torque_hip = RAIBERT_CONTROLLER_DATA(data)->stance_hip_p_gain * (des_hip_ang - input->hip_angle)
                - RAIBERT_CONTROLLER_DATA(data)->stance_hip_d_gain * input->hip_angle_vel;

	RAIBERT_CONTROLLER_STATE(state)->last_leg_len = cos( ( 2.*PI + input->leg_angleA - input->leg_angleB ) / 2. );

}


extern void takedown_raibert_controller (ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA 	 = 0.;
	output->motor_torqueB    = 0.;
	output->motor_torque_hip = 0.;
}

void RAIBERT_flight_controller (ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	RAIBERT_CONTROLLER_STATE(state)->stance_time = 0.0;
	float stance_trigger_height = 0.910; 
 //   float des_mtr_angA = RAIBERT_CONTROLLER_DATA(data)->des_hor_vel + PI/2 - PI + acos(0.9);
 //   float des_mtr_angB = RAIBERT_CONTROLLER_DATA(data)->des_hor_vel + PI/2 + PI - acos(0.9);
	// calculate moving average of z-velocity
	movavg2=movavg1;
	movavg1=movavg;
	movavg=(movavg2+movavg1+input->zVelocity)/3; //current averaged z-velocity
	// find apex
	if (movavg1 > 0 ) && (movavg <=0 )
	{
			tapex=input->time;
			yapex=input->zPosition;
	}
	// adjust TD-angle
	if (movavg<=0) && (tapex != input->time) && (((yapex-9.81/2*(input->time-tapex)^2)/RAIBERT_CONTROLLER_DATA(data)->leg_length)<=1)
	{
		des_leg_angle=asin((yapex-9.81/2*((input->time-tapex)*1000)^2)/RAIBERT_CONTROLLER_DATA(data)->leg_length);
	}
	else
	{
		des_leg_ang=PI/2;
	{
	// clamp TD-angle
	des_leg_ang = CLAMP( des_leg_ang, 1.2, PI/2 );
	// convert into motor angle
	float des_mtr_angA = des_leg_angle + PI/2 - PI + acos(0.9);
    float des_mtr_angB = des_leg_angle + PI/2 + PI - acos(0.9);
	float des_hip_ang = 0.99366*input->body_angle + 0.03705;
	
	printk("A: %d, B: %d\n",(int)(des_mtr_angA*100),(int)(des_mtr_angB*100));

	//	if ((des_hip_ang < -0.2007) || (des_hip_ang > 0.148))
	des_hip_ang = CLAMP(des_hip_ang,-0.2007,0.148);

	output->motor_torqueA = RAIBERT_CONTROLLER_DATA(data)->flight_p_gain * (des_mtr_angA - input->motor_angleA) 
		- RAIBERT_CONTROLLER_DATA(data)->flight_d_gain * input->motor_velocityA;
	output->motor_torqueB = RAIBERT_CONTROLLER_DATA(data)->flight_p_gain * (des_mtr_angB - input->motor_angleB) 
		- RAIBERT_CONTROLLER_DATA(data)->flight_d_gain * input->motor_velocityB;

	output->motor_torque_hip = RAIBERT_CONTROLLER_DATA(data)->stance_hip_p_gain * (des_hip_ang - input->hip_angle)
		- RAIBERT_CONTROLLER_DATA(data)->stance_hip_d_gain * input->hip_angle_vel;
//	printk("                                                                                              %d\n",(int)((des_hip_ang)*1000));
	if (( input->zPosition <= stance_trigger_height) && (input->toe_switch == 1))
	{
		// Check to see if ground contact has occured.
		PRINT_MSG("TD!\n");

		RAIBERT_CONTROLLER_STATE(state)->in_flight = false;
	}

}

void RAIBERT_stance_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	float des_mtr_angA = (input->leg_angleA + input->leg_angleB)/2.0 - PI + acos(0.9);
	float des_mtr_angB = (input->leg_angleA + input->leg_angleB)/2.0 + PI - acos(0.9);
	float des_hip_ang = 0.99366*input->body_angle + 0.03705;
	float stance_trigger_height = 0.910; 

//	if ((des_hip_ang < -0.2007) || (des_hip_ang > 0.148))
	des_hip_ang = CLAMP(des_hip_ang,-0.2007,0.148);

	output->motor_torqueA = RAIBERT_CONTROLLER_DATA(data)->stance_p_gain * (des_mtr_angA - input->motor_angleA) 
		- RAIBERT_CONTROLLER_DATA(data)->stance_d_gain * input->motor_velocityA;
	output->motor_torqueB = RAIBERT_CONTROLLER_DATA(data)->stance_p_gain * (des_mtr_angB - input->motor_angleB) 
		- RAIBERT_CONTROLLER_DATA(data)->stance_d_gain * input->motor_velocityB;

	output->motor_torque_hip = RAIBERT_CONTROLLER_DATA(data)->stance_hip_p_gain * (des_hip_ang - input->hip_angle)
		- RAIBERT_CONTROLLER_DATA(data)->stance_hip_d_gain * input->hip_angle_vel;
//	printk("                                                                                              %d\n",(int)((des_hip_ang)*1000));
	if ((RAIBERT_CONTROLLER_STATE(state)->stance_time > 50) && (input->zPosition >= stance_trigger_height-0.01) && (input->toe_switch == 0))
	{
		// Check to see if lift off has occured.

		PRINT_MSG("LO!\n");

		RAIBERT_CONTROLLER_STATE(state)->in_flight = true;


	}	
	RAIBERT_CONTROLLER_STATE(state)->stance_time += 1.0;

}

