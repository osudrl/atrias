// Devin Koepl

#include <atrias_controllers/controller.h>

#define ESTIMATED_SPRING_STIFFNESS			0.
#define ESTIMATED_GEAR_RATIO				20

void flight_controller(ControllerInput *, ControllerOutput *, ControllerState *, ControllerData *);
void stance_controller(ControllerInput *, ControllerOutput *, ControllerState *, ControllerData *);

extern void initialize_raibert_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	RAIBERT_CONTROLLER_STATE(state)->in_flight = true;
	RAIBERT_CONTROLLER_STATE(state)->after_mid_stance = false;

	RAIBERT_CONTROLLER_STATE(state)->peak_ht = 1.0;

	output->motor_torqueA = output->motor_torqueB 				= 0.;

	PRINT_MSG("Raibert Controller Initialized.\n");
}


extern void update_raibert_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	//RAIBERT_CONTROLLER_STATE(state)->in_flight = false;

	if ( RAIBERT_CONTROLLER_STATE(state)->in_flight )
	{
		flight_controller(input, output, state, data);
	}
	else
	{
		stance_controller(input, output, state, data);
	}	

	RAIBERT_CONTROLLER_STATE(state)->last_leg_len = cos( ( 2.*PI + input->leg_angleA - input->leg_angleB ) / 2. );
}

extern void takedown_raibert_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
}

void flight_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	float des_leg_ang = PI/2. + RAIBERT_CONTROLLER_DATA(data)->leg_ang_gain * input->horizontal_velocity 
		- RAIBERT_CONTROLLER_DATA(data)->hor_vel_gain * RAIBERT_CONTROLLER_DATA(data)->des_hor_vel;

	// Generate the motor torques.
	float des_mtr_angA = des_leg_ang - PI + acos(RAIBERT_CONTROLLER_DATA(data)->preferred_leg_len);
	float des_mtr_angB = des_leg_ang + PI - acos(RAIBERT_CONTROLLER_DATA(data)->preferred_leg_len); 

	output->motor_torqueA = RAIBERT_CONTROLLER_DATA(data)->flight_p_gain * (des_mtr_angA - input->motor_angleA) 
		- RAIBERT_CONTROLLER_DATA(data)->flight_d_gain * input->motor_velocityA;
	output->motor_torqueB = RAIBERT_CONTROLLER_DATA(data)->flight_p_gain * (des_mtr_angB - input->motor_angleB) 
		- RAIBERT_CONTROLLER_DATA(data)->flight_d_gain * input->motor_velocityB;

	//=========================================================================//

	// Figure out the next state.

	if ( (ABS(input->motor_angleA - input->leg_angleA) > RAIBERT_CONTROLLER_DATA(data)->stance_spring_threshold) 
		|| (ABS(input->motor_angleB - input->leg_angleB) > RAIBERT_CONTROLLER_DATA(data)->stance_spring_threshold) )
	{
		// Check to see if ground contact has occured.
		PRINT_MSG("TD!");

		RAIBERT_CONTROLLER_STATE(state)->in_flight = false;
	}

	// Check to see if we have reached a new peak height.
	RAIBERT_CONTROLLER_STATE(state)->peak_ht = MAX( input->height, RAIBERT_CONTROLLER_STATE(state)->peak_ht );
}

void stance_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	// Spring deflections for force control.  These can be problematic on the real robot, since they require good sensor calibration.
	float spring_defA = input->leg_angleA - input->motor_angleA;
	float spring_defB = input->leg_angleB - input->motor_angleB;
	
	//float spring_def_velA = input->leg_velocityA - input->motor_velocityA;
	//float spring_def_velB = input->leg_velocityB - input->motor_velocityB;

	// Limit the desired leg length to help prevent smacking hardstops.
	float leg_len			= cos( ( 2.*PI + input->leg_angleA - input->leg_angleB ) / 2. );
	float zf_leg_len	= cos( ( 2.*PI + input->motor_angleA - input->motor_angleB ) / 2. ); // zero force leg length
	float zf_leg_len_vel = -sin( ( 2.*PI + input->motor_angleA - input->motor_angleB ) / 4. 
		* ( input->motor_velocityA - input->motor_velocityB ) );

	// Check to see if the robot has reached midstance.  If it has, set the after mid stance flag.
	if ( ( !RAIBERT_CONTROLLER_STATE(state)->after_mid_stance ) && ( leg_len > RAIBERT_CONTROLLER_STATE(state)->last_leg_len ) )
	{
		RAIBERT_CONTROLLER_STATE(state)->after_mid_stance = true;
	}

	// Find the leg extension during stance to add energy back into the system.
	float leg_ext = 0.;

	// If the robot has reach midstance, extend the leg.
	if ( RAIBERT_CONTROLLER_STATE(state)->after_mid_stance )
	{
		leg_ext = RAIBERT_CONTROLLER_DATA(data)->hop_ht_gain * ( RAIBERT_CONTROLLER_DATA(data)->des_hop_ht - RAIBERT_CONTROLLER_STATE(state)->peak_ht );
	}

	// Limit the desired leg length.
	float des_leg_len = CLAMP( RAIBERT_CONTROLLER_DATA(data)->preferred_leg_len + leg_ext, 0.51, 1.005 );
	float torque = RAIBERT_CONTROLLER_DATA(data)->stance_p_gain * (des_leg_len - zf_leg_len ) 
		- RAIBERT_CONTROLLER_DATA(data)->stance_d_gain * zf_leg_len_vel 
		+ ESTIMATED_SPRING_STIFFNESS * (zf_leg_len - leg_len) / ESTIMATED_GEAR_RATIO;

	output->motor_torqueA =	 -torque;
	output->motor_torqueB =	 torque;

	//float des_leg_ang = (input->leg_angleA + input->leg_angleB) / 2.;
	//float des_leg_ang_vel = (input->leg_velocityA + input->leg_velocityB) / 2.;		

	// Deadband for force control.
	/*if ( ABS( des_leg_ang - (input->motor_angleA + input->motor_angleB) / 2. ) < 0.015 )
	{
		des_leg_ang = (input->motor_angleA + input->motor_angleB) / 2.;
	}

	if ( ABS(des_leg_ang_vel ) < 0.1 )
	{
		des_leg_ang_vel = 0.;
	}*/

	//float des_mtr_angA = des_leg_ang - PI + acos(des_leg_len);
	//float des_mtr_angB = des_leg_ang + PI - acos(des_leg_len); 

	// Compute the leg torque for zero hip moment and maintaining hopping height.
	//output->motor_torqueA = RAIBERT_CONTROLLER_DATA(data)->stance_p_gain * (des_mtr_angA - input->motor_angleA) 
	//	+ RAIBERT_CONTROLLER_DATA(data)->stance_d_gain * (des_leg_ang_vel - input->motor_velocityA) + ESTIMATED_SPRING_STIFFNESS * spring_defA / ESTIMATED_GEAR_RATIO;
	//output->motor_torqueB = RAIBERT_CONTROLLER_DATA(data)->stance_p_gain * (des_mtr_angB - input->motor_angleB) 
	//	+ RAIBERT_CONTROLLER_DATA(data)->stance_d_gain * (des_leg_ang_vel - input->motor_velocityB) + ESTIMATED_SPRING_STIFFNESS * spring_defB / ESTIMATED_GEAR_RATIO;

	// Clamp the torques for now, for added safety.
	//output->motor_torqueA = CLAMP( output->motor_torqueA, -3., 3. );
	//output->motor_torqueB = CLAMP( output->motor_torqueB, -3., 3. );

	if ( ( ABS(spring_defA) < RAIBERT_CONTROLLER_DATA(data)->flight_spring_threshold )
		&& ( ABS(spring_defB) < RAIBERT_CONTROLLER_DATA(data)->flight_spring_threshold ) )
	{
		// Check to see if lift off has occured.

		PRINT_MSG("LO!");

		RAIBERT_CONTROLLER_STATE(state)->in_flight = true;

		// Reset peak height.
		RAIBERT_CONTROLLER_STATE(state)->peak_ht = 0.;

		RAIBERT_CONTROLLER_STATE(state)->after_mid_stance = false;
	}
}
