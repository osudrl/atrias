// Devin Koepl

#include <atrias_controllers/raibert_controller.h>

#define ESTIMATED_SPRING_STIFFNESS			0.
#define ESTIMATED_GEAR_RATIO				20

void flight_controller(ControllerInput *, ControllerOutput *, ControllerState *, ControllerData *);
void stance_controller(ControllerInput *, ControllerOutput *, ControllerState *, ControllerData *);

void initialize_raibert_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	RAIBERT_CONTROLLER_STATE(state)->in_flight = true;
	RAIBERT_CONTROLLER_STATE(state)->after_mid_stance = false;

	RAIBERT_CONTROLLER_STATE(state)->peak_ht = 1.0;

	output->motor_torqueA    = 0.;
	output->motor_torqueB    = 0.;
	output->motor_torque_hip = 0.;

	PRINT_MSG("Raibert Controller Initialized.\n");

	RAIBERT_CONTROLLER_STATE(state)->stance_time = RAIBERT_CONTROLLER_STATE(state)->time;
}


void update_raibert_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
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
	

	// Regardless of if we are in stance or flight we control the hip the same
	// Do that now.
	float des_hip_ang = 0.99366*input->body_angle + 0.03705;

	// REMOVED BY HUBICKI!  Resulted in oscillation ourside of range
       // if ((des_hip_ang < -0.2007) || (des_hip_ang > 0.148))
       //        des_hip_ang = input->body_angle;

	//  Added Hubicki
	des_hip_ang = CLAMP( des_hip_ang, -0.2007, 0.148 );
	// End Hubicki	


	output->motor_torque_hip = RAIBERT_CONTROLLER_DATA(data)->stance_hip_p_gain * (des_hip_ang - input->hip_angle)
                - RAIBERT_CONTROLLER_DATA(data)->stance_hip_d_gain * input->hip_angle_vel;

	RAIBERT_CONTROLLER_STATE(state)->last_leg_len = cos( ( 2.*PI + input->leg_angleA - input->leg_angleB ) / 2. );

}

void takedown_raibert_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA 	 = 0.;
	output->motor_torqueB    = 0.;
	output->motor_torque_hip = 0.;
}

void flight_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
   
    	// Spring deflections for force control.  These can be problematic on the real robot, since they require good sensor calibration.
	float spring_defA = input->leg_angleA - input->motor_angleA;
	float spring_defB = input->leg_angleB - input->motor_angleB;
        
	// Negated 02/09
	float des_leg_ang = PI/2. - RAIBERT_CONTROLLER_DATA(data)->leg_ang_gain * input->xVelocity 
		+ RAIBERT_CONTROLLER_DATA(data)->hor_vel_gain * RAIBERT_CONTROLLER_DATA(data)->des_hor_vel;

	// Generate the motor torques.
	float des_mtr_angA = des_leg_ang - PI + acos(RAIBERT_CONTROLLER_DATA(data)->preferred_leg_len);
	float des_mtr_angB = des_leg_ang + PI - acos(RAIBERT_CONTROLLER_DATA(data)->preferred_leg_len); 

	float leg_angle = ( input->leg_angleA + input->leg_angleB ) / 2.;
	float leg_length = - 0.5 * sin( input->leg_angleA ) - 0.5 * sin( input->leg_angleB );

	// XXX: This is a hack to keep the robot in one place.
	// GCF = gain control factor
	float gcf = CLAMP(MAX(ABS(des_mtr_angA - input->motor_angleA), 
			      ABS(des_mtr_angB - input->motor_angleB)) / 0.05, 0, 1);

	output->motor_torqueA = gcf * RAIBERT_CONTROLLER_DATA(data)->flight_p_gain * (des_mtr_angA - input->motor_angleA) 
		- RAIBERT_CONTROLLER_DATA(data)->flight_d_gain * input->motor_velocityA;
	output->motor_torqueB = gcf * RAIBERT_CONTROLLER_DATA(data)->flight_p_gain * (des_mtr_angB - input->motor_angleB) 
		- RAIBERT_CONTROLLER_DATA(data)->flight_d_gain * input->motor_velocityB;

	//=========================================================================//

	// Figure out the next state.
        //PRINT_MSG("00<%f> <%f>", ABS(spring_defA), ABS(spring_defB));
	//if ( ( input->zPosition - leg_length * sin( leg_angle ) < 0.02 ) && ( (ABS(input->motor_angleA - input->leg_angleA) > RAIBERT_CONTROLLER_DATA(data)->stance_spring_threshold) 
	//	|| (ABS(input->motor_angleB - input->leg_angleB) > RAIBERT_CONTROLLER_DATA(data)->stance_spring_threshold) ) )
	if ( input->toe_switch == 1)
	{
		// Check to see if ground contact has occured.
		PRINT_MSG("TD!\n");

		RAIBERT_CONTROLLER_STATE(state)->in_flight = false;
	}

	// Check to see if we have reached a new peak height.
	RAIBERT_CONTROLLER_STATE(state)->peak_ht = MAX( input->zPosition, RAIBERT_CONTROLLER_STATE(state)->peak_ht );
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

//HUBICKI DEBUG ADD
//RAIBERT_CONTROLLER_STATE(state)->after_mid_stance = false;
// END DEBUG!


	// Find the leg extension during stance to add energy back into the system.
	float leg_ext = 0.;

	// If the robot has reach midstance, extend the leg.
	if ( RAIBERT_CONTROLLER_STATE(state)->after_mid_stance )
	{
		leg_ext = RAIBERT_CONTROLLER_DATA(data)->hop_ht_gain * ( RAIBERT_CONTROLLER_DATA(data)->des_hop_ht - RAIBERT_CONTROLLER_STATE(state)->peak_ht );
	}

	// Limit the desired leg length.
	float des_leg_len = CLAMP( RAIBERT_CONTROLLER_DATA(data)->preferred_leg_len + leg_ext, 0.51, 0.97 );
	float torque = RAIBERT_CONTROLLER_DATA(data)->stance_p_gain * (des_leg_len - zf_leg_len ) 
		- RAIBERT_CONTROLLER_DATA(data)->stance_d_gain * zf_leg_len_vel 
		+ ESTIMATED_SPRING_STIFFNESS * (zf_leg_len - leg_len) / ESTIMATED_GEAR_RATIO;

	float leg_angle = ( input->leg_angleA + input->leg_angleB ) / 2.;
	float leg_length = - 0.5 * sin( input->leg_angleA ) - 0.5 * sin( input->leg_angleB );

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

        //PRINT_MSG("!!<%f> <%f>", ABS(spring_defA), ABS(spring_defB));
       
	//if ( ( input->zPosition - leg_length * sin( leg_angle ) > -0.02 ) && ( ( ABS(spring_defA) < RAIBERT_CONTROLLER_DATA(data)->flight_spring_threshold )
	//	&& ( ABS(spring_defB) < RAIBERT_CONTROLLER_DATA(data)->flight_spring_threshold ) ) )
	if ( input->toe_switch == 1 )
	{
		RAIBERT_CONTROLLER_STATE(state)->stance_time = RAIBERT_CONTROLLER_STATE(state)->time;
	}

	//if ( input->toe_switch == 0 && 1000 < (RAIBERT_CONTROLLER_STATE(state)->time - RAIBERT_CONTROLLER_STATE(state)->stance_time))
	if ( input->toe_switch == 0 )
	{
		// Check to see if lift off has occured.

		PRINT_MSG("LO!\n");

		RAIBERT_CONTROLLER_STATE(state)->in_flight = true;

		// Reset peak height.
		RAIBERT_CONTROLLER_STATE(state)->peak_ht = 0.;

		RAIBERT_CONTROLLER_STATE(state)->after_mid_stance = false;
	}
}
