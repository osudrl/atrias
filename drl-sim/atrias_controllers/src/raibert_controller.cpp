// FORCE CONTROL FOR VERTICAL HOPPING - ATRIAS 2.0 - By Mikhail Jones
// Code is originally from Devin Koepl's Raibert controller for ATRIAS 1.0



// INSTANSTIATE
#include <atrias_controllers/controller.h>

void flight_controller(ControllerInput *, ControllerOutput *, ControllerState *, ControllerData *);
void stance_controller(ControllerInput *, ControllerOutput *, ControllerState *, ControllerData *);



// INTIALIZATION ROUTINE
extern void initialize_raibert_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
	RAIBERT_CONTROLLER_STATE(state)->in_flight = true;
	RAIBERT_CONTROLLER_STATE(state)->stance_time = 0.0;
	RAIBERT_CONTROLLER_STATE(state)->spring_def_sumA = 0.0;
	RAIBERT_CONTROLLER_STATE(state)->spring_def_sumB = 0.0;
	RAIBERT_CONTROLLER_STATE(state)->toe_switch_debounce = 0.0;

	output->motor_torqueA = 0.0;
	output->motor_torqueB = 0.0;
	output->motor_torque_hip = 0.0;
}



// UPDATE ROUTINE
extern void update_raibert_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
	// UPDATE LEG CONTROL
	if (RAIBERT_CONTROLLER_STATE(state)->in_flight)
	{
		// We reset the integral error sums for each hop.		
		RAIBERT_CONTROLLER_STATE(state)->spring_def_sumA = 0.0;
		RAIBERT_CONTROLLER_STATE(state)->spring_def_sumB = 0.0;
		
		// Call flight controller.
		flight_controller(input, output, state, data);

		// Debounce toe switch
		if (input->toe_switch == 0.0)
		{
			RAIBERT_CONTROLLER_STATE(state)->toe_switch_debounce += 0.0;
		}
		else
		{
			RAIBERT_CONTROLLER_STATE(state)->toe_switch_debounce += 1.0;
		}

		// Check to see if we are still in flight, if not we go into stance.
		// Based on spring deflection. (Will not work as is for multiple hops with current flight spring oscillations)
		//if ((ABS(input->motor_angleA - input->leg_angleA) > 0.007) && (ABS(input->motor_angleB - input->leg_angleB) > 0.007))
		// Based on toe switch.
		//if ((input->toe_switch == 1.0) && (RAIBERT_CONTROLLER_STATE(state)->toe_switch_debounce >= 3.0))
		// Based on z position.	(Will not work as is for horizontal hopping)	
		if (input->zPosition <= 0.908)
		{
			RAIBERT_CONTROLLER_STATE(state)->in_flight = false;
		}
	}
	else
	{
		// Call stance controller.
		stance_controller(input, output, state, data);

		// Debounce toe switch
		if (input->toe_switch == 1.0)
		{
			RAIBERT_CONTROLLER_STATE(state)->toe_switch_debounce += 0.0;
		}
		else
		{
			RAIBERT_CONTROLLER_STATE(state)->toe_switch_debounce += 1.0;
		}

		// Check to see if we are in stance, if not we go into flight.
		// Based on toe switch and time of stance.
		//if  ((input->toe_switch == 0.0) && (RAIBERT_CONTROLLER_STATE(state)->stance_time > 50.0))
		// Based on toe switch.
		//if ((input->toe_switch == 0.0) && (RAIBERT_CONTROLLER_STATE(state)->toe_switch_debounce >= 3.0))
		// Based on z position and time of stance.
		if  ((input->zPosition > 0.908) && (RAIBERT_CONTROLLER_STATE(state)->stance_time > 50.0))
		{
			RAIBERT_CONTROLLER_STATE(state)->in_flight = true;
		}
	}	
	
	// UPDATE HIP CONTROL
	// We control the hip the same in flight and stance, the equation is based on experimental test to prevent lateral forces on the knee joints.	
	float desired_hip_angle = 0.99366 * input->body_angle + 0.03705;
	desired_hip_angle = CLAMP(desired_hip_angle, -0.2007, 0.148);

	// Hip PD control.
	output->motor_torque_hip = (RAIBERT_CONTROLLER_DATA(data)->stance_hip_p_gain * (desired_hip_angle - input->hip_angle)) - (RAIBERT_CONTROLLER_DATA(data)->stance_hip_d_gain * input->hip_angle_vel);

	// MISC
	// Make data avaiable to log.
	input->phase = RAIBERT_CONTROLLER_STATE(state)->in_flight;
}



// TAKEDOWN ROUTINE
extern void takedown_raibert_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
	// Set all motor torque's to zero.
	output->motor_torqueA = 0.0;
	output->motor_torqueB = 0.0;
	output->motor_torque_hip = 0.0;
}



// FLIGHT CONTROLLER
void flight_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
	// Reset stance based variables.
	RAIBERT_CONTROLLER_STATE(state)->stance_time = 0.0;
	RAIBERT_CONTROLLER_STATE(state)->spring_def_sumA = 0.0;
	RAIBERT_CONTROLLER_STATE(state)->spring_def_sumB = 0.0;

	// Get values from GUI
	float leg_angle_gain = RAIBERT_CONTROLLER_DATA(data)->hor_vel_gain;
  
	// Desired leg angle during flight.
	float desired_leg_angle = PI / 2.0 + leg_angle_gain;

	// Calculate desired motor angles during flight.
	input->desired_motor_angleA = desired_leg_angle - PI + acos(RAIBERT_CONTROLLER_DATA(data)->preferred_leg_len);
	input->desired_motor_angleB = desired_leg_angle + PI - acos(RAIBERT_CONTROLLER_DATA(data)->preferred_leg_len);

	// Leg PD control.
	output->motor_torqueA = (RAIBERT_CONTROLLER_DATA(data)->flight_p_gain * (input->desired_motor_angleA - input->motor_angleA)) - (RAIBERT_CONTROLLER_DATA(data)->flight_d_gain * input->motor_velocityA);
	output->motor_torqueB = (RAIBERT_CONTROLLER_DATA(data)->flight_p_gain * (input->desired_motor_angleB - input->motor_angleB)) - (RAIBERT_CONTROLLER_DATA(data)->flight_d_gain * input->motor_velocityB);

	input->desired_spring_defA = 0.0;
	input->desired_spring_defB = 0.0;
}



// STANCE CONTROLLER
void stance_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
	
	//*********************
	//   SAFETY VALUES
	//*********************

	// Values for safety limits.  If outside of these, a damping controller engages

	// Leg length limits
	float safety_length_long = 0.97;
	float safety_length_short = 0.6;

	//  Motor A angle limits
	float safety_angleA_short = -2./3.*PI;
	float safety_angleA_long = 0.;	

	// Motor B angle limits
	float safety_angleB_short = PI;
	float safety_angleB_long = 5./3.*PI;	



	//*********************
	//     CONSTANTS
	//*********************

	float SPRING_CONSTANT = 1301.9;



	//************************
	//  STATE CALCULATIONS
	//***********************
	
	// Leg length calculation
	float leg_length = cos((2.0 * PI + input->leg_angleA - input->leg_angleB ) / 2.0);

	// Calculate spring deflections.
	float spring_defA = input->motor_angleA - input->leg_angleA;
	float spring_defB = input->motor_angleB - input->leg_angleB;
	

	//************************
	//      GUI VALUES
	//************************

	// Get values from GUI
	float HOP_GAIN = RAIBERT_CONTROLLER_DATA(data)->des_hop_ht;
	float FORCE_P_GAIN = RAIBERT_CONTROLLER_DATA(data)->hop_ht_gain;
	float FORCE_D_GAIN = RAIBERT_CONTROLLER_DATA(data)->leg_ang_gain;
	float FORCE_I_GAIN = RAIBERT_CONTROLLER_DATA(data)->des_hor_vel;
	

	//****************************************
	//  FORCE/TORQUE/DEFLECTION COMPUTATIONS
	//****************************************

	// Quartic Coefficients fitted to MATLAB Simulation Force Profile
	float coeffA = -1.4442*pow(10,6);
	float coeffB =  6.4643*pow(10,5);
	float coeffC = -1.6339*pow(10,5);
	float coeffD =  2.0380*pow(10,4);
	float coeffE =  0;

	float t = RAIBERT_CONTROLLER_STATE(state)->stance_time;
	// Quartic approximation of force profile
	float desired_force = coeffA*pow(t,4) + coeffB*pow(t,3) + coeffC*pow(t,2) + coeffD*pow(t,1) + coeffE;

	// Forces to Torques based on current kinematic configuration
	float desired_torque = 0.5*desired_force*cos(PI + asin(leg_length));

	// Desired Spring Deflections
	input->desired_spring_defA = CLAMP(HOP_GAIN * (desired_torque/SPRING_CONSTANT), -0.3, 0.0);
	input->desired_spring_defB = CLAMP(HOP_GAIN * (-desired_torque/SPRING_CONSTANT), 0.0, 0.3);


	// Spring deflection profile based on data from a 1m z-position drop with 0.90m leg length. 
//	input->desired_spring_defA = CLAMP(HOP_GAIN * (0.0000149620790 * pow(RAIBERT_CONTROLLER_STATE(state)->stance_time, 2.0) - 0.00318251866 * RAIBERT_CONTROLLER_STATE(state)->stance_time), -0.3, 0.0);
//	input->desired_spring_defB = CLAMP(HOP_GAIN * (-0.0000149620790 * pow(RAIBERT_CONTROLLER_STATE(state)->stance_time, 2.0) + 0.00318251866 * RAIBERT_CONTROLLER_STATE(state)->stance_time), 0.0, 0.3);


	//************************
	//    CONTROL SIGNAL
	//************************

	// Compute feed-forward torque signal for force tracking (Soley based on spring deflection, later could include motor velocity and inertia.)
	float feed_forward_torqueA = 310.0 * input->desired_spring_defA;
	float feed_forward_torqueB = 310.0 * input->desired_spring_defB;

	// Deflection sum error for integral gain
	RAIBERT_CONTROLLER_STATE(state)->spring_def_sumA = CLAMP(RAIBERT_CONTROLLER_STATE(state)->spring_def_sumA + (input->desired_spring_defA - spring_defA), -10.0, 10.0);
	RAIBERT_CONTROLLER_STATE(state)->spring_def_sumB = CLAMP(RAIBERT_CONTROLLER_STATE(state)->spring_def_sumB + (input->desired_spring_defB - spring_defB), -10.0, 10.0);

	// When leg length reaches virtual safety limits, add damping to slow motors before real hardstops are hit.
	if(leg_length != CLAMP(leg_length, safety_length_short, safety_length_long) || 
		input->motor_angleA != CLAMP(input->motor_angleA, safety_angleA_short, safety_angleA_long) || 
		input->motor_angleB != CLAMP(input->motor_angleB, safety_angleB_short, safety_angleB_long))
	{
		output->motor_torqueA = - 100.0 * input->motor_velocityA;
		output->motor_torqueB = - 100.0 * input->motor_velocityB;
	}
	else
	{
		// Leg A, PID control with feed forward.
		input->desired_motor_angleA = input->motor_angleA + (input->desired_spring_defA - spring_defA);
		output->motor_torqueA = (FORCE_P_GAIN * (input->desired_motor_angleA - input->motor_angleA)) - (FORCE_D_GAIN * (input->motor_velocityA)) + (FORCE_I_GAIN * RAIBERT_CONTROLLER_STATE(state)->spring_def_sumA) + feed_forward_torqueA;

		// Leg B, PID control with feed forward.
		input->desired_motor_angleB = input->motor_angleB + (input->desired_spring_defB - spring_defB);
		output->motor_torqueB = (FORCE_P_GAIN * (input->desired_motor_angleB - input->motor_angleB)) - (FORCE_D_GAIN * (input->motor_velocityB)) + (FORCE_I_GAIN * RAIBERT_CONTROLLER_STATE(state)->spring_def_sumB) + feed_forward_torqueB;
	}

	// Update stance time.
	RAIBERT_CONTROLLER_STATE(state)->stance_time += 1.0;
}
