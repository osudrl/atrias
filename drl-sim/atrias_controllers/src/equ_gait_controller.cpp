// Devin Koepl

#include <atrias_robot/controller.h>

extern void initialize_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	input->motor_angleA = input->motor_angleB = input->leg_angleA = input->leg_angleB 
		= input->horizontal_velocity = input->vertical_velocity = 0.;

	output->motor_torqueA = output->motor_torqueB = 0;

	state->last_input = *input;
	state->in_flight = TRUE;

	state->last_input = *input;

	state->lut = (LUT_2dof*)MALLOC(sizeof(LUT_2dof));

	create2dof_lut(state->lut);

//	state->in_flight = true;
}


//////////////////////////////////////////////////////////////////////////
//	Equilibrium Gait Leg Angle Controller.
extern void update_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	/*if ((ABS(input->leg_angleA) > MIN_CONTACT_DISPLACEMENT) || (ABS(input->leg_angleB) > MIN_CONTACT_DISPLACEMENT))
	{
		stanceController(input, output, state);
	}
	else
	{
		flightController(input, output, state);
	}

	state->last_input = *input;*/

	//output->motor_torqueA += 0.0001;
	//output->motor_torqueB += 0.0001;

	//PRINT_MSG("\n\n\n************ Controller Update **************\n\n\n");

	output->motor_torqueA = 1. - output->motor_torqueA;
}


extern void flightController(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	float desired_horizontal_velocity = -2.0;
	float desired_leg_angle = linear_interp(state->lut, input->horizontal_velocity, input->vertical_velocity); 
		//- 0.15 * (desired_horizontal_velocity - input->horizontal_velocity);


	float desired_motor_angleA =  desired_leg_angle - PI/2. + FOUR_BAR_ANGLE;
	float desired_motor_angleB = -desired_leg_angle + PI/2. + FOUR_BAR_ANGLE; 
	
	float spring_compensating_torqueA = -SPRING_STIFFNESS * input->leg_angleA / GEAR_RATIO;
	float spring_compensating_torqueB = -SPRING_STIFFNESS * input->leg_angleB / GEAR_RATIO;

	float timestep = input->timestamp - state->last_input.timestamp; 

	// Approximate the velocity of the motors.
	float vel_motorA = (input->motor_angleA - state->last_input.motor_angleA) / timestep;
	float vel_motorB = (input->motor_angleB - state->last_input.motor_angleB) / timestep;

	float PD_torqueA = KP_GAIN * (desired_motor_angleA - input->motor_angleA) - KD_GAIN * vel_motorA;
	float PD_torqueB = KP_GAIN * (desired_motor_angleB - input->motor_angleB) - KD_GAIN * vel_motorB;

	output->motor_torqueA = spring_compensating_torqueA + PD_torqueA;
	output->motor_torqueB = spring_compensating_torqueB + PD_torqueB;
}


void stanceController(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
	//float spring_compensating_torque = -SPRING_STIFFNESS * (input->leg_angleA + input->leg_angleB) / GEAR_RATIO;
	float maintain_hopping_height_torque_scalor = 1.0175;

	//float P_hinge = 0.;
	//float hinge_error = SPRING_STIFFNESS * (input->leg_angleA - input->leg_angleB) / GEAR_RATIO; 		

	/*if (input->leg_angleA > input->leg_angleB)
	{
		output->motor_torqueA = spring_compensating_torque * maintain_hopping_height_torque_scalor;
		output->motor_torqueB = 0.;
	}
	else
	{
		output->motor_torqueA = 0.;
		output->motor_torqueB = spring_compensating_torque * maintain_hopping_height_torque_scalor;
	}*/

	//ROS_INFO("saA = %.3f,\tsaB = %.3f,\tmtA = %.3f,\tmtB = %.3f", input->leg_angleA, input->leg_angleB, output->motor_torqueA, output->motor_torqueB);

	//output->motor_torqueA = spring_compensating_torque * maintain_hopping_height_torque_scalor / 2.;
	//output->motor_torqueB = spring_compensating_torque * maintain_hopping_height_torque_scalor / 2.;

	output->motor_torqueA = -SPRING_STIFFNESS * input->leg_angleB * maintain_hopping_height_torque_scalor / GEAR_RATIO;
	output->motor_torqueB = -SPRING_STIFFNESS * input->leg_angleA * maintain_hopping_height_torque_scalor / GEAR_RATIO;

}

//////////////////////////////////////////////////////////////////////////


////	Simple PD position control of the motors.
//void update_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
//{
//	float desired_angle = 0.;

//	// Approximate the velocity of the motors.
//	float vel_motorA = (input->motor_angleA - state->last_input.motor_angleA) / TIMESTEP;
//	float vel_motorB = (input->motor_angleB - state->last_input.motor_angleB) / TIMESTEP;

//	output->motor_torqueA = KP_GAIN * (desired_angle - input->motor_angleA) - KD_GAIN * vel_motorA;
//	output->motor_torqueB = KP_GAIN * (desired_angle - input->motor_angleB) - KD_GAIN * vel_motorB;

//	state->last_input = *input;
//}



////	Spring compensating control with PD position control of motors.
//void update_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
//{
//	float desired_motor_angle = 0.6;

////	ROS_INFO("Theta = %.3f rad\n\tvx = %.3f m/s, vy = %.3f m/s.", desired_leg_angle, input->horizontal_velocity, input->vertical_velocity);
//	
//	float spring_compensating_torqueA = -SPRING_STIFFNESS * input->leg_angleA / GEAR_RATIO;
//	float spring_compensating_torqueB = -SPRING_STIFFNESS * input->leg_angleB / GEAR_RATIO;

//	// Approximate the velocity of the motors.
//	float vel_motorA = (input->motor_angleA - state->last_input.motor_angleA) / TIMESTEP;
//	float vel_motorB = (input->motor_angleB - state->last_input.motor_angleB) / TIMESTEP;

//	float PD_torqueA = KP_GAIN * (desired_motor_angle - input->motor_angleA) - KD_GAIN * vel_motorA;
//	float PD_torqueB = KP_GAIN * (desired_motor_angle - input->motor_angleB) - KD_GAIN * vel_motorB;

//	output->motor_torqueA = spring_compensating_torqueA + PD_torqueA;
//	output->motor_torqueB = spring_compensating_torqueB + PD_torqueB;

//	state->last_input = *input;
//}




////	Floppy leg.
//void update_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
//{
//	output->motor_torqueA = 0.;
//	output->motor_torqueB = 0.;

//	state->last_input = *input;
//}



void takedown_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
	//FREE(state->lut);
}
