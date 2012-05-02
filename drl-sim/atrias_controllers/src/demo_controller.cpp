// Devin Koepl

#include <atrias_controllers/demo_controller.h>

#define MID_MOT_ANG_A	 -1.0254
#define MID_MOT_ANG_B		4.167
#define F_SLOW .1

#define START_TIME 5.0

typedef struct
{
	float time;
} SineData;

void initialize_demo_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
	DEMO_CONTROLLER_STATE(state)->time = 0.;

	output->motor_torqueA = output->motor_torqueB = output->motor_torque_hip = 0.;

	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_ang = 0.0; 
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_ang_vel = 0.0;
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_len = 0.0;
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_len_vel = 0.0;
	DEMO_CONTROLLER_STATE(state)->desiredPos.hip_ang = 0.0; 
	DEMO_CONTROLLER_STATE(state)->desiredPos.hip_ang_vel = 0.0;

	DEMO_CONTROLLER_STATE(state)->currentState = DEMO_STATE_STOPPED;
	DEMO_CONTROLLER_STATE(state)->currentDemo = DEMO_CONTROLLER_DATA(data)->commandedDemo;
}


void update_demo_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	// We use a constant timestep of 1ms because this is the rate that this function is actually being called at.
	DEMO_CONTROLLER_STATE(state)->time += 0.001;

	if (DEMO_CONTROLLER_STATE(state)->currentState == DEMO_STATE_STOPPED)
	{
		// If the current state is stopped then we can switch to another controller
		DEMO_CONTROLLER_STATE(state)->currentDemo = DEMO_CONTROLLER_DATA(data)->commandedDemo;

		if (DEMO_CONTROLLER_DATA(data)->commandedState == 1) 
		{
			DEMO_CONTROLLER_STATE(state)->time = 0.;
			DEMO_CONTROLLER_STATE(state)->currentState = DEMO_STATE_STARTING;
			printk("ENABLING\n");
		}
	}

	else if (DEMO_CONTROLLER_STATE(state)->currentState == DEMO_STATE_STARTING)
	{
		start_demo(input, output, state, data);
	}

	else if (DEMO_CONTROLLER_STATE(state)->currentState == DEMO_STATE_RUNNING)
	{
		if (DEMO_CONTROLLER_STATE(data)->currentDemo == DEMO1)
			DEMO_CONTROLLER_STATE(state)->desiredPos = demo1(input, output, state, data, DEMO_CONTROLLER_STATE(state)->time);
		if (DEMO_CONTROLLER_STATE(data)->currentDemo == DEMO2)
			DEMO_CONTROLLER_STATE(state)->desiredPos = demo2(input, output, state, data, DEMO_CONTROLLER_STATE(state)->time);
		if (DEMO_CONTROLLER_STATE(data)->currentDemo == DEMO3)
			DEMO_CONTROLLER_STATE(state)->desiredPos = demo3(input, output, state, data, DEMO_CONTROLLER_STATE(state)->time);
		if (DEMO_CONTROLLER_STATE(data)->currentDemo == DEMO4)
			DEMO_CONTROLLER_STATE(state)->desiredPos = demo4(input, output, state, data, DEMO_CONTROLLER_STATE(state)->time);
			

		// If we are currently running a demo and the stop demo button is pressed, go to demo_stopping
		if (DEMO_CONTROLLER_DATA(data)->commandedState == 0)
		{
			DEMO_CONTROLLER_STATE(state)->lastDemoPos = DEMO_CONTROLLER_STATE(state)->desiredPos;
			DEMO_CONTROLLER_STATE(state)->currentState = DEMO_STATE_STOPPING;
			DEMO_CONTROLLER_STATE(state)->time = 0.0;
		}
	}

	else if (DEMO_CONTROLLER_STATE(state)->currentState == DEMO_STATE_STOPPING)
	{
		stop_demo(input, output, state, data);
	}


	// If we are not in the stopped state, then decode the desired leg angle, leg length, and hip angle into motor torques
	if (DEMO_CONTROLLER_STATE(state)->currentState != DEMO_STATE_STOPPED) 
	{
		float des_mtr_angA = DEMO_CONTROLLER_STATE(state)->desiredPos.leg_ang - PI + acos(DEMO_CONTROLLER_STATE(state)->desiredPos.leg_len);
		float des_mtr_angB = DEMO_CONTROLLER_STATE(state)->desiredPos.leg_ang + PI - acos(DEMO_CONTROLLER_STATE(state)->desiredPos.leg_len); 

		float des_mtr_ang_velA = -DEMO_CONTROLLER_STATE(state)->desiredPos.leg_len_vel / 2. / sin(des_mtr_angA - des_mtr_angB) - DEMO_CONTROLLER_STATE(state)->desiredPos.leg_ang_vel / 2.;
		float des_mtr_ang_velB = DEMO_CONTROLLER_STATE(state)->desiredPos.leg_len_vel / 2. / sin(des_mtr_angA - des_mtr_angB) - DEMO_CONTROLLER_STATE(state)->desiredPos.leg_ang_vel / 2.;

		output->motor_torqueA = DEMO_CONTROLLER_DATA(data)->p_gain * (des_mtr_angA - input->motor_angleA) 
			+ DEMO_CONTROLLER_DATA(data)->d_gain * (des_mtr_ang_velA - input->motor_velocityA);
		output->motor_torqueB = DEMO_CONTROLLER_DATA(data)->p_gain * (des_mtr_angB - input->motor_angleB) 
			+ DEMO_CONTROLLER_DATA(data)->d_gain * (des_mtr_ang_velB - input->motor_velocityB);

		output->motor_torque_hip = DEMO_CONTROLLER_DATA(data)->hip_p_gain * (DEMO_CONTROLLER_STATE(state)->desiredPos.hip_ang - input->hip_angle) 
			+ DEMO_CONTROLLER_DATA(data)->hip_d_gain * (DEMO_CONTROLLER_STATE(state)->desiredPos.hip_ang_vel - input->hip_angle_vel);
	}
	else
	{
		output->motor_torqueA = 0.0;
		output->motor_torqueB = 0.0;
		output->motor_torque_hip = 0.0;
	}

}

void start_demo(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
	RobotPosition demoStart;
	switch (DEMO_CONTROLLER_STATE(state)->currentDemo)
	{
		case DEMO1: demoStart = demo1(input, output, state, data, 0.0);
		case DEMO2: demoStart = demo2(input, output, state, data, 0.0);
		case DEMO3: demoStart = demo3(input, output, state, data, 0.0);
		case DEMO4: demoStart = demo4(input, output, state, data, 0.0);
	}

	// calculate current position
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_ang = ((demoStart.leg_ang-(PI/2))/START_TIME) * DEMO_CONTROLLER_STATE(state)->time + (PI/2);
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_len = ((demoStart.leg_len-0.9)/START_TIME) * DEMO_CONTROLLER_STATE(state)->time + 0.9;
	DEMO_CONTROLLER_STATE(state)->desiredPos.hip_ang = ((demoStart.hip_ang)/START_TIME) * DEMO_CONTROLLER_STATE(state)->time;
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_ang_vel = 0.0;
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_len_vel = 0.0;
	DEMO_CONTROLLER_STATE(state)->desiredPos.hip_ang_vel = 0.0;

	// If we are at the staring position then go to the running state
	if (DEMO_CONTROLLER_STATE(state)->time >= START_TIME)
	{
		DEMO_CONTROLLER_STATE(state)->time = 0.;
		DEMO_CONTROLLER_STATE(state)->currentState = DEMO_STATE_RUNNING;
	}
}

void stop_demo(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data)
{
	// calculate current position
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_ang = (((PI/2) - DEMO_CONTROLLER_STATE(state)->lastDemoPos.leg_ang)/START_TIME) * DEMO_CONTROLLER_STATE(state)->time + DEMO_CONTROLLER_STATE(state)->lastDemoPos.leg_ang;
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_len = ((0.9 - DEMO_CONTROLLER_STATE(state)->lastDemoPos.leg_len)/START_TIME) * DEMO_CONTROLLER_STATE(state)->time + DEMO_CONTROLLER_STATE(state)->lastDemoPos.leg_len;
	DEMO_CONTROLLER_STATE(state)->desiredPos.hip_ang = ((0.0 - DEMO_CONTROLLER_STATE(state)->lastDemoPos.hip_ang)/START_TIME) * DEMO_CONTROLLER_STATE(state)->time + DEMO_CONTROLLER_STATE(state)->lastDemoPos.hip_ang;
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_ang_vel = 0.0;
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_len_vel = 0.0;
	DEMO_CONTROLLER_STATE(state)->desiredPos.hip_ang_vel = 0.0;

	// If we are at the zero position, then go to the stopped state
	if (DEMO_CONTROLLER_STATE(state)->time >= START_TIME)
	{
		DEMO_CONTROLLER_STATE(state)->time = 0.;
		DEMO_CONTROLLER_STATE(state)->currentState = DEMO_STATE_STOPPED;
	}
}

RobotPosition demo1(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data, float time)
{
	CartPosition desPos;

	desPos.x=0.05*cos(2*PI*0);
	desPos.y=0.05*sin(2*PI*0);
	desPos.z=-0.75;
	desPos.x_vel = 0.0;
	desPos.y_vel = 0.0;
	desPos.z_vel = 0.0;


	return cartesianToRobot(desPos);
}

RobotPosition demo2(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data, float time)
{	
	CartPosition desPos;


	return cartesianToRobot(desPos);

}

RobotPosition demo3(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data, float time)
{	
	CartPosition desPos;


	return cartesianToRobot(desPos);

}

RobotPosition demo4(ControllerInput *input, ControllerOutput *output, ControllerState *state, ControllerData *data, float time)
{	
	CartPosition desPos;


	return cartesianToRobot(desPos);

}

RobotPosition cartesianToRobot(CartPosition posIn)
{
	RobotPosition posOut;

	// Store Cartesian Values	
	float x = posIn.x;
	float y = posIn.y;
	float z = posIn.z;

	float dx = posIn.x_vel;
	float dy = posIn.y_vel;
	float dz = posIn.x_vel;

	// Constants
	float lh = 0.15;  // Hip Link Length (m)
		
	
	// ***********************
	// Compute Robot Positions
	// ***********************

	// Useful Values
	float lf = CLAMP(sqrt(pow(z,2) + pow(y,2) - pow(lh,2)), 0.3, 1.0);

	// Robot Coordinate Computations
	posOut.hip_ang = CLAMP( PI/2. - asin(lh/lf) - atan2(-z,y) , -0.1, 0.1);
	posOut.leg_ang = CLAMP( atan2(lf, x) , 0.1, PI-0.1); 
	posOut.leg_len = CLAMP(lf/sin(leg_ang),0.5,0.95);

	// *************************
	// Compute Robot Velocities
	// *************************

	// Useful Values
	float dlf = (z*dz + y*dy)/lf;
	float dlf_inv = (-z*dz - y*dy)/pow(lf,3);
	float dlf_x = (dlf*x - lf*dx)/pow(x,2);

	posOut.leg_ang_vel = 0.0;	
	posOut.leg_len_vel = 0.0;
	posOut.hip_ang_vel = 0.0;

//	leg_ang_vel = (-lh*dlf_inv)/sqrt(1-pow(lh/lf,2)) + (dz*y - z*dy)/(pow(y,2) + pow(z,2));
//	hip_ang_vel = dlf_x/(1 + pow(lf/x,2));	
//	leg_len_vel = dlf/sin(leg_ang) - lf*hip_ang_vel/(sin(leg_ang)*tan(leg_ang));
	

	return posOut;	
}

void takedown_demo_controller(ControllerInput *input, ControllerOutput *output, ControllerState *state, 
	ControllerData *data)
{
	output->motor_torqueA = output->motor_torqueB = output->motor_torque_hip = 0.;
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_ang = 0.0; 
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_ang_vel = 0.0;
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_len = 0.0;
	DEMO_CONTROLLER_STATE(state)->desiredPos.leg_len_vel = 0.0;
	DEMO_CONTROLLER_STATE(state)->desiredPos.hip_ang = 0.0;
	DEMO_CONTROLLER_STATE(state)->desiredPos.hip_ang_vel = 0.0;

}
