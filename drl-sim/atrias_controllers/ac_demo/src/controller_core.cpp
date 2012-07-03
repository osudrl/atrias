/*
 * controller_core.cpp
 *
 * Demo Controller
 *
 *  Created on: May 6, 2012
 *      Author: Michael Anderson
 */

#include <ac_demo/controller_core.h>

ControllerInitResult controllerInit()
{
    elapsedTime = 0.;
    
    desTorqueA = 0.;
	desTorqueB = 0.;
	currentState = DEMO_STATE_STOPPED;
	currentDemo = 0;

	ControllerInitResult cir;
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    cir.error = false;
    return cir;
}


void controllerUpdate(robot_state state, ByteArray input, ControllerOutput *output, ByteArray &status)
{
    InputData *id = BYTE_ARRAY_TO_STRUCT(input, InputData*);

	// We use a constant timestep of 1ms because this is the rate that this function is actually being called at.
	elapsedTime += 0.001;

	if (currentState == DEMO_STATE_STOPPED)
	{
		// If the current state is stopped then we can switch to another controller
		currentDemo = id->commandedDemo;

		if (id->commandedState == 1)
		{
			printf("Starting controller %i\n", (int)id->commandedDemo);
			elapsedTime = 0.;
			currentState = DEMO_STATE_STARTING;
			lastDemoPos.leg_ang = LEG_ANGLE(state.motor_angleA, state.motor_angleB);
			lastDemoPos.leg_len = LEG_LENGTH(state.motor_angleA, state.motor_angleB);
			lastDemoPos.hip_ang = state.motor_angle_hip;
		}
	}

	else if (currentState == DEMO_STATE_STARTING)
	{
		start_demo(state, id, output);
	}

	else if (currentState == DEMO_STATE_RUNNING)
	{
		if (currentDemo == DEMO1)
			desiredPos = demo1(state, id, output, elapsedTime);
		if (currentDemo == DEMO2)
			desiredPos = demo2(state, id, output, elapsedTime);
		if (currentDemo == DEMO3)
			desiredPos = demo3(state, id, output, elapsedTime);
		if (currentDemo == DEMO4)
			desiredPos = demo4(state, id, output, elapsedTime);
		if (currentDemo == DEMO5)
			desiredPos = demo5(state, id, output, elapsedTime);
		if (currentDemo == DEMO6)
			desiredPos = demo6(state, id, output, elapsedTime);
		
		if ((uint32_t)(elapsedTime * 1000) % 1000 == 0)	
			printf("Desired position leg length: %f\n", desiredPos.leg_len);


		// If we are currently running a demo and the stop demo button is pressed, go to demo_stopping
		if (id->commandedState == 0)
		{
			lastDemoPos = desiredPos;
			currentState = DEMO_STATE_STOPPING;
			elapsedTime = 0.0;
		}
		else
		{
			// This is a hack, but for DEMO4, the force controller demo, we don't want to use this position controller code. Instead the demo will directly set desiredTorque variables
			if (currentDemo == DEMO4)
			{
				output->motor_torqueA = desTorqueA;
				output->motor_torqueB = desTorqueB;
			}
			else
			{
				float des_mtr_angA = desiredPos.leg_ang - PI + acos(desiredPos.leg_len);
				float des_mtr_angB = desiredPos.leg_ang + PI - acos(desiredPos.leg_len);

				float des_mtr_ang_velA = -desiredPos.leg_len_vel / 2. / sin(des_mtr_angA - des_mtr_angB) - desiredPos.leg_ang_vel / 2.;
				float des_mtr_ang_velB = desiredPos.leg_len_vel / 2. / sin(des_mtr_angA - des_mtr_angB) - desiredPos.leg_ang_vel / 2.;

				output->motor_torqueA = 1000. * (des_mtr_angA - state.motor_angleA)
					+ 20. * (des_mtr_ang_velA - state.motor_velocityA);
				output->motor_torqueB = 1000. * (des_mtr_angB - state.motor_angleB)
					+ 20. * (des_mtr_ang_velB - state.motor_velocityB);
			}
		}

		output->motor_torque_hip = 2000. * (desiredPos.hip_ang - state.motor_angle_hip)
			+ 20. * (desiredPos.hip_ang_vel - state.motor_velocity_hip);
	}

	else if (currentState == DEMO_STATE_STOPPING)
	{
		stop_demo(state, id, output);
	}
}

void start_demo(robot_state& state, InputData* id, ControllerOutput* output)
{
	RobotPosition demoStart;
	switch (currentDemo)
	{
		case DEMO1: demoStart = demo1(state, id, output, 0.0); break;
		case DEMO2: demoStart = demo2(state, id, output, 0.0); break;
		case DEMO3: demoStart = demo3(state, id, output, 0.0); break;
		case DEMO4: demoStart = demo4(state, id, output, 0.0); break;
		case DEMO5: demoStart = demo5(state, id, output, 0.0); break;
		case DEMO6: demoStart = demo6(state, id, output, 0.0); break;
	}

	// calculate current position
	desiredPos.leg_ang = ((demoStart.leg_ang-lastDemoPos.leg_ang)/START_TIME) * elapsedTime + lastDemoPos.leg_ang;
	desiredPos.leg_len = ((demoStart.leg_len-lastDemoPos.leg_len)/START_TIME) * elapsedTime + lastDemoPos.leg_len;
	desiredPos.hip_ang = ((demoStart.hip_ang-lastDemoPos.hip_ang)/START_TIME) * elapsedTime + lastDemoPos.hip_ang;
	desiredPos.leg_ang_vel = 0.0;
	desiredPos.leg_len_vel = 0.0;
	desiredPos.hip_ang_vel = 0.0;

	// If we are at the staring position then go to the running state
	if (elapsedTime >= START_TIME)
	{
		elapsedTime = 0.;
		printf("Controller has started!\n");
		currentState = DEMO_STATE_RUNNING;
	}
}

void stop_demo(robot_state& state, InputData* id, ControllerOutput* output)
{
	// calculate current position
	desiredPos.leg_ang = (((PI/2) - lastDemoPos.leg_ang)/START_TIME) * elapsedTime + lastDemoPos.leg_ang;
	desiredPos.leg_len = ((0.9 - lastDemoPos.leg_len)/START_TIME) * elapsedTime + lastDemoPos.leg_len;
	desiredPos.hip_ang = ((0.135 - lastDemoPos.hip_ang)/START_TIME) * elapsedTime + lastDemoPos.hip_ang;
	desiredPos.leg_ang_vel = 0.0;
	desiredPos.leg_len_vel = 0.0;
	desiredPos.hip_ang_vel = 0.0;

	// If we are at the zero position, then go to the stopped state
	if (elapsedTime >= START_TIME)
	{
		elapsedTime = 0.;
		currentState = DEMO_STATE_STOPPED;
	}
}

RobotPosition demo1(robot_state& state, InputData* id, ControllerOutput* output, float time)
{
	// Planer elipse
	CartPosition desPos;

	desPos.x=0.35*cos(2*PI*time);
	desPos.y=0.07*sin(2*PI*time) + 0.15;
	desPos.z=-0.75;
	desPos.x_vel = 0.0;
	desPos.y_vel = 0.0;
	desPos.z_vel = 0.0;

	return cartesianToRobot(desPos);
}

RobotPosition demo2(robot_state& state, InputData* id, ControllerOutput* output, float time)
{
	CartPosition desPos;

	// Planar Figure eight
	/*
	desPos.x = 0.25*sin(2.0*PI*1.0*time);
	desPos.y = 0.1*cos(2.0*PI*1.0*time)*sin(2.0*PI*1.0*time) + 0.15;
	desPos.z = -0.75;
	desPos.x_vel = 0.0;
	desPos.y_vel = 0.0;
	desPos.z_vel = 0.0; */

	// Piriform Curve (Running Shape)
	float rads = 2.0*PI*id->amplitude*time;

	desPos.x = 0.30*(((1+cos(-rads))*cos(0.5)) + ((sin(-rads)*0.6*(1+cos(-rads)))*-sin(0.5)))-0.27;
	desPos.y = 0.2 + 0.05*sin(rads);
	desPos.z = 0.25*(((1+cos(-rads))*sin(0.5)) + ((sin(-rads)*0.6*(1+cos(-rads)))*cos(0.5)))-0.91;
	desPos.x_vel = 0.0;
	desPos.y_vel = 0.0;
	desPos.z_vel = 0.0;


	return cartesianToRobot(desPos);

}

RobotPosition demo3(robot_state& state, InputData* id, ControllerOutput* output, float time)
{
	CartPosition desPos;

	float rads = 2.0*PI*id->amplitude*time;

	desPos.x = (0.2*sin(rads))*cos(rads/20);
	desPos.y = 0.07*cos(rads) + 0.15;
	desPos.z = (-.2*sin(rads))*sin(rads/20)-.75;
	desPos.x_vel = 0.0;
	desPos.y_vel = 0.0;
	desPos.z_vel = 0.0;

/*
	desPos.x = (0.2*sin(rads));
	desPos.y = 0.07*cos(rads) + 0.15;
	desPos.z = (-.2*sin(rads))-.75;
	desPos.x_vel = 0.0;
	desPos.y_vel = 0.0;
	desPos.z_vel = 0.0;
*/
	return cartesianToRobot(desPos);

}

RobotPosition demo4(robot_state& state, InputData* id, ControllerOutput* output, float time)
{
	// Leg length limits
	float safety_length_long = 0.97;
	float safety_length_short = 0.4;

	//  Motor A angle limits
	float safety_angleA_short = -2./3.*PI;
	float safety_angleA_long = 0.;

	// Motor B angle limits
	float safety_angleB_short = PI;
	float safety_angleB_long = 5./3.*PI;

	// Leg length calculation
	float leg_length = cos((2.0 * PI + state.leg_angleA - state.leg_angleB ) / 2.0);


	// If outside of safety limits, Engage damping controller
	// Logic:  if value is not equal to its "CLAMPED" value, it must be out of range
	if(LEG_LENGTH(state.motor_angleA, state.motor_angleB) != CLAMP(LEG_LENGTH(state.motor_angleA, state.motor_angleB), safety_length_short, safety_length_long) ||
		state.motor_angleA != CLAMP(state.motor_angleA, safety_angleA_short, safety_angleA_long) ||
		state.motor_angleB != CLAMP(state.motor_angleB, safety_angleB_short, safety_angleB_long))
	{
		// Damping controller engages, with hard-coded gains
		desTorqueA = - 100.0 * state.motor_velocityA;
		desTorqueB = - 100.0 * state.motor_velocityB;
	}
	// Else, engage normal controller
	else
	{
		// Get values from GUI.
		float desired_force = 0;
		//float FORCE_P_GAIN = id->p_gain;
		//float FORCE_D_GAIN = id->d_gain;

		// Calculate spring deflections.
		float spring_defA = state.motor_angleA - state.leg_angleA;
		float spring_defB = state.motor_angleB - state.leg_angleB;

		// Known Values
		float MOTOR_CONSTANT = 0.12226; // Nm per A
		//float SPRING_CONSTANT = 310.; // A per rad  via rough calibration 2012-03-01
		float SPRING_CONSTANT = 1175.*1.108; // Nm per rad  via spring tester 2012-03-28
		float GEAR_RATIO = 50.;

		// Using the current leg configuration, converts the desired axial force into a desired leg torque
		float desired_torque = 0.5*desired_force*cos(PI + asin(LEG_LENGTH(state.motor_angleA, state.motor_angleB)));

		// Inverse spring function:  Converts desired torques into desired deflection angles.
		//		float desired_spring_def = desired_torque/(MOTOR_CONSTANT*SPRING_CONSTANT*GEAR_RATIO);
		float desired_spring_def = desired_torque/(SPRING_CONSTANT);

		// NOTE: defA is NEGATIVE when compressed.  Should be opposite sign of defB.
		state.desired_spring_defA = desired_spring_def;
		state.desired_spring_defB = -1.0*desired_spring_def;

		// Leg A, PD control.
		state.desired_motor_angleA = state.motor_angleA + (state.desired_spring_defA - spring_defA);
		desTorqueA = (FORCE_CONTROLLER_P_GAIN * (state.desired_motor_angleA - state.motor_angleA)) - (FORCE_CONTROLLER_D_GAIN * (state.motor_velocityA));

		// Leg B, PD control.
		state.desired_motor_angleB = state.motor_angleB + (state.desired_spring_defB - spring_defB);
		desTorqueB = (FORCE_CONTROLLER_P_GAIN * (state.desired_motor_angleB - state.motor_angleB)) - (FORCE_CONTROLLER_D_GAIN * (state.motor_velocityB));
	}

	RobotPosition desPos;
	if (time == 0)	// Return our desired starting position
	{
		desPos.leg_ang = PI/2;
		desPos.leg_ang_vel = 0.0;
		desPos.leg_len = 0.9;
		desPos.leg_len_vel = 0.0;
		desPos.hip_ang = 0.0;
		desPos.hip_ang_vel = 0.0;
	}
	else
	{
		desPos.leg_ang = LEG_ANGLE(state.motor_angleA, state.motor_angleB);
		desPos.leg_ang_vel = 0.0;
		desPos.leg_len = LEG_LENGTH(state.motor_angleA, state.motor_angleB);
		desPos.leg_len_vel = 0.0;
		desPos.hip_ang = 0.0;
		desPos.hip_ang_vel = 0.0;
	}

	return desPos;

}

RobotPosition demo5(robot_state& state, InputData* id, ControllerOutput* output, float time)
{
	// Example of cartesian demo
	CartPosition desPos;

	desPos.x= 0.0;
	desPos.y= 0.15;
	desPos.z= 0.75;
	desPos.x_vel = 0.0;
	desPos.y_vel = 0.0;
	desPos.z_vel = 0.0;

	return cartesianToRobot(desPos);
}

RobotPosition demo6(robot_state& state, InputData* id, ControllerOutput* output, float time)
{
	RobotPosition desPos;

	// Example of robot coordinate demo

	desPos.leg_ang = PI/2.0;
	desPos.leg_len = 0.8;
	desPos.hip_ang = 0.0;
	desPos.leg_ang_vel = 0.0;
	desPos.leg_len_vel = 0.0;
	desPos.hip_ang_vel = 0.0;


	return desPos;

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
	posOut.hip_ang = CLAMP(-1.0* (PI/2. - asin(lh/lf) - atan2(-z,y)) , -0.195, 0.125);
	posOut.leg_ang = CLAMP( atan2(lf, x) , 0.1, PI-0.1); 
	posOut.leg_len = CLAMP(lf/sin(posOut.leg_ang),0.5,0.95);

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

//	leg_ang_vel = -1.0*((-lh*dlf_inv)/sqrt(1-pow(lh/lf,2)) + (dz*y - z*dy)/(pow(y,2) + pow(z,2)));
//	hip_ang_vel = dlf_x/(1 + pow(lf/x,2));	
//	leg_len_vel = dlf/sin(posOut.leg_ang) - lf*hip_ang_vel/(sin(posOut.leg_ang)*tan(posOut.leg_ang));
	

	return posOut;	
}

void controllerTakedown() {

}
