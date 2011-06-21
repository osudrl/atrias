// Devin Koepl

#include <atrias_controllers/controller_wrapper_states.h>

/*****************************************************************************/

// Control wrapper variables.

uint16_t			tranA_off;
uint16_t			tranB_off;

int32_t				boom_pan_off = 0;
int32_t				boom_tilt_off = 0;

uint16_t			last_boom_pan_cnt;
uint16_t			first_boom_pan_cnt;
uint16_t			last_boom_tilt_cnt;

float				boom_pan_angle = 0.;
float				boom_tilt_angle = 0.;

float				last_boom_pan_angle = 0.;

uint16_t			last_tranA_cnt;
uint16_t			last_tranB_cnt;

float				last_motor_angleA = 0.;
float				last_motor_angleB = 0.;
float				last_leg_angleA		= 0.;
float				last_leg_angleB		= 0.;

float				hor_vel;
float				hor_vel_buffer[HOR_VEL_WINDOW];
int					hor_vel_index = 0;

/*****************************************************************************/

void control_wrapper_state_machine( uControllerInput ** in, uControllerOutput ** out )
{
	// Keep a copy of the states in memory.
	static unsigned char last_state = WAKE_UP_STATE;
	static unsigned char next_state = WAKE_UP_STATE;

	//rtai_print_to_screen( "CWSM: %u\n", next_state );

	switch ( next_state )
	{
		case WAKE_UP_STATE:
			next_state = wake_up_state( in, out, last_state );
			last_state = WAKE_UP_STATE;
			break;
		case RESTART_STATE:
			next_state = restart_state( in, out, last_state );
			last_state = RESTART_STATE;
			break;
		case CHECK_STATE:
			next_state = check_state( in, out, last_state );
			last_state = CHECK_STATE;
			break;
		case INITIALIZE_STATE:
			next_state = initialize_state( in, out, last_state );
			last_state = INITIALIZE_STATE;
			break;
		case RUN_STATE:
			next_state = run_state( in, out, last_state );
			last_state = RUN_STATE;
			break;
		case ERROR_STATE:
			next_state = error_state( in, out, last_state );
			last_state = ERROR_STATE;
			break;
		default:
			next_state = ERROR_STATE;
	}
	
	// Handle controller data change requests.
	if ( req_data_index_chg )
	{
		data_index ^= 1;
		req_data_index_chg = false;
	}
}

/*****************************************************************************/

unsigned char wake_up_state( uControllerInput ** in, uControllerOutput ** out, unsigned char last_state )
{
	int i;

	//rtai_print_to_screen( "Wake up.\n" );

	// Check to see if Medullas are awake by sending them a bad command (0).
	for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
	{
		in[i]->command = 0;
		
		// If a Medulla does not report the bad command, then fail.
		if ( !( out[i]->status & STATUS_BADCMD ) )
			return WAKE_UP_STATE;
	}

	// If successful, Medullas are ready to receive restarts.
	return RESTART_STATE;
}

/*****************************************************************************/

unsigned char restart_state( uControllerInput ** in, uControllerOutput ** out, unsigned char last_state )
{
	int i;
	
	//rtai_print_to_screen( "Restart.\n" );

	// Send restart commands.
	for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
	{
		in[i]->command = CMD_RESTART;
	}

	// Open the leg, and have the hip behave like a pin joint.
	in[A_INDEX]->MOTOR_TORQUE	= PWM_OPEN;
	in[B_INDEX]->MOTOR_TORQUE 	= PWM_OPEN;
	in[HIP_INDEX]->HIP_MTR_CMD 	= HIP_CMD_PIN;

	if ( last_state == RESTART_STATE )
		return CHECK_STATE;

	return RESTART_STATE;
}

/*****************************************************************************/

unsigned char check_state( uControllerInput ** in, uControllerOutput ** out, unsigned char last_state )
{
	int i;

	//rtai_print_to_screen( "Check.\n" );

	for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
	{
		// Send command disables.
		in[i]->command = CMD_DISABLE;

		// Check Medulla states.  They have to be disabled to move on to the next state.
		if ( out[i]->status != STATUS_DISABLED )
			return CHECK_STATE;
	}

	// Hold the leg open, and the hip floppy.
	in[A_INDEX]->MOTOR_TORQUE	= PWM_OPEN;
	in[B_INDEX]->MOTOR_TORQUE 	= PWM_OPEN;
	in[HIP_INDEX]->HIP_MTR_CMD 	= HIP_CMD_PIN;

	// Verify Medullas in their correct locations.
	if ( ( out[A_INDEX]->id != MEDULLA_A_ID ) || ( out[B_INDEX]->id != MEDULLA_B_ID )		
		|| ( out[HIP_INDEX]->id != MEDULLA_HIP_ID )	|| ( out[BOOM_INDEX]->id != MEDULLA_BOOM_ID ) )
		return CHECK_STATE;


	return INITIALIZE_STATE;
}

/*****************************************************************************/

// Initialize the encoder counters.
unsigned char initialize_state( uControllerInput ** in, uControllerOutput ** out, unsigned char last_state )
{
	int i;

	//rtai_print_to_screen( "Initialize.\n" );

	// Send command disables.
	for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
	{
		in[i]->command = CMD_DISABLE;
	}

	// Hold the leg open, and the hip floppy.
	in[A_INDEX]->MOTOR_TORQUE	= PWM_OPEN;
	in[B_INDEX]->MOTOR_TORQUE 	= PWM_OPEN;
	in[HIP_INDEX]->HIP_MTR_CMD 	= HIP_CMD_PIN;

	// Initialize sensors that can rollover, by considering the sensors that don't.
	// First find the leg segment angles.
	controller_input[io_index].leg_angleA = UNDISCRETIZE(	out[A_INDEX]->LEG_SEG_ANGLE,
		MAX_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_COUNT, MAX_LEG_SEG_A_COUNT);

	controller_input[io_index].leg_angleB = UNDISCRETIZE(	out[B_INDEX]->LEG_SEG_ANGLE,
		MIN_LEG_SEG_B_ANGLE, MAX_LEG_SEG_B_ANGLE, MIN_LEG_SEG_B_COUNT, MAX_LEG_SEG_B_COUNT);

	// Now use the leg segment angles to estimate the transmission output counts, by assuming small spring deflections.
	tranA_off = DISCRETIZE(controller_input[io_index].leg_angleA,
		MAX_TRAN_A_ANGLE, MIN_TRAN_A_ANGLE, MIN_TRAN_A_COUNT, MAX_TRAN_A_COUNT);

	tranB_off = DISCRETIZE(controller_input[io_index].leg_angleB,
		MIN_TRAN_B_ANGLE, MAX_TRAN_B_ANGLE, MIN_TRAN_B_COUNT, MAX_TRAN_B_COUNT);

	// Use the estimated encoder counts to figure out the offsets to accomodate for rollovers.
	tranA_off = (tranA_off / MAX_13BIT) * MAX_13BIT;
	tranB_off = (tranB_off / MAX_13BIT) * MAX_13BIT;

	// Set the last transmission counts, so that a false rollover is not tripped immediately.
	last_tranA_cnt = out[A_INDEX]->TRANS_ANGLE;
	last_tranB_cnt = out[B_INDEX]->TRANS_ANGLE;

	// Grab the initial pan count, so that we know how far the robot has moved about the room.
	first_boom_pan_cnt = out[BOOM_INDEX]->BOOM_PAN_CNT;

	// Tell the control switch to initialize.
	controller_state[0].state = controller_state[1].state = STATE_INIT;

	return RUN_STATE;
}

/*****************************************************************************/

unsigned char run_state( uControllerInput ** in, uControllerOutput ** out, unsigned char last_state )
{
	int i;
	static unsigned char command;

	if ( last_state == INITIALIZE_STATE )
		command = CMD_RUN;
	
	// Send run command.
	for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
	{
		in[i]->command = command;
	}
	// Toggle the run bit.
	command ^= CMD_RUN_TOGGLE_bm;

	// Check for sensor rollovers.
	// Transmission A rollovers.
	if ( (out[A_INDEX]->TRANS_ANGLE > last_tranA_cnt) 
		&& (out[A_INDEX]->TRANS_ANGLE - last_tranA_cnt > ROLLOVER13BIT_THRESHOLD) )
	{
		tranA_off -= MAX_13BIT;
	}
	if ( (last_tranA_cnt > out[A_INDEX]->TRANS_ANGLE)
		&& (last_tranA_cnt - out[A_INDEX]->TRANS_ANGLE > ROLLOVER13BIT_THRESHOLD) )
	{
		tranA_off += MAX_13BIT;
	}
	// Transmission B rollovers.
	if ( (out[B_INDEX]->TRANS_ANGLE > last_tranB_cnt) 
		&& (out[B_INDEX]->TRANS_ANGLE - last_tranB_cnt > ROLLOVER13BIT_THRESHOLD) )
	{
		tranB_off -= MAX_13BIT;
	}
	if ( (last_tranB_cnt > out[B_INDEX]->TRANS_ANGLE)
		&& (last_tranB_cnt - out[B_INDEX]->TRANS_ANGLE > ROLLOVER13BIT_THRESHOLD) )
	{
		tranB_off += MAX_13BIT;
	}
	// Boom Pan
	if ( (out[BOOM_INDEX]->BOOM_PAN_CNT > last_boom_pan_cnt) 
		&& (out[BOOM_INDEX]->BOOM_PAN_CNT - last_boom_pan_cnt > ROLLOVER16BIT_THRESHOLD) )
	{
		boom_pan_off -= MAX_16BIT;
	}
	if ( (last_boom_pan_cnt > out[BOOM_INDEX]->BOOM_PAN_CNT)
		&& (last_boom_pan_cnt - out[BOOM_INDEX]->BOOM_PAN_CNT > ROLLOVER16BIT_THRESHOLD) )
	{
		boom_pan_off += MAX_16BIT;
	}
	// Boom Tilt
	if ( (out[BOOM_INDEX]->BOOM_TILT_CNT > last_boom_tilt_cnt) 
		&& (out[BOOM_INDEX]->BOOM_TILT_CNT - last_boom_tilt_cnt > ROLLOVER16BIT_THRESHOLD) )
	{
		boom_tilt_off -= MAX_16BIT;
	}
	if ( (last_boom_tilt_cnt > out[BOOM_INDEX]->BOOM_TILT_CNT)
		&& (last_boom_tilt_cnt - out[BOOM_INDEX]->BOOM_TILT_CNT > ROLLOVER16BIT_THRESHOLD) )
	{
		boom_tilt_off += MAX_16BIT;
	}

	// Keep track of the last counts for sensors that could rollover.
	last_tranA_cnt = out[A_INDEX]->TRANS_ANGLE;
	last_tranB_cnt = out[B_INDEX]->TRANS_ANGLE;
	last_boom_pan_cnt		= out[BOOM_INDEX]->BOOM_PAN_CNT;
	last_boom_tilt_cnt	= out[BOOM_INDEX]->BOOM_TILT_CNT;

	// Generate controller input
	controller_input[io_index].leg_angleA		= UNDISCRETIZE(
		out[A_INDEX]->LEG_SEG_ANGLE,
		MAX_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_COUNT, MAX_LEG_SEG_A_COUNT);

	controller_input[io_index].leg_angleB		= UNDISCRETIZE(
		out[B_INDEX]->LEG_SEG_ANGLE,
		MIN_LEG_SEG_B_ANGLE, MAX_LEG_SEG_B_ANGLE, MIN_LEG_SEG_B_COUNT, MAX_LEG_SEG_B_COUNT);

	controller_input[io_index].motor_angleA 	= UNDISCRETIZE(
		tranA_off + out[A_INDEX]->TRANS_ANGLE,
		MAX_TRAN_A_ANGLE, MIN_TRAN_A_ANGLE, MIN_TRAN_A_COUNT, MAX_TRAN_A_COUNT) + TRAN_A_OFF_ANGLE;

	controller_input[io_index].motor_angleB 	= UNDISCRETIZE(
		tranB_off + out[B_INDEX]->TRANS_ANGLE,
		MIN_TRAN_B_ANGLE, MAX_TRAN_B_ANGLE, MIN_TRAN_B_COUNT, MAX_TRAN_B_COUNT) + TRAN_B_OFF_ANGLE;				

	controller_input[io_index].motor_velocityA = (controller_input[io_index].motor_angleA - last_motor_angleA)
		/ ( (float)out[A_INDEX]->timestep * SEC_PER_CNT );
	controller_input[io_index].motor_velocityB = (controller_input[io_index].motor_angleB - last_motor_angleB) 
		/ ( (float)out[B_INDEX]->timestep * SEC_PER_CNT );

	controller_input[io_index].leg_velocityA = (controller_input[io_index].leg_angleA - last_leg_angleA)
		/ ( (float)out[A_INDEX]->timestep * SEC_PER_CNT );	
	controller_input[io_index].leg_velocityB = (controller_input[io_index].leg_angleB - last_leg_angleB)
		/ ( (float)out[B_INDEX]->timestep * SEC_PER_CNT );					

	boom_pan_angle	= DISCRETIZE_LOCATION( out[BOOM_INDEX]->BOOM_PAN_CNT,
		first_boom_pan_cnt, 0., MAX_16BIT, BOOM_PAN_GEAR_RATIO);
	// Filter the horizontal velocity.
	hor_vel = BOOM_LENGTH * (boom_pan_angle - last_boom_pan_angle)
		/ ( (float)out[BOOM_INDEX]->timestep * SEC_PER_CNT );
	if ( ABS(hor_vel) > 200. )
	{
		// Not a valid velocity.
	}
	else
	{
		hor_vel_buffer[hor_vel_index] = hor_vel;
		hor_vel_index++;
		hor_vel_index = hor_vel_index % HOR_VEL_WINDOW;

		// Average the values in the buffer.
		controller_input[io_index].horizontal_velocity = 0.;
		for (i = 0; i < HOR_VEL_WINDOW; i++)
		{
			controller_input[io_index].horizontal_velocity += hor_vel_buffer[i] / (float)HOR_VEL_WINDOW;
		}
		//HOR_VEL_FILTER_EPS * hor_vel + (1. - HOR_VEL_FILTER_EPS) * controller_input[io_index].horizontal_velocity; 
	}				
	last_boom_pan_angle = boom_pan_angle;

	boom_tilt_angle = DISCRETIZE_LOCATION( out[BOOM_INDEX]->BOOM_TILT_CNT, 
		BOOM_KNOWN_TILT_CNT, BOOM_KNOWN_TILT_ANGLE, MAX_16BIT, BOOM_TILT_GEAR_RATIO);
	controller_input[io_index].height = BOOM_LENGTH * sin(boom_tilt_angle) + BOOM_PIVOT_HEIGHT;

	last_motor_angleA = controller_input[io_index].motor_angleA;
	last_motor_angleB = controller_input[io_index].motor_angleB;
	last_leg_angleA		= controller_input[io_index].leg_angleA;
	last_leg_angleB		= controller_input[io_index].leg_angleB;			

	// Controller update.
	control_switcher_state_machine(&controller_input[io_index], &controller_output[io_index],
		&controller_state[state_index], &controller_data[data_index]);
	// Clamp the motor torques.
	controller_output[io_index].motor_torqueA = CLAMP(controller_output[io_index].motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ);
	controller_output[io_index].motor_torqueB = CLAMP(controller_output[io_index].motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ);
	//controller_output[io_index].motor_torqueA = CLAMP(controller_output[io_index].motor_torqueA, -3., 3.);
	//controller_output[io_index].motor_torqueB = CLAMP(controller_output[io_index].motor_torqueB, -3., 3.);
	//controller_output[io_index].motor_torqueA = 0.;
	//controller_output[io_index].motor_torqueB = 0.;			

	// Send motor torques only when all of the Medullas status's are okay..
	if ( out[A_INDEX]->status || out[B_INDEX]->status
		|| out[HIP_INDEX]->status || out[BOOM_INDEX]->status )
	{
		// At least one of the Medullas is unhappy, so don't send motor torques.

		in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
			0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
		in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
			0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);

		// Move to error state.
		//return ERROR_STATE;
	}
	else
	{
		// All of the Medullas are okay, so we go ahead and send the torques.

		// If both torques are below the threshold, then send a small torque to keep the robot off of its hardstops.
		if ( ( ABS(controller_output[io_index].motor_torqueA) < MIN_TRQ_THRESH ) 
			&& ( ABS(controller_output[io_index].motor_torqueB) < MIN_TRQ_THRESH ) )
		{
			in[A_INDEX]->MOTOR_TORQUE = PWM_OPEN;
			in[B_INDEX]->MOTOR_TORQUE = PWM_OPEN;
		}
		else
		{
			// Send motor torques.
			in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
				controller_output[io_index].motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
			in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
				-controller_output[io_index].motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
			//in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
			//	0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
			//in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
			//	0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
		}
	}

	// Send hip command.  Do this regardless of the status of the Medullas to try and protect the knee from moments.
	//if ( controller_input[io_index].height < 1.0 )
	//{
	//	in[HIP_INDEX]->HIP_MTR_CMD = HIP_CMD_PIN;
	//}
	//else
	//{
		in[HIP_INDEX]->HIP_MTR_CMD = HIP_CMD_RIGID;
	//}

	// Increment and rollover i/o index for datalogging.
	io_index = (++io_index) % SIZE_OF_DATA_RING_BUFFER;

	return RUN_STATE;
}

/*****************************************************************************/

unsigned char error_state( uControllerInput ** in, uControllerOutput ** out, unsigned char last_state )
{
	//rtai_print_to_screen( "Error.\n" );

	// Send zero motor torques.
	in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
		0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
	in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
		0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);

	// No coming back from this one yet.
	return ERROR_STATE;
}

/*****************************************************************************/

bool atrias_gui_callback(atrias_controllers::atrias_srv::Request &req, atrias_controllers::atrias_srv::Response &res)
{
	int i;

	//*************************************************************************

	// Load request into controller's data.

	controller_data[ 1 - data_index ].command					= req.command;
	controller_data[ 1 - data_index ].controller_requested		= req.controller_requested;

	for (i = 0; i < SIZE_OF_CONTROLLER_DATA; i++)
	{
		controller_data[ 1 - data_index ].data[i] = req.control_data[i];
	}

	// Request controller switches data.
	req_data_index_chg 	= true;

	//*************************************************************************

	// Populate response for GUI with the last controller I/O.

	res.body_angle		= controller_input[io_index - 1].body_angle;
	res.motor_angleA	= controller_input[io_index - 1].motor_angleA;
	res.motor_angleB	= controller_input[io_index - 1].motor_angleB;
	res.leg_angleA		= controller_input[io_index - 1].leg_angleA;
	res.leg_angleB		= controller_input[io_index - 1].leg_angleB;
	res.hor_vel			= controller_input[io_index - 1].horizontal_velocity;
	res.height			= controller_input[io_index - 1].height;

	res.motor_torqueA	= controller_output[io_index - 1].motor_torqueA;
	res.motor_torqueB	= controller_output[io_index - 1].motor_torqueB;
	
	//*************************************************************************

	return true;
}

/****************************************************************************/

void datalog( void )
{
	int i;

	FILE *fp = fopen( QUOTEME(LOG_FILENAME), "w" );

	// This print cannot be a ROS_INFO, because of a problem in the ROS header file?
	printf( "Writing %u log entries to %s.", io_index, QUOTEME(LOG_FILENAME) );

	// Create file header.
	fprintf( fp, "Body Angle, Motor Angle A, Motor Angle B, Leg Angle A, Leg Angle B, Body Angular Velocity, Motor Velocity A, Motor Velocity B, Leg Velocity A, Leg Velocity B, Height, Horizontal Velocity, Vertical Velocity, Motor Torque A, Motor Torque B, Counter, Shm Index\n");

	for ( i = 0; i < io_index; i++ )
	{

		fprintf( fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", controller_input[i].body_angle, controller_input[i].motor_angleA, controller_input[i].motor_angleB,
			controller_input[i].leg_angleA, controller_input[i].leg_angleB, controller_input[i].body_ang_vel, controller_input[i].motor_velocityA, controller_input[i].motor_velocityB,
			controller_input[i].leg_velocityA, controller_input[i].leg_velocityB, controller_input[i].height, controller_input[i].horizontal_velocity, controller_input[i].vertical_velocity,
			controller_output[i].motor_torqueA, controller_output[i].motor_torqueB );

	}

	fclose(fp);
}

