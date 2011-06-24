// Devin Koepl

#include "controller_wrapper_states.h"

/*****************************************************************************/

#define STATE_WAKEUP						0
#define STATE_RESTART 						1
#define STATE_CHECK 						2
#define STATE_INITIALIZE 					3
#define STATE_RUN 							4
#define STATE_ERROR 						5

/*****************************************************************************/

static Shm * shm;

/*****************************************************************************/

// Control wrapper variables.

unsigned short int	tranA_off;
unsigned short int	tranB_off;

int					boom_pan_off = 0;
int					boom_tilt_off = 0;

unsigned short int	last_boom_pan_cnt;
unsigned short int	first_boom_pan_cnt;
unsigned short int	last_boom_tilt_cnt;

float				boom_pan_angle 		= 0.;
float				boom_tilt_angle 	= 0.;

float				last_boom_pan_angle = 0.;

unsigned short int	last_tranA_cnt;
unsigned short int	last_tranB_cnt;

float				last_motor_angleA 	= 0.;
float				last_motor_angleB 	= 0.;
float				last_leg_angleA		= 0.;
float				last_leg_angleB		= 0.;

float				hor_vel;
float				hor_vel_buffer[HOR_VEL_WINDOW];
int					hor_vel_index 		= 0;

float				leg_angle;
float				leg_length;

static unsigned char command = CMD_RUN;

ControllerInput 	* c_in;
ControllerOutput 	* c_out;

/*****************************************************************************/

// Initialize SHM.  Return true upon success.

unsigned char initialize_shm( void )
{
	// To Uspace SHM
	if ( !( shm = ( Shm * )rt_shm_alloc( nam2num( SHM_NAME ), sizeof( Shm ), USE_VMALLOC ) ) )
		return false;
	memset( shm, 0, sizeof( Shm ) );

	shm->controller_data[0].command 				= shm->controller_data[1].command 				= CMD_DISABLE;
	shm->controller_data[0].controller_requested 	= shm->controller_data[1].controller_requested 	= NO_CONTROLLER;

	return true;
}

/*****************************************************************************/

void takedown_shm( void )
{
	rt_shm_free( nam2num( SHM_NAME ) );
}

/*****************************************************************************/

void control_wrapper_state_machine( uControllerInput ** uc_in, uControllerOutput ** uc_out )
{
	// Keep a copy of the states in memory.
	static unsigned char last_state = STATE_WAKEUP;
	static unsigned char next_state = STATE_WAKEUP;

	//rt_printk( "Next state: %u\n", next_state );

	switch ( next_state )
	{
		case STATE_WAKEUP:
			next_state = state_wakeup( uc_in, uc_out, last_state );
			last_state = STATE_WAKEUP;
			break;
		case STATE_RESTART:
			next_state = state_restart( uc_in, uc_out, last_state );
			last_state = STATE_RESTART;
			break;
		case STATE_CHECK:
			next_state = state_check( uc_in, uc_out, last_state );
			last_state = STATE_CHECK;
			break;
		case STATE_INITIALIZE:
			next_state = state_initialize( uc_in, uc_out, last_state );
			last_state = STATE_INITIALIZE;
			break;
		case STATE_RUN:
			next_state = state_run( uc_in, uc_out, last_state );
			last_state = STATE_RUN;
			break;
		case STATE_ERROR:
			next_state = state_error( uc_in, uc_out, last_state );
			last_state = STATE_ERROR;
			break;
		default:
			next_state = STATE_ERROR;
	}
	
	if ( shm->controller_data[shm->control_index].command == CMD_RESTART )
		next_state =  STATE_WAKEUP;

	// Handle controller data change requests.
	if ( shm->req_switch )
	{
		shm->control_index 	= 1 - shm->control_index;
		shm->req_switch 	= false;
	}
}

/*****************************************************************************/

unsigned char state_wakeup( uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state )
{
	int i;

	// Check to see if Medullas are awake by sending them a bad command (0).
	for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
	{
		uc_in[i]->command = 0;
		
		// If a Medulla does not report the bad command, then fail.
		if ( !( uc_out[i]->status & STATUS_BADCMD ) )
		{
			rt_printk( "Medulla %u did not report the bad command.\n", i );
			return STATE_WAKEUP;
		}
	}

	// If successful, Medullas are ready to receive restarts.
	return STATE_RESTART;
}

/*****************************************************************************/

unsigned char state_restart( uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state )
{
	int i;

	// Send restart commands.
	for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
		uc_in[i]->command = CMD_RESTART;

	// Open the leg, and have the hip behave like a pin joint.
	uc_in[A_INDEX]->MOTOR_TORQUE	= PWM_OPEN;
	uc_in[B_INDEX]->MOTOR_TORQUE 	= PWM_OPEN;
	uc_in[HIP_INDEX]->HIP_MTR_CMD 	= HIP_CMD_PIN;

	if ( last_state == STATE_RESTART )
		return STATE_CHECK;

	return STATE_RESTART;
}

/*****************************************************************************/

unsigned char state_check( uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state )
{
	int i;

	for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
	{
		// Send command disables.
		uc_in[i]->command = CMD_DISABLE;

		// Check Medulla states.  They have to be disabled to move on to the next state.
		if ( uc_out[i]->status != STATUS_DISABLED )
		{
			rt_printk( "Medulla not disabled.\n" );
			return STATE_CHECK;
		}
	}

	// Hold the leg open, and the hip floppy.
	uc_in[A_INDEX]->MOTOR_TORQUE	= PWM_OPEN;
	uc_in[B_INDEX]->MOTOR_TORQUE 	= PWM_OPEN;
	uc_in[HIP_INDEX]->HIP_MTR_CMD 	= HIP_CMD_PIN;

	// Verify Medullas in their correct locations.
	if ( ( uc_out[A_INDEX]->id != MEDULLA_A_ID ) || ( uc_out[B_INDEX]->id != MEDULLA_B_ID )		
		|| ( uc_out[HIP_INDEX]->id != MEDULLA_HIP_ID )	|| ( uc_out[BOOM_INDEX]->id != MEDULLA_BOOM_ID ) )
	{
		rt_printk( "Medulla in wrong location.\n" );
		return STATE_CHECK;
	}

	return STATE_INITIALIZE;
}

/*****************************************************************************/

// Initialize the encoder counters.
unsigned char state_initialize( uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state )
{
	int i;

	// Tell the control switch to initialize.
	shm->controller_state.state = 3;

	// Create pointers to controller I/O
	c_in	= &shm->controller_input[shm->io_index];
	c_out 	= &shm->controller_output[shm->io_index];

	// Send command disables.
	for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
	{
		uc_in[i]->command = CMD_DISABLE;
	}

	// Hold the leg open, and the hip floppy.
	uc_in[A_INDEX]->MOTOR_TORQUE	= PWM_OPEN;
	uc_in[B_INDEX]->MOTOR_TORQUE 	= PWM_OPEN;
	uc_in[HIP_INDEX]->HIP_MTR_CMD 	= HIP_CMD_PIN;

	// Initialize sensors that can rollover, by considering the sensors that don't.
	// First find the leg segment angles.
	c_in->leg_angleA = UNDISCRETIZE(	uc_out[A_INDEX]->LEG_SEG_ANGLE,
		MAX_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_COUNT, MAX_LEG_SEG_A_COUNT);

	c_in->leg_angleB = UNDISCRETIZE(	uc_out[B_INDEX]->LEG_SEG_ANGLE,
		MIN_LEG_SEG_B_ANGLE, MAX_LEG_SEG_B_ANGLE, MIN_LEG_SEG_B_COUNT, MAX_LEG_SEG_B_COUNT);

	// Now use the leg segment angles to estimate the transmission output counts, by assuming small spring deflections.
	tranA_off = DISCRETIZE(c_in->leg_angleA,
		MAX_TRAN_A_ANGLE, MIN_TRAN_A_ANGLE, MIN_TRAN_A_COUNT, MAX_TRAN_A_COUNT);

	tranB_off = DISCRETIZE(c_in->leg_angleB,
		MIN_TRAN_B_ANGLE, MAX_TRAN_B_ANGLE, MIN_TRAN_B_COUNT, MAX_TRAN_B_COUNT);

	// Use the estimated encoder counts to figure out the offsets to accomodate for rollovers.
	tranA_off = (tranA_off / MAX_13BIT) * MAX_13BIT;
	tranB_off = (tranB_off / MAX_13BIT) * MAX_13BIT;

	// Set the last transmission counts, so that a false rollover is not tripped immediately.
	last_tranA_cnt = uc_out[A_INDEX]->TRANS_ANGLE;
	last_tranB_cnt = uc_out[B_INDEX]->TRANS_ANGLE;
	last_boom_pan_cnt	= uc_out[BOOM_INDEX]->BOOM_PAN_CNT;
	last_boom_tilt_cnt	= uc_out[BOOM_INDEX]->BOOM_TILT_CNT;

	// Grab the initial pan count, so that we know how far the robot has moved about the room.
	first_boom_pan_cnt = uc_out[BOOM_INDEX]->BOOM_PAN_CNT;

	return STATE_RUN;
}

/*****************************************************************************/

unsigned char state_run( uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state )
{
	int i;

	// Create pointers to controller I/O
	c_in	= &shm->controller_input[shm->io_index];
	c_out 	= &shm->controller_output[shm->io_index];

	if ( last_state == state_initialize )
		command = CMD_RUN;
	
	// Send run command.
	for ( i = 0; i < NUM_OF_MEDULLAS_ON_ROBOT; i++ )
	{
		uc_in[i]->command = command;
	}
	// Toggle the run bit.
	command ^= CMD_RUN_TOGGLE_bm;

	// Check for sensor rollovers.
	// Transmission A rollovers.
	if ( (uc_out[A_INDEX]->TRANS_ANGLE > last_tranA_cnt) 
		&& (uc_out[A_INDEX]->TRANS_ANGLE - last_tranA_cnt > ROLLOVER13BIT_THRESHOLD) )
	{
		tranA_off -= MAX_13BIT;
	}
	if ( (last_tranA_cnt > uc_out[A_INDEX]->TRANS_ANGLE)
		&& (last_tranA_cnt - uc_out[A_INDEX]->TRANS_ANGLE > ROLLOVER13BIT_THRESHOLD) )
	{
		tranA_off += MAX_13BIT;
	}
	// Transmission B rollovers.
	if ( (uc_out[B_INDEX]->TRANS_ANGLE > last_tranB_cnt) 
		&& (uc_out[B_INDEX]->TRANS_ANGLE - last_tranB_cnt > ROLLOVER13BIT_THRESHOLD) )
	{
		tranB_off -= MAX_13BIT;
	}
	if ( (last_tranB_cnt > uc_out[B_INDEX]->TRANS_ANGLE)
		&& (last_tranB_cnt - uc_out[B_INDEX]->TRANS_ANGLE > ROLLOVER13BIT_THRESHOLD) )
	{
		tranB_off += MAX_13BIT;
	}
	// Boom Pan
	if ( (uc_out[BOOM_INDEX]->BOOM_PAN_CNT > last_boom_pan_cnt) 
		&& (uc_out[BOOM_INDEX]->BOOM_PAN_CNT - last_boom_pan_cnt > ROLLOVER16BIT_THRESHOLD) )
	{
		boom_pan_off -= MAX_16BIT;
	}
	if ( (last_boom_pan_cnt > uc_out[BOOM_INDEX]->BOOM_PAN_CNT)
		&& (last_boom_pan_cnt - uc_out[BOOM_INDEX]->BOOM_PAN_CNT > ROLLOVER16BIT_THRESHOLD) )
	{
		boom_pan_off += MAX_16BIT;
	}
	// Boom Tilt
	if ( (uc_out[BOOM_INDEX]->BOOM_TILT_CNT > last_boom_tilt_cnt) 
		&& (uc_out[BOOM_INDEX]->BOOM_TILT_CNT - last_boom_tilt_cnt > ROLLOVER16BIT_THRESHOLD) )
	{
		boom_tilt_off -= MAX_16BIT;
	}
	if ( (last_boom_tilt_cnt > uc_out[BOOM_INDEX]->BOOM_TILT_CNT)
		&& (last_boom_tilt_cnt - uc_out[BOOM_INDEX]->BOOM_TILT_CNT > ROLLOVER16BIT_THRESHOLD) )
	{
		boom_tilt_off += MAX_16BIT;
	}

	// Keep track of the last counts for sensors that could rollover.
	last_tranA_cnt 		= uc_out[A_INDEX]->TRANS_ANGLE;
	last_tranB_cnt 		= uc_out[B_INDEX]->TRANS_ANGLE;
	last_boom_pan_cnt	= uc_out[BOOM_INDEX]->BOOM_PAN_CNT;
	last_boom_tilt_cnt	= uc_out[BOOM_INDEX]->BOOM_TILT_CNT;

	// Generate controller input
	c_in->leg_angleA		= UNDISCRETIZE(
		uc_out[A_INDEX]->LEG_SEG_ANGLE,
		MAX_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_COUNT, MAX_LEG_SEG_A_COUNT);

	c_in->leg_angleB		= UNDISCRETIZE(
		uc_out[B_INDEX]->LEG_SEG_ANGLE,
		MIN_LEG_SEG_B_ANGLE, MAX_LEG_SEG_B_ANGLE, MIN_LEG_SEG_B_COUNT, MAX_LEG_SEG_B_COUNT);

	c_in->motor_angleA 	= UNDISCRETIZE(
		tranA_off + uc_out[A_INDEX]->TRANS_ANGLE,
		MAX_TRAN_A_ANGLE, MIN_TRAN_A_ANGLE, MIN_TRAN_A_COUNT, MAX_TRAN_A_COUNT) + TRAN_A_OFF_ANGLE;

	c_in->motor_angleB 	= UNDISCRETIZE(
		tranB_off + uc_out[B_INDEX]->TRANS_ANGLE,
		MIN_TRAN_B_ANGLE, MAX_TRAN_B_ANGLE, MIN_TRAN_B_COUNT, MAX_TRAN_B_COUNT) + TRAN_B_OFF_ANGLE;				

	c_in->motor_velocityA = (c_in->motor_angleA - last_motor_angleA)
		/ ( (float)uc_out[A_INDEX]->timestep * SEC_PER_CNT );
	c_in->motor_velocityB = (c_in->motor_angleB - last_motor_angleB) 
		/ ( (float)uc_out[B_INDEX]->timestep * SEC_PER_CNT );

	c_in->leg_velocityA = (c_in->leg_angleA - last_leg_angleA)
		/ ( (float)uc_out[A_INDEX]->timestep * SEC_PER_CNT );	
	c_in->leg_velocityB = (c_in->leg_angleB - last_leg_angleB)
		/ ( (float)uc_out[B_INDEX]->timestep * SEC_PER_CNT );					

	boom_pan_angle	= DISCRETIZE_LOCATION( uc_out[BOOM_INDEX]->BOOM_PAN_CNT,
		first_boom_pan_cnt, 0., MAX_16BIT, BOOM_PAN_GEAR_RATIO);
	// Filter the horizontal velocity.
	hor_vel = BOOM_LENGTH * (boom_pan_angle - last_boom_pan_angle)
		/ ( (float)uc_out[BOOM_INDEX]->timestep * SEC_PER_CNT );
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
		c_in->horizontal_velocity = 0.;
		for (i = 0; i < HOR_VEL_WINDOW; i++)
		{
			c_in->horizontal_velocity += hor_vel_buffer[i] / (float)HOR_VEL_WINDOW;
		}
		//HOR_VEL_FILTER_EPS * hor_vel + (1. - HOR_VEL_FILTER_EPS) * c_in->horizontal_velocity; 
	}				
	last_boom_pan_angle = boom_pan_angle;

	boom_tilt_angle = DISCRETIZE_LOCATION( boom_tilt_off + uc_out[BOOM_INDEX]->BOOM_TILT_CNT, 
		BOOM_KNOWN_TILT_CNT, BOOM_KNOWN_TILT_ANGLE, MAX_16BIT, BOOM_TILT_GEAR_RATIO);
	c_in->height = BOOM_LENGTH * sin(boom_tilt_angle) + BOOM_PIVOT_HEIGHT;

	// Grab motor torque readings.
	c_in->motor_currentA = uc_out[A_INDEX]->therm1;
	c_in->motor_currentB = uc_out[B_INDEX]->therm1;

	last_motor_angleA = c_in->motor_angleA;
	last_motor_angleB = c_in->motor_angleB;
	last_leg_angleA		= c_in->leg_angleA;
	last_leg_angleB		= c_in->leg_angleB;			

	// Send hip command.  Do this regardless of the status of the Medullas to try and protect the knee from moments.
	leg_angle = ( 2. * PI + c_in->leg_angleA + c_in->leg_angleB ) / 2. - PI;
	leg_length = - 0.5 * sin( c_in->leg_angleA ) - 0.5 * sin( c_in->leg_angleB );

	if ( c_in->height > leg_length * sin( leg_angle ) )
	{
		// Flight
		uc_in[HIP_INDEX]->HIP_MTR_CMD = HIP_CMD_RIGID;
		c_in->toe_switch = false;
	}
	else
	{
		// Stance
		uc_in[HIP_INDEX]->HIP_MTR_CMD = HIP_CMD_PIN;
		c_in->toe_switch = false;
	}

	c_in->command = shm->controller_data[shm->control_index].command;

	// Controller update.
	control_switcher_state_machine( c_in, c_out,
		&shm->controller_state, &shm->controller_data[shm->control_index] );
	// Clamp the motor torques.
	c_out->motor_torqueA = CLAMP(c_out->motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ);
	c_out->motor_torqueB = CLAMP(c_out->motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ);
	//c_out->motor_torqueA = CLAMP(c_out->motor_torqueA, -3., 3.);
	//c_out->motor_torqueB = CLAMP(c_out->motor_torqueB, -3., 3.);
	//c_out->motor_torqueA = 0.;
	//c_out->motor_torqueB = 0.;			

	// Send motor torques only when all of the Medullas status's are okay..
	if ( uc_out[A_INDEX]->status || uc_out[B_INDEX]->status
		|| uc_out[HIP_INDEX]->status || uc_out[BOOM_INDEX]->status )
	{
		// At least one of the Medullas is unhappy, so don't send motor torques.

		uc_in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
			0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
		uc_in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
			0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);

		// Print the Medullas' status bytes, because at least one of them has a problem.
		rt_printk( "Status: A: %u, B: %u, Hip: %u, Boom: %u\n", uc_out[A_INDEX]->status, uc_out[B_INDEX]->status,
			uc_out[HIP_INDEX]->status, uc_out[BOOM_INDEX]->status );

		// Move to error state.
		//return state_error;
	}
	else
	{
		// All of the Medullas are okay, so we go ahead and send the torques.

		// If both torques are below the threshold, then send a small torque to keep the robot off of its hardstops.
		if ( ( ABS(c_out->motor_torqueA) < MIN_TRQ_THRESH ) 
			&& ( ABS(c_out->motor_torqueB) < MIN_TRQ_THRESH ) )
		{
			uc_in[A_INDEX]->MOTOR_TORQUE = PWM_OPEN;
			uc_in[B_INDEX]->MOTOR_TORQUE = PWM_OPEN;
		}
		else
		{
			// Send motor torques.
			uc_in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
				c_out->motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
			uc_in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
				-c_out->motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
			//uc_in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
			//	0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
			//uc_in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
			//	0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
		}
	}

	// Increment and rollover i/o index for datalogging.
	shm->io_index = (++shm->io_index) % SHM_TO_USPACE_ENTRIES;

	return STATE_RUN;
}

/*****************************************************************************/

unsigned char state_error( uControllerInput ** uc_in, uControllerOutput ** uc_out, unsigned char last_state )
{
	// Send zero motor torques.
	uc_in[A_INDEX]->MOTOR_TORQUE = DISCRETIZE(
		0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
	uc_in[B_INDEX]->MOTOR_TORQUE = DISCRETIZE(
		0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);

	// No coming back from this one yet.
	return STATE_ERROR;
}
