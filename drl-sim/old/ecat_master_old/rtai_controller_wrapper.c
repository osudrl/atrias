// Devin Koepl

#include "rtai_controller_wrapper.h"

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    rt_sem_wait(&master_sem);
    ecrt_master_state(master, &ms);
    rt_sem_signal(&master_sem);

    if (ms.slaves_responding != master_state.slaves_responding)
        printk(KERN_INFO PFX "%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printk(KERN_INFO PFX "AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printk(KERN_INFO PFX "Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/*****************************************************************************/

unsigned char wake_up_medullas()
{
	// Check to see if Medullas are awake by sending them a bad command (0).

	// receive process data
	rt_sem_wait(&master_sem);
	ecrt_master_receive(master);
	ecrt_domain_process(domain1);
	rt_sem_signal(&master_sem);

	((uControllerInput *)(domain1_pd + off_medullaA_rx))->command = 0;
	((uControllerInput *)(domain1_pd + off_medullaB_rx))->command = 0;
	((uControllerInput *)(domain1_pd + off_medulla_hip_rx))->command = 0;
	((uControllerInput *)(domain1_pd + off_medulla_boom_rx))->command = 0;

	rt_sem_wait(&master_sem);
	ecrt_domain_queue(domain1);
	rt_sem_signal(&master_sem);
	ecrt_master_send(master);

	rt_task_wait_period();

	rt_printk("Waking up Medullas.\n");

	// If all of the Medullas report the bad command, then return success.
	if ( (((uControllerOutput *)(domain1_pd + off_medullaA_tx))->status & STATUS_BADCMD)
		&& (((uControllerOutput *)(domain1_pd + off_medullaB_tx))->status & STATUS_BADCMD)
		&& (((uControllerOutput *)(domain1_pd + off_medulla_hip_tx))->status & STATUS_BADCMD)
		&& (((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->status & STATUS_BADCMD) )
			return true;

	return false;
}

/*****************************************************************************/

unsigned char check_medullas()
{
	// The result of the check.  Clear if any problems with the Medullas are detected.
	unsigned char result = 1;

	// Verify Medullas in their correct locations.

	// receive process data
	rt_sem_wait(&master_sem);
	ecrt_master_receive(master);
	ecrt_domain_process(domain1);
	rt_sem_signal(&master_sem);

	((uControllerInput *)(domain1_pd + off_medullaA_rx))->command = CMD_DISABLE;
	((uControllerInput *)(domain1_pd + off_medullaB_rx))->command = CMD_DISABLE;
	((uControllerInput *)(domain1_pd + off_medulla_hip_rx))->command = CMD_DISABLE;
	((uControllerInput *)(domain1_pd + off_medulla_boom_rx))->command = CMD_DISABLE;

	((uControllerInput *)(domain1_pd + off_medullaA_rx))->MOTOR_TORQUE = PWM_OPEN;
	((uControllerInput *)(domain1_pd + off_medullaB_rx))->MOTOR_TORQUE = PWM_OPEN;
	((uControllerInput *)(domain1_pd + off_medulla_hip_rx))->HIP_MTR_CMD = HIP_CMD_PIN;

	rt_sem_wait(&master_sem);
	ecrt_domain_queue(domain1);
	rt_sem_signal(&master_sem);
	ecrt_master_send(master);

	rt_task_wait_period();

	// If any of the Medullas are not in their disabled state, return failure.
	//if ( (((uControllerOutput *)(domain1_pd + off_medullaA_tx))->status != STATUS_DISABLED)
	//	|| (((uControllerOutput *)(domain1_pd + off_medullaB_tx))->status != STATUS_DISABLED)
	//	|| (((uControllerOutput *)(domain1_pd + off_medulla_hip_tx))->status != STATUS_DISABLED)
	//	|| (((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->status != STATUS_DISABLED) )
	//	return false;
	if (((uControllerOutput *)(domain1_pd + off_medullaA_tx))->status != STATUS_DISABLED)
	{
		rt_printk("Medulla A not in disabled state.\n");
		result = 0;
	}
	if (((uControllerOutput *)(domain1_pd + off_medullaB_tx))->status != STATUS_DISABLED)
	{
		rt_printk("Medulla B not in disabled state.\n");
		result = 0;
	}
	if (((uControllerOutput *)(domain1_pd + off_medulla_hip_tx))->status != STATUS_DISABLED)
	{
		rt_printk("Medulla Hip not in disabled state.\n");
		result = 0;
	}
	if (((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->status != STATUS_DISABLED)
	{
		rt_printk("Medulla Boom not in disabled state.\n");
		result = 0;
	}

	// If the Medullas are reporting a configuration other than the one expected, return failure.
	//if ( ( ((uControllerOutput *)(domain1_pd + off_medullaA_tx))->id != MEDULLA_A_ID )
	//	|| ( ((uControllerOutput *)(domain1_pd + off_medullaB_tx))->id != MEDULLA_B_ID )		
	//	|| ( ((uControllerOutput *)(domain1_pd + off_medulla_hip_tx))->id != MEDULLA_HIP_ID )	
	//	|| ( ((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->id != MEDULLA_BOOM_ID )	)
	//	return false;
	if ( ((uControllerOutput *)(domain1_pd + off_medullaA_tx))->id != MEDULLA_A_ID )
	{
		rt_printk("Medulla A reporting wrong ID.\n");
		result = 0;
	}
	if ( ((uControllerOutput *)(domain1_pd + off_medullaB_tx))->id != MEDULLA_B_ID )
	{
		rt_printk("Medulla B reporting wrong ID.\n");
		result = 0;
	}
	if ( ((uControllerOutput *)(domain1_pd + off_medulla_hip_tx))->id != MEDULLA_HIP_ID )
	{
		rt_printk("Medulla Hip reporting wrong ID.\n");
		result = 0;
	}
	if ( ((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->id != MEDULLA_BOOM_ID )
	{
		rt_printk("Medulla Boom reporting wrong ID.\n");
		result = 0;
	}

	return result;
}

/*****************************************************************************/

void run(long data)
{
	int i;

	/*************************************************************************/

	while ( !wake_up_medullas() );

	// Now restart the medullas.  The boom needs 2 command restarts to respond.  Firmware not fast enough?
	for ( i = 0; i < 2; i++ )
	{
		// receive process data
		rt_sem_wait(&master_sem);
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
		rt_sem_signal(&master_sem);

		((uControllerInput *)(domain1_pd + off_medullaA_rx))->command = CMD_RESTART;
		((uControllerInput *)(domain1_pd + off_medullaB_rx))->command = CMD_RESTART;
		((uControllerInput *)(domain1_pd + off_medulla_hip_rx))->command = CMD_RESTART;
		((uControllerInput *)(domain1_pd + off_medulla_boom_rx))->command = CMD_RESTART;

		((uControllerInput *)(domain1_pd + off_medullaA_rx))->MOTOR_TORQUE = PWM_OPEN;
		((uControllerInput *)(domain1_pd + off_medullaB_rx))->MOTOR_TORQUE = PWM_OPEN;
		((uControllerInput *)(domain1_pd + off_medulla_hip_rx))->HIP_MTR_CMD = HIP_CMD_PIN;

		rt_sem_wait(&master_sem);
		ecrt_domain_queue(domain1);
		rt_sem_signal(&master_sem);
		ecrt_master_send(master);

		rt_task_wait_period();
	}

	// Wait for all the Medullas to wake up.
	while ( !check_medullas() );

	rt_printk("Medullas Checked.\n");

	// Initialize the encoder counters.

	// receive process data
	rt_sem_wait(&master_sem);
	ecrt_master_receive(master);
	ecrt_domain_process(domain1);
	rt_sem_signal(&master_sem);

	((uControllerInput *)(domain1_pd + off_medullaA_rx))->command = CMD_DISABLE;
	((uControllerInput *)(domain1_pd + off_medullaB_rx))->command = CMD_DISABLE;
	((uControllerInput *)(domain1_pd + off_medulla_hip_rx))->command = CMD_DISABLE;
	((uControllerInput *)(domain1_pd + off_medulla_boom_rx))->command = CMD_DISABLE;

	((uControllerInput *)(domain1_pd + off_medullaA_rx))->MOTOR_TORQUE = PWM_OPEN;
	((uControllerInput *)(domain1_pd + off_medullaB_rx))->MOTOR_TORQUE = PWM_OPEN;
	((uControllerInput *)(domain1_pd + off_medulla_hip_rx))->HIP_MTR_CMD = HIP_CMD_PIN;

	//rt_printk("A: %u,\tB: %u\n", ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->LEG_SEG_ANGLE,
	//	((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->LEG_SEG_ANGLE);

	// Initialize sensors that can rollover, by considering the sensors that don't.
	// First find the leg segment angles.
	controller_input.leg_angleA = UNDISCRETIZE(
		((uControllerOutput *)(domain1_pd + off_medullaA_tx))->LEG_SEG_ANGLE,
		MAX_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_COUNT, MAX_LEG_SEG_A_COUNT);

	controller_input.leg_angleB = UNDISCRETIZE(
		((uControllerOutput *)(domain1_pd + off_medullaB_tx))->LEG_SEG_ANGLE,
		MIN_LEG_SEG_B_ANGLE, MAX_LEG_SEG_B_ANGLE, MIN_LEG_SEG_B_COUNT, MAX_LEG_SEG_B_COUNT);

	// Now use the leg segment angles to estimate the transmission output counts, by assuming small spring deflections.
	tranA_off = DISCRETIZE(controller_input.leg_angleA,
		MAX_TRAN_A_ANGLE, MIN_TRAN_A_ANGLE, MIN_TRAN_A_COUNT, MAX_TRAN_A_COUNT);

	tranB_off = DISCRETIZE(controller_input.leg_angleB,
		MIN_TRAN_B_ANGLE, MAX_TRAN_B_ANGLE, MIN_TRAN_B_COUNT, MAX_TRAN_B_COUNT);

	// Use the estimated encoder counts to figure out the offsets to accomodate for rollovers.
	tranA_off = (tranA_off / MAX_13BIT) * MAX_13BIT;
	tranB_off = (tranB_off / MAX_13BIT) * MAX_13BIT;

	// Set the last transmission counts, so that a false rollover is not tripped immediately.
	last_tranA_cnt = ((uControllerOutput *)(domain1_pd + off_medullaA_tx))->TRANS_ANGLE;
	last_tranB_cnt = ((uControllerOutput *)(domain1_pd + off_medullaB_tx))->TRANS_ANGLE;

	// Grab the initial pan count, so that we know how far the robot has moved about the room.
	first_boom_pan_cnt = ((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_PAN_CNT;

	rt_sem_wait(&master_sem);
	ecrt_domain_queue(domain1);
	rt_sem_signal(&master_sem);
	ecrt_master_send(master);

	rt_task_wait_period();

	/*************************************************************************/

	// Now start controlling the robot
	while (true) {
		t_last_cycle = get_cycles();

		// receive process data
		rt_sem_wait(&master_sem);
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
		rt_sem_signal(&master_sem);

		// Toggle the command bit.
		((uControllerInput *)(domain1_pd + off_medullaA_rx))->command = command;	
		((uControllerInput *)(domain1_pd + off_medullaB_rx))->command = command;	
		((uControllerInput *)(domain1_pd + off_medulla_hip_rx))->command = command;
		((uControllerInput *)(domain1_pd + off_medulla_boom_rx))->command = command;
		command ^= CMD_RUN_TOGGLE_bm;

		// Check for sensor rollovers.
		// Transmission A rollovers.
		if ( (((uControllerOutput *)(domain1_pd + off_medullaA_tx))->TRANS_ANGLE > last_tranA_cnt) 
			&& (((uControllerOutput *)(domain1_pd + off_medullaA_tx))->TRANS_ANGLE - last_tranA_cnt > ROLLOVER13BIT_THRESHOLD) )
		{
			tranA_off -= MAX_13BIT;
		}
		if ( (last_tranA_cnt > ((uControllerOutput *)(domain1_pd + off_medullaA_tx))->TRANS_ANGLE)
			&& (last_tranA_cnt - ((uControllerOutput *)(domain1_pd + off_medullaA_tx))->TRANS_ANGLE > ROLLOVER13BIT_THRESHOLD) )
		{
			tranA_off += MAX_13BIT;
		}
		// Transmission B rollovers.
		if ( (((uControllerOutput *)(domain1_pd + off_medullaB_tx))->TRANS_ANGLE > last_tranB_cnt) 
			&& (((uControllerOutput *)(domain1_pd + off_medullaB_tx))->TRANS_ANGLE - last_tranB_cnt > ROLLOVER13BIT_THRESHOLD) )
		{
			tranB_off -= MAX_13BIT;
		}
		if ( (last_tranB_cnt > ((uControllerOutput *)(domain1_pd + off_medullaB_tx))->TRANS_ANGLE)
			&& (last_tranB_cnt - ((uControllerOutput *)(domain1_pd + off_medullaB_tx))->TRANS_ANGLE > ROLLOVER13BIT_THRESHOLD) )
		{
			tranB_off += MAX_13BIT;
		}
		// Boom Pan
		if ( (((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_PAN_CNT > last_boom_pan_cnt) 
			&& (((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_PAN_CNT - last_boom_pan_cnt > ROLLOVER16BIT_THRESHOLD) )
		{
			boom_pan_off -= MAX_16BIT;
		}
		if ( (last_boom_pan_cnt > ((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_PAN_CNT)
			&& (last_boom_pan_cnt - ((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_PAN_CNT > ROLLOVER16BIT_THRESHOLD) )
		{
			boom_pan_off += MAX_16BIT;
		}
		// Boom Tilt
		if ( (((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_TILT_CNT > last_boom_tilt_cnt) 
			&& (((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_TILT_CNT - last_boom_tilt_cnt > ROLLOVER16BIT_THRESHOLD) )
		{
			boom_tilt_off -= MAX_16BIT;
		}
		if ( (last_boom_tilt_cnt > ((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_TILT_CNT)
			&& (last_boom_tilt_cnt - ((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_TILT_CNT > ROLLOVER16BIT_THRESHOLD) )
		{
			boom_tilt_off += MAX_16BIT;
		}

		// Keep track of the last counts for sensors that could rollover.
		last_tranA_cnt = ((uControllerOutput *)(domain1_pd + off_medullaA_tx))->TRANS_ANGLE;
		last_tranB_cnt = ((uControllerOutput *)(domain1_pd + off_medullaB_tx))->TRANS_ANGLE;
		last_boom_pan_cnt		= ((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_PAN_CNT;
		last_boom_tilt_cnt	= ((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_TILT_CNT;

		// Generate controller input
		controller_input.leg_angleA		= UNDISCRETIZE(
			((uControllerOutput *)(domain1_pd + off_medullaA_tx))->LEG_SEG_ANGLE,
			MAX_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_COUNT, MAX_LEG_SEG_A_COUNT);

		controller_input.leg_angleB		= UNDISCRETIZE(
			((uControllerOutput *)(domain1_pd + off_medullaB_tx))->LEG_SEG_ANGLE,
			MIN_LEG_SEG_B_ANGLE, MAX_LEG_SEG_B_ANGLE, MIN_LEG_SEG_B_COUNT, MAX_LEG_SEG_B_COUNT);

		controller_input.motor_angleA 	= UNDISCRETIZE(
			tranA_off + ((uControllerOutput *)(domain1_pd + off_medullaA_tx))->TRANS_ANGLE,
			MAX_TRAN_A_ANGLE, MIN_TRAN_A_ANGLE, MIN_TRAN_A_COUNT, MAX_TRAN_A_COUNT) + TRAN_A_OFF_ANGLE;

		controller_input.motor_angleB 	= UNDISCRETIZE(
			tranB_off + ((uControllerOutput *)(domain1_pd + off_medullaB_tx))->TRANS_ANGLE,
			MIN_TRAN_B_ANGLE, MAX_TRAN_B_ANGLE, MIN_TRAN_B_COUNT, MAX_TRAN_B_COUNT) + TRAN_B_OFF_ANGLE;				

		controller_input.motor_velocityA = (controller_input.motor_angleA - last_motor_angleA)
			/ ( (float)((uControllerOutput *)(domain1_pd + off_medullaA_tx))->timestep * SEC_PER_CNT );
		controller_input.motor_velocityB = (controller_input.motor_angleB - last_motor_angleB) 
			/ ( (float)((uControllerOutput *)(domain1_pd + off_medullaB_tx))->timestep * SEC_PER_CNT );

		controller_input.leg_velocityA = (controller_input.leg_angleA - last_leg_angleA)
			/ ( (float)((uControllerOutput *)(domain1_pd + off_medullaA_tx))->timestep * SEC_PER_CNT );	
		controller_input.leg_velocityB = (controller_input.leg_angleB - last_leg_angleB)
			/ ( (float)((uControllerOutput *)(domain1_pd + off_medullaB_tx))->timestep * SEC_PER_CNT );					

		boom_pan_angle	= DISCRETIZE_LOCATION( ((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_PAN_CNT,
			first_boom_pan_cnt, 0., MAX_16BIT, BOOM_PAN_GEAR_RATIO);
		// Filter the horizontal velocity.
		hor_vel = BOOM_LENGTH * (boom_pan_angle - last_boom_pan_angle)
			/ ( (float)((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->timestep * SEC_PER_CNT );
		if ( ABS(hor_vel) > 200. )
		{
			// Not a valid velocity.
			rt_printk("Horizontal velocity was invalid, not passing to filter.\n");
		}
		else
		{
			hor_vel_buffer[hor_vel_index] = hor_vel;
			hor_vel_index++;
			hor_vel_index = hor_vel_index % HOR_VEL_WINDOW;

			// Average the values in the buffer.
			controller_input.horizontal_velocity = 0.;
			for (i = 0; i < HOR_VEL_WINDOW; i++)
			{
				controller_input.horizontal_velocity += hor_vel_buffer[i] / (float)HOR_VEL_WINDOW;
			}
			//HOR_VEL_FILTER_EPS * hor_vel + (1. - HOR_VEL_FILTER_EPS) * controller_input.horizontal_velocity; 
		}				
		last_boom_pan_angle = boom_pan_angle;

		//rt_printk( "Pan: %u\n", ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_PAN_CNT );

		boom_tilt_angle = DISCRETIZE_LOCATION( ((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->BOOM_TILT_CNT, 
			BOOM_KNOWN_TILT_CNT, BOOM_KNOWN_TILT_ANGLE, MAX_16BIT, BOOM_TILT_GEAR_RATIO);
		controller_input.height = BOOM_LENGTH * sin(boom_tilt_angle) + BOOM_PIVOT_HEIGHT;

		last_motor_angleA = controller_input.motor_angleA;
		last_motor_angleB = controller_input.motor_angleB;
		last_leg_angleA		= controller_input.leg_angleA;
		last_leg_angleB		= controller_input.leg_angleB;

		// Receive commands from user space.
		if ( !to_kern_shm->lock )
		{
			to_kern_shm->lock = true;

			controller_data.command = to_kern_shm->command;
			controller_data.controller_requested = to_kern_shm->controller_requested;

			// Copy the controller data out of shm.
			for (i = 0; i < SIZE_OF_CONTROLLER_DATA; i++)
			{
				controller_data.data[i] = to_kern_shm->controller_data[i];
			}
	
			to_kern_shm->lock = false;

			// Reset the spin lock counter.
			spin_lock_cnt = 0;
		}
		else
		{
			spin_lock_cnt++;

			// If the module has been locked out of shm for more than a second.  Unlock the shm, because it is stuck.
			if ( spin_lock_cnt > 1000 )			
			{
				to_kern_shm->lock = false;

				// Reset the spin lock counter.
				spin_lock_cnt = 0;
			}	
		}			

		// Controller update.
		control_switcher_state_machine(&controller_input, &controller_output, &controller_state, &controller_data);
		// Clamp the motor torques.
		controller_output.motor_torqueA = CLAMP(controller_output.motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ);
		controller_output.motor_torqueB = CLAMP(controller_output.motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ);
		//controller_output.motor_torqueA = CLAMP(controller_output.motor_torqueA, -3., 3.);
		//controller_output.motor_torqueB = CLAMP(controller_output.motor_torqueB, -3., 3.);
		//controller_output.motor_torqueA = 0.;
		//controller_output.motor_torqueB = 0.;			

		// Send state to user space for datalogging.
		to_uspace_shm[to_uspace_index]->cnt					= to_uspace_cnt++;
		to_uspace_shm[to_uspace_index]->index				= to_uspace_index;
		to_uspace_shm[to_uspace_index]->controller_input 	= controller_input;
		to_uspace_shm[to_uspace_index]->controller_output 	= controller_output;
		to_uspace_shm[to_uspace_index]->fresh = true;

		//if ( to_uspace_index == 0 )
		//	rt_printk( "Cnt: %u\n", to_uspace_shm[0]->cnt );
		//rt_printk( "Index: %u, Cnt: %u\n", to_uspace_index, to_uspace_cnt );

		// Increment and roll the ring buffer index over, when it reaches the end of the buffer.
		to_uspace_index = (++to_uspace_index) % SHM_TO_USPACE_ENTRIES;

		// Send motor torques only when all of the Medullas status's are okay..
		if ( ((uControllerOutput *)(domain1_pd + off_medullaA_tx))->status 
			|| ((uControllerOutput *)(domain1_pd + off_medullaB_tx))->status
			|| ((uControllerOutput *)(domain1_pd + off_medulla_hip_tx))->status
			|| ((uControllerOutput *)(domain1_pd + off_medulla_boom_tx))->status )
		{
			// At least one of the Medullas is unhappy, so don't send motor torques.

			((uControllerInput *)(domain1_pd + off_medullaA_rx))->MOTOR_TORQUE = DISCRETIZE(
				0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
			((uControllerInput *)(domain1_pd + off_medullaB_rx))->MOTOR_TORQUE = DISCRETIZE(
				0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);

			//rt_printk("\nUnhappy Medulla detected.  Status: A: %02X, B: %02X, Hip: %02X, Boom: %02X\n\n",
				//((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->status,
				//((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->status,
				//((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_hip_index]))->status,
				//((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->status );
		}
		else
		{
			// All of the Medullas are okay, so we go ahead and send the torques.

			// If both torques are below the threshold, then send a small torque to keep the robot off of its hardstops.
			if ( ( ABS(controller_output.motor_torqueA) < MIN_TRQ_THRESH ) 
				&& ( ABS(controller_output.motor_torqueB) < MIN_TRQ_THRESH ) )
			{
				((uControllerInput *)(domain1_pd + off_medullaA_rx))->MOTOR_TORQUE = PWM_OPEN;
				((uControllerInput *)(domain1_pd + off_medullaB_rx))->MOTOR_TORQUE = PWM_OPEN;
			}
			else
			{
				// Send motor torques.
				((uControllerInput *)(domain1_pd + off_medullaA_rx))->MOTOR_TORQUE = DISCRETIZE(
					controller_output.motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
				((uControllerInput *)(domain1_pd + off_medullaB_rx))->MOTOR_TORQUE = DISCRETIZE(
					-controller_output.motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
				//((uControllerInput *)(domain1_pd + off_medullaA_rx))->MOTOR_TORQUE = DISCRETIZE(
				//	0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
				//((uControllerInput *)(domain1_pd + off_medullaB_rx))->MOTOR_TORQUE = DISCRETIZE(
				//	0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
			}
		}

		// Send hip command.  Do this regardless of the status of the Medullas to try and protect the knee from moments.
		//if ( controller_input.height < 1.0 )
		//{
		//	((uControllerInput *)(domain1_pd + off_medulla_hip_rx))->HIP_MTR_CMD = HIP_CMD_PIN;
		//}
		//else
		//{
			((uControllerInput *)(domain1_pd + off_medulla_hip_rx))->HIP_MTR_CMD = HIP_CMD_RIGID;
		//}

		// Print the status bytes.
		//rt_printk("Status Bytes:\nA: %02X\tB: %02X\thip: %02X\n",
		//	((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->status,
		//	((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->status,
		//	((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_hip_index]))->status);
		//rt_printk("A: %u\n", ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE);		

		rt_sem_wait(&master_sem);
		ecrt_domain_queue(domain1);
		rt_sem_signal(&master_sem);
		ecrt_master_send(master);

		rt_task_wait_period();
	}
}

/*****************************************************************************/

void request_lock_callback(void *cb_data)
{
    rt_sem_wait(&master_sem);
}

/*****************************************************************************/

void release_lock_callback(void *cb_data)
{
    rt_sem_signal(&master_sem);
}

/*****************************************************************************/

int __init init_mod(void)
{
	int i, ret = -1;
	RTIME tick_period, requested_ticks, now;

	//****************************************************************************

	// Define variables.

	command = CMD_RUN;
	controller_state.state = STATE_INIT;

	to_uspace_index = 0;

	boom_pan_off = 0;
	boom_tilt_off = 0;

	boom_pan_angle = 0.;
	boom_tilt_angle = 0.;

	last_boom_pan_angle = 0.;

	last_motor_angleA = 0.;
	last_motor_angleB = 0.;
	last_leg_angleA		= 0.;
	last_leg_angleB		= 0.;

	hor_vel;
	hor_vel_buffer[HOR_VEL_WINDOW];
	hor_vel_index = 0;

	//****************************************************************************

	printk("\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
	printk(KERN_INFO PFX "Starting...\n");

	//****************************************************************************

	// To Kernel SHM
	to_kern_shm = (DataToKern *)rt_shm_alloc( SHM_TO_KERN_KEY, sizeof(DataToKern), USE_VMALLOC );
	if (to_kern_shm == NULL)
		return -ENOMEM;
	memset(to_kern_shm, 0, sizeof(DataToKern));

	to_kern_shm->lock 					= false;
	to_kern_shm->command 	 			= CMD_DISABLE;
	to_kern_shm->controller_requested	= NO_CONTROLLER;
	
	// To Uspace SHM
	for ( i = 0; i < SHM_TO_USPACE_ENTRIES; i++ )
	{
		to_uspace_shm[i] = (DataToUspace *)rt_shm_alloc( SHM_TO_USPACE_KEY + i, sizeof(DataToUspace), USE_VMALLOC );
		if (to_uspace_shm[i] == NULL)
			return -ENOMEM;
		memset(to_uspace_shm[i], 0, sizeof(DataToUspace));
	}

	//****************************************************************************

	rt_sem_init(&master_sem, 1);

	t_critical = cpu_khz * 1000 / FREQUENCY - cpu_khz * INHIBIT_TIME / 1000;

	master = ecrt_request_master(0);
	if (!master) {
	  ret = -EBUSY; 
	  printk(KERN_ERR PFX "Requesting master 0 failed!\n");
	  goto out_return;
	}

	ecrt_master_callbacks(master, request_lock_callback, release_lock_callback, master);

	printk(KERN_INFO PFX "Registering domain...\n");
	if (!(domain1 = ecrt_master_create_domain(master))) {
	  printk(KERN_ERR PFX "Domain creation failed!\n");
	  goto out_release_master;
	}

	// Verify that four slaves are responding, which means that we expect to be attached to ATRIAS.
	check_master_state();
	if ( master_state.slaves_responding != 4 )
	{
		printk("%u Medullas detected.\n", master_state.slaves_responding );	
		goto out_release_master;
	}

	//****************************************************************************

	if (!(sc_medulla_boom = ecrt_master_slave_config(master, MEDULLA_BOOM_POS, VENDOR_ID, PRODUCT_CODE))) {
		printk(KERN_ERR PFX "Failed to get slave configuration.\n");
		goto out_release_master;
	}

	if (ecrt_slave_config_pdos(sc_medulla_boom, EC_END, medulla_boom_sync)) {
		printk(KERN_ERR PFX "Failed to configure PDOs.\n");
		goto out_release_master;
	}

	if (!(sc_medullaB = ecrt_master_slave_config(master, MEDULLA_B_POS, VENDOR_ID, PRODUCT_CODE))) {
		printk(KERN_ERR PFX "Failed to get slave configuration.\n");
		goto out_release_master;
	}

	if (ecrt_slave_config_pdos(sc_medullaB, EC_END, medullaB_sync)) {
		printk(KERN_ERR PFX "Failed to configure PDOs.\n");
		goto out_release_master;
	}

	if (!(sc_medullaA = ecrt_master_slave_config(master, MEDULLA_A_POS, VENDOR_ID, PRODUCT_CODE))) {
		printk(KERN_ERR PFX "Failed to get slave configuration.\n");
		goto out_release_master;
	}

	if (ecrt_slave_config_pdos(sc_medullaA, EC_END, medullaA_sync)) {
		printk(KERN_ERR PFX "Failed to configure PDOs.\n");
		goto out_release_master;
	}

	if (!(sc_medulla_hip = ecrt_master_slave_config(master, MEDULLA_HIP_POS, VENDOR_ID, PRODUCT_CODE))) {
		printk(KERN_ERR PFX "Failed to get slave configuration.\n");
		goto out_release_master;
	}

	if (ecrt_slave_config_pdos(sc_medulla_hip, EC_END, medulla_hip_sync)) {
		printk(KERN_ERR PFX "Failed to configure PDOs.\n");
		goto out_release_master;
	}

	//****************************************************************************

	printk(KERN_INFO PFX "Registering PDO entries...\n");
	if (ecrt_domain_reg_pdo_entry_list(domain1, domain_regs)) 
	{
		printk(KERN_ERR PFX "PDO entry registration failed!\n");
		goto out_release_master;
	}

	printk(KERN_INFO PFX "Activating master...\n");
	if (ecrt_master_activate(master)) 
	{
		printk(KERN_ERR PFX "Failed to activate master!\n");
		goto out_release_master;
	}

	// Get internal process data for domain
	domain1_pd = ecrt_domain_data(domain1);

	// Start the RT task.
	printk(KERN_INFO PFX "Starting cyclic sample thread...\n");
	requested_ticks = nano2count(TIMERTICKS);
	tick_period = start_rt_timer(requested_ticks);
	printk(KERN_INFO PFX "RT timer started with %i/%i ticks.\n",
		 (int) tick_period, (int) requested_ticks);

	if ( rt_task_init_cpuid(&task, run, 0, 10000, 0, 1, NULL, 1) ) 
	{
	  printk(KERN_ERR PFX "Failed to init RTAI task!\n");
	  goto out_stop_timer;
	}

	now = rt_get_time();
	if (rt_task_make_periodic(&task, now + tick_period, tick_period)) 
	{
	  printk(KERN_ERR PFX "Failed to run RTAI task!\n");
	  goto out_stop_task;
	}

	printk(KERN_INFO PFX "Initialized.\n");
	return 0;

out_stop_task:
  rt_task_delete(&task);
out_stop_timer:
  stop_rt_timer();
out_release_master:
  printk(KERN_ERR PFX "Releasing master...\n");
  ecrt_release_master(master);
out_return:
  rt_sem_delete(&master_sem);
  printk(KERN_ERR PFX "Failed to load. Aborting.\n");
  return ret;
}

/*****************************************************************************/

void __exit cleanup_mod(void)
{
	int i;

    printk(KERN_INFO PFX "Stopping...\n");

	rt_busy_sleep(10000000);

	rt_shm_free( SHM_TO_KERN_KEY );

	for ( i = 0; i < SHM_TO_USPACE_ENTRIES; i++ )
	{
		rt_shm_free( SHM_TO_USPACE_KEY + i );
	}

    rt_task_delete(&task);
    stop_rt_timer();
    ecrt_release_master(master);
    rt_sem_delete(&master_sem);

    printk(KERN_INFO PFX "Unloading.\n");
}

/*****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Devin Koepl <fp@igh-essen.com>");
MODULE_DESCRIPTION("EtherCAT RTAI Controller Wrapper");

module_init(init_mod);
module_exit(cleanup_mod);

/*****************************************************************************/
