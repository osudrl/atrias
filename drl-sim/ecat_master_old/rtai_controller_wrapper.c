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
	int i;

	for (i = 0; i < 100; i++);
	{
		// receive process data
		rt_sem_wait(&master_sem);
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
		rt_sem_signal(&master_sem);

		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->command = CMD_DISABLE;
		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_B_index]))->command = CMD_DISABLE;
		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_hip_index]))->command = CMD_DISABLE;
		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_boom_index]))->command = CMD_DISABLE;

		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->MOTOR_TORQUE = PWM_OPEN;
		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_B_index]))->MOTOR_TORQUE = PWM_OPEN;
		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_hip_index]))->HIP_MTR_CMD = HIP_CMD_PIN;

		rt_sem_wait(&master_sem);
		ecrt_domain_queue(domain1);
		rt_sem_signal(&master_sem);
		ecrt_master_send(master);

		rt_task_wait_period();
	}

	// If all of the Medullas came up, and are now disabled, report success.
	if ( (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->status == STATUS_DISABLED)
		&& (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->status == STATUS_DISABLED)
		&& (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_hip_index]))->status == STATUS_DISABLED)
		&& (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->status == STATUS_DISABLED) )
	{
		
		return true;
	}

	if (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->status != STATUS_DISABLED )
	{
		rt_printk("Medulla A sad: %.2X\n", ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->status);
	}

	if (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->status != STATUS_DISABLED )
	{
		rt_printk("Medulla B sad: %.2X\n", ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->status);
	}

	if (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_hip_index]))->status != STATUS_DISABLED )
	{
		rt_printk("Medulla Hip sad: %.2X\n", ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_hip_index]))->status);
	}

	if (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->status != STATUS_DISABLED )
	{
		rt_printk("Medulla Boom sad: %.2X\n", ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->status);
	}

	return false;
}

/*****************************************************************************/

unsigned char identify_medullas()
{
	int								i;

	uint8_t 					medulla_identified;

	// Figure out what slaves are attached where.  It may take a while for all
	// of the slaves to go into OP mode and start advertising their id byte.
	for (i = 0; i < num_of_medullas; i++)
	{
		medulla_identified = false;

		while (!medulla_identified)
		{
			// receive process data
			rt_sem_wait(&master_sem);
			ecrt_master_receive(master);
			ecrt_domain_process(domain1);
			rt_sem_signal(&master_sem);

			((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->command = CMD_DISABLE;
			((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_B_index]))->command = CMD_DISABLE;
			((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_hip_index]))->command = CMD_DISABLE;
			((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_boom_index]))->command = CMD_DISABLE;

			((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->MOTOR_TORQUE = PWM_OPEN;
			((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_B_index]))->MOTOR_TORQUE = PWM_OPEN;
			((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_hip_index]))->HIP_MTR_CMD = HIP_CMD_PIN;

			switch (((uControllerOutput *)(domain1_pd + off_medulla_tx[i]))->id)
			{
				case 0:
					break;
				case MEDULLA_A_ID:
					rt_printk("\n\n\nMedulla A found.\n\n\n\n");
					medulla_A_index = i;
					medulla_identified = true;
					break;
				case MEDULLA_B_ID:
					printk("\n\n\nMedulla B found.\n\n\n\n");
					medulla_B_index = i;
					medulla_identified = true;
					break;
				case MEDULLA_HIP_ID:
					printk("\n\n\nMedulla hip found.\n\n\n\n");
					medulla_hip_index = i;
					medulla_identified = true;
					break;
				case MEDULLA_BOOM_ID:
					printk("\n\n\nMedulla boom found.\n\n\n\n");
					medulla_boom_index = i;
					medulla_identified = true;
					break;
				default:
					rt_printk("Medulla ID %02X not recognized.\n\n\n\n", ((uControllerOutput *)(domain1_pd + off_medulla_tx[i]))->id);
			}

      rt_sem_wait(&master_sem);
      ecrt_domain_queue(domain1);
      rt_sem_signal(&master_sem);
      ecrt_master_send(master);

      rt_task_wait_period();
		}
	}

	return true;
}

/*****************************************************************************/

void run(long data)
{
	int								i;

	uint8_t						command;

	uint16_t					tranA_off;
	uint16_t					tranB_off;

	int32_t						boom_pan_off = 0;
	int32_t						boom_tilt_off = 0;

	uint16_t					last_boom_pan_cnt;
	uint16_t					first_boom_pan_cnt;
	uint16_t					last_boom_tilt_cnt;

	float							boom_pan_angle = 0.;
	float							boom_tilt_angle = 0.;

	float							last_boom_pan_angle = 0.;

	uint16_t					last_tranA_cnt;
	uint16_t					last_tranB_cnt;

	float							last_motor_angleA = 0.;
	float							last_motor_angleB = 0.;
	float							last_leg_angleA		= 0.;
	float							last_leg_angleB		= 0.;

	float							hor_vel;
	float							hor_vel_buffer[HOR_VEL_WINDOW];
	int								hor_vel_index = 0;

	ControllerInput	 	controller_input;
	ControllerOutput 	controller_output;
	ControllerState	 	controller_state;
	ControllerData	 	controller_data;
	
	while ( !wake_up_medullas() );

	while ( !identify_medullas() );

	//while ( true ) // Leave in while loop to send command disables.
	//{

		// receive process data
		rt_sem_wait(&master_sem);
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
		rt_sem_signal(&master_sem);

		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->command = CMD_DISABLE;
		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_B_index]))->command = CMD_DISABLE;
		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_hip_index]))->command = CMD_DISABLE;
		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_boom_index]))->command = CMD_DISABLE;

		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->MOTOR_TORQUE = PWM_OPEN;
		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_B_index]))->MOTOR_TORQUE = PWM_OPEN;
		((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_hip_index]))->HIP_MTR_CMD = HIP_CMD_PIN;

		rt_printk("A: %u,\tB: %u\n", ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->LEG_SEG_ANGLE,
			((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->LEG_SEG_ANGLE);

		// Initialize sensors that can rollover, by considering the sensors that don't.
		// First find the leg segment angles.
		controller_input.leg_angleA = UNDISCRETIZE(
			((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->LEG_SEG_ANGLE,
			MAX_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_COUNT, MAX_LEG_SEG_A_COUNT);

		controller_input.leg_angleB = UNDISCRETIZE(
			((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->LEG_SEG_ANGLE,
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
		last_tranA_cnt = ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE;
		last_tranB_cnt = ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->TRANS_ANGLE;

		if ( ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_TILT_CNT < TILT_CNT_THRESH )
		{
			boom_tilt_off = MAX_16BIT;
		}

		// Grab the initial pan count, so that we know how far the robot has moved about the room.
		first_boom_pan_cnt = ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_PAN_CNT;

		rt_sem_wait(&master_sem);
		ecrt_domain_queue(domain1);
		rt_sem_signal(&master_sem);
		ecrt_master_send(master);

		rt_task_wait_period();

	//}

	command = CMD_RUN;
	controller_state.state = STATE_INIT;

	// Now start controlling the robot
  while (true) {
		t_last_cycle = get_cycles();

		// receive process data
		rt_sem_wait(&master_sem);
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
		rt_sem_signal(&master_sem);

		rt_sem_wait(&master_sem);
		ecrt_domain_state(domain1, &domain1_state);
		rt_sem_signal(&master_sem);

		if (domain1_state.working_counter == EC_WC_ZERO) //EC_WC_COMPLETE
		{
			// Toggle the command bit.
			((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->command = command;	
			((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_B_index]))->command = command;	
			((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_hip_index]))->command = command;
			((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_boom_index]))->command = command;
			command ^= CMD_RUN_TOGGLE_bm;

			// Check for sensor rollovers.
			// Transmission A rollovers.
			if ( (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE > last_tranA_cnt) 
				&& (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE - last_tranA_cnt > ROLLOVER13BIT_THRESHOLD) )
			{
				tranA_off -= MAX_13BIT;
			}
			if ( (last_tranA_cnt > ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE)
				&& (last_tranA_cnt - ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE > ROLLOVER13BIT_THRESHOLD) )
			{
				tranA_off += MAX_13BIT;
			}
			// Transmission B rollovers.
			if ( (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->TRANS_ANGLE > last_tranB_cnt) 
				&& (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->TRANS_ANGLE - last_tranB_cnt > ROLLOVER13BIT_THRESHOLD) )
			{
				tranB_off -= MAX_13BIT;
			}
			if ( (last_tranB_cnt > ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->TRANS_ANGLE)
				&& (last_tranB_cnt - ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->TRANS_ANGLE > ROLLOVER13BIT_THRESHOLD) )
			{
				tranB_off += MAX_13BIT;
			}
			// Boom Pan
			if ( (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_PAN_CNT > last_boom_pan_cnt) 
				&& (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_PAN_CNT - last_boom_pan_cnt > ROLLOVER16BIT_THRESHOLD) )
			{
				boom_pan_off -= MAX_16BIT;
			}
			if ( (last_boom_pan_cnt > ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_PAN_CNT)
				&& (last_boom_pan_cnt - ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_PAN_CNT > ROLLOVER16BIT_THRESHOLD) )
			{
				boom_pan_off += MAX_16BIT;
			}
			// Boom Tilt
			if ( (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_TILT_CNT > last_boom_tilt_cnt) 
				&& (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_TILT_CNT - last_boom_tilt_cnt > ROLLOVER16BIT_THRESHOLD) )
			{
				boom_tilt_off -= MAX_16BIT;
			}
			if ( (last_boom_tilt_cnt > ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_TILT_CNT)
				&& (last_boom_tilt_cnt - ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_TILT_CNT > ROLLOVER16BIT_THRESHOLD) )
			{
				boom_tilt_off += MAX_16BIT;
			}

			// Keep track of the last counts for sensors that could rollover.
			last_tranA_cnt = ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE;
			last_tranB_cnt = ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->TRANS_ANGLE;
			last_boom_pan_cnt		= ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_PAN_CNT;
			last_boom_tilt_cnt	= ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_TILT_CNT;

			// Generate controller input
			controller_input.leg_angleA		= UNDISCRETIZE(
				((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->LEG_SEG_ANGLE,
				MAX_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_COUNT, MAX_LEG_SEG_A_COUNT);

			controller_input.leg_angleB		= UNDISCRETIZE(
				((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->LEG_SEG_ANGLE,
				MIN_LEG_SEG_B_ANGLE, MAX_LEG_SEG_B_ANGLE, MIN_LEG_SEG_B_COUNT, MAX_LEG_SEG_B_COUNT);

			controller_input.motor_angleA 	= UNDISCRETIZE(
				tranA_off + ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE,
				MAX_TRAN_A_ANGLE, MIN_TRAN_A_ANGLE, MIN_TRAN_A_COUNT, MAX_TRAN_A_COUNT) + TRAN_A_OFF_ANGLE;

			controller_input.motor_angleB 	= UNDISCRETIZE(
				tranB_off + ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->TRANS_ANGLE,
				MIN_TRAN_B_ANGLE, MAX_TRAN_B_ANGLE, MIN_TRAN_B_COUNT, MAX_TRAN_B_COUNT) + TRAN_B_OFF_ANGLE;				

			controller_input.motor_velocityA = (controller_input.motor_angleA - last_motor_angleA)
				/ ( (float)((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->timestep * SEC_PER_CNT );
			controller_input.motor_velocityB = (controller_input.motor_angleB - last_motor_angleB) 
				/ ( (float)((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->timestep * SEC_PER_CNT );

			controller_input.leg_velocityA = (controller_input.leg_angleA - last_leg_angleA)
				/ ( (float)((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->timestep * SEC_PER_CNT );	
			controller_input.leg_velocityB = (controller_input.leg_angleB - last_leg_angleB)
				/ ( (float)((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->timestep * SEC_PER_CNT );					

			boom_pan_angle	= DISCRETIZE_LOCATION( ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_PAN_CNT,
				first_boom_pan_cnt, 0., MAX_16BIT, BOOM_PAN_GEAR_RATIO);
			// Filter the horizontal velocity.
			hor_vel = BOOM_LENGTH * (boom_pan_angle - last_boom_pan_angle)
				/ ( (float)((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->timestep * SEC_PER_CNT );
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

			boom_tilt_angle = DISCRETIZE_LOCATION( ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->BOOM_TILT_CNT + boom_tilt_off, 
				BOOM_KNOWN_TILT_CNT, BOOM_KNOWN_TILT_ANGLE, MAX_16BIT, BOOM_TILT_GEAR_RATIO);
			controller_input.height = BOOM_LENGTH * sin(boom_tilt_angle) + BOOM_PIVOT_HEIGHT;

			last_motor_angleA = controller_input.motor_angleA;
			last_motor_angleB = controller_input.motor_angleB;
			last_leg_angleA		= controller_input.leg_angleA;
			last_leg_angleB		= controller_input.leg_angleB;

			// Send motor torques only when all of the Medullas status's are okay..
			if ( ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->status 
				|| ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->status
				|| ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_hip_index]))->status
				|| ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->status )
			{
				// At least one of the Medullas is unhappy, so don't send motor torques.

				((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->MOTOR_TORQUE = DISCRETIZE(
					0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
				((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_B_index]))->MOTOR_TORQUE = DISCRETIZE(
					0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);

				rt_printk("\nUnhappy Medulla detected.  Status: A: %02X, B: %02X, Hip: %02X, Boom: %02X\n\n",
					((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->status,
					((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->status,
					((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_hip_index]))->status,
					((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_boom_index]))->status );
			}
			else
			{
				// All of the Medullas are okay, so we go ahead and send the torques.

				// If both torques are below the threshold, then send a small torque to keep the robot off of its hardstops.
				if ( ( ABS(controller_output.motor_torqueA) < MIN_TRQ_THRESH ) 
					&& ( ABS(controller_output.motor_torqueB) < MIN_TRQ_THRESH ) )
				{
					((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->MOTOR_TORQUE = PWM_OPEN;
					((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_B_index]))->MOTOR_TORQUE = PWM_OPEN;
				}
				else
				{
					// Send motor torques.
					((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->MOTOR_TORQUE = DISCRETIZE(
						controller_output.motor_torqueA, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
					// Sign change on torque B?
					//		5/4/2011 2:00pm
					((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_B_index]))->MOTOR_TORQUE = DISCRETIZE(
						-controller_output.motor_torqueB, MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
					//((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->MOTOR_TORQUE = DISCRETIZE(
					//	0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
					//((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_B_index]))->MOTOR_TORQUE = DISCRETIZE(
					//	0., MTR_MIN_TRQ, MTR_MAX_TRQ, MTR_MIN_CNT, MTR_MAX_CNT);
				}
			}

			// Send hip command.  Do this regardless of the status of the Medullas to try and protect the knee from moments.
			if ( controller_input.height < 1.0 )
			{
				((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_hip_index]))->HIP_MTR_CMD = HIP_CMD_PIN;
			}
			else
			{
				((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_hip_index]))->HIP_MTR_CMD = HIP_CMD_RIGID;
			}

		}
		else
		{
			// Uspace shm
			if (!uspace_shm->lock)
			{
				uspace_shm->lock								= true;

				if ( controller_state.state == STATE_ENABLED )
				{
					uspace_shm->status						= CMD_RUN;
				}
				else
				{
					uspace_shm->status						= CMD_DISABLE;
				}

				uspace_shm->time							 += 0.001;

				uspace_shm->leg_angleA					= controller_input.leg_angleA;
				uspace_shm->leg_angleB					= controller_input.leg_angleB;
				uspace_shm->motor_angleA 				= controller_input.motor_angleA;
				uspace_shm->motor_angleB 				= controller_input.motor_angleB;

				uspace_shm->hor_vel						= controller_input.horizontal_velocity;
				uspace_shm->height						= controller_input.height;

				uspace_shm->motor_torqueA				= controller_output.motor_torqueA;
				uspace_shm->motor_torqueB				= controller_output.motor_torqueB;

				// Copy the controller state into shm.
				for (i = 0; i < SIZE_OF_CONTROLLER_STATE_DATA; i++)
				{
					uspace_shm->controller_state[i] = controller_state.data[i];
				}					

				controller_data.command = uspace_shm->command;
				controller_data.controller_requested = uspace_shm->controller_requested;

				// Copy the controller data out of shm.
				for (i = 0; i < SIZE_OF_CONTROLLER_DATA; i++)
				{
					controller_data.data[i] = uspace_shm->controller_data[i];
				}

				uspace_shm->fresh = true;
				uspace_shm->lock  = false;
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


			// Print the status bytes.
			//rt_printk("Status Bytes:\nA: %02X\tB: %02X\thip: %02X\n",
			//	((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->status,
			//	((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_B_index]))->status,
			//	((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_hip_index]))->status);
			//rt_printk("A: %u\n", ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE);		
		}

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
    ec_master_t *m = (ec_master_t *) cb_data;
    rt_sem_wait(&master_sem);
}

/*****************************************************************************/

void release_lock_callback(void *cb_data)
{
    ec_master_t *m = (ec_master_t *) cb_data;
    rt_sem_signal(&master_sem);
}

/*****************************************************************************/

int __init init_mod(void)
{
	int i, ret = -1;
  RTIME tick_period, requested_ticks, now;

  ec_slave_config_t *sc;

	ec_sync_info_t medulla_sync[] = {
		{0, EC_DIR_OUTPUT, 0, NULL,	EC_WD_DISABLE},
		{1, EC_DIR_INPUT , 0, NULL,	EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, NULL,	EC_WD_ENABLE},
		{3, EC_DIR_INPUT , 1, NULL,	EC_WD_ENABLE},
		{0xff}
	};

	printk("\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  printk(KERN_INFO PFX "Starting...\n");

	//****************************************************************************

	// Prepare the SHM
	uspace_shm = (UspaceKernShm *)rtai_kmalloc(nam2num(SHMNAM), sizeof(UspaceKernShm));
	uspace_shm = (UspaceKernShm *)rtai_kmalloc(nam2num(SHMNAM), sizeof(UspaceKernShm));
	uspace_shm = (UspaceKernShm *)rtai_kmalloc(nam2num(SHMNAM), sizeof(UspaceKernShm));
	uspace_shm = (UspaceKernShm *)rtai_kmalloc(nam2num(SHMNAM), sizeof(UspaceKernShm));
	if (uspace_shm == NULL)
		return -ENOMEM;
	memset(uspace_shm, 0, sizeof(UspaceKernShm));

	uspace_shm->lock 									= false;
	uspace_shm->fresh 								= true;
	uspace_shm->command 	 						= CMD_DISABLE;
	uspace_shm->controller_requested	= NO_CONTROLLER;

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

	// Check to see how many slaves are responding to figure out what we are attached to.
	check_master_state();
	num_of_medullas = master_state.slaves_responding;
	printk("%u Medullas detected.\n", num_of_medullas);		

	for (i = 0; i < num_of_medullas; i++)
	{
		medulla_rxpdo_entries[i].index = 0x6126;
		medulla_rxpdo_entries[i].subindex = 0x21 + i;
		medulla_rxpdo_entries[i].bit_length = BITS_IN_A_BYTE * sizeof(uControllerInput);		
	
		medulla_txpdo_entries[i].index = 0x6130;
		medulla_txpdo_entries[i].subindex = 0x01 + i;
		medulla_txpdo_entries[i].bit_length = BITS_IN_A_BYTE * sizeof(uControllerOutput);

		medulla_rx_pdos[i].index = 0x1600 + i;
		medulla_rx_pdos[i].n_entries = 1;
		medulla_rx_pdos[i].entries = &medulla_rxpdo_entries[i];

		medulla_tx_pdos[i].index = 0x1A00 + i;
		medulla_tx_pdos[i].n_entries = 1;
		medulla_tx_pdos[i].entries = &medulla_txpdo_entries[i];

		medulla_sync[2].pdos = &medulla_rx_pdos[i];		
		medulla_sync[3].pdos = &medulla_tx_pdos[i];

	  if (!(sc = ecrt_master_slave_config(master, MEDULLA_POS + i, VENDOR_ID, PRODUCT_CODE))) {
	      printk(KERN_ERR PFX "Failed to get slave configuration.\n");
	      goto out_release_master;
	  }

	  if (ecrt_slave_config_pdos(sc, EC_END, medulla_sync)) {
	      printk(KERN_ERR PFX "Failed to configure PDOs.\n");
	      goto out_release_master;
	  }
	}

  printk(KERN_INFO PFX "Registering PDO entries...\n");
	switch (num_of_medullas)
	{
		case 1:
			if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
			    printk(KERN_ERR PFX "PDO entry registration failed!\n");
			    goto out_release_master;
			}
			break;
		case 2:
			if (ecrt_domain_reg_pdo_entry_list(domain1, domain2_regs)) {
			    printk(KERN_ERR PFX "PDO entry registration failed!\n");
			    goto out_release_master;
			}
			break;
		case 3:
			if (ecrt_domain_reg_pdo_entry_list(domain1, domain3_regs)) {
			    printk(KERN_ERR PFX "PDO entry registration failed!\n");
			    goto out_release_master;
			}
			break;
		case 4:
			if (ecrt_domain_reg_pdo_entry_list(domain1, domain4_regs)) {
			    printk(KERN_ERR PFX "PDO entry registration failed!\n");
			    goto out_release_master;
			}
	}

  printk(KERN_INFO PFX "Activating master...\n");
  if (ecrt_master_activate(master)) {
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

  if (rt_task_init(&task, run, 0, 2000, 0, 1, NULL)) {
      printk(KERN_ERR PFX "Failed to init RTAI task!\n");
      goto out_stop_timer;
  }

  now = rt_get_time();
  if (rt_task_make_periodic(&task, now + tick_period, tick_period)) {
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
    printk(KERN_INFO PFX "Stopping...\n");

		rt_busy_sleep(10000000);
		rtai_kfree(nam2num(SHMNAM));
		rtai_kfree(nam2num(SHMNAM));
		rtai_kfree(nam2num(SHMNAM));
		rtai_kfree(nam2num(SHMNAM));
		rtai_kfree(nam2num(SHMNAM));
		rtai_kfree(nam2num(SHMNAM));

    rt_task_delete(&task);
    stop_rt_timer();
    ecrt_release_master(master);
    rt_sem_delete(&master_sem);

    printk(KERN_INFO PFX "Unloading.\n");
}

/*****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florian Pose <fp@igh-essen.com>");
MODULE_DESCRIPTION("EtherCAT RTAI sample module");

module_init(init_mod);
module_exit(cleanup_mod);

/*****************************************************************************/
