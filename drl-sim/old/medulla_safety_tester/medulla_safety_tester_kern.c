// Devin Koepl

#include "medulla_safety_tester_kern.h"

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    rt_sem_wait(&master_sem);
    ecrt_domain_state(domain1, &ds);
    rt_sem_signal(&master_sem);

    if (ds.working_counter != domain1_state.working_counter)
        printk(KERN_INFO PFX "Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printk(KERN_INFO PFX "Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

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

void run(long data)
{
	int							i;

	uint8_t					command;

	uint8_t 				medulla_identified;

	uint16_t				tranA_off;

	uint16_t				last_tranA_cnt;

	ControllerInput	controller_input;

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

			// Startup the slaves in disabled mode.
			((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->command = CMD_DISABLE;	

			switch (((uControllerOutput *)(domain1_pd + off_medulla_tx[i]))->id)
			{
				case 0:
					break;
				case MEDULLA_A_ID:
					rt_printk("Medulla A found.\n");
					medulla_A_index = i;
					medulla_identified = true;
					break;
				case MEDULLA_B_ID:
					printk("Medulla B found.\n");
					medulla_B_index = i;
					medulla_identified = true;
					break;
				case MEDULLA_HIP_ID:
					printk("Medulla hip found.\n");
					medulla_hip_index = i;
					medulla_identified = true;
					break;
				case MEDULLA_BOOM_ID:
					printk("Medulla boom found.\n");
					medulla_boom_index = i;
					medulla_identified = true;
					break;
				default:
					rt_printk("Medulla ID %02X not recognized.\n", ((uControllerOutput *)(domain1_pd + off_medulla_tx[i]))->id);
			}

      rt_sem_wait(&master_sem);
      ecrt_domain_queue(domain1);
      rt_sem_signal(&master_sem);
      ecrt_master_send(master);

      rt_task_wait_period();
		}
	}

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

		rt_sem_wait(&master_sem);
		ecrt_domain_queue(domain1);
		rt_sem_signal(&master_sem);
		ecrt_master_send(master);

		rt_task_wait_period();
	}

	// receive process data
	rt_sem_wait(&master_sem);
	ecrt_master_receive(master);
	ecrt_domain_process(domain1);
	rt_sem_signal(&master_sem);

	// Initialize sensors that can rollover, by considering the sensors that don't.
	// First find the leg segment angles.
	controller_input.leg_angleA = UNDISCRETIZE(
		((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->LEG_SEG_ANGLE,
		MAX_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_ANGLE, MIN_LEG_SEG_A_COUNT, MAX_LEG_SEG_A_COUNT);

	// Now use the leg segment angles to estimate the transmission output counts, by assuming small spring deflections.
	last_tranA_cnt = DISCRETIZE(controller_input.leg_angleA,
		MIN_TRAN_A_ANGLE, MAX_TRAN_A_ANGLE, MIN_TRAN_A_COUNT, MAX_TRAN_A_COUNT);

	// Use the estimated encoder counts to figure out the offsets to accomodate for rollovers.
	tranA_off = (last_tranA_cnt / MAX_13BIT) * MAX_13BIT;

	// Set the last transmission counts, so that a false rollover is not tripped immediately.
	last_tranA_cnt = ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE;

  rt_sem_wait(&master_sem);
  ecrt_domain_queue(domain1);
  rt_sem_signal(&master_sem);
  ecrt_master_send(master);

  rt_task_wait_period();

	command = CMD_RUN;

  while (true) 
	{
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
				// Put the controller here.

				// Toggle the command bit.
				((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_A_index]))->command = command;	
				((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_B_index]))->command = command;	
				((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_hip_index]))->command = CMD_DISABLE;
				((uControllerInput *)(domain1_pd + off_medulla_rx[medulla_boom_index]))->command = CMD_DISABLE;
				command ^= CMD_RUN_TOGGLE_bm;

				// Check for sensor rollovers.
				// Transmission A rollovers.
				if (((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE - last_tranA_cnt > ROLLOVER13BIT_THRESHOLD)
				{
					tranA_off -= MAX_13BIT;
				}
				if (last_tranA_cnt - ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE > ROLLOVER13BIT_THRESHOLD)
				{
					tranA_off += MAX_13BIT;
				}
			
				// Keep track of the last counts for sensors that could rollover.
				last_tranA_cnt = ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE;
			}
			else
			{
				// Deal with userspace here.

				if (!uspace_shm->lock)
				{
					uspace_shm->lock	= true;

					uspace_shm->medullaA_biss_encoder		= ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->LEG_SEG_ANGLE;
					uspace_shm->medullaA_ssi_encoder 		= ((uControllerOutput *)(domain1_pd + off_medulla_tx[medulla_A_index]))->TRANS_ANGLE; 

					uspace_shm->fresh = true;
					uspace_shm->lock	= false;
				}
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
    //ec_master_t *m = (ec_master_t *) cb_data;
    rt_sem_wait(&master_sem);
}

/*****************************************************************************/

void release_lock_callback(void *cb_data)
{
    //ec_master_t *m = (ec_master_t *) cb_data;
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

	uspace_shm->lock 			 = false;
	uspace_shm->fresh 		 = true;

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

	// Move sensor initialization and what not here.  That way we do not enter the RT task until everyone is in OP.

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
