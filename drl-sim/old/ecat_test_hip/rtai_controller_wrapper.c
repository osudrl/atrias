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

	((uControllerInput *)(domain1_pd + off_medulla_rx))->command = 0;

	rt_sem_wait(&master_sem);
	ecrt_domain_queue(domain1);
	rt_sem_signal(&master_sem);
	ecrt_master_send(master);

	rt_task_wait_period();

	rt_printk("Waking up Medullas.\n");

	// If all of the Medullas report the bad command, then return success.
	if (((uControllerOutput *)(domain1_pd + off_medulla_tx))->status & STATUS_BADCMD)
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

	((uControllerInput *)(domain1_pd + off_medulla_rx))->command = CMD_DISABLE;

	((uControllerInput *)(domain1_pd + off_medulla_rx))->HIP_MTR_CMD = HIP_CMD_PIN;

	rt_sem_wait(&master_sem);
	ecrt_domain_queue(domain1);
	rt_sem_signal(&master_sem);
	ecrt_master_send(master);

	rt_task_wait_period();

	if (((uControllerOutput *)(domain1_pd + off_medulla_tx))->status != STATUS_DISABLED)
	{
		rt_printk("Medulla A not in disabled state.\n");
		result = 0;
	}

	if ( ((uControllerOutput *)(domain1_pd + off_medulla_tx))->id != MEDULLA_HIP_ID )
	{
		rt_printk("Medulla A reporting wrong ID.\n");
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

		((uControllerInput *)(domain1_pd + off_medulla_rx))->command = CMD_RESTART;

		((uControllerInput *)(domain1_pd + off_medulla_rx))->HIP_MTR_CMD = HIP_CMD_PIN;

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

	((uControllerInput *)(domain1_pd + off_medulla_rx))->HIP_MTR_CMD = HIP_CMD_PIN;

	rt_sem_wait(&master_sem);
	ecrt_domain_queue(domain1);
	rt_sem_signal(&master_sem);
	ecrt_master_send(master);

	rt_task_wait_period();

	/*************************************************************************/

	// Now start controlling the robot
	while (true) 
	{
		// receive process data
		rt_sem_wait(&master_sem);
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
		rt_sem_signal(&master_sem);	

		// Toggle the command bit.
		((uControllerInput *)(domain1_pd + off_medulla_rx))->command = command;	
		command ^= CMD_RUN_TOGGLE_bm;

		((uControllerInput *)(domain1_pd + off_medulla_rx))->HIP_MTR_CMD = HIP_CMD_PIN;

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

	//****************************************************************************

	printk("\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
	printk(KERN_INFO PFX "Starting...\n");

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
	if ( master_state.slaves_responding != NUM_OF_MEDULLAS_ON_ROBOT )
	{
		printk("%u Medullas detected.\n", master_state.slaves_responding );	
		goto out_release_master;
	}

	//****************************************************************************

	if (!(sc_medulla = ecrt_master_slave_config(master, MEDULLA_POS, VENDOR_ID, PRODUCT_CODE))) {
		printk(KERN_ERR PFX "Failed to get slave configuration.\n");
		goto out_release_master;
	}

	if (ecrt_slave_config_pdos(sc_medulla, EC_END, medulla_sync)) {
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

	//rt_shm_free( SHM_TO_KERN_KEY );

	//rt_shm_free( SHM_TO_USPACE_KEY );

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
