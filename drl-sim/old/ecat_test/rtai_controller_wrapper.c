// Devin Koepl

#include "rtai_controller_wrapper.h"

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

void check_slave_config_states(void)
{
	ec_slave_config_state_t s;

	rt_sem_wait(&master_sem);
	ecrt_slave_config_state(sc_medulla, &s);
	rt_sem_signal(&master_sem);

	if (s.al_state != sc_medulla_state.al_state)
		printk(KERN_INFO PFX "AnaIn: State 0x%02X.\n", s.al_state);
	if (s.online != sc_medulla_state.online)
		printk(KERN_INFO PFX "AnaIn: %s.\n", s.online ? "online" : "offline");
	if (s.operational != sc_medulla_state.operational)
		printk(KERN_INFO PFX "AnaIn: %soperational.\n",
				s.operational ? "" : "Not ");

	sc_medulla_state = s;
}

/*****************************************************************************/

void run(long data) {
	int i;

	while (1) {
		t_last_cycle = get_cycles();

		// receive process data
        rt_sem_wait(&master_sem);
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        rt_sem_signal(&master_sem);

		//rt_printk("It worked.\n");

		// check process data state (optional)
		check_domain1_state();		

        ecrt_master_sync_slave_clocks(master);
        ecrt_domain_queue(domain1);
        rt_sem_signal(&master_sem);
        ecrt_master_send(master);

		rt_task_wait_period();
	}
}

/*****************************************************************************/

/*void send_callback(void *cb_data)
{
	ec_master_t *m = (ec_master_t *) cb_data;

	// too close to the next real time cycle: deny access...
	if (get_cycles() - t_last_cycle <= t_critical) {
		rt_sem_wait(&master_sem);
		ecrt_master_send(m);
		rt_sem_signal(&master_sem);

		rt_printk("Cannot send, too close to next RT cycle.\n");
	}

    //ec_master_t *m = (ec_master_t *) cb_data;
    //rt_sem_signal(&master_sem);
}*/

/*****************************************************************************/

/*void receive_callback(void *cb_data)
{
	ec_master_t *m = (ec_master_t *) cb_data;

	// too close to the next real time cycle: deny access...
	if (get_cycles() - t_last_cycle <= t_critical) {
		rt_sem_wait(&master_sem);
		ecrt_master_receive(m);
		rt_sem_signal(&master_sem);

		rt_printk("Cannot receive, too close to next RT cycle.\n");
	}

    //ec_master_t *m = (ec_master_t *) cb_data;
    //rt_sem_wait(&master_sem);
}*/

/*****************************************************************************/

int __init init_mod(void) {

	int ret = -1;
	RTIME tick_period, requested_ticks, now;
	ec_slave_config_t *sc;


	txpdo.ch1 = 0;
	txpdo.ch2 = 0;
	txpdo.ch3 = 0;
	
	rxpdo.ch1 = 0;
	rxpdo.ch2 = 0;

	printk(KERN_INFO PFX "Starting...\n");

	rt_sem_init(&master_sem, 1);

	t_critical = cpu_khz * 1000 / FREQUENCY - cpu_khz * INHIBIT_TIME / 1000;

	master = ecrt_request_master(0);
	if (!master) {
		ret = -EBUSY; 
		printk(KERN_ERR PFX "Requesting master 0 failed!\n");
		goto out_return;
	}

	//ecrt_master_callbacks(master, send_callback, receive_callback, master);

	printk(KERN_INFO PFX "Registering domain...\n");
	if (!(domain1 = ecrt_master_create_domain(master))) {
		printk(KERN_ERR PFX "Domain creation failed!\n");
		goto out_release_master;
	}

	if (!(sc_medulla = ecrt_master_slave_config( master, MEDULLA_POS, MEDULLA_IDPC))) {
		printk(KERN_ERR PFX "Failed to get slave configuration.\n");
		goto out_release_master;
	}

	printk(KERN_INFO PFX "Configuring PDOs...\n");
	if (ecrt_slave_config_pdos( sc_medulla, EC_END, medulla_syncs)) {
		printk(KERN_ERR PFX "Failed to configure PDOs.\n");
		goto out_release_master;
	}

//	if (!(sc = ecrt_master_slave_config(master, DigOutSlavePos, Beckhoff_EL2004))) {
//		printk(KERN_ERR PFX "Failed to get slave configuration.\n");
//		goto out_release_master;
//	}

//	if (ecrt_slave_config_pdos(sc, EC_END, el2004_syncs)) {
//		printk(KERN_ERR PFX "Failed to configure PDOs.\n");
//		goto out_release_master;
//s	}

	printk(KERN_INFO PFX "Registering PDO entries...\n");
	if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
		printk(KERN_ERR PFX "PDO entry registration failed!\n");
		goto out_release_master;
	}

	printk(KERN_INFO PFX "Activating master...\n");
	if (ecrt_master_activate(master)) {
		printk(KERN_ERR PFX "Failed to activate master!\n");
		goto out_release_master;
	}

	// Get internal process data for domain
	domain1_pd = ecrt_domain_data(domain1);

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

	rt_task_delete(&task);
	stop_rt_timer();
	ecrt_release_master(master);
	rt_sem_delete(&master_sem);

	printk(KERN_INFO PFX "Unloading.\n");
}

/*****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Devin Koepl <devin.koepl@gmail.com>");
MODULE_DESCRIPTION("EtherCAT Tester");

module_init(init_mod);
module_exit(cleanup_mod);

/*****************************************************************************/
