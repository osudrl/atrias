// Devin Koepl

/****************************************************************************/

#include "ec_control_wrapper.h"

/****************************************************************************/

void endme(int dummy)
{
	printf( "\nStopping...\n" );

	stop_rt_timer();

	signal(SIGINT, SIG_DFL);

	exit(1);
}

/****************************************************************************/

void *control_thread(void *arg)
{
	int i;

	RT_TASK * task;

	// Map I/O pointers to sync manager memory.
	uControllerInput * in[] = 
	{ 
		( uControllerInput * )( domain1_pd + off_medullaA_rx ), ( uControllerInput * )( domain1_pd + off_medullaB_rx ),
		( uControllerInput * )( domain1_pd + off_medulla_hip_rx ), ( uControllerInput * )( domain1_pd + off_medulla_boom_rx ) 
	};
	uControllerOutput *	out[] = 
	{
		( uControllerOutput * )( domain1_pd + off_medullaA_tx ), ( uControllerOutput * )( domain1_pd + off_medullaB_tx ),
		( uControllerOutput * )( domain1_pd + off_medulla_hip_tx ),	( uControllerOutput * )( domain1_pd + off_medulla_boom_tx )
	};

	printf( "Control thread initialized...\n" );

 	if ( !( task = rt_task_init_schmod( nam2num( "control_task" ), 0, 1024, 0, SCHED_FIFO, 0xFF ) ) ) 
	{
		printf( "CANNOT INIT TASK: %d\n", task );
		exit(1);
	}

	rt_make_hard_real_time();
	rt_task_make_periodic( task, rt_get_time() + nano2count(1000000), nano2count(1000000) );

	while ( 1 )
	{		
		// receive process data
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
	
		control_wrapper_state_machine( in, out );

		//rtai_print_to_screen( "test: %d\n", test_global );

		//log[i] = rt_get_cpu_time_ns();

		// send process data
		ecrt_domain_queue(domain1);
		ecrt_master_send(master);

		rt_task_wait_period();
	}

	rt_make_soft_real_time();
	rt_task_delete( task );

	printf( "Finished task...\n" );

	return NULL;
}

/****************************************************************************/

int main(int argc, char **argv)
{
	// RTAI
	int arg = 0;
	pthread_t control_task;

	// End on interrupt. 
	signal(SIGINT, endme);

	printf( "Initializing\n" );
   
    master = ecrt_request_master(0);
    if (!master)
        return -1;

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
        return -1;

	//****************************************************************************

	// Configure ECAT.

	if (!(sc_medulla_boom = ecrt_master_slave_config(master, MEDULLA_BOOM_POS, VENDOR_ID, PRODUCT_CODE))) {
		fprintf( stderr, "Failed to get boom Medulla configuration.\n" );
		return -1;
	}

	if (ecrt_slave_config_pdos(sc_medulla_boom, EC_END, medulla_boom_sync)) {
		fprintf( stderr, "Failed to configure boom Medulla PDOs.\n" );
		return -1;
	}

	if (!(sc_medullaB = ecrt_master_slave_config(master, MEDULLA_B_POS, VENDOR_ID, PRODUCT_CODE))) {
		fprintf( stderr, "Failed to get Medulla B configuration.\n" );
		return -1;
	}

	if (ecrt_slave_config_pdos(sc_medullaB, EC_END, medullaB_sync)) {
		fprintf( stderr, "Failed to configure Medulla B PDOs.\n" );
		return -1;
	}

	if (!(sc_medullaA = ecrt_master_slave_config(master, MEDULLA_A_POS, VENDOR_ID, PRODUCT_CODE))) {
		fprintf( stderr, "Failed to get Medulla A configuration.\n" );
		return -1;
	}

	if (ecrt_slave_config_pdos(sc_medullaA, EC_END, medullaA_sync)) {
		fprintf( stderr, "Failed to configure Medulla A PDOs.\n" );
		return -1;
	}

	if (!(sc_medulla_hip = ecrt_master_slave_config(master, MEDULLA_HIP_POS, VENDOR_ID, PRODUCT_CODE))) {
		fprintf( stderr, "Failed to get hip Medulla configuration.\n" );
		return -1;
	}

	if (ecrt_slave_config_pdos(sc_medulla_hip, EC_END, medulla_hip_sync)) {
		fprintf( stderr, "Failed to configure hip Medulla PDOs.\n" );
		return -1;
	}

	//****************************************************************************	

	// Start ECAT master.

	printf( "Registering PDO entries...\n" );
	if (ecrt_domain_reg_pdo_entry_list(domain1, domain_regs)) 
	{
		fprintf( stderr, "PDO entry registration failed!\n" );
		return -1;
	}

	printf( "Activating master...\n" );
	if (ecrt_master_activate(master)) 
	{
		fprintf( stderr, "Failed to activate master!\n" );
		return -1;
	}

	// Get internal process data for domain
	if ( !( domain1_pd = ecrt_domain_data( domain1 ) ) )
	{
		fprintf(stderr, "Failed to get domain data!\n" );
		return -1;
	}

	//****************************************************************************	

	// RTAI

	rt_set_oneshot_mode();
	start_rt_timer( 0 );

	if ( ! ( control_task = rt_thread_create( control_thread, &arg, 10000 ) ) ) 
	{
		printf( "ERROR IN CREATING THREAD\n" );
		exit(1);
	}  

	while ( 1 )
	{
		pause();
	}

    return 0;
}

/****************************************************************************/
