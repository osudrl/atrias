// Devin Koepl

/****************************************************************************/

#include <atrias_controllers/rt_atrias_controller_wrapper.h>

/****************************************************************************/

void *control_thread(void *arg)
{
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

	ROS_INFO( "Control thread initialized..." );

 	if ( !( task = rt_task_init_schmod( nam2num( "control_task" ), 0, 1024, 0, SCHED_FIFO, 0xFF ) ) ) 
		ROS_ERROR( "CANNOT INIT TASK: %d", task );

	rt_make_hard_real_time();
	rt_task_make_periodic( task, rt_get_time() + nano2count(1000000), nano2count(1000000) );

	while ( ros::ok() )
	{		
		// receive process data
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);

		control_wrapper_state_machine( in, out );

		//log[i] = rt_get_cpu_time_ns();

		// send process data
		ecrt_domain_queue(domain1);
		ecrt_master_send(master);

		rt_task_wait_period();
	}

	rt_make_soft_real_time();
	rt_task_delete( task );

	ROS_INFO( "Stopping..." );

	stop_rt_timer();

	datalog();

	return NULL;
}

/****************************************************************************/

int main(int argc, char **argv)
{
	// RTAI
	int arg = 0;
	pthread_t control_task;

	// End on interrupt. 
	//signal(SIGINT, endme);

	ROS_INFO( "Initializing..." );
   
    master = ecrt_request_master(0);
    if (!master)
        return -1;

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
        return -1;

	//****************************************************************************

	// Configure ECAT.

	if (!(sc_medulla_boom = ecrt_master_slave_config(master, MEDULLA_BOOM_POS, VENDOR_ID, PRODUCT_CODE)))
		ROS_ERROR( "Failed to get boom Medulla configuration." );

	if (ecrt_slave_config_pdos(sc_medulla_boom, EC_END, medulla_boom_sync))
		ROS_ERROR( "Failed to configure boom Medulla PDOs." );

	if (!(sc_medullaB = ecrt_master_slave_config(master, MEDULLA_B_POS, VENDOR_ID, PRODUCT_CODE)))
		ROS_ERROR( "Failed to get Medulla B configuration." );

	if (ecrt_slave_config_pdos(sc_medullaB, EC_END, medullaB_sync))
		ROS_ERROR( "Failed to configure Medulla B PDOs." );

	if (!(sc_medullaA = ecrt_master_slave_config(master, MEDULLA_A_POS, VENDOR_ID, PRODUCT_CODE)))
		ROS_ERROR( "Failed to get Medulla A configuration." );

	if (ecrt_slave_config_pdos(sc_medullaA, EC_END, medullaA_sync))
		ROS_ERROR( "Failed to configure Medulla A PDOs." );

	if (!(sc_medulla_hip = ecrt_master_slave_config(master, MEDULLA_HIP_POS, VENDOR_ID, PRODUCT_CODE)))
		ROS_ERROR( "Failed to get hip Medulla configuration." );

	if (ecrt_slave_config_pdos(sc_medulla_hip, EC_END, medulla_hip_sync))
		ROS_ERROR( "Failed to configure hip Medulla PDOs." );

	//****************************************************************************	

	// Start ECAT master.

	ROS_INFO( "Registering PDO entries..." );
	if (ecrt_domain_reg_pdo_entry_list(domain1, domain_regs)) 
		ROS_ERROR( "PDO entry registration failed!" );

	ROS_INFO( "Activating master..." );
	if (ecrt_master_activate(master)) 
		ROS_ERROR( "Failed to activate master!" );

	// Get internal process data for domain
	if ( !( domain1_pd = ecrt_domain_data( domain1 ) ) )
		ROS_ERROR( "Failed to get domain data!" );

	//****************************************************************************	

	// RTAI

	rt_set_oneshot_mode();
	start_rt_timer( 0 );

	if ( ! ( control_task = rt_thread_create( (void *)control_thread, &arg, 10000 ) ) ) 
		ROS_ERROR( "ERROR IN CREATING THREAD" );

	//*************************************************************************

	// Startup ROS and interface service.

	ros::init(argc, argv, "gui_interface");
	ros::NodeHandle nh;
	ros::ServiceServer gui_srv = nh.advertiseService("gui_interface_srv", atrias_gui_callback);

	ROS_INFO("ROS Service Advertised.");

	//*************************************************************************

	// We could do something here if we wanted to.

	while ( ros::ok() )
	{
		rt_msg_print();

		ros::spinOnce();
	}

	//*************************************************************************

	// Wait for the control task to finish.
	rt_thread_join( control_task );

    return 0;
}

/****************************************************************************/
