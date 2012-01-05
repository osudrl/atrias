#ifndef OROCOS_ATRIAS_CONTROLLER_WRAPPER_COMPONENT_HPP
#define OROCOS_ATRIAS_CONTROLLER_WRAPPER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <iostream>



// ============================================================================
// rtai_controller_wrapper
// ============================================================================

// EtherCAT
#include <ecrt.h>

// ATRIAS
#include <atrias/ucontroller.h>


/*****************************************************************************/

// Module parameters

#define FREQUENCY 										1000 // task frequency in Hz
#define INHIBIT_TIME 									20

#define TIMERTICKS 										(1000000000 / FREQUENCY)

// Optional features (comment to disable)
#define CONFIGURE_PDOS

#define PFX 											"ec_rtai_controller: "

#define MEDULLA_BOOM_POS								0, 0
#define MEDULLA_B_POS									0, 1
#define MEDULLA_A_POS									0, 2
#define MEDULLA_HIP_POS									0, 3

#define VENDOR_ID										0x00000777
#define PRODUCT_CODE						 			0x02628111

#define NUM_OF_MEDULLAS_ON_ROBOT 						4 
#define NUM_OF_SLAVES_IN_SIMULATION_MACHINE 			1

#define BITS_IN_A_BYTE 									8

#define HOR_VEL_FILTER_EPS								0.003
#define HOR_VEL_WINDOW									100

/*****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

// Slave Configurations
static ec_slave_config_t *sc_medulla_boom;
static ec_slave_config_t *sc_medullaB;
static ec_slave_config_t *sc_medullaA;
static ec_slave_config_t *sc_medulla_hip;

// RTAI
static RT_TASK task;
static SEM master_sem;
static cycles_t t_last_cycle = 0, t_critical;

/*****************************************************************************/

// process data
static uint8_t *domain1_pd; // process data memory

// Slave offsets
static unsigned int off_medulla_boom_rx;
static unsigned int off_medullaB_rx;
static unsigned int off_medullaA_rx;
static unsigned int off_medulla_hip_rx;

static unsigned int off_medulla_boom_tx;
static unsigned int off_medullaB_tx;
static unsigned int off_medullaA_tx;
static unsigned int off_medulla_hip_tx;

const static ec_pdo_entry_reg_t domain_regs[] =
{
	{MEDULLA_BOOM_POS,	VENDOR_ID, PRODUCT_CODE, 0x6126, 0x21, &off_medulla_boom_rx},
	{MEDULLA_BOOM_POS,	VENDOR_ID, PRODUCT_CODE, 0x6130, 0x01, &off_medulla_boom_tx},
	{MEDULLA_B_POS	 ,	VENDOR_ID, PRODUCT_CODE, 0x6126, 0x22, &off_medullaB_rx},
	{MEDULLA_B_POS	 ,	VENDOR_ID, PRODUCT_CODE, 0x6130, 0x02, &off_medullaB_tx},
	{MEDULLA_A_POS	 ,	VENDOR_ID, PRODUCT_CODE, 0x6126, 0x23, &off_medullaA_rx},
	{MEDULLA_A_POS	 ,	VENDOR_ID, PRODUCT_CODE, 0x6130, 0x03, &off_medullaA_tx},
	{MEDULLA_HIP_POS ,	VENDOR_ID, PRODUCT_CODE, 0x6126, 0x24, &off_medulla_hip_rx},
	{MEDULLA_HIP_POS ,	VENDOR_ID, PRODUCT_CODE, 0x6130, 0x04, &off_medulla_hip_tx},
	{}
};

static unsigned int counter = 0;

/*****************************************************************************/

// RX PDO entries
static ec_pdo_entry_info_t medulla_boom_rxpdo_entries = {0x6126, 0x21,  BITS_IN_A_BYTE * sizeof(uControllerInput)};
static ec_pdo_entry_info_t medullaB_rxpdo_entries = {0x6126, 0x22,  BITS_IN_A_BYTE * sizeof(uControllerInput)};
static ec_pdo_entry_info_t medullaA_rxpdo_entries = {0x6126, 0x23,  BITS_IN_A_BYTE * sizeof(uControllerInput)};
static ec_pdo_entry_info_t medulla_hip_rxpdo_entries = {0x6126, 0x24,  BITS_IN_A_BYTE * sizeof(uControllerInput)};

// TX PDO Entries
static ec_pdo_entry_info_t medulla_boom_txpdo_entries = {0x6130, 0x01, BITS_IN_A_BYTE * sizeof(uControllerOutput)};
static ec_pdo_entry_info_t medullaB_txpdo_entries = {0x6130, 0x02, BITS_IN_A_BYTE * sizeof(uControllerOutput)};
static ec_pdo_entry_info_t medullaA_txpdo_entries = {0x6130, 0x03, BITS_IN_A_BYTE * sizeof(uControllerOutput)};
static ec_pdo_entry_info_t medulla_hip_txpdo_entries = {0x6130, 0x04, BITS_IN_A_BYTE * sizeof(uControllerOutput)};

// RX PDO Info
static ec_pdo_info_t medulla_boom_rx_pdos = {0x1600, 1, &medulla_boom_rxpdo_entries};
static ec_pdo_info_t medullaB_rx_pdos = {0x1601, 1, &medullaB_rxpdo_entries};
static ec_pdo_info_t medullaA_rx_pdos = {0x1602, 1, &medullaA_rxpdo_entries};
static ec_pdo_info_t medulla_hip_rx_pdos = {0x1603, 1, &medulla_hip_rxpdo_entries};

// TX PDO Info
static ec_pdo_info_t medulla_boom_tx_pdos = {0x1A00, 1, &medulla_boom_txpdo_entries};
static ec_pdo_info_t medullaB_tx_pdos = {0x1A01, 1, &medullaB_txpdo_entries};
static ec_pdo_info_t medullaA_tx_pdos = {0x1A02, 1, &medullaA_txpdo_entries};
static ec_pdo_info_t medulla_hip_tx_pdos = {0x1A03, 1, &medulla_hip_txpdo_entries};

/*****************************************************************************/

static ec_sync_info_t medulla_boom_sync[] = 
{
	{0, EC_DIR_OUTPUT, 0, NULL,	EC_WD_DISABLE},
	{1, EC_DIR_INPUT , 0, NULL,	EC_WD_DISABLE},
	{2, EC_DIR_OUTPUT, 1, &medulla_boom_rx_pdos, EC_WD_ENABLE},
	{3, EC_DIR_INPUT , 1, &medulla_boom_tx_pdos, EC_WD_ENABLE},
	{0xff}
};

static ec_sync_info_t medullaB_sync[] = 
{
	{0, EC_DIR_OUTPUT, 0, NULL,	EC_WD_DISABLE},
	{1, EC_DIR_INPUT , 0, NULL,	EC_WD_DISABLE},
	{2, EC_DIR_OUTPUT, 1, &medullaB_rx_pdos, EC_WD_ENABLE},
	{3, EC_DIR_INPUT , 1, &medullaB_tx_pdos, EC_WD_ENABLE},
	{0xff}
};

static ec_sync_info_t medullaA_sync[] = 
{
	{0, EC_DIR_OUTPUT, 0, NULL,	EC_WD_DISABLE},
	{1, EC_DIR_INPUT , 0, NULL,	EC_WD_DISABLE},
	{2, EC_DIR_OUTPUT, 1, &medullaA_rx_pdos, EC_WD_ENABLE},
	{3, EC_DIR_INPUT , 1, &medullaA_tx_pdos, EC_WD_ENABLE},
	{0xff}
};

static ec_sync_info_t medulla_hip_sync[] = 
{
	{0, EC_DIR_OUTPUT, 0, NULL,	EC_WD_DISABLE},
	{1, EC_DIR_INPUT , 0, NULL,	EC_WD_DISABLE},
	{2, EC_DIR_OUTPUT, 1, &medulla_hip_rx_pdos, EC_WD_ENABLE},
	{3, EC_DIR_INPUT , 1, &medulla_hip_tx_pdos, EC_WD_ENABLE},
	{0xff}
};

/*****************************************************************************/

// Microcontroller I/O
uControllerInput * uc_in[NUM_OF_MEDULLAS_ON_ROBOT ];
uControllerOutput *	uc_out[NUM_OF_MEDULLAS_ON_ROBOT ];

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


// ============================================================================
// controller_wrapper_states
// ============================================================================





class Atrias_controller_wrapper
    : public RTT::TaskContext
{
 public:
    Atrias_controller_wrapper(string const& name)
        : TaskContext(name)
    {
        std::cout << "Atrias_controller_wrapper constructed !" <<std::endl;
    }

    bool configureHook() {
        int i, ret = -1;
        RTIME tick_period, requested_ticks, now;

        //****************************************************************************

        printk("\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        printk(KERN_INFO PFX "Starting...\n");

        //****************************************************************************

        // Initialize SHM
        if ( !initialize_shm() )
        {
            printk( KERN_ERR PFX "Failed to initialize shm.\n" );
            goto out_return;
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
        if ( master_state.slaves_responding != NUM_OF_MEDULLAS_ON_ROBOT )
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

        //****************************************************************************

        // Map I/O pointers to sync manager memory.
        uc_in[0] = ( uControllerInput * )( domain1_pd + off_medullaA_rx );
        uc_in[1] = ( uControllerInput * )( domain1_pd + off_medullaB_rx );
        uc_in[2] = ( uControllerInput * )( domain1_pd + off_medulla_hip_rx );
        uc_in[3] = ( uControllerInput * )( domain1_pd + off_medulla_boom_rx );

        uc_out[0] = ( uControllerOutput * )( domain1_pd + off_medullaA_tx );
        uc_out[1] = ( uControllerOutput * )( domain1_pd + off_medullaB_tx );
        uc_out[2] = ( uControllerOutput * )( domain1_pd + off_medulla_hip_tx );
        uc_out[3] = ( uControllerOutput * )( domain1_pd + off_medulla_boom_tx );

        //****************************************************************************

        // Start the RT task.
        printk(KERN_INFO PFX "Starting cyclic sample thread...\n");
        requested_ticks = nano2count(TIMERTICKS);
        tick_period = start_rt_timer(requested_ticks);
        printk(KERN_INFO PFX "RT timer started with %i/%i ticks.\n",
             (int) tick_period, (int) requested_ticks);

        if ( rt_task_init_cpuid(&task, run, 0, 2000, 0, 1, NULL, 1) ) 
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

    //out_stop_task:
    //  rt_task_delete(&task);
    //out_stop_timer:
    //  stop_rt_timer();
    //out_release_master:
    //  printk(KERN_ERR PFX "Releasing master...\n");
    //  ecrt_release_master(master);
    //out_return:
    //  rt_sem_delete(&master_sem);
    //  printk(KERN_ERR PFX "Failed to load. Aborting.\n");
    //  return ret;


        std::cout << "Atrias_controller_wrapper configured !" <<std::endl;
        return true;
    }

    bool startHook() {
        std::cout << "Atrias_controller_wrapper started !" <<std::endl;
        return true;
    }

    void updateHook() {
		// receive process data
		rt_sem_wait(&master_sem);
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
		rt_sem_signal(&master_sem);

		control_wrapper_state_machine( uc_in, uc_out );

		rt_sem_wait(&master_sem);
		ecrt_domain_queue(domain1);
		rt_sem_signal(&master_sem);
		ecrt_master_send(master);

        std::cout << "Atrias_controller_wrapper executes updateHook !" <<std::endl;
    }

    void stopHook() {
        std::cout << "Atrias_controller_wrapper executes stopping !" <<std::endl;
    }

    void cleanupHook() {
	    takedown_shm;
        ecrt_release_master(master);
        std::cout << "Atrias_controller_wrapper cleaning up !" <<std::endl;
    }
};

#endif
