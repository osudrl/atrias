// Devin Koepl

#ifndef FUNCS_H_RTAI_CONTROLLER_WRAPPER
#define FUNCS_H_RTAI_CONTROLLER_WRAPPER

#include <errno.h>
//#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
//#include <sys/time.h>
//#include <sys/types.h>
#include <unistd.h>

// EtherCAT
#include <ecrt.h>

// ATRIAS robot
#include <atrias/ucontroller.h>

// ATRIAS controllers
//#include <atrias_controllers/controller.h>
//#include <atrias_controllers/control_switcher_state_machine.h>
//#include <atrias_controllers/uspace_kern_shm.h>
//#include <atrias_controllers/simulation_ecat_interface.h>

// DRL Library
//#include <drl_library/discretize.h>
//#include <drl_library/drl_math.h>

// Local
//#include "controller_wrapper_states.h"

/*****************************************************************************/

// Module parameters

#define FREQUENCY 										1000 // task frequency in Hz
//#define INHIBIT_TIME 									20

//#define TIMERTICKS 										(1000000000 / FREQUENCY)

// Optional features (comment to disable)
#define CONFIGURE_PDOS                                  1

#define PFX 											"ec_rtai_controller: "

#define MEDULLA_BOOM_POS								0, 0
#define MEDULLA_B_POS									0, 1
#define MEDULLA_A_POS									0, 0
#define MEDULLA_HIP_POS									0, 0

#define VENDOR_ID										0x00000777
#define PRODUCT_CODE						 			0x02628111

#define NUM_OF_MEDULLAS_ON_ROBOT 						2
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

// Timer
static unsigned int sig_alarms = 0;
static unsigned int user_alarms = 0;

// RTAI
//static RT_TASK task;
//static SEM master_sem;
//static cycles_t t_last_cycle = 0, t_critical;

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

const static ec_pdo_entry_reg_t domain1_regs[] =
{
	{MEDULLA_BOOM_POS,	VENDOR_ID, PRODUCT_CODE, 0x6126, 0x21, &off_medulla_boom_rx},
	{MEDULLA_BOOM_POS,	VENDOR_ID, PRODUCT_CODE, 0x6130, 0x01, &off_medulla_boom_tx},
	{MEDULLA_B_POS	 ,	VENDOR_ID, PRODUCT_CODE, 0x6126, 0x34, &off_medullaB_rx},
	{MEDULLA_B_POS	 ,	VENDOR_ID, PRODUCT_CODE, 0x6130, 0x02, &off_medullaB_tx},
	{MEDULLA_A_POS	 ,	VENDOR_ID, PRODUCT_CODE, 0x6126, 0x33, &off_medullaA_rx},
	{MEDULLA_A_POS	 ,	VENDOR_ID, PRODUCT_CODE, 0x6130, 0x01, &off_medullaA_tx},
	{MEDULLA_HIP_POS ,	VENDOR_ID, PRODUCT_CODE, 0x6126, 0x33, &off_medulla_hip_rx},
	{MEDULLA_HIP_POS ,	VENDOR_ID, PRODUCT_CODE, 0x6130, 0x01, &off_medulla_hip_tx},
	{}
};

static unsigned int counter = 0;

/*****************************************************************************/

#if CONFIGURE_PDOS

// RX PDO entries
static ec_pdo_entry_info_t medulla_boom_rxpdo_entries = {0x6126, 0x21,  BITS_IN_A_BYTE * sizeof(uControllerInput)};
static ec_pdo_entry_info_t medullaB_rxpdo_entries = {0x6126, 0x34,  BITS_IN_A_BYTE * sizeof(uControllerInput)};
static ec_pdo_entry_info_t medullaA_rxpdo_entries = {0x6126, 0x33,  BITS_IN_A_BYTE * sizeof(uControllerInput)};
static ec_pdo_entry_info_t medulla_hip_rxpdo_entries = {0x6126, 0x33,  BITS_IN_A_BYTE * sizeof(uControllerInput)};

// TX PDO Entries
static ec_pdo_entry_info_t medulla_boom_txpdo_entries = {0x6130, 0x01, BITS_IN_A_BYTE * sizeof(uControllerOutput)};
static ec_pdo_entry_info_t medullaB_txpdo_entries = {0x6130, 0x02, BITS_IN_A_BYTE * sizeof(uControllerOutput)};
static ec_pdo_entry_info_t medullaA_txpdo_entries = {0x6130, 0x01, BITS_IN_A_BYTE * sizeof(uControllerOutput)};
static ec_pdo_entry_info_t medulla_hip_txpdo_entries = {0x6130, 0x01, BITS_IN_A_BYTE * sizeof(uControllerOutput)};

// RX PDO Info
static ec_pdo_info_t medulla_boom_rx_pdos = {0x1600, 1, &medulla_boom_rxpdo_entries};
static ec_pdo_info_t medullaB_rx_pdos = {0x1601, 1, &medullaB_rxpdo_entries};
static ec_pdo_info_t medullaA_rx_pdos = {0x1600, 1, &medullaA_rxpdo_entries};
static ec_pdo_info_t medulla_hip_rx_pdos = {0x1600, 1, &medulla_hip_rxpdo_entries};

// TX PDO Info
static ec_pdo_info_t medulla_boom_tx_pdos = {0x1A00, 1, &medulla_boom_txpdo_entries};
static ec_pdo_info_t medullaB_tx_pdos = {0x1A01, 1, &medullaB_txpdo_entries};
static ec_pdo_info_t medullaA_tx_pdos = {0x1A00, 1, &medullaA_txpdo_entries};
static ec_pdo_info_t medulla_hip_tx_pdos = {0x1A00, 1, &medulla_hip_txpdo_entries};

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

#endif // CONFIGURE_PDOS

/*****************************************************************************/

// Microcontroller I/O
uControllerInput * uc_in[NUM_OF_MEDULLAS_ON_ROBOT];
uControllerOutput *	uc_out[NUM_OF_MEDULLAS_ON_ROBOT];

/*****************************************************************************/

void check_master_state(void);
void cyclic_task(void);
//void request_lock_callback(void *cb_data);
//void release_lock_callback(void *cb_data);
//int  init_mod(void);
//void cleanup_mod(void);

#endif // FUNCS_H_RTAI_CONTROLLER_WRAPPER
