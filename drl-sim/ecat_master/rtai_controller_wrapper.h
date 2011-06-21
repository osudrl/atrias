// Devin Koepl

#ifndef FUNCS_H_RTAI_CONTROLLER_WRAPPER
#define FUNCS_H_RTAI_CONTROLLER_WRAPPER

// Linux
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>

// RTAI
#include <rtai.h>
#include <rtai_sched.h>
#include <rtai_shm.h>
#include <rtai_nam2num.h>
#include <rtai_sched.h>
#include <rtai_sem.h>
#include <rtai_math.h>

// EtherCAT
#include <ecrt.h>

// ATRIAS robot
#include <atrias/ucontroller.h>

// ATRIAS controllers
#include <atrias_controllers/controller.h>
#include <atrias_controllers/control_switcher_state_machine.h>
#include <atrias_controllers/uspace_kern_shm.h>
#include <atrias_controllers/simulation_ecat_interface.h>

// DRL Library
#include <drl_library/discretize.h>
#include <drl_library/drl_math.h>

/*****************************************************************************/

// Module parameters

#define FREQUENCY 									1000 // task frequency in Hz
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

static DataToKern	*to_kern_shm;
static DataToUspace *to_uspace_shm;

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

// Controller structs.

ControllerInput	 	controller_input;
ControllerOutput 	controller_output;
ControllerState	 	controller_state;
ControllerData	 	controller_data;

// Control wrapper variables.

uint8_t				command;

uint16_t			tranA_off;
uint16_t			tranB_off;

int32_t				boom_pan_off = 0;
int32_t				boom_tilt_off = 0;

uint16_t			last_boom_pan_cnt;
uint16_t			first_boom_pan_cnt;
uint16_t			last_boom_tilt_cnt;

float				boom_pan_angle = 0.;
float				boom_tilt_angle = 0.;

float				last_boom_pan_angle = 0.;

uint16_t			last_tranA_cnt;
uint16_t			last_tranB_cnt;

float				last_motor_angleA = 0.;
float				last_motor_angleB = 0.;
float				last_leg_angleA		= 0.;
float				last_leg_angleB		= 0.;

float				hor_vel;
float				hor_vel_buffer[HOR_VEL_WINDOW];
int					hor_vel_index = 0;

// Keep track of the number of consecutive cycles that the program has been locked out of shm.
int					spin_lock_cnt 	= 0;
int					to_uspace_index = 0;
unsigned int		to_uspace_cnt 	= 0;

/*****************************************************************************/

void check_master_state(void);

void run(long data);

void request_lock_callback(void *cb_data);

void release_lock_callback(void *cb_data);

int init_mod(void);

void cleanup_mod(void);

#endif // FUNCS_H_RTAI_CONTROLLER_WRAPPER
