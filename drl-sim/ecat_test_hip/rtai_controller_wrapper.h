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

/*****************************************************************************/

// Module parameters

#define FREQUENCY 										1000 // task frequency in Hz
#define INHIBIT_TIME 									20

#define TIMERTICKS 										(1000000000 / FREQUENCY)

// Optional features (comment to disable)
#define CONFIGURE_PDOS

#define PFX 											"ec_rtai_controller: "

#define MEDULLA_POS										0, 0

#define VENDOR_ID										0x00000777
#define PRODUCT_CODE						 			0x02628111

#define NUM_OF_MEDULLAS_ON_ROBOT 						1

#define BITS_IN_A_BYTE									8

/*****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

// Slave Configurations
static ec_slave_config_t *sc_medulla;

// RTAI
static RT_TASK task;
static SEM master_sem;
static cycles_t t_last_cycle = 0, t_critical;

/*****************************************************************************/

// process data
static uint8_t *domain1_pd; // process data memory

// Slave offsets
static unsigned int off_medulla_rx;

static unsigned int off_medulla_tx;

const static ec_pdo_entry_reg_t domain_regs[] =
{
	{MEDULLA_POS,	VENDOR_ID, PRODUCT_CODE, 0x6126, 0x21, &off_medulla_rx},
	{MEDULLA_POS,	VENDOR_ID, PRODUCT_CODE, 0x6130, 0x01, &off_medulla_tx},
	{}
};

static unsigned int counter = 0;

/*****************************************************************************/

// RX PDO entries
static ec_pdo_entry_info_t medulla_rxpdo_entries = {0x6126, 0x21,  BITS_IN_A_BYTE * sizeof(uControllerInput)};

// TX PDO Entries
static ec_pdo_entry_info_t medulla_txpdo_entries = {0x6130, 0x01, BITS_IN_A_BYTE * sizeof(uControllerOutput)};

// RX PDO Info
static ec_pdo_info_t medulla_rx_pdos = {0x1600, 1, &medulla_rxpdo_entries};

// TX PDO Info
static ec_pdo_info_t medulla_tx_pdos = {0x1A00, 1, &medulla_txpdo_entries};

/*****************************************************************************/

static ec_sync_info_t medulla_sync[] = 
{
	{0, EC_DIR_OUTPUT, 0, NULL,	EC_WD_DISABLE},
	{1, EC_DIR_INPUT , 0, NULL,	EC_WD_DISABLE},
	{2, EC_DIR_OUTPUT, 1, &medulla_rx_pdos, EC_WD_ENABLE},
	{3, EC_DIR_INPUT , 1, &medulla_tx_pdos, EC_WD_ENABLE},
	{0xff}
};

/*****************************************************************************/

// Control wrapper variables.

uint8_t				command;

/*****************************************************************************/

void check_master_state(void);

void run(long data);

void request_lock_callback(void *cb_data);

void release_lock_callback(void *cb_data);

int init_mod(void);

void cleanup_mod(void);

#endif // FUNCS_H_RTAI_CONTROLLER_WRAPPER
