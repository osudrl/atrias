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

#include <atrias/ucontroller.h>

#include <atrias_controllers/controller.h>

#include <drl_library/discretize.h>

#include "medulla_safety_tester.h"

/*****************************************************************************/

// Module parameters

#define FREQUENCY 2000 // task frequency in Hz
#define INHIBIT_TIME 20

#define TIMERTICKS (1000000000 / FREQUENCY)

// Optional features (comment to disable)
#define CONFIGURE_PDOS

#define PFX "ec_rtai_sample: "

#define MEDULLA_POS													0, 0
#define VENDOR_ID														0x00000777
#define PRODUCT_CODE						 						0x02628111

#define NUM_OF_MEDULLAS_ON_ROBOT 						4 
#define NUM_OF_SLAVES_IN_SIMULATION_MACHINE 1

#define BITS_IN_A_BYTE 						8

#define MAX_13BIT									8192
#define MAX_16BIT									65536

#define ROLLOVER13BIT_THRESHOLD		4096
#define ROLLOVER16BIT_THRESHOLD		32768

/*****************************************************************************/

static UspaceKernShm *uspace_shm;

/*****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_medulla[4];
static ec_slave_config_state_t sc_medulla_state[4] = {};

// RTAI
static RT_TASK task;
static SEM master_sem;
static cycles_t t_last_cycle = 0, t_critical;

/*****************************************************************************/

// process data
static uint8_t *domain1_pd; // process data memory

// Slave offsets
static unsigned int off_medulla_rx[4];
static unsigned int off_medulla_tx[4];

static unsigned int medulla_A_index;
static unsigned int medulla_B_index;
static unsigned int medulla_hip_index;
static unsigned int medulla_boom_index;

const static ec_pdo_entry_reg_t domain1_regs[] =
{
	{MEDULLA_POS		,  0x00000777, 0x02628111, 0x6126, 0x21, &off_medulla_rx[0]},
	{MEDULLA_POS		,	 0x00000777, 0x02628111, 0x6130, 0x01, &off_medulla_tx[0]},
	{}
};

const static ec_pdo_entry_reg_t domain2_regs[] =
{
	{MEDULLA_POS		,  0x00000777, 0x02628111, 0x6126, 0x21, &off_medulla_rx[0]},
	{MEDULLA_POS		,	 0x00000777, 0x02628111, 0x6130, 0x01, &off_medulla_tx[0]},
	{MEDULLA_POS + 1,  0x00000777, 0x02628111, 0x6126, 0x22, &off_medulla_rx[1]},
	{MEDULLA_POS + 1,  0x00000777, 0x02628111, 0x6130, 0x02, &off_medulla_tx[1]},
	{}
};

const static ec_pdo_entry_reg_t domain3_regs[] =
{
	{MEDULLA_POS		,  0x00000777, 0x02628111, 0x6126, 0x21, &off_medulla_rx[0]},
	{MEDULLA_POS		,	 0x00000777, 0x02628111, 0x6130, 0x01, &off_medulla_tx[0]},
	{MEDULLA_POS + 1,  0x00000777, 0x02628111, 0x6126, 0x22, &off_medulla_rx[1]},
	{MEDULLA_POS + 1,  0x00000777, 0x02628111, 0x6130, 0x02, &off_medulla_tx[1]},
	{MEDULLA_POS + 2,  0x00000777, 0x02628111, 0x6126, 0x23, &off_medulla_rx[2]},
	{MEDULLA_POS + 2,  0x00000777, 0x02628111, 0x6130, 0x03, &off_medulla_tx[2]},
	{}
};

const static ec_pdo_entry_reg_t domain4_regs[] =
{
	{MEDULLA_POS		,  0x00000777, 0x02628111, 0x6126, 0x21, &off_medulla_rx[0]},
	{MEDULLA_POS		,	 0x00000777, 0x02628111, 0x6130, 0x01, &off_medulla_tx[0]},
	{MEDULLA_POS + 1,  0x00000777, 0x02628111, 0x6126, 0x22, &off_medulla_rx[1]},
	{MEDULLA_POS + 1,  0x00000777, 0x02628111, 0x6130, 0x02, &off_medulla_tx[1]},
	{MEDULLA_POS + 2,  0x00000777, 0x02628111, 0x6126, 0x23, &off_medulla_rx[2]},
	{MEDULLA_POS + 2,  0x00000777, 0x02628111, 0x6130, 0x03, &off_medulla_tx[2]},
	{MEDULLA_POS + 3,  0x00000777, 0x02628111, 0x6126, 0x24, &off_medulla_rx[3]},
	{MEDULLA_POS + 3,  0x00000777, 0x02628111, 0x6130, 0x04, &off_medulla_tx[3]},
	{}
};

static unsigned int counter = 0;
static unsigned int blink = 0;

static unsigned int num_of_medullas;

/*****************************************************************************/

static ec_pdo_entry_info_t medulla_rxpdo_entries[NUM_OF_MEDULLAS_ON_ROBOT];
static ec_pdo_entry_info_t medulla_txpdo_entries[NUM_OF_MEDULLAS_ON_ROBOT];

static ec_pdo_info_t medulla_rx_pdos[NUM_OF_MEDULLAS_ON_ROBOT];
static ec_pdo_info_t medulla_tx_pdos[NUM_OF_MEDULLAS_ON_ROBOT];

/*****************************************************************************/

void check_master_state(void);

void run(long data);

void request_lock_callback(void *cb_data);

void release_lock_callback(void *cb_data);

int init_mod(void);

void cleanup_mod(void);

#endif // FUNCS_H_RTAI_CONTROLLER_WRAPPER
