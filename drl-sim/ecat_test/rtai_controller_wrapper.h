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

#define FREQUENCY 									1000 // task frequency in Hz
#define INHIBIT_TIME 									20

#define TIMERTICKS 									(1000000000 / FREQUENCY)

// Optional features (comment to disable)
#define CONFIGURE_PDOS

#define PFX 										"ec_rtai_controller_wrapper: "

#define MEDULLA_POS									0, 0

#define VENDOR_ID									0x00000777
#define PRODUCT_CODE						 			0x02628111

#define BITS_IN_A_BYTE 									8

/*****************************************************************************/

typedef struct {
	uint8_t ch1;
	uint16_t ch2;
} rxpdo_struct;

typedef struct {
	uint8_t ch1;
	uint16_t ch2;
	uint32_t ch3;
} txpdo_struct;

/*****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_medulla = NULL;
static ec_slave_config_state_t sc_medulla_state = {};

// RTAI
static RT_TASK task;
static SEM master_sem;
static cycles_t t_last_cycle = 0, t_critical;

/*****************************************************************************/

// process data
static uint8_t *domain1_pd; // process data memory

#define MEDULLA_POS	0, 0
#define MEDULLA_IDPC	0x00000777, 0x02628111

// offsets for PDO entries
static unsigned int off_medulla_rx;
static unsigned int off_medulla_tx;


const static ec_pdo_entry_reg_t domain1_regs[] = {
	{MEDULLA_POS,  MEDULLA_IDPC, 0x6126, 0x21,	&off_medulla_rx},
	{MEDULLA_POS,  MEDULLA_IDPC, 0x6130, 0x01,	&off_medulla_tx},
	{}
};

static unsigned int	counter	= 0;
static unsigned char	blink	= 0xF0;

txpdo_struct	txpdo;
rxpdo_struct	rxpdo;

/*****************************************************************************/


static ec_pdo_entry_info_t medulla_txpdo_entries[] = {
	{0x6130, 0x01,  8}, // channel 1
	{0x6130, 0x02, 16}, // channel 2
	{0x6130, 0x03, 32}  // channel 3
};

static ec_pdo_entry_info_t medulla_rxpdo_entries[] = {
	{0x6126, 0x21,  8}, // channel 1
	{0x6126, 0x22, 16}  // channel 2
};

static ec_pdo_info_t medulla_tx_pdos[] = {
	{0x1A00 , 1, medulla_txpdo_entries},	// Ch1
	{0x1A01 , 1, medulla_txpdo_entries+1},	// Ch2
	{0x1A02 , 1, medulla_txpdo_entries+2}	// Ch3
};

static ec_pdo_info_t medulla_rx_pdos[] = {
	{0x1600 , 1, medulla_rxpdo_entries},	// Ch1
	{0x1601 , 1, medulla_rxpdo_entries+1}	// Ch2
};

static ec_sync_info_t medulla_syncs[] = {
	{0, EC_DIR_OUTPUT,	0, NULL,		EC_WD_DISABLE},
	{1, EC_DIR_INPUT,	0, NULL,		EC_WD_DISABLE},
	{2, EC_DIR_OUTPUT,	2, medulla_rx_pdos,	EC_WD_ENABLE},
	{3, EC_DIR_INPUT,	3, medulla_tx_pdos,	EC_WD_ENABLE},
	{0xff}
};

/*****************************************************************************/

void check_master_state(void);

void run(long data);

void request_lock_callback(void *cb_data);

void release_lock_callback(void *cb_data);

int init_mod(void);

void cleanup_mod(void);

#endif //FUNCS_H_RTAI_CONTROLLER_WRAPPER

