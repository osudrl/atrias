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

/*****************************************************************************/

// Module parameters

#define FREQUENCY 										1 // task frequency in Hz
#define INHIBIT_TIME 									20

#define TIMERTICKS 										(1000000000 / FREQUENCY)

#define PFX 											"ec_rtai_controller_wrapper: "

#define MEGS    4
#define SHMNAM  "MYSHM"
#define SHM_ENTRIES 1000

/*****************************************************************************/

typedef struct
{
	unsigned char fresh;

	int data;
} DataEntry;

DataEntry *shm;

/*****************************************************************************/

// RTAI
static RT_TASK task;
static SEM master_sem;
static cycles_t t_last_cycle = 0, t_critical;

/*****************************************************************************/

void run(long data);

int init_mod(void);

void cleanup_mod(void);

#endif //FUNCS_H_RTAI_CONTROLLER_WRAPPER

