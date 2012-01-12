#ifndef MEDULLA_CONTROLLER_H
#define MEDULLA_CONTROLLER_H

#define F_CPU 32000000UL

#include <avr/interrupt.h>
#include <stdio.h>

#include "ecat.h"
#include "timer.h"
#include "estop.h"
#include "uart.h"
#include "clock.h"

#define STEP_TIMER		&TCE1
#define	WATCHDOG_TIMER	&TCD1

typedef enum {IDLE, INIT, RUN, STOP, ERROR_DAMPING, ERROR} MedullaState;
uint8_t eStop;
/**
 * @ breif medulla_run is the state machine loop for the medulla
 */
void medulla_run(void* in, void* out);
void init_eCAT(void);
void eCATWriteData(void* out);
void eCATReadData(void* in);
	
/*
 * The rest of the functions below need to be implemented in main program. They may 
 * not be blocking functions. 
 */
void init(void);
void updateInput(void);
void setTimeStep(uint16_t time);
void setState(MedullaState state);
MedullaState getState(void);
void timerOverflow(void); // This function gets called if something freezes
 
/* The following functions are state macheince functionsEach function returns a
 * suggested next state. This allows the program to switch states if it wants to.
 * However medulla_controller can still enter an error state if needed.
 */
MedullaState state_idle(void);
MedullaState state_init(void);
MedullaState state_run(void);
void state_stop(void);
MedullaState state_error_damping(void);
MedullaState state_error(void);

#endif