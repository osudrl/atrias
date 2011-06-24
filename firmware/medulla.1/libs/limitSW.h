// Kevin Kemper
//
//
////////////////////////////////////////////////////////////////////////////////
#ifndef LIMITSW_H
#define LIMITSW_H

#include <stdio.h>
#include <avr/io.h>

#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"
#include "menial_io.h"


//#define LimitDis()	(PORT_LIMIT.INTCTRL = ( PORT_LIMIT.INTCTRL & ~PORT_INT0LVL_gm ) | PORT_INT0LVL_OFF_gc)

// disable the limit switch interrupt
void limitDis() {
	PORT_LIMIT.INTCTRL = ( PORT_LIMIT.INTCTRL & ~PORT_INT0LVL_gm ) | PORT_INT0LVL_OFF_gc;
}

// Enable the limit switch interupt
void limitEn() {

	// clear the global var containg the limit state
	global_flags.limits = 0;

	// clear any pending flags
	PORT_LIMIT.INTFLAGS = 0x03;

	
	PORT_LIMIT.INT0MASK = LIMIT_ENABLE_bm;
	
	// Configure Interrupt0 to have medium interrupt level, triggered by any enabled limits.
	PORT_LIMIT.INTCTRL = ( PORT_LIMIT.INTCTRL & ~PORT_INT0LVL_gm ) | PORT_INT0LVL_HI_gc;
	
}

// initilize the limit switch inputs
void initLimitSW() {

	// Enable pullups and trigger on a rising edge
	PORTCFG.MPCMASK		= LIMIT_ENABLE_bm;
	PORT_LIMIT.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc | PORT_INVEN_bm;

	// Set all pins to input (not really necessary, the chip restarts in this mode)               
	PORT_LIMIT.DIR		= 0x00;
	
}


// Interrupt handeler for the pin change
ISR(LIMIT_VECT) {

	PWMdis();


	global_flags.status	|= STATUS_LIMITSW;
	
	// Store which switch was trggered.  The value is inverted because the limits
	// are active high.
	global_flags.limits	= ~PORT_LIMIT.IN;
	
	// Disable the interrupt so that we don't hang if the switch is held closed.
	// Once is enough.
	limitDis();

}


#endif // !LIMITSW_H
