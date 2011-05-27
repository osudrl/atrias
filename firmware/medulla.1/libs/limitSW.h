// Kevin Kemper

#include <stdio.h>
#include <avr/io.h>

#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"
#include "menial_io.h"

#define LimitDis()	(PORT_LIMIT.INTCTRL = ( PORT_LIMIT.INTCTRL & ~PORT_INT0LVL_gm ) | PORT_INT0LVL_OFF_gc)

//XXX
ISR(LIMIT_VECT) {

//	DisablePWM();
//	PWMdis();

	global_flags.status	|= STATUS_LIMITSW;	
	global_flags.limits	= ~PORT_LIMIT.IN;
	
	LimitDis();
	//	printf("limit\n");
}

void limitEn() {

	global_flags.limits		= 0;

	// clear any pending flags
	PORT_LIMIT.INTFLAGS = 0x03;

	PORT_LIMIT.INT0MASK = LIMIT_ENABLE_bm;
	
	// Configure Interrupt0 to have medium interrupt level, triggered by any enabled limits.
	PORT_LIMIT.INTCTRL = ( PORT_LIMIT.INTCTRL & ~PORT_INT0LVL_gm ) | PORT_INT0LVL_HI_gc;
	
}

void initLimitSW() {

	// Enable pullups and trigger on a rising edge
	PORTCFG.MPCMASK = LIMIT_ENABLE_bm;
	PORT_LIMIT.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc | PORT_INVEN_bm;
	
//	PORTCFG.MPCMASK = ~LIMIT_ENABLE_bm;
//	PORT_LIMIT.PIN0CTRL	= PORT_OPC_PULLDOWN_gc;

	// Set all pins to input (not really necessary, the chip restarts in this mode)               
	PORT_LIMIT.DIR		= 0x00;


	//limitEn();
	
}


