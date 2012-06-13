// Kevin Kemper
// Modifications by Kit Morton
//
////////////////////////////////////////////////////////////////////////////////

#include "limitSW.h"

// disable the limit switch interrupt
void limitDis() {
	_limitSWPort->INTCTRL = ( _limitSWPort->INTCTRL & ~PORT_INT0LVL_gm ) | PORT_INT0LVL_OFF_gc;
}

// Enable the limit switch interupt
void limitEn() {
	// clear any pending flags
	_limitSWPort->INTFLAGS = 0x03;

	
	_limitSWPort->INT0MASK = 0xFF;
	
	// Configure Interrupt0 to have medium interrupt level, triggered by any enabled limits.
	_limitSWPort->INTCTRL = ( _limitSWPort->INTCTRL & ~PORT_INT0LVL_gm ) | PORT_INT0LVL_HI_gc;
	
}

// initilize the limit switch inputs
void initLimitSW(PORT_t* port) {
	_limitSWPort = port;
	// Enable pullups and trigger on a rising edge
	PORTCFG.MPCMASK		= 0xFF;
	port->PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc | PORT_INVEN_bm;

	// Set all pins to input (not really necessary, the chip restarts in this mode)               
	port->DIR		= 0x00;
	
}

uint8_t checkLimitSW(void) {
	return ~_limitSWPort->IN;
}
