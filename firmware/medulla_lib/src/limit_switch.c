#include "limit_switch.h"

limit_sw_port_t limit_sw_init_port(PORT_t *limit_sw_port, uint8_t pin_mask, void *counter_pntr, void (*callback)(void)) {
	limit_sw_port_t port;

	port.limit_sw_port = limit_sw_port;
	port.pin_mask = pin_mask;
	port.callback = callback;
	port.counter_pntr = counter_pntr;

	// make sure all the pins in the mask are inputs
	limit_sw_port->OUTSET &= ~pin_mask;

	// we now will enable the pullup resistor and input sense configuration on all the mask pins. To do this we will set the MPCMASK register so we don't have to manually configure each pullup.
	PORTCFG.MPCMASK = pin_mask;
	limit_sw_port->PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_RISING_gc;
	PORTCFG.MPCMASK = 0;

	((TC1_t*)(port.counter_pntr))->PER = 32000;
	// Pins are now configured, now we have to set the interrupt mask
	limit_sw_port->INT0MASK = pin_mask;

	return port;		
}

inline int limit_sw_enable_port(limit_sw_port_t *limit_sw_port) {
	PORTC.OUTSET = 0b10;
	if (limit_sw_port->limit_sw_port->INTCTRL & PORT_INT0LVL_gm)
		return -1; // the interrupt is already in use

	// Unfortunately we need to use a switch statements to put the pointers in the right places
	switch ((intptr_t)(limit_sw_port->limit_sw_port)) {
		case (intptr_t)(&PORTA): _limit_sw_PORTA = limit_sw_port; break; 
		case (intptr_t)(&PORTB): _limit_sw_PORTB = limit_sw_port; break; 
		case (intptr_t)(&PORTC): _limit_sw_PORTC = limit_sw_port; break; 
		case (intptr_t)(&PORTD): _limit_sw_PORTD = limit_sw_port; break; 
		case (intptr_t)(&PORTE): _limit_sw_PORTE = limit_sw_port; break; 
		case (intptr_t)(&PORTF): _limit_sw_PORTF = limit_sw_port; break; 
		case (intptr_t)(&PORTH): _limit_sw_PORTH = limit_sw_port; break; 
		case (intptr_t)(&PORTJ): _limit_sw_PORTJ = limit_sw_port; break; 
		case (intptr_t)(&PORTK): _limit_sw_PORTK = limit_sw_port; 
	}

	switch ((intptr_t)(limit_sw_port->counter_pntr)) {
		case (intptr_t)(&TCC0): _limit_sw_TCC0 = limit_sw_port; break; 
		case (intptr_t)(&TCC1): _limit_sw_TCC1 = limit_sw_port; break; 
		case (intptr_t)(&TCD0): _limit_sw_TCD0 = limit_sw_port; break; 
		case (intptr_t)(&TCD1): _limit_sw_TCD1 = limit_sw_port; break; 
		case (intptr_t)(&TCE0): _limit_sw_TCE0 = limit_sw_port; break; 
		case (intptr_t)(&TCE1): _limit_sw_TCE1 = limit_sw_port; break; 
		case (intptr_t)(&TCF0): _limit_sw_TCF0 = limit_sw_port; break; 
		case (intptr_t)(&TCF1): _limit_sw_TCF1 = limit_sw_port; break; 
	}

	// Turn on the interrupt for the port
	limit_sw_port->limit_sw_port->INTFLAGS = 1;
	limit_sw_port->limit_sw_port->INTCTRL = PORT_INT0LVL_HI_gc;

	// Turn on the interrupt for counter	
	((TC1_t*)(limit_sw_port->counter_pntr))->INTCTRLA = TC_OVFINTLVL_HI_gc;

	return 0;
}

inline void limit_sw_disable_port(limit_sw_port_t *limit_sw_port) {
	limit_sw_port->limit_sw_port->INTCTRL = PORT_INT0LVL_OFF_gc;
}

inline uint8_t limit_sw_get_port(limit_sw_port_t *limit_sw_port) {
	return ~(limit_sw_port->limit_sw_port->IN);
}
