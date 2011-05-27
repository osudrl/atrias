// Kevin Kemper

#include <stdio.h>
#include <avr/io.h>


#include "menial_io.h"


// Setting the panic pin to high will trigger the panic line connected to all
//  other medulla bords to go low, causing wide-spread pandemonium and terror.


#define PanicSet()	(PORT_PANIC.OUTSET = PANIC_bm)
#define PanicClr()	(PORT_PANIC.OUTCLR = PANIC_bm)

//#define PanicEn()	(PORT_PANIC.INT0MASK = PANIC_SENSE_bm)
//#define PanicDis()	(PORT_PANIC.INT0MASK = 0x00)

//#define PanicEn()	(PORT_PANIC.INTCTRL = ( PORT_PANIC.INTCTRL & ~PORT_INT0LVL_gm ) | PORT_INT0LVL_MED_gc)
#define PanicDis()	(PORT_PANIC.INTCTRL = ( PORT_PANIC.INTCTRL & ~PORT_INT0LVL_gm ) | PORT_INT0LVL_OFF_gc)

#define PANIC_LIMIT 100

//XXX
//ISR(PANIC_VECT) {
	// Disable PWM
//	printf("panic\n");

//	if ( global_flags.error_cnt == PANIC_LIMIT ) {
//		DisablePWM();
//		global_flags.status	|= STATUS_PANIC;
//		PanicSet();
///		PanicDis();
//	}
//	else
//		global_flags.error_cnt++;
//}


void panicEn () {

//	PanicClr();

	// clear any pending flags
	PORT_PANIC.INTFLAGS = 0x01;

	// ... triggered by the sense pin.
	PORT_PANIC.INT0MASK = PANIC_SENSE_bm;
	
	// Configure Interrupt0 to have medium interrupt level
	PORT_PANIC.INTCTRL = ( PORT_PANIC.INTCTRL & ~PORT_INT0LVL_gm ) | PORT_INT0LVL_HI_gc;
	
}


void initPanic() {
	
	// Enable pullups and trigger on level
//	PORTCFG.MPCMASK			= PANIC_SENSE_bm;
//	PORT_PANIC.PIN0CTRL		= PORT_OPC_TOTEM_gc;// | PORT_ISC_LEVEL_gc;
	
	// disable the interrupt
	PanicDis();
	
//	PORTCFG.MPCMASK			= PANIC_bm;
//	PORT_PANIC.PIN0CTRL		= PORT_OPC_WIREDAND_gc;
              
	PORT_PANIC.DIRCLR		= PANIC_SENSE_bm;

//	PanicSet();																	// make sure the panic line is set before changing the pin dir
	PORT_PANIC.DIRSET		= PANIC_bm;											// set the panic line as an output
	PanicSet();

}


