#include "estop.h"

PORT_t * eStop_port;
uint8_t eStop_panicPin;
uint8_t eStop_eStopPin;

// Initilizes the IO for the eStop system. After initilization the
// EStop line is asserted, and must be cleared.
void initEStop(PORT_t * port, uint8_t panicPin, uint8_t eStopPin) {
	eStop_port = port;
	eStop_panicPin = panicPin;
	eStop_eStopPin = eStopPin;
	port->DIRSET = (1 << panicPin);	// Make the panic pin an output
}

// assertEStop asserts the eStop/Panic lines to tell the other medullas to stop
void assertEStop(void) {
	eStop_port->OUTCLR = (1 << eStop_panicPin);
}

// clearEStop deasserts the eStop/Panic line
void clearEStop(void) {
	eStop_port->OUTSET = (1 << eStop_panicPin);
}

// checkPanic returns 1 if the panic line as been asserted by another medulla
// otherwise it returns 0.
int  checkEStop(void) {
	//if ((eStop_port->IN & (1<<eStop_eStopPin))==0) 
	if ((PORTF.IN &0b10) == 0)
		return 0;
	else
		return 1;
}