#ifndef ESTOP_H
#define ESTOP_H

#include <avr/io.h>

PORT_t * eStop_port;
uint8_t eStop_panicPin;
uint8_t eStop_eStopPin;

// Initilizes the IO for the eStop system. After initilization the
// EStop line is asserted, and must be cleared.
void initEStop(PORT_t * port, uint8_t panicPin, uint8_t eStopPin);

// assertEStop asserts the eStop/Panic lines to tell the other medullas to stop
void assertEStop(void);

// clearEStop deasserts the eStop/Panic line
void clearEStop(void);

// checkPanic returns 1 if the panic line as been asserted by another medulla
// otherwise it returns 0.
int  checkEStop(void);

#endif