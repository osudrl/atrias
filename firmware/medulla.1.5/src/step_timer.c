// Kevin Kemper
//
// Basic timer running on TC1
////////////////////////////////////////////////////////////////////////////////
#include "step_timer.h"

// stops the step timer counter and zeros it
void stepTimer_Stop(void) {
	
	TCD1.CTRLA = ( TCD1.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_OFF_gc;
	TCD1.CNT = 0;
	
}

// starts the step timer at clk/4 -> 8 MHz
void stepTimer_Start(void) {
	// so we don't have to set the divider all the time:
	if (TCD1.CTRLA == 0)	
		TCD1.CTRLA = ( TCD1.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV4_gc;	
}

// inits the step timer, sets the timer ISR to a high-level interrupt
void initStepTimer(void) {

	// make sure the timer is stopped (probably not necessary but why not?)
	stepTimer_Stop();
	resetStepTimer();
	
	// Set the overflow interrupt as high level.
	TCD1.INTCTRLA = ( TCD1.INTCTRLA & ~TC1_OVFINTLVL_gm ) | TC_OVFINTLVL_HI_gc;
	

}

// Get the current value of the step timer
uint16_t stepTimerValue(void) {
	return TCD1.CNT;
}

// Clear the step timer and start counting again from zero
void resetStepTimer(void) {
	TCD1.CNT = 0;
}
