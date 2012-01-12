// Kevin Kemper
//
// Basic timer running on TC1
////////////////////////////////////////////////////////////////////////////////
#include "timer.h"

// stops the step timer counter and zeros it
void timer_Stop(TC1_t * timer) {
	
	timer->CTRLA = ( timer->CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_OFF_gc;
	timer->CNT = 0;
	
}

// starts the step timer
void timer_Start(TC1_t * timer, uint8_t prescaler) {
	// so we don't have to set the divider all the time:
	resetTimer(timer);
	timer->PER = 0xFFFF;
	if (timer->CTRLA == 0)	
		timer->CTRLA = ( timer->CTRLA & ~TC1_CLKSEL_gm ) | prescaler;	
	// Set the overflow interrupt as high level.
}

// inits the step timer, sets the timer ISR to a high-level interrupt
void initTimer(TC1_t * timer) {

	// make sure the timer is stopped (probably not necessary but why not?)
	timer_Stop(timer);
	resetTimer(timer);
}

// Get the current value of the step timer
uint16_t timerValue(TC1_t * timer) {
	return timer->CNT;
}

// Clear the step timer and start counting again from zero
void resetTimer(TC1_t * timer) {
	timer->CNT = 0;
}
