// Kevin Kemper
//
//
////////////////////////////////////////////////////////////////////////////////
#ifndef TIMER_H
#define TIMER_H

#include <stdio.h>
#include <avr/io.h>

#include "menial_io.h"
#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"



// Interrupt handeler for the timer overflow.
ISR(TC_STEP_OVF_vect) {

	PWMdis();
	
	global_flags.status	|= STATUS_TCOVF;

}

// stops the step timer counter and zeros it
void tc_Stop() {
	
	TC_STEP.CTRLA = ( TC_STEP.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_OFF_gc;
	TC_STEP.CNT = 0;
	
}

// starts the step timer at clk/4 -> 8 MHz
void tc_Start() {
	// so we don't have to set the divider all the time:
	if (TC_STEP.CTRLA == 0)	
		TC_STEP.CTRLA = ( TC_STEP.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV4_gc;	
}

// inits the step timer, sets the timer ISR to a high-level interrupt
void initTimer() {

	// make sure the timer is stopped (probably not necessary but why not?)
	tc_Stop();
	
	// Set the overflow interrupt as high level.
	TC_STEP.INTCTRLA = ( TC_STEP.INTCTRLA & ~TC1_OVFINTLVL_gm ) | TC_OVFINTLVL_HI_gc;

}

#endif
