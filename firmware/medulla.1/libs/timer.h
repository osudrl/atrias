// Kevin Kemper

#include <stdio.h>
#include <avr/io.h>

#include "menial_io.h"
#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"


//#define TC_Stop()	(TC_STEP.CTRLA = ( TC_STEP.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_OFF_gc)
//#define TC_Start()	(TC_STEP.CTRLA = ( TC_STEP.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV64_gc)


// Interrupt handeler for the timer overflow.
ISR(TC_STEP_OVF_vect) {

	PWMdis();
	
	global_flags.status	|= STATUS_TCOVF;

//	printf("\n\t\t\t\t\ttc_ovf\n");
}


void tc_Stop() {
	
	TC_STEP.CTRLA = ( TC_STEP.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_OFF_gc;
	TC_STEP.CNT = 0;
}

void tc_Start() {
	TC_STEP.CTRLA = ( TC_STEP.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV4_gc;	
}

// Configures PWM output on compare a for single slope pwm, with hires, and clk source as sys clk
void initTimer() {

	// Set period/TOP value
//	TC_SetPeriod( &TC_STEP	, 0x1000 );

	tc_Stop();
	
	// Set the overflow interrupt as high level.
	TC_STEP.INTCTRLA = ( TC_STEP.INTCTRLA & ~TC1_OVFINTLVL_gm ) | TC_OVFINTLVL_HI_gc;
	
	
}


