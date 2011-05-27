// Kevin Kemper
#define F_CPU 32000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"
#include "menial_io.h"

#define PERIOD		20000
#define PWM_ZERO	(PERIOD/2)

//#define PWMdis()	(TC_PWM.CTRLB	&= ~TC0_CCAEN_bm)
#define PWMen()		(TC_PWM.CTRLB 	|= TC0_CCAEN_bm)

// pass a duty value of 0-5000 to these functions to control the duty cycle of each phase
//#define SetPWM(_compareValue ) ( TC_PWM.CCABUF = (_compareValue) )

//XXX
void setPWM(uint16_t val) {
	
	if (val > PERIOD) {
		val = 0;
		global_flags.status	|= STATUS_BADPWM;
	}


	TC_PWM.CCABUF = val;

}

void setDirection(uint8_t dir) {
	
	if (dir == 0)
		PORT_AMP_CTL.OUTCLR = DIRECTION_bm;
	else
		PORT_AMP_CTL.OUTSET = DIRECTION_bm;

}



// Configures PWM output on compare a for single slope pwm, with hires, and clk source as sys clk
void initPWM(uint16_t duty) {

	PORT_AMP_CTL.OUTCLR	= PWM_bm;
	PORT_AMP_CTL.DIRSET	= PWM_bm;												// PWM -> output


	TC_PWM.CTRLA	= ( TC_PWM.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;

	HIRES_PWM.CTRL	= HIRES_HREN_TC0_gc;
	
	TC_PWM.PER = PERIOD;
	
	TC_PWM.CTRLB = ( TC_PWM.CTRLB & ~TC0_WGMODE_gm ) | TC_WGMODE_SS_gc;
	
	// Enable channel A
	TC_PWM.CTRLB |= TC0_CCAEN_bm;
	
	TC_PWM.INTCTRLA = ( TC_PWM.INTCTRLA & ~TC0_OVFINTLVL_gm ) | TC_OVFINTLVL_LO_gc;
	
	//~ TC0_SetCCAIntLevel (tc, TC_CCAINTLVL_LO_gc);
//	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	
	setPWM(duty);
}

