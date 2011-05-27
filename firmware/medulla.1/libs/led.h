// Kevin Kemper

#ifndef LED_H
#define LED_H

#include <stdio.h>
#include <avr/io.h>

#include "menial_io.h"


#define TC_CLKSEL_BLINK			TC_CLKSEL_DIV64_gc
#define TC_CLKSEL_SOLID			TC_CLKSEL_DIV8_gc

//#define TC_SetPeriod( _tc, _period ) ( (_tc)->PER = (_period) )

//#define BlinkLED() ( TC_LED.PER = 0x7FFF )
//#define SolidLED() ( TC_LED.PER = 0xFFFF )
//#define BlinkLED()					(TC_LED.CTRLA = ( TC_LED.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_BLINK)
//#define SolidLED()					(TC_LED.CTRLA = ( TC_LED.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_SOLID)

#define SetDutyR( _compareValue )	( TC_LED.CCABUF = (_compareValue) )
#define SetDutyG( _compareValue )	( TC_LED.CCBBUF = (_compareValue) )
#define SetDutyB( _compareValue )	( TC_LED.CCCBUF = (_compareValue) )

// Interrupt handeler for the timer overflow.
//ISR(TC_LED_OVF_vect) {
	// Turn on each led
//	printf("leds on\n");
//	PORT_LED.OUTCLR = LED_R_bm | LED_G_bm | LED_B_bm;
//}

// Interrupt handeler for red compare match
ISR(TC_LED_R_vect) {
	PORT_LED.OUTSET	= LED_R_bm;
//	printf("r\n");
}

// Interrupt handeler for green
ISR(TC_LED_G_vect) {
	PORT_LED.OUTSET	= LED_G_bm;
//	printf("g\n");
}

// Interrupt handeler for blue
ISR(TC_LED_B_vect) {
	PORT_LED.OUTSET	= LED_B_bm;
//	printf("b\n");
}

ISR(TCC0_CCD_vect) {
	PORT_LED.OUTCLR = LED_R_bm | LED_G_bm | LED_B_bm;
//	TC_LED.CNT = 0;
//	printf("b\n");
}


//#define BlinkLED()					(TC_LED.CTRLA = ( TC_LED.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_BLINK)
//#define SolidLED()					(TC_LED.CTRLA = ( TC_LED.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_SOLID)

void blinkLED() {
	TC_LED.CTRLA = ( TC_LED.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_BLINK;
//	TC_LED.CTRLFSET = TC_CMD_RESTART_gc;
}

void solidLED() {
	TC_LED.CTRLA = ( TC_LED.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_SOLID;
//	TC_LED.CTRLFSET = TC_CMD_RESTART_gc;
}


// Configures PWM output on compare a for single slope pwm, with hires, and clk source as sys clk
void initLED() {
	
	// stop the counter
	TC_LED.CTRLA = ( TC_LED.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_OFF_gc;

	// init LED pins
	PORT_LED.OUTSET = LED_R_bm | LED_G_bm | LED_B_bm;
	PORT_LED.DIRSET	= LED_R_bm | LED_G_bm | LED_B_bm;							// set LED pins to outputs

	// Enable the compare channels.
	TC_LED.CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC_WGMODE_NORMAL_gc;	// TODO: make this easier to config...
	
	TC_LED.CTRLD = 0x00;
	
	// Set the overflow interrupt as high level.
	TC_LED.INTCTRLA = TC_OVFINTLVL_OFF_gc;
	
	TC_LED.INTCTRLB = TC_CCAINTLVL_MED_gc | TC_CCBINTLVL_MED_gc | TC_CCCINTLVL_MED_gc | TC_CCDINTLVL_MED_gc;
	
	// Enable high level interrupts
//	PMIC.CTRL |= PMIC_LOLVLEN_bm;
//	TC_LED.CCDBUF = 0x0000;
	
	SetDutyR(0x0FFF);
	SetDutyG(0x0FFF);
	SetDutyB(0x0FFF);
	
//	blinkLED();
	solidLED();
}

#endif

