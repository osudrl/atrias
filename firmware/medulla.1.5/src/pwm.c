#include "pwm.h"

void initPWM(PORT_t * port, TC1_t * timer, HIRES_t * hires, uint8_t pwm_pin, uint8_t dir_pin) {
	port->OUTCLR	= (1 << pwm_pin) | (1 << dir_pin);
	port->DIRSET	= 1 << pwm_pin | (1 << dir_pin);


	timer->CTRLA	= ( timer->CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;

	hires->CTRL	= HIRES_HREN_TC1_gc;
	
	timer->PER = PERIOD;
	
	timer->CTRLB = ( timer->CTRLB & ~TC1_WGMODE_gm ) | TC_WGMODE_SS_gc;
	
	// Enable channel A
	timer->CTRLB |= TC1_CCAEN_bm;
	
	timer->INTCTRLA = ( timer->INTCTRLA & ~TC1_OVFINTLVL_gm ) | TC_OVFINTLVL_LO_gc;

	
	setPWM(0,0,port,timer,dir_pin);
}


void setPWM(uint16_t duty, uint8_t dir, PORT_t * port, TC1_t * timer, uint8_t dir_pin) {
	timer->CCABUF = duty;
	
	if (dir == 0)
		port->OUTCLR = 1<< dir_pin;
	else
		port->OUTSET = 1<< dir_pin;
	
}

void disablePWM(TC1_t * timer) {
	timer->CTRLB	&= ~TC0_CCAEN_bm;
}