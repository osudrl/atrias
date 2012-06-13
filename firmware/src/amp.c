#include "amp.h"

void initAmp(USART_t * amp_usart, PORT_t * usart_port, PORT_t * pwm_port, TC1_t * timer, HIRES_t * hires, uint8_t pwm_pin, uint8_t dir_pin) {
	// Init usart communication with amplifier
	initUART(usart_port,amp_usart,1152);
	
	// Init counter for PWM generation
	pwm_port->OUTCLR	= (1 << pwm_pin) | (1 << dir_pin);
	pwm_port->DIRSET	= 1 << pwm_pin | (1 << dir_pin);


	timer->CTRLA	= ( timer->CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;

	hires->CTRL	= HIRES_HREN_TC1_gc;
	
	timer->PER = PERIOD;
	
	timer->CTRLB = ( timer->CTRLB & ~TC1_WGMODE_gm ) | TC_WGMODE_SS_gc;
}


void setPWM(uint16_t duty, uint8_t dir, PORT_t * port, TC1_t * timer, uint8_t dir_pin) {
	timer->CCABUF = duty;
	
	if (dir == 0)
		port->OUTCLR = 1<< dir_pin;
	else
		port->OUTSET = 1<< dir_pin;
	
}

void enablePWM(TC1_t * timer) {
	// Enable PWM generation
	timer->CTRLB |= TC1_CCAEN_bm;
	
	timer->INTCTRLA = ( timer->INTCTRLA & ~TC1_OVFINTLVL_gm ) | TC_OVFINTLVL_LO_gc;
	
	timer->CCABUF = 0;
}

void disablePWM(TC1_t * timer) {
	timer->CTRLB	&= ~TC0_CCAEN_bm;
}

void enableAmp(USART_t * amp_usart) {	
	//0xA5 0x3F 0x02 0x07 0x00 0x01 0xB3 0xE7 0x0F 0x00 0x10 0x3E
	UARTWriteChar(amp_usart, 0xA5);
	UARTWriteChar(amp_usart, 0x3F);
	UARTWriteChar(amp_usart, 0x02);
	UARTWriteChar(amp_usart, 0x07);
	UARTWriteChar(amp_usart, 0x00);
	UARTWriteChar(amp_usart, 0x01);
	UARTWriteChar(amp_usart, 0xB3);
	UARTWriteChar(amp_usart, 0xE7);
	UARTWriteChar(amp_usart, 0x0F);
	UARTWriteChar(amp_usart, 0x00);
	UARTWriteChar(amp_usart, 0x10);
	UARTWriteChar(amp_usart, 0x3E);
	_delay_ms(10);
	//0xA5 0x3F 0x02 0x01 0x00 0x01 0x01 0x47 0x00 0x00 0x00 0x00
	UARTWriteChar(amp_usart, 0xA5);
	UARTWriteChar(amp_usart, 0x3F);
	UARTWriteChar(amp_usart, 0x02);
	UARTWriteChar(amp_usart, 0x01);
	UARTWriteChar(amp_usart, 0x00);
	UARTWriteChar(amp_usart, 0x01);
	UARTWriteChar(amp_usart, 0x01);
	UARTWriteChar(amp_usart, 0x47);
	UARTWriteChar(amp_usart, 0x00);
	UARTWriteChar(amp_usart, 0x00);
	UARTWriteChar(amp_usart, 0x00);
	UARTWriteChar(amp_usart, 0x00);
}