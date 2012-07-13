#include "pwm.h"
#include "stdio.h"

pwm_output_t pwm_initilize_output(io_pin_t pwm_pin, pwm_clk_div_t clock_divider, uint16_t cnt_max) {
	pwm_output_t pwm;	// Create a pwm struct to return
	
	// Figure out which counter to use, and set up the registers
	if (pwm_pin.io_port == &PORTC) {
		pwm.tc0_register = &TCC0;
		pwm.tc1_register = &TCC1;
	}
	else if (pwm_pin.io_port == &PORTD) {
		pwm.tc0_register = &TCD0;
		pwm.tc1_register = &TCD1;
	}
	else if (pwm_pin.io_port == &PORTE) {
		pwm.tc0_register = &TCE0;
		pwm.tc1_register = &TCE1;
	}
	else if (pwm_pin.io_port == &PORTF) {
		pwm.tc0_register = &TCF0;
		pwm.tc1_register = &TCF1;
	}
	else {
		pwm.tc0_register = 0;
		pwm.tc1_register = 0;
		pwm.cc_register = 0; 
		return pwm; // This port doesn't have pwm on it
	}

	// Now that we have both counter registers for this port, then we have to figure out which counter to use
	if (pwm_pin.pin < 4) {// Use TC0 if we are on pins 0-3
		pwm.tc1_register = 0; // Clear the tc1 register value to indicate we are using TC0
		pwm.cc_register = (uint16_t*)((uint8_t*)pwm.tc0_register + 0x28 + 0x2*pwm_pin.pin); // Note: We need to cast the tcx_register to a uint8_t pointer so that we can add to the memory address normally. (If we don't C will add two bytes for every 1 integer we add.
		
		// Now configure the counter
		pwm.tc0_register->PER = cnt_max;
		pwm.tc0_register->CTRLB = (pwm.tc0_register->CTRLB & ~TC0_WGMODE_gm) | TC_WGMODE_SS_gc;
		pwm.tc0_register->CTRLA = (pwm.tc0_register->CTRLA & ~TC0_CLKSEL_gm) | clock_divider;
	}
	else if (pwm_pin.pin < 6) { // If the pin is either 4 or 5 then use pins TC1
		pwm.tc0_register = 0;
		pwm.cc_register = (uint16_t*)((uint8_t*)pwm.tc1_register + 0x28 + 0x02*(pwm_pin.pin-4));

		// Now configure the counter
		pwm.tc1_register->PER = cnt_max;
		pwm.tc1_register->CTRLB = (pwm.tc1_register->CTRLB & ~TC1_WGMODE_gm) | TC_WGMODE_SS_gc; 
		pwm.tc1_register->CTRLA = (pwm.tc1_register->CTRLA & ~TC1_CLKSEL_gm) | clock_divider;
	}
	else {
		pwm.tc0_register = 0;
		pwm.tc1_register = 0;
		pwm.cc_register = 0; 
		return pwm; // This pin cannot generate PWM
	}

	// If we have we have gotten this far without returning, then we are good to configure the pin as an output
	io_set_direction(pwm_pin,io_output);
	
	return pwm;
}

void pwm_enable_output(pwm_output_t *pwm_output) {
	if (pwm_output->tc0_register)
		// Now we have to do some magic pointer math to get a bit position from the cc register address
		pwm_output->tc0_register->CTRLB |= 1<<((((intptr_t)pwm_output->cc_register - ((intptr_t)pwm_output->tc0_register + 0x28))/0x2)+4);
	else if (pwm_output->tc1_register)
		// Now we have to do some magic pointer math to get a bit position from the cc register address
		pwm_output->tc1_register->CTRLB |= 1<<((((intptr_t)pwm_output->cc_register - (intptr_t)pwm_output->tc1_register - 0x28)/0x2)+4);
}

void pwm_disable_output(pwm_output_t *pwm_output) {
	if (pwm_output->tc0_register)
		// Now we have to do some magic pointer math to get a bit position from the cc register address
		pwm_output->tc0_register->CTRLB &= ~(1<<((((intptr_t)pwm_output->cc_register - (intptr_t)pwm_output->tc0_register - 0x28)/0x2)+4));
	else if (pwm_output->tc1_register)
		// Now we have to do some magic pointer math to get a bit position from the cc register address
		pwm_output->tc1_register->CTRLB |= ~(1<<((((intptr_t)pwm_output->cc_register - (intptr_t)pwm_output->tc1_register - 0x28)/0x2)+4));

}

void pwm_set_output(pwm_output_t *pwm_output, uint16_t value) {
	if (pwm_output->cc_register) // Make sure that the cc pointer actually points to something before we use it
		*(pwm_output->cc_register) = value;
}

