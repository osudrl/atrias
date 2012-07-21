#include "quadrature_encoder.h"

quadrature_encoder_t quadrature_encoder_init(io_pin_t quadrature_base_pin,io_pin_t index_pin, bool uses_index, void *counter_reg, uint16_t lines_per_rev) {
	quadrature_encoder_t encoder;
	
	// setup the encoder struct first
	encoder.quadrature_base_pin = quadrature_base_pin;
	encoder.index_pin = index_pin;
	encoder.uses_index = uses_index;
	encoder.counter_reg = (TC0_t*)counter_reg;
	encoder.lines_per_rev = lines_per_rev;

	// Set up the io pins for the quadrature decoding
	quadrature_base_pin.io_port->OUTCLR = 0b11 << quadrature_base_pin.pin;	

	PORTCFG.MPCMASK = 0b11 << quadrature_base_pin.pin;
	*(quadrature_base_pin.control_reg) = (*(quadrature_base_pin.control_reg) & ~PORT_ISC_gm) | PORT_ISC_LEVEL_gc;

	// Setup the event system for quadrature decoding
	switch ((intptr_t)(quadrature_base_pin.io_port)) { // We need a switch statement to set the event mux for the correct port.
		case ((intptr_t)(&PORTA)): EVSYS.CH0MUX = 0b01010000; break;
		case ((intptr_t)(&PORTB)): EVSYS.CH0MUX = 0b01011000; break;
		case ((intptr_t)(&PORTC)): EVSYS.CH0MUX = 0b01100000; break;
		case ((intptr_t)(&PORTD)): EVSYS.CH0MUX = 0b01101000; break;
		case ((intptr_t)(&PORTE)): EVSYS.CH0MUX = 0b01110000; break;
		case ((intptr_t)(&PORTF)): EVSYS.CH0MUX = 0b01111000; break;
	}
	EVSYS.CH0MUX |= quadrature_base_pin.pin; // Now that we have the upper bits set, we can just or it with pin.
	
	EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;
	
	// If we are using the index, then we set that up now
	if (uses_index) {
		io_set_direction(index_pin,io_input);
		*(index_pin.control_reg) = (*(index_pin.control_reg) & ~PORT_ISC_gm) | PORT_ISC_BOTHEDGES_gc;

		switch ((intptr_t)(index_pin.io_port)) { // We need a switch statement to set the event mux for the correct port.
                	case ((intptr_t)(&PORTA)): EVSYS.CH1MUX = 0b01010000; break;
	                case ((intptr_t)(&PORTB)): EVSYS.CH1MUX = 0b01011000; break;
        	        case ((intptr_t)(&PORTC)): EVSYS.CH1MUX = 0b01100000; break;
                	case ((intptr_t)(&PORTD)): EVSYS.CH1MUX = 0b01101000; break;
	                case ((intptr_t)(&PORTE)): EVSYS.CH1MUX = 0b01110000; break;                         
        	        case ((intptr_t)(&PORTF)): EVSYS.CH1MUX = 0b01111000; break;
        	}

		EVSYS.CH1MUX |= index_pin.pin;
		EVSYS.CH1CTRL = EVSYS_DIGFILT_2SAMPLES_gc;
		EVSYS.CH0CTRL |= EVSYS_QDIEN_bm;
	}

	// Now setup the timer
	((TC0_t*)counter_reg)->CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;
	((TC0_t*)counter_reg)->PER = (lines_per_rev * 4) - 1;
	((TC0_t*)counter_reg)->CTRLA = TC_CLKSEL_DIV1_gc;

	return encoder; // Now that everything is setup, return the encoder struct

}

inline uint16_t quadrature_encoder_get_value(quadrature_encoder_t *encoder) {
	return encoder->counter_reg->CNT;
}

