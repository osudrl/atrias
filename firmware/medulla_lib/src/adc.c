#include "adc.h"

adc_port_t adc_init_port(ADC_t *adc) {
	// setup the port struct
	adc_port_t port;
	port.adc_port = adc;
	port.pin_mask = 0;
	port.adc_output_pointers[0] = 0;
	port.adc_output_pointers[1] = 0;
	port.adc_output_pointers[2] = 0;
	port.adc_output_pointers[3] = 0;
	port.adc_output_pointers[4] = 0;
	port.adc_output_pointers[5] = 0;
	port.adc_output_pointers[6] = 0;
	port.adc_output_pointers[7] = 0;

	// Setup control registers
	adc->CTRLA = ADC_ENABLE_bm;
	adc->CTRLB = 0;

	// Set reference voltage to PORTA pin 0
	adc->REFCTRL = ADC_REFSEL_AREFA_gc;
	
	// setup prescaler
	adc->PRESCALER = ADC_PRESCALER_DIV16_gc;

	// Set the gain for the channels
	adc->CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	adc->CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	adc->CH2.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	adc->CH3.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;

	return port;
}

void adc_connect_port(adc_port_t *adc) {
	if (adc->adc_port == &ADCA)
		_adc_port_ADCA = adc;
	else if (adc->adc_port == &ADCB)
		_adc_port_ADCB = adc;
	
	// Enable the interrupt for this port
	adc->adc_port->CH0.INTCTRL = ADC_CH_INTLVL_LO_gc;
	adc->adc_port->CH1.INTCTRL = ADC_CH_INTLVL_LO_gc;
	adc->adc_port->CH2.INTCTRL = ADC_CH_INTLVL_LO_gc;
	adc->adc_port->CH3.INTCTRL = ADC_CH_INTLVL_LO_gc;
}

void adc_init_pin(adc_port_t *adc, uint8_t pin, uint16_t *pin_data) {
	adc->adc_output_pointers[pin] = pin_data;
}

void adc_start_read(adc_port_t *adc, uint8_t pin_mask) {
	PORTC.OUTSET = 0b10;
	adc->pin_mask = pin_mask;
	
	// setup and start channel 1 if it's used
	if (pin_mask & 0b00000001) {
		adc->adc_port->CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc;
		adc->adc_port->CH0.CTRL |= ADC_CH_START_bm;
	}
	else if (pin_mask & 0b00010000) {
		adc->adc_port->CH0.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc;
		adc->adc_port->CH0.CTRL |= ADC_CH_START_bm;
	}

	PORTC.OUTCLR = 0b10;
	// setup and start channel 2 if it's used
	if (pin_mask & 0b00000010) {
		adc->adc_port->CH1.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
		adc->adc_port->CH1.CTRL |= ADC_CH_START_bm;
	}
	else if (pin_mask & 0b00100000) {
		adc->adc_port->CH1.MUXCTRL = ADC_CH_MUXPOS_PIN5_gc;
		adc->adc_port->CH1.CTRL |= ADC_CH_START_bm;
	}

	// setup and start channel 3 if it's used
	if (pin_mask & 0b00000100) {
		adc->adc_port->CH2.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;
		adc->adc_port->CH2.CTRL |= ADC_CH_START_bm;
	}
	else if (pin_mask & 0b01000000) {
		adc->adc_port->CH2.MUXCTRL = ADC_CH_MUXPOS_PIN6_gc;
		adc->adc_port->CH2.CTRL |= ADC_CH_START_bm;
	}

	// setup and start channel 4 if it's used
	if (pin_mask & 0b00001000) {
		adc->adc_port->CH3.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
		adc->adc_port->CH3.CTRL |= ADC_CH_START_bm;
	}
	else if (pin_mask & 0b10000000) {
		adc->adc_port->CH3.MUXCTRL = ADC_CH_MUXPOS_PIN7_gc;
		adc->adc_port->CH3.CTRL |= ADC_CH_START_bm;
	}
}

bool adc_read_complete(adc_port_t *adc) {
	return adc->pin_mask == 0;
}

