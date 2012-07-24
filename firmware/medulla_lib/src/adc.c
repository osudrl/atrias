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

	// Set up the interrupt
	adc->CH3.INTCTRL = ADC_CH_INTLVL_LO_gc;
	
	// return the adc struc
	return port;
}

void adc_init_pin(adc_port_t *adc, uint8_t pin, uint16_t *pin_data) {
	adc->adc_output_pointers[pin] = pin_data;
	adc->pin_mask |= (1<<pin);
}

void adc_start_read(adc_port_t *adc) {
	// We signal that a buffer contains old data, by setting the msb. The user will never know
	// So let's mark all the buffers as having old data
	if (adc->adc_port == &ADCA) {
		_adc_buffer_ADCA[0] |= 0x8000;
		_adc_buffer_ADCA[1] |= 0x8000;
		_adc_buffer_ADCA[2] |= 0x8000;
		_adc_buffer_ADCA[3] |= 0x8000;
		_adc_buffer_ADCA[4] |= 0x8000;
		_adc_buffer_ADCA[5] |= 0x8000;
		_adc_buffer_ADCA[6] |= 0x8000;
		_adc_buffer_ADCA[7] |= 0x8000;
	}
	else if (adc->adc_port == &ADCB) {
		_adc_buffer_ADCB[0] |= 0x8000;
		_adc_buffer_ADCB[1] |= 0x8000;
		_adc_buffer_ADCB[2] |= 0x8000;
		_adc_buffer_ADCB[3] |= 0x8000;
		_adc_buffer_ADCB[4] |= 0x8000;
		_adc_buffer_ADCB[5] |= 0x8000;
		_adc_buffer_ADCB[6] |= 0x8000;
		_adc_buffer_ADCB[7] |= 0x8000;
	}
	
	// rest the ADC channel mux registers to read the first four pins
	adc->adc_port->CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc;
	adc->adc_port->CH1.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
	adc->adc_port->CH2.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;
	adc->adc_port->CH3.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;

	// Start all the channels reading, when they are all done, the channel interrupt will fire.
	adc->adc_port->CTRLA |= ADC_CH0START_bm | ADC_CH1START_bm | ADC_CH2START_bm | ADC_CH3START_bm;
}

bool adc_read_complete(adc_port_t *adc) {
	// All the data has been read when the MSb of pin 7 in the hardware buffer is 0
	uint16_t *data_buffer;
	if (adc->adc_port == &ADCA)
		data_buffer = _adc_buffer_ADCA;
	else if (adc->adc_port == &ADCB)
		data_buffer = _adc_buffer_ADCB;
	else 
		return false;

	if (data_buffer[7] & 0x8000)
		return false; // Still reading
	else {
		// done reading, so copy the desired buffers. Mask out the upper bits so the user doesn't have to worry about our funky singlas
		if (adc->pin_mask & 0b00000001)
			*(adc->adc_output_pointers[0]) = data_buffer[0] & 0xFFF;
		if (adc->pin_mask & 0b00000010)
			*(adc->adc_output_pointers[1]) = data_buffer[1] & 0xFFF;
		if (adc->pin_mask & 0b00000100)
			*(adc->adc_output_pointers[2]) = data_buffer[2] & 0xFFF;
		if (adc->pin_mask & 0b00001000)
			*(adc->adc_output_pointers[3]) = data_buffer[3] & 0xFFF;
		if (adc->pin_mask & 0b00010000)
			*(adc->adc_output_pointers[4]) = data_buffer[4] & 0xFFF;
		if (adc->pin_mask & 0b00100000)
			*(adc->adc_output_pointers[5]) = data_buffer[5] & 0xFFF;
		if (adc->pin_mask & 0b01000000)
			*(adc->adc_output_pointers[6]) = data_buffer[6] & 0xFFF;
		if (adc->pin_mask & 0b1000000)
			*(adc->adc_output_pointers[7]) = data_buffer[7] & 0xFFF;
		return true;
	}
}

