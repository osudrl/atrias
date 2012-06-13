#include "adc.h"

void initADC(ADC_t * adc) {
	adc->CTRLA = ADC_ENABLE_bm;  			// Enable this ADC
	adc->CTRLB = ADC_RESOLUTION1_bm | ADC_FREERUN_bm; 		// Set 8 bit resolution
	adc->EVCTRL = 0b11000000;				// Set free run channels to sweep
	adc->REFCTRL = ADC_REFSEL_AREFA_gc; 	// Set the reference voltage to read from pin A0
	adc->PRESCALER	= ADC_PRESCALER_DIV16_gc;					// Scale the ADC clock by 4, this is as fast as it can run
}

void initADC_CH(ADC_CH_t * channel, uint8_t pin) {
	channel->MUXCTRL = pin<<3;				// Connect this channel to the input pin
	channel->CTRL = 0x01;					// Put the channel in singled ended input mode
}