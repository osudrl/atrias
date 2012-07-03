/*
Xmega ADC Driver Source

Daniel Sidlauskas Miller

*/

#include "adc.h"

/*ADC Initiate*/
/*
Note - Initialize ADC to unsigned mode, can be changed with modification of CTRLB
	
	12 bit right adjusted
	
	Set to External Reference on PORTA, can be Changed with Modification of REFCTRL

	Prescaler Set to div4, can be modified by changing PRESCALER

	Make sure to set input channels as inputs

Params-
	adc - pointer to the ADC module to be initialized (eg ADC0)
*/
void adcInit(ADC_t * adc){
	adc->CTRLA |= 1;
	adc->CTRLB |= 0;

	adc->REFCTRL |= 0b00010000;
	adc->EVCTRL |= 0b11000000;

	adc->PRESCALER |= 0b00000111;

	(&(adc->CH0))->CTRL = 1;
}


/*Read all 4 ADC Channels*/
/*
Note- Could be modified to read both ADC modules channels since all adc conversions can be
		done in parrallel

Params-
	adc - pointer to adc module that was initialized and is to be read

	data - pointer to integer array where channel results are to be stored

*/
void adcRead(ADC_t * adc, int * data){
	adc->CTRLA |= 0b00111100;

	//Wait for ADC Channel 1
	while(!(adc->INTFLAGS & 1));
		data[0] = adc->CH0RES;
		adc->INTFLAGS |= 1;
	
	//Wait for ADC Channel 1
	while(!(adc->INTFLAGS & 2));
		data[1] = adc->CH1RES;
		adc->INTFLAGS |= 2;
	
	//etc..
	while(!(adc->INTFLAGS & 4));
		data[2] = adc->CH2RES;
		adc->INTFLAGS |= 4;
	
	//etc
	while(!(adc->INTFLAGS & 8));
		data[3] = adc->CH3RES;
		adc->INTFLAGS |= 8;
	
}
	
