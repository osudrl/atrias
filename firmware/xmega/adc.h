/*
Xmega ADC Driver Header

Daniel Sidlauskas Miller

More comments on functions in source file
*/



#include "avr_compiler.h"

void adcInit(ADC_t * adc);

void adcRead(ADC_t * adc, int * data);
