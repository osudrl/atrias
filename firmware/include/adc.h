#ifndef ADC_H
#define ADC_H

#include <avr/io.h>

void initADC(ADC_t * adc);
void initADC_CH(ADC_CH_t * channel, uint8_t pin);
#define readADC_CH(channel) channel.RESL

#endif