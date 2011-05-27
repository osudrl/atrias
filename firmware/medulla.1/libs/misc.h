#include <stdio.h>
#include <avr/io.h>
#define F_CPU 32000000UL
#include <util/delay.h>


// Battery Sensor
#define BATTERY_ADC	7

#define MOTOR_DAC	3 //PB3

void initBattery() {
// setup adc for single ended one shot mode
	ADCA.CTRLA			= ADC_ENABLE_bm;						// enable adc
	ADCA.CTRLB			= ADC_RESOLUTION1_bm | ADC_FREERUN_bm;	// 8-bit result
	ADCA.CH0.CTRL		= 0x1;									// single ended
	ADCA.CH0.MUXCTRL	= BATTERY_ADC << 3;						// PORTA:2
//	ADCA.REFCTRL		= 										// Internal 1v ref
	ADCA.PRESCALER		= 0x0;									// peripheral clk/8

	PORTA.DIR			&= ~(1<<BATTERY_ADC);

}

unsigned char readBattery() {
	return ADCA.CH0RESL;
}



void initDAC() {

	PORTB.DIR	|= (1<<MOTOR_DAC);
	PORTB.OUT	&= ~(1<<MOTOR_DAC);

	DACB.CTRLC	= DAC_REFSEL_AVCC_gc;

	DACB.CTRLB	= DAC_CHSEL_DUAL_gc;
	
	DACB.CTRLA	= DAC_CH1EN_bm | DAC_ENABLE_bm;

}

void setDAC(int val) {

	DACB.CH1DATA = val;
}



