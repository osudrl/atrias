// Kevin Kemper

#include <avr/io.h>

#include "./menial_io.h"

void initADC() {

	// ADC: voltages
	// setup adc for free running mode
	ADC_VOLTAGE.CTRLA			= ADC_ENABLE_bm;						// enable adc
	ADC_VOLTAGE.CTRLB			= ADC_RESOLUTION1_bm;					// 8-bit result
//	ADC_VOLTAGE.REFCTRL			= 										// Internal 1v ref
	ADC_VOLTAGE.PRESCALER		= 0x00;									// peripheral clk
//	ADC_VOLTAGE.EVCTRL			= ADC_SWEEP0_bm;

	ADC_VOLTAGE.CH0.CTRL		= 0x01;									// single ended
	ADC_VOLTAGE.CH0.MUXCTRL		= ADC_LOGIC << 3;

//	ADC_VOLTAGE.CH1.CTRL		= 0x01;									// single ended
//	ADC_VOLTAGE.CH1.MUXCTRL		= ADC_POW << 3;

	PORT_VOLTAGE.DIRCLR			= ADC_LOGIC_bm;// | ADC_POW_bm;


	// ADC: Thermistors
//	ADC_THERM.CTRLA				= ADC_ENABLE_bm;						// enable adc
//	ADC_THERM.CTRLB				= ADC_RESOLUTION1_bm;					// 8-bit result
//	ADC_THERM.REFCTRL			= 										// Internal 1v ref
//	ADC_THERM.PRESCALER			= 0x00;									// peripheral clk
//	ADC_THERM.EVCTRL			= ADC_SWEEP1_bm;

//	ADC_THERM.CH0.CTRL			= 0x01;									// single ended
//	ADC_THERM.CH0.MUXCTRL		= THERM0 << 3;

//	ADC_THERM.CH1.CTRL			= 0x01;									// single ended
//	ADC_THERM.CH1.MUXCTRL		= THERM1 << 3;

//	ADC_THERM.CH2.CTRL			= 0x01;									// single ended
//	ADC_THERM.CH2.MUXCTRL		= THERM2 << 3;

//	PORT_THERM.DIRCLR			= THERM0_bm | THERM1_bm | THERM2_bm;
	

	ADC_LOGIC_START;
//	ADC_POW_START;
	
//	THERM0_START;
//	THERM1_START;
//	THERM2_START;
	

}



