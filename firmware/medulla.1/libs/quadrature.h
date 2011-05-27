// Kevin Kemper

#include <stdio.h>
#include <avr/io.h>


#include "menial_io.h"

#define GetCaptureValue(_tc)  ( _tc.CCA )



// Configures PWM output on compare a for single slope pwm, with hires, and clk source as sys clk
void initQuad() {


	// ENC_A,B,Z to inputs
	PORT_ENCODER.DIRCLR = ENC_A_bm | ENC_B_bm | ENC_Z_bm;

	// Configure Index signal sensing
	PORTCFG.MPCMASK = ENC_Z_bm;
	PORT_ENCODER.PIN0CTRL = (PORT_ENCODER.PIN0CTRL & ~PORT_ISC_gm) | PORT_ISC_BOTHEDGES_gc;

	// Set QDPH0 and QDPH1 sensing level
	PORTCFG.MPCMASK = ENC_A_bm | ENC_B_bm;
	PORT_ENCODER.PIN0CTRL = (PORT_ENCODER.PIN0CTRL & ~PORT_ISC_gm) | PORT_ISC_LEVEL_gc;
	
	
	/* Configure event channel 0 for quadrature decoding of pins. */
	EVSYS.CH0MUX = ENC_EVSYS_CHMUX_A;
	EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;

	/*  Configure event channel 1 as index channel. Note
	 *  that when enabling Index in channel n, the channel
	 *  n+1 must be configured for the index signal.*/
	EVSYS.CH1MUX = ENC_EVSYS_CHMUX_Z;
	EVSYS.CH1CTRL = EVSYS_DIGFILT_2SAMPLES_gc;
	EVSYS.CH0CTRL |= (uint8_t) EVSYS_QDIRM_00_gc | EVSYS_QDIEN_bm;
		

	// Configure TC as a quadrature counter
	TC_ENC.CTRLD = (uint8_t) TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;
	TC_ENC.PER = (LINECOUNT * 4) - 1;
	TC_ENC.CTRLA = TC_CLKSEL_DIV1_gc;



// velocity stuff
	// Configure channel 2 to input pin for freq calculation. */
//	EVSYS.CH2MUX = ENC_EVSYS_CHMUX_A;
//	EVSYS.CH2CTRL = EVSYS_DIGFILT_4SAMPLES_gc;

	/* Configure TC to capture frequency. */
//	TCF1.CTRLD = (uint8_t) TC_EVACT_FRW_gc | TC_EVSEL_CH2_gc;
//	TCF1.PER = 0xFFFF;
//	TCF1.CTRLB = TC1_CCAEN_bm;
//	TCF1.CTRLA = CLOCK_DIV_bm;
	
	
}



