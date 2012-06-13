// Kevin Kemper

#include "quadrature.h"


// Configures PWM output on compare a for single slope pwm, with hires, and clk source as sys clk
void initQuad() {

	// ENC_A,B to inputs
	PORT_ENCODER.DIRCLR = ENC_A_bm | ENC_B_bm;

	// Set QDPH0 and QDPH1 sensing level
	PORTCFG.MPCMASK = ENC_A_bm | ENC_B_bm;
	PORT_ENCODER.PIN0CTRL = (PORT_ENCODER.PIN0CTRL & ~PORT_ISC_gm) | PORT_ISC_BOTHEDGES_gc;
	
	
	/* Configure event channel 0 for quadrature decoding of pins. */
	EVSYS.CH0MUX = ENC_EVSYS_CHMUX_A;
	EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;

	// Configure TC as a quadrature counter
	TC_ENC.CTRLD = (uint8_t) TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;
	TC_ENC.PER = 0xFFFF;
	TC_ENC.CTRLA = TC_CLKSEL_DIV1_gc;
	TC_ENC.CNT = 1;
	inc_encoder_base = 0;

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