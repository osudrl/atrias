// Kevin Kemper
//
////////////////////////////////////////////////////////////////////////////////
#include "clock.h"

void Config32MHzClock(void) {
	CCP = CCP_IOREG_gc;															// security Signature to modify clock 

	// initialize clock source to be 32MHz internal oscillator (no PLL)
	OSC.CTRL = OSC_RC32MEN_bm;													// enable internal 32MHz oscillator

	while(!(OSC.STATUS & OSC_RC32MRDY_bm));										// wait for oscillator ready
	CCP = CCP_IOREG_gc;															// Security Signature to modify clock 
	CLK.CTRL = 0x01;															// select sysclock 32MHz osc
}



void Config32KHzRTC(void) {
	CCP = CCP_IOREG_gc;															// security Signature to modify clock 

	OSC.CTRL |= OSC_RC32KEN_bm;													// enable internal 32KHz Osc

	while(!(OSC.STATUS & OSC_RC32KRDY_bm));										// wait for oscillator ready

	// select RTC clk source
	CLK.RTCCTRL = (0x2<<1) | 0x1;
	RTC.CTRL = 0x02;

	// wait for RTC SYNC status not busy before returning
	while(RTC.STATUS);
}
