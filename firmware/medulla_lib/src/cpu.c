#include "cpu.h"

#define _CPU_OSC_TIMEOUT 100 // Number of loop counts to wait for the the oscillator to start

bool cpu_set_clock_source(cpu_clock_source_t clk_source) {
	// The bit mask for the desired oscillator source is the upper 8 bits of the clk_source value
	OSC.CTRL = clk_source>>8;

	uint8_t wait_cnt = 0; // wait loop counter
	while(!(OSC.STATUS & clk_source>>8)) { // wait for oscillator ready
		if (_CPU_OSC_TIMEOUT < wait_cnt++)
			// we have waited long enough, give up
			return false;
	}

	CCP = CCP_IOREG_gc; // Security Signature to modify clock 
	CLK.CTRL = clk_source & 0xFF; // The desired clock source is in the lower 8 bits of clk_source
	return true;
}



bool cpu_configure_rtc(bool enabled) {
	if (enabled) {
		// We are trying to eanble the RTC
		OSC.CTRL |= OSC_RC32KEN_bm; // enable internal 32KHz Osc

		uint8_t wait_cnt = 0;
		while(!(OSC.STATUS & OSC_RC32KRDY_bm)) { 
			// wait for oscillator ready
			if (_CPU_OSC_TIMEOUT < wait_cnt++)
				// we have waited long enough, give up
				return false;
		}

		// select RTC clk source
		CLK.RTCCTRL = (0x2<<1) | 0x1;
		RTC.CTRL = 0x01;

		// wait for RTC SYNC status not busy before returning
		while(RTC.STATUS);
	}
	else {
		// We are disabling the RTC
		CLK.RTCCTRL = 0;
		RTC.CTRL = 0;
	}
	
	// If we didn't fail waiting for the 32Khz oscillator to start, then we succeded, return true
	return true;
}

void cpu_configure_interrupt_level(cpu_interrupt_level_t interrupt_level, bool enable) {
	if (enable)
		// we are enabling, so set the interrupt level's bit in the PMIC control register
		PMIC.CTRL |= interrupt_level;
	else
		// we are disabling the interrupt level, so and with interrupt_level's inverse
		PMIC.CTRL &= ~interrupt_level;
}

inline void cpu_reset(void) {
	RST.CTRL = 1;
}

