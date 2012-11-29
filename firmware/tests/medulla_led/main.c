#include <avr/io.h>
#include <util/delay.h>

#include "cpu.h"

int main(void) {
	cpu_set_clock_source(cpu_32mhz_clock);

	PORTC.DIRSET = 0b111;
	
	uint8_t count = 0;
	while (1) {
		PORTC.OUT = (count++) & 0b111;
		_delay_ms(500);
	}

	while(1);
	return 1;
}

