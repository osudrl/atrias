#include <avr/io.h>
#include <util/delay.h>

#include "cpu.h"
#include "uart.h"
#include "limit_switch.h"

UART_USES_PORT(USARTE0)

int main(void) {
	cpu_set_clock_source(cpu_32mhz_clock);
	cpu_configure_interrupt_level(cpu_interrupt_level_medium, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_low, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_high, true);

	uint8_t outbuffer[128];
	uint8_t inbuffer[128];
	uart_port_t debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, outbuffer, 128, inbuffer, 128);
	uart_connect_port(&debug_port, true);
	
	printf("Medulla Limit Swtich test.....\n> ");

	PORTCFG.MPCMASK = 0xFF;
	PORTK.PIN0CTRL = 0b11<<3;

	while (1) {
		printf("%d%d%d%d%d%d%d%d\n", (PORTK.IN>>7)&1, (PORTK.IN>>6)&1, (PORTK.IN>>5)&1, (PORTK.IN>>4)&1, (PORTK.IN>>3)&1, (PORTK.IN>>2)&1, (PORTK.IN>>1)&1, PORTK.IN&1);
		_delay_ms(100); 
	}

	while(1);
	return 1;
}

