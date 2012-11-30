#include <avr/io.h>
#include <util/delay.h>

#include "cpu.h"
#include "uart.h"

UART_USES_PORT(USARTE0)

int main(void) {
	cpu_set_clock_source(cpu_32mhz_clock);
	cpu_configure_interrupt_level(cpu_interrupt_level_medium, true);

	uint8_t outbuffer[128];
	uint8_t inbuffer[128];
	uart_port_t debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, outbuffer, 128, inbuffer, 128);
	uart_connect_port(&debug_port, true);
	
	printf("Medulla UART Test.....\n> ");

	uint8_t newchar;
	while (1) {
		if (uart_received_bytes(&debug_port) > 0) {
			newchar = uart_rx_byte(&debug_port);
			if (newchar == 13)
				printf("\n> ");
			else
				uart_tx_byte(&debug_port,newchar);
		}
	}

	while(1);
	return 1;
}

