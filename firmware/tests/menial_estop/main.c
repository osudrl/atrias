#include <avr/io.h>
#include <util/delay.h>

#include "cpu.h"
#include "uart.h"
#include "estop.h"

UART_USES_PORT(USARTE0)
ESTOP_USES_PORT(PORTJ)
ESTOP_USES_COUNTER(TCE0)

void main_estop() {
	PORTC.OUTSET = 0b1;
}

int main(void) {
	cpu_set_clock_source(cpu_32mhz_clock);
	cpu_configure_interrupt_level(cpu_interrupt_level_medium, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_low, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_high, true);

	uint8_t outbuffer[128];
	uint8_t inbuffer[128];
	uart_port_t debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, outbuffer, 128, inbuffer, 128);
	uart_connect_port(&debug_port, true);
	
	printf("Menial e-stop test.....\n> ");
	PORTC.DIRSET = 0b111;
	estop_port_t estop_port = estop_init_port(io_init_pin(&PORTJ,6),io_init_pin(&PORTJ,7),&TCE0,main_estop);
	estop_enable_port(&estop_port);


	PORTCFG.MPCMASK = 0xFF;
	PORTK.PIN0CTRL = 0b11<<3;

	while(1);
	return 1;
}

