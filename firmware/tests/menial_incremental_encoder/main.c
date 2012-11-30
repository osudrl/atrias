#include <avr/io.h>
#include <util/delay.h>

#include "cpu.h"
#include "uart.h"
#include "quadrature_encoder.h"

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
	
	printf("Menial incremental encoder test.....\n> ");

	quadrature_encoder_t encoder = quadrature_encoder_init(io_init_pin(&PORTD,0),io_init_pin(&PORTD,5),false,&TCF1,16384); 

	while (1) {
		printf("%d\n", quadrature_encoder_get_value(&encoder));
		_delay_ms(100);
	}

	while(1);
	return 1;
}

