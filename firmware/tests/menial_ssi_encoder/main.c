#include <avr/io.h>
#include <util/delay.h>

#include "cpu.h"
#include "uart.h"
#include "spi.h"

UART_USES_PORT(USARTE0)
SPI_USES_PORT(SPIC)
SPI_USES_PORT(SPID)
SPI_USES_PORT(SPIF)

int main(void) {
	cpu_set_clock_source(cpu_32mhz_clock);
	cpu_configure_interrupt_level(cpu_interrupt_level_medium, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_low, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_high, true);

	uint8_t outbuffer[128];
	uint8_t inbuffer[128];
	uart_port_t debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, outbuffer, 128, inbuffer, 128);
	uart_connect_port(&debug_port, true);
	
	printf("Menial SSI encoder test.....\n> ");

	spi_port_t spi_c = spi_init_port(&PORTC, &SPIC, spi_div32, false);
	spi_port_t spi_d = spi_init_port(&PORTD, &SPID, spi_div32, false);
	spi_port_t spi_f = spi_init_port(&PORTF, &SPIF, spi_div32, false);

	uint8_t data_c, data_d, data_f;

	while (1) {
		spi_start_receive(&spi_c, &data_c, 1);
		spi_start_receive(&spi_d, &data_d, 1);
		spi_start_receive(&spi_f, &data_f, 1);
		while (spi_c.transaction_underway);
		while (spi_d.transaction_underway);
		while (spi_f.transaction_underway);

		printf("%d, %d, %d\n", data_c, data_d, data_f);
		_delay_ms(100);
	}

	while(1);
	return 1;
}

