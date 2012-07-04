#include <avr/io.h>
#include <util/delay.h>

#define UART_USES_E0
#define SPI_USES_PORTF
#define SPI_USES_PORTC

#include "uart.h"
#include "spi.h"
#include "clock.h"

int main(void) {
	Config32MHzClock();	
	PORTC.DIRSET = 0b11;
	PMIC.CTRL = PMIC_MEDLVLEN_bm;
	sei();


	uint8_t outbuffer[128];
	uint8_t inbuffer[128];
	uart_port_t debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, outbuffer, 128, inbuffer, 128);
	spi_port_t spi_port = spi_init_port(&PORTF, &SPIF, false);
	spi_port_t spi_portc = spi_init_port(&PORTC, &SPIC, false);

	uart_connect_port(&debug_port, true);

	uint8_t barf_buffer[] = "Hello world, this is a test, Hello Wold this is a test, Hello world this is a test";

	uint8_t out = 'A';
	uint8_t in = 'X';

	while (1) {
		spi_start_transmit(&spi_portc,barf_buffer,6);
		PORTC.OUTSET = 1;
		spi_start_transmit_receive(&spi_port, &out, 1, &in, 1);
		PORTC.OUTCLR = 1;
		printf("RX: %c\n", in);
		_delay_us(500);
	}

	return 1;
}

