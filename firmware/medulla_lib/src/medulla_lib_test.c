#include <avr/io.h>
#include <util/delay.h>

#define UART_USES_F0

#include "uart.h"
#include "clock.h"

int main(void) {
	Config32MHzClock();	
	PORTC.DIRSET = 0b11;
	PMIC.CTRL = PMIC_MEDLVLEN_bm;
	sei();


	uint8_t outbuffer[128];
	uint8_t inbuffer[128];
	uart_port_t debug_port = uart_init_port(&PORTF, &USARTF0, uart_baud_115200, outbuffer, 32, inbuffer, 128);

	uart_connect_port(&debug_port, true);

	while (1) {
		printf("Hello\n");
		_delay_ms(1000);
	}
	return 1;
}

