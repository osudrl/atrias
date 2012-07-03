#include <avr/io.h>
#include <util/delay.h>

#define UART_USES_E0

#include "uart.h"
#include "clock.h"

int main(void) {
	Config32MHzClock();	
	PORTC.DIRSET = 0b11;
	PMIC.CTRL = PMIC_MEDLVLEN_bm;
	sei();


	uint8_t outbuffer[128] = {'A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A','A'};
	uint8_t inbuffer[128];
	uint8_t data[] = {'0','1','2','3','4','5','6','7','8','9',13,11};
	uint16_t size = 0;
	uart_port_t debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, outbuffer, 128, inbuffer, 128);

	uart_connect_port(&debug_port, true);
	while (1) {
		size = uart_rx_data(&debug_port, data, 7);
		uart_tx_data(&debug_port,data,size) ;
		//_delay_ms(1000);
	}
	return 1;
}

