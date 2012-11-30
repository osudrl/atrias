#include <avr/io.h>
#include <util/delay.h>

#include "cpu.h"
#include "uart.h"
#include "usart_adc.h"

UART_USES_PORT(USARTE0)
USART_ADC_USES_PORT(USARTF0);

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

	uint16_t ch1, ch2, ch3, ch4;
	usart_adc_t adc = usart_adc_init(&PORTF, &USARTF0, io_init_pin(&PORTD,4), &ch1, &ch2, &ch3, &ch4);

	while (1) {
		usart_adc_start_read(&adc);

		while (!usart_adc_read_complete(&adc));
		usart_adc_process_data(&adc);
		printf("%04d\n",ch4);
		_delay_ms(100);
	}

	while(1);
	return 1;
}

