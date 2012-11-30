#include <avr/io.h>
#include <util/delay.h>

#include "cpu.h"
#include "uart.h"
#include "adc.h"

UART_USES_PORT(USARTE0)
ADC_USES_PORT(ADCB)
ADC_USES_PORT(ADCA)


int main(void) {
	cpu_set_clock_source(cpu_32mhz_clock);
	cpu_configure_interrupt_level(cpu_interrupt_level_medium, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_low, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_high, true);

	uint8_t outbuffer[128];
	uint8_t inbuffer[128];
	uart_port_t debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, outbuffer, 128, inbuffer, 128);
	uart_connect_port(&debug_port, true);
	
	printf("Medulla ADC test.....\n> ");

	uint16_t adcb0,adcb1,adcb2,adcb3,adcb4,adcb5,adcb6,adcb7;
	uint16_t adca1,adca2,adca3,adca4,adca5,adca6,adca7;
	adc_port_t adcb = adc_init_port(&ADCB);
	adc_port_t adca = adc_init_port(&ADCA);
	adc_init_pin(&adcb, 0, &adcb0);
	adc_init_pin(&adcb, 1, &adcb1);
	adc_init_pin(&adcb, 2, &adcb2);
	adc_init_pin(&adcb, 3, &adcb3);
	adc_init_pin(&adcb, 4, &adcb4);
	adc_init_pin(&adcb, 5, &adcb5);
	adc_init_pin(&adcb, 6, &adcb6);
	adc_init_pin(&adcb, 7, &adcb7);

	adc_init_pin(&adca, 1, &adca1);
	adc_init_pin(&adca, 2, &adca2);
	adc_init_pin(&adca, 3, &adca3);
	adc_init_pin(&adca, 4, &adca4);
	adc_init_pin(&adca, 5, &adca5);
	adc_init_pin(&adca, 6, &adca6);
	adc_init_pin(&adca, 7, &adca7);

	while (1) {
		PORTC.OUTSET = 0b10;
		adc_start_read(&adcb);
		adc_start_read(&adca);

		while ( (!adc_read_complete(&adcb)) || (!adc_read_complete(&adca))){};
		PORTC.OUTCLR = 0b10;
		printf("%04u %04u %04u %04u %04u %04u %04u %04u ",adcb0,adcb1,adcb2,adcb3,adcb4,adcb5,adcb6,adcb7); 
		printf("%04u %04u %04u %04u %04u %04u %04u\n",adca1,adca2,adca3,adca4,adca5,adca6,adca7); 
		_delay_ms(100);
	}

	while(1);
	return 1;
}

