#include <avr/io.h>
#include <util/delay.h>

#include "cpu.h"
#include "uart.h"
#include "biss_encoder.h"

UART_USES_PORT(USARTE0)
BISS_ENCODER_USES_PORT(SPIC)

ISR(TCC0_OVF_vect) {
	PORTC.OUTTGL = 1;
	PORTC.OUTTGL = 0b10;
	PORTC.OUTTGL = 0b100;
}

int main(void) {
	cpu_set_clock_source(cpu_32mhz_clock);
	cpu_configure_interrupt_level(cpu_interrupt_level_medium, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_low, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_high, true);
	PORTC.DIRSET = 0b111;
	uint8_t outbuffer[128];
	uint8_t inbuffer[128];
	uart_port_t debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, outbuffer, 128, inbuffer, 128);
	uart_connect_port(&debug_port, true);
	
	printf("Menial BISS encoder test.....\n> ");
	TCC0.CTRLA = TC_CLKSEL_DIV2_gc;
	TCC0.INTCTRLA = TC_OVFINTLVL_HI_gc;
	uint32_t encoder_data;
	uint16_t encoder_timestamp;
	biss_encoder_t encoder = biss_encoder_init(&PORTC,&SPIC,&TCC0,32,&encoder_data,&encoder_timestamp);

	uint32_t old_val;
	biss_encoder_start_reading(&encoder);
	while(!biss_encoder_read_complete(&encoder));
	biss_encoder_process_data(&encoder);
	old_val = encoder_data;

	while (1) {
		biss_encoder_start_reading(&encoder);
		while(!biss_encoder_read_complete(&encoder));
		biss_encoder_process_data(&encoder);
		printf("%010lu\n",encoder_data);
/*
		if (old_val > encoder_data) {
			if ((old_val - encoder_data) > 10000)
				printf("%010lu -> %010lu\n",old_val,encoder_data);
		}
		else {
			if ((encoder_data - old_val) > 10000)
				printf("%010lu -> %010lu\n",old_val,encoder_data);
		}
		old_val = encoder_data;*/
		_delay_ms(100);
	}

	while(1);
	return 1;
}

