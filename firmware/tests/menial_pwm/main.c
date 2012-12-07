#include <avr/io.h>
#include <util/delay.h>

#include "cpu.h"
#include "uart.h"
#include "pwm.h"

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
	
	printf("Menial PWM test.....\n> ");

	pwm_output_t pwm_output = pwm_initilize_output(io_init_pin(&PORTC, 4), pwm_div256, 1000);
	pwm_enable_output(&pwm_output);
	PORTC.DIRSET = 1<<3;

	while (1) {
		PORTC.OUTTGL = 1<<3;
		pwm_set_output(&pwm_output, 100);
		_delay_ms(1000); 
		PORTC.OUTTGL = 1<<3;
		pwm_set_output(&pwm_output, 500);
		_delay_ms(1000); 
	}

	while(1);
	return 1;
}

