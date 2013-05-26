#include <avr/io.h>
#include <util/delay.h>

#include "uart.h"
#include "cpu.h"
#include "io_pin.h"
#include "ad7193.h" 

UART_USES_PORT(USARTE0)
UART_USES_PORT(USARTF0)

uart_port_t imu_port;
io_pin_t msync_pin;

int main(void) {
	cpu_set_clock_source(cpu_32mhz_clock);
	cpu_configure_interrupt_level(cpu_interrupt_level_medium, true);
	//cpu_configure_interrupt_level(cpu_interrupt_level_high, true);
	//cpu_configure_interrupt_level(cpu_interrupt_level_low, true);
	sei();
	uint8_t outbuffer[128];
	uint8_t inbuffer[128];
	uart_port_t debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, outbuffer, 128, inbuffer, 128);
	uart_connect_port(&debug_port, true);
	printf("Starting...\n");

	imu_port = uart_init_port(&PORTF, &USARTF0, uart_baud_921600, inbuffer, 128, outbuffer, 128);
	uart_connect_port(&imu_port, false);

	msync_pin = io_init_pin(&PORTF, 1);   // Initialize master sync pin for IMU.
	PORTF.DIR = PORTF.DIR | (1<<1);   // msync pin is output.

	while (1) {
		PORTF.OUT = PORTF.OUT | (1<<1);   // Pull msync pin high.
		_delay_us(30);   // ..for at least 30 us.
		PORTF.OUT = PORTF.OUT & ~(1<<1);   // Pull msync pin low.

		_delay_us(100);   // Wait for IMU to finish sending data. Scope shows about 60 us delay from rising edge of msync to first bit of packet.

		uint8_t bytes_received = uart_received_bytes(&imu_port);   // DEBUG
		uart_rx_data(&imu_port, inbuffer, uart_received_bytes(&imu_port));

		int a=28;
		printf("[Medulla IMU] %u %x %x %x %x\n", bytes_received, inbuffer[a+0], inbuffer[a+1], *(inbuffer+a+2), *(inbuffer+a+3));

		_delay_ms(2);
	}

	while(1);
	return 1;
}

