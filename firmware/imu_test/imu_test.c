#include <avr/io.h>
#include <util/delay.h>

#include "medulla.h"
#include "adc.h"
#include "uart.h"
#include "cpu.h"
#include "io_pin.h"
#include "ad7193.h" 

#include <medulla_ahrs.h>

#define IMU_MSYNC EXT   // EXT or IMU

//--- Define the interrupt functions ---//
UART_USES_PORT(USARTE0)   // Debug port
UART_USES_PORT(USARTF0)   // IMU port

//The ESTOP is on port J and uses TCE0 as it's debounce timer
ESTOP_USES_PORT(PORTJ)
ESTOP_USES_COUNTER(TCE0)

// Ethercat on port E
#ifdef ENABLE_ECAT
ECAT_USES_PORT(SPIE);
#endif


io_pin_t debug_pin;

int main(void) {
	// Initilize the clock to 32 Mhz oscillator
	if(cpu_set_clock_source(cpu_32mhz_clock) == false) {
		PORTC.DIRSET = 1;
		PORTC.OUTSET = 1;
	}

	// Configure and enable all the interrupts   NOTE: Using external
	// interrupts may mess up timing!
	cpu_configure_interrupt_level(cpu_interrupt_level_medium, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_high, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_low, true);
	sei();

	uint8_t unused_outbuffer[128];
	uint8_t unused_inbuffer[128];
	uint8_t debug_outbuffer[128];
	uint8_t debuginbuffer[128];
	uint8_t imu_outbuffer[128];
	uint8_t imu_inbuffer[128];
	uint8_t bytes_received = 0;

	debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_460800, unused_outbuffer, 128, unused_inbuffer, 128);
	uart_connect_port(&debug_port, true);

	imu_port = uart_init_port(&PORTF, &USARTF0, uart_baud_921600, unused_outbuffer, 128, unused_inbuffer, 128);
	uart_connect_port(&imu_port, false);

	msync_pin = io_init_pin(&PORTF, 1);   // Initialize master sync pin for IMU.
	PORTF.DIR = PORTF.DIR | (1<<1);   // msync pin is output.

	debug_pin = io_init_pin(&PORTK, 1);   // Initialize debug pin.
	PORTK.DIR = PORTK.DIR | (1<<1);   // Debug pin

	_delay_ms(1800);   // Wait for IMU to be ready.

	while (1) {
		#if IMU_MSYNC == EXT
		PORTF.OUT = PORTF.OUT | (1<<1);   // Pull msync pin high.
		_delay_us(30);   // ..for at least 30 us.
		PORTF.OUT = PORTF.OUT & ~(1<<1);   // Pull msync pin low.

		//_delay_us(1);   // Wait for IMU to finish sending data. Scope shows about 60 us delay from rising edge of msync to first bit of packet. NOTE: This doesn't work as intended when external interrupts are on, since UART will interrupt the delay!
		// It takes 390.6 us for the IMU to send 36 bytes at 921600 baud.

		bytes_received = uart_received_bytes(&imu_port);
		uart_rx_data(&imu_port, imu_inbuffer, bytes_received);   // This takes 200 us.

		int a=4;

		printf("%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
				imu_inbuffer[a+0], imu_inbuffer[a+1], imu_inbuffer[a+2], imu_inbuffer[a+3],
				imu_inbuffer[a+4], imu_inbuffer[a+5], imu_inbuffer[a+6], imu_inbuffer[a+7],
				imu_inbuffer[a+8], imu_inbuffer[a+9], imu_inbuffer[a+10], imu_inbuffer[a+11]);
		_delay_us(559);   // Use the scope to determine this number for 500 Hz operation. Yeah it's a hack.
		#endif // IMU_MSYNC == EXT

		#if IMU_MSYNC == IMU
		#endif // IMU_MSYNC == IMU
	}

	while(1);
	return 1;
}

