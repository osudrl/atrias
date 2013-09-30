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
#define DEBUG_PRINT DCM   // DCM or IMU
#define DT_IMU 0.0025   // 400 Hz

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

// The DCM
float dcm[3][3];

int main(void) {
	m_init_identity(dcm);   // Initialize the DCM.

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

	debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_460800, debug_uart_tx_buffer, 128, debug_uart_rx_buffer, 128);
	uart_connect_port(&debug_port, true);

	debug_pin = io_init_pin(&PORTK, 1);   // Initialize debug pin.
	PORTK.DIR = PORTK.DIR | (1<<1);   // Debug pin

	setup_ahrs();   // Set up AHRS.

	_delay_ms(1800);   // Wait for IMU to be ready.

	while (1) {
		#if IMU_MSYNC == EXT
		update_ahrs(DT_IMU, dcm);
		_delay_us(30);   // Use the scope to determine this number for 500 Hz operation. Yeah it's a hack.
		#endif // IMU_MSYNC == EXT

		#if IMU_MSYNC == IMU
		// TODO: Implement same packet parsing code here and printf only whole
		// packets to debug.
		#endif // IMU_MSYNC == IMU

		//#if DEBUG_PRINT == IMU
		//print_imu();
		//#elif DEBUG_PRINT == DCM
		print_dcm(dcm);
		//#endif // DEBUG_PRINT
		_delay_us(366);
	}

	while(1);
	return 1;
}

