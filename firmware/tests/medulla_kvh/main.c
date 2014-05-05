#include <avr/io.h>
#include <util/delay.h>

#include "cpu.h"
#include "uart.h"

UART_USES_PORT(USARTE0)
UART_USES_PORT(USARTF0)

int main(void) {
	//cpu_set_clock_source(cpu_32mhz_clock);

	// Enable external 16 MHz oscillator.
	OSC.XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc; /* configure the XTAL input */
	OSC.CTRL |= OSC_XOSCEN_bm; /* start XTAL */
	while (!(OSC.STATUS & OSC_XOSCRDY_bm)); /* wait until ready */
	OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | 0x2; /* XTAL->PLL, 2x multiplier */
	OSC.CTRL |= OSC_PLLEN_bm; /* start PLL */
	_delay_us(1);   /* Wait a bit per Kit's code */
	while (!(OSC.STATUS & OSC_PLLRDY_bm)); /* wait until ready */
	CCP = CCP_IOREG_gc; /* allow changing CLK.CTRL */
	CLK.CTRL = CLK_SCLKSEL_PLL_gc; /* use PLL output as system clock */

	cpu_configure_interrupt_level(cpu_interrupt_level_low, true);

	uint8_t outbuffer[128];
	uint8_t inbuffer[128];
	uint8_t imu_outbuffer[128];
	uint8_t imu_inbuffer[128];
	uart_port_t debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, outbuffer, 128, inbuffer, 128);
	uart_connect_port(&debug_port, true);
	uart_port_t imu_port = uart_init_port(&PORTF, &USARTF0, uart_baud_921600, imu_outbuffer, 128, imu_inbuffer, 128);
	uart_connect_port(&imu_port, true);
	
	printf("Medulla UART Test.....\n> ");

	uint8_t newchar;
	while (1) {
		if (uart_received_bytes(&debug_port) > 0) {
			newchar = uart_rx_byte(&debug_port);
			if (newchar == 13)
				printf("\n> ");
			else
				uart_tx_byte(&imu_port,newchar);
		}

		uart_tx_byte(&imu_port, 'U');
		_delay_us(10000);
	}

	while(1);
	return 1;
}

