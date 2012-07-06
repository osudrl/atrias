#include <avr/io.h>
#include <util/delay.h>

#define UART_USES_E0
#define SPI_USES_PORTE

#include "uart.h"
#include "cpu.h"
#include "ethercat.h"
#include "io_pin.h"

uint8_t *command;
uint16_t *motor_current;
uint16_t *timestep;
uint32_t *encoder0;
uint8_t status;

int main(void) {
	cpu_set_clock_source(cpu_32mhz_clock);
	PORTC.DIRSET = 0b11;
	PMIC.CTRL = PMIC_MEDLVLEN_bm;
	sei();

	uint8_t outbuffer[128];
	uint8_t inbuffer[128];
	uint8_t rx_sm[6];
	uint8_t tx_sm[6];
	uart_port_t debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, outbuffer, 128, inbuffer, 128);
	uart_connect_port(&debug_port, true);
	printf("Starting\n");

	io_pin_t eeprom = io_init_pin(PORTE,0);
	io_pin_t irq = io_init_pin(PORTE,1);
	ecat_slave_t ecat = ecat_init_slave(&PORTE,&SPIE,eeprom,irq);
	ecat_init_sync_managers(&ecat,rx_sm,6,0x1000,tx_sm,6,0x2000);

	ecat_pdo_entry_t rx_pdos[] = {{&command,1},{&motor_current,2}};
	ecat_pdo_entry_t tx_pdos[] = {{&timestep,2},{&encoder0,4}};
	ecat_configure_pdo_entries(&ecat,rx_pdos,2,tx_pdos,2);

	while (1) {
		ecat_update_status(&ecat);
		ecat_read_rx_sm(&ecat);
		*timestep = *command;
		*encoder0 = ((uint32_t)(*command))*1000;
		ecat_write_tx_sm(&ecat);
	}

	while(1);
	return 1;
}

