#include <avr/io.h>
#include <util/delay.h>

#include "uart.h"
#include "cpu.h"
#include "ethercat.h"
#include "io_pin.h"
#include "pwm.h"
#include "limit_switch.h"
#include "estop.h"
#include "ssi_encoder.h"

UART_USES_PORT(USARTE0)
ECAT_USES_PORT(SPIE)
//LIMIT_SW_USES_PORT(PORTK)
//LIMIT_SW_USES_COUNTER(TCC0)
ESTOP_USES_PORT(PORTJ)
//ESTOP_USES_COUNTER(TCC0)
SSI_ENCODER_USES_PORT(SPIC)

uint8_t *command;
uint16_t *motor_current;
uint16_t *timestep;
uint32_t *encoder0;
uint8_t status;

void handle_estop() {
	PORTA.OUTTGL = 1<<7;
}

int main(void) {
	cpu_set_clock_source(cpu_32mhz_clock);
	cpu_configure_interrupt_level(cpu_interrupt_level_medium, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_high, true);
	sei();
	PORTA.DIRSET = 1<<7;
	//PORTC.DIRSET = 0b1111;
//	PORTC.OUTSET = 0b100;
	//if (limit_sw_enable_port(&limitSW) == 0)
//		PORTC.OUTSET = 0b1;

	uint8_t outbuffer[128];
	uint8_t inbuffer[128];
	uart_port_t debug_port = uart_init_port(&PORTE, &USARTE0, uart_baud_115200, outbuffer, 128, inbuffer, 128);
	uart_connect_port(&debug_port, true);
	printf("Starting\n");

	estop_port_t estop = estop_init_port(io_init_pin(&PORTJ,6),io_init_pin(&PORTJ,7),&TCC0,&handle_estop);
	estop_enable_port(&estop);

	//limit_sw_port_t limitSW = limit_sw_init_port(&PORTK, 0b100, &TCC0, *handle_estop);
	//_delay_ms(10);
	//limit_sw_enable_port(&limitSW);

/*	
	uint8_t rx_sm[6];
	uint8_t tx_sm[6];

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
*/
//	io_pin_t pwm_pin = io_init_pin(PORTF,5);
//	pwm_output_t pwm = pwm_initilize_output(pwm_pin,pwm_div64,1000);

//	pwm_enable_output(&pwm);

//	uint16_t cnt = 0;
//	while(1) {
//		cnt = (cnt+1)%1024;
//		pwm_set_output(&pwm,cnt);
//		_delay_ms(10);
//	}


	uint32_t enc_val;
	uint16_t timer_val;

	TCC1.CTRLA = TC_CLKSEL_DIV8_gc;

	ssi_encoder_t encoder = ssi_encoder_init(&PORTC,&SPIC,&TCC1,&enc_val,17,&timer_val);
	while(1) {
		ssi_encoder_start_reading(&encoder);
		// wait for read to complete
		while (!ssi_encoder_read_complete(&encoder));
		ssi_encoder_process_data(&encoder);
		printf("Encoder: %0lu Timestamp: %u\n",enc_val,timer_val);
		
		_delay_ms(100);
	}

	while(1);
	return 1;
}

