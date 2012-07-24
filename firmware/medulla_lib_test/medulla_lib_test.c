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
#include "quadrature_encoder.h"
#include "dzralte_comm.h"
#include "adc.h"

UART_USES_PORT(USARTE0)
UART_USES_PORT(USARTC0)
ADC_USES_PORT(ADCB)
ADC_USES_PORT(ADCA)
//ECAT_USES_PORT(SPIE)
//LIMIT_SW_USES_PORT(PORTK)
//LIMIT_SW_USES_COUNTER(TCC0)
//ESTOP_USES_PORT(PORTJ)
//ESTOP_USES_COUNTER(TCC0)
//SSI_ENCODER_USES_PORT(SPIC)

uint8_t *command;
uint16_t *motor_current;
uint16_t *timestep;
uint32_t *encoder0;
uint8_t status;

dzralte_message_list_t message_list;

void handle_estop() {
	PORTA.OUTTGL = 1<<7;
}

int main(void) {
	cpu_set_clock_source(cpu_32mhz_clock);
	cpu_configure_interrupt_level(cpu_interrupt_level_medium, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_high, true);
	cpu_configure_interrupt_level(cpu_interrupt_level_low, true);
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
	printf("Starting...\n");
/*
	uart_port_t amp_port = uart_init_port(&PORTC, &USARTC0, uart_baud_115200, outbuffer, 128, inbuffer, 128);
	uart_connect_port(&amp_port, true);

	uint16_t write_command = 0xF;
	dzralte_generate_message(&message_list, 63,0,dzralte_write_cmd, 0x07,0x00,&write_command,2);
	dzralte_send_message(&message_list,0,0,&amp_port,false);

	while (!dzralte_response_received(&message_list,0,0)) {
		dzralte_check_responses(&message_list,&amp_port);
		_delay_ms(100);
	}
	uart_tx_data(&debug_port,message_list.message[0].response_header,8);
*/
//	estop_port_t estop = estop_init_port(io_init_pin(&PORTJ,6),io_init_pin(&PORTJ,7),&TCC0,&handle_estop);
//	estop_enable_port(&estop);

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

/*
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
*/
/*
	quadrature_encoder_t encoder = quadrature_encoder_init(io_init_pin(&PORTD,0),io_init_pin(&PORTD,4),true,&TCD0,2048);
	while (1) {
		printf("%u\n",quadrature_encoder_get_value(&encoder));
		_delay_ms(100);
	}
*/
	PORTC.DIRSET = 0b11;
	uint16_t adc0,adc1,adc2,adc3,adc4,adc5,adc6,adc7;
	uint16_t adca1,adca2,adca3,adca4,adca5,adca6,adca7;
	adc_port_t adcb = adc_init_port(&ADCB);
	adc_port_t adca = adc_init_port(&ADCA);
	adc_init_pin(&adcb, 0, &adc0);
	adc_init_pin(&adcb, 1, &adc1);
	adc_init_pin(&adcb, 2, &adc2);
	adc_init_pin(&adcb, 3, &adc3);
	adc_init_pin(&adcb, 4, &adc4);
	adc_init_pin(&adcb, 5, &adc5);
	adc_init_pin(&adcb, 6, &adc6);
	adc_init_pin(&adcb, 7, &adc7);

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
		printf("%04u %04u %04u %04u %04u %04u %04u %04u ",adc0,adc1,adc2,adc3,adc4,adc5,adc6,adc7); 
		printf("%04u %04u %04u %04u %04u %04u %04u\n",adca1,adca2,adca3,adca4,adca5,adca6,adca7); 
		_delay_ms(100);
	} 

	while(1);
	return 1;
}

