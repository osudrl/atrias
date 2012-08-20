#include "amplifier.h"
#include "util/delay.h"

uint16_t amp_write_command = 0xF;
uint16_t amp_enable_command = 0x00;
uint16_t amp_disable_command = 0x01;
uint32_t amp_change_baud_command = 0x04;

dzralte_message_list_t amp_message_list;
uart_port_t amp_uart_port;
pwm_output_t amp_pwm_output;
io_pin_t amp_direction_pin;

uint8_t amp_tx_buffer[256];
uint8_t amp_rx_buffer[256];

void initilize_amp(bool second_amp, int16_t *amp_63_current, int16_t *amp_62_current) {
	// Initilize the PWM and direction outputs
	//amp_direction_pin = io_init_pin(&PORTC,5);
	PORTC.DIRSET = 1<<3;
	amp_pwm_output = pwm_initilize_output(io_init_pin(&PORTC,4),pwm_div1,20000);

	// Initilize the amplifier communication uart at 115200 baud
	amp_uart_port = uart_init_port(&PORTD,&USARTD0,uart_baud_115200,amp_tx_buffer,256,amp_rx_buffer,256);
	uart_connect_port(&amp_uart_port,false);

	// Generate the amplifier messages
	dzralte_generate_message(&amp_message_list,63,AMP_GET_WRITE_ACCESS_63,dzralte_write_cmd,0x07,0x00,&amp_write_command,2);
	dzralte_generate_message(&amp_message_list,62,AMP_GET_WRITE_ACCESS_62,dzralte_write_cmd,0x7,0x00,&amp_write_command,2);
	dzralte_generate_message(&amp_message_list,63,AMP_ENABLE_63,dzralte_write_cmd,0x01,0x00,&amp_enable_command,2);
	dzralte_generate_message(&amp_message_list,62,AMP_ENABLE_62,dzralte_write_cmd,0x01,0x00,&amp_enable_command,2);
	dzralte_generate_message(&amp_message_list,63,AMP_DISABLE_63,dzralte_write_cmd,0x01,0x00,&amp_disable_command,2);
	dzralte_generate_message(&amp_message_list,62,AMP_DISABLE_62,dzralte_write_cmd,0x01,0x00,&amp_disable_command,2);
	dzralte_generate_message(&amp_message_list,63,AMP_GET_CURRENT_63,dzralte_read_cmd,0x10,0x03,amp_63_current,2);
	dzralte_generate_message(&amp_message_list,62,AMP_GET_CURRENT_62,dzralte_read_cmd,0x10,0x03,amp_62_current,2);
	dzralte_generate_message(&amp_message_list,63,AMP_CHANGE_BAUD_63,dzralte_write_cmd,0x05,0x03,&amp_change_baud_command,4);
	dzralte_generate_message(&amp_message_list,62,AMP_CHANGE_BAUD_62,dzralte_write_cmd,0x05,0x03,&amp_change_baud_command,4);

	// Send the packets to gain write access to the amplifiers
	dzralte_send_message(&amp_message_list,0,AMP_GET_WRITE_ACCESS_63,&amp_uart_port,false);
	if (second_amp)
		dzralte_send_message(&amp_message_list,0,AMP_GET_WRITE_ACCESS_62,&amp_uart_port,false);

	_delay_ms(5);

	// Change the baud rate of the amps
	//dzralte_send_message(&amp_message_list,0,AMP_CHANGE_BAUD_63,&amp_uart_port,false);	
	//if (second_amp)
	//	dzralte_send_message(&amp_message_list,0,AMP_CHANGE_BAUD_62,&amp_uart_port,false);

	//_delay_ms(5);

	// Now switch the serial baud rate to 921600 baud
	//amp_uart_port = uart_init_port(&PORTD,&USARTD0,uart_baud_921600,amp_tx_buffer,256,amp_rx_buffer,256);
}

inline void enable_pwm() {
	pwm_enable_output(&amp_pwm_output);
}

inline void disable_pwm() {
	pwm_disable_output(&amp_pwm_output);
}

void enable_amp(bool second_amp) {
	pwm_enable_output(&amp_pwm_output);

	dzralte_send_message(&amp_message_list,0,AMP_ENABLE_63,&amp_uart_port,false);
	if (second_amp)
		dzralte_send_message(&amp_message_list,0,AMP_ENABLE_62,&amp_uart_port,false);
}

void disable_amp(bool second_amp) {
	pwm_disable_output(&amp_pwm_output);

	dzralte_send_message(&amp_message_list,0,AMP_DISABLE_63,&amp_uart_port,false);
	if (second_amp)
		dzralte_send_message(&amp_message_list,0,AMP_DISABLE_62,&amp_uart_port,false);
}

void set_amp_output(int32_t value) {
	if (value < 0) {
		// negitive value so set direction pin to reverse
//		io_set_output(amp_direction_pin,io_low);
		PORTC.OUTCLR = 1<<3;
		pwm_set_output(&amp_pwm_output,(uint16_t)(value*-1));
	}
	else {
		// positive value, so set direction to forward
		//io_set_output(amp_direction_pin,io_high);
		PORTC.OUTSET = 1<<3;
		pwm_set_output(&amp_pwm_output,(uint16_t)value);
	}
}

void send_current_read(bool second_amp) {
	dzralte_send_message(&amp_message_list,0,AMP_GET_CURRENT_62,&amp_uart_port,false);
	if (second_amp)
		dzralte_send_message(&amp_message_list,0,AMP_GET_WRITE_ACCESS_62,&amp_uart_port,false);
}

bool check_current_read(bool second_amp) {
	// First check for new packets and process them
	dzralte_check_responses(&amp_message_list,&amp_uart_port);

	// Now check if there has been a response to the packets
	if (dzralte_response_received(&amp_message_list,0,AMP_GET_CURRENT_63)) {
		return dzralte_response_received(&amp_message_list,0,AMP_GET_WRITE_ACCESS_62);
	}
	return false;
}

