#include "uart.h"

// Define internal functions and variables, so they cannot be used outside the library
/// Internal function to get a pointer to the hardware buffer struct
static _uart_buffer_t* _uart_get_hw_buffer(uart_port_t *port);

/// Internal function used by printf for sending characters
static int _uart_stdio_put_char(char c, FILE *stream);

/// Internal function used by scanf to read characters out of the rx buffer
static int _uart_stdio_get_char(FILE *stream);

// static variables for use with printf
static FILE _uart_stdio_file = FDEV_SETUP_STREAM(_uart_stdio_put_char,_uart_stdio_get_char,_FDEV_SETUP_RW);
uart_port_t *_uart_stdio_port;

uart_port_t uart_init_port(PORT_t *port_reg, USART_t *uart_reg, uart_baud_t baud_rate, void *tx_buffer, uint8_t tx_buffer_length, void *rx_buffer, uint8_t rx_buffer_length) {
	uart_port_t port;
	port.uart_port = port_reg;
	port.uart_register = uart_reg;
	port.baud_rate = baud_rate;
	port.tx_buffer = tx_buffer;
	port.tx_buffer_size = tx_buffer_length;
	port.rx_buffer = rx_buffer;
	port.rx_buffer_size = rx_buffer_length;

	if ( ((unsigned short)(uart_reg) & 0x0010) > 0) {
		// If this is a USART1, then use pins 6 and 7
		port.uart_port->OUTSET = (1<<7);
		port.uart_port->DIRSET = (1<<7);
		port.uart_port->DIRCLR = (1<<6);
	}
	else {
		// If this is a USART0, then use ins 2 and 3
		port.uart_port->OUTSET = (1<<3);
		port.uart_port->DIRSET = (1<<3);
		port.uart_port->DIRCLR = (1<<2);
	}

	// Set the baud registers based upon the given baud rate.
	// The values in the enum, relate to the vaues of these registers, so we can just use that
	port.uart_register->BAUDCTRLA = baud_rate & 0xFF;
	port.uart_register->BAUDCTRLB = baud_rate >> 8;

	// Enable the tx driver and rx receiver
	port.uart_register->CTRLB = USART_RXEN_bm | USART_TXEN_bm;

	// Configure the port for 8 bit mode
	port.uart_register->CTRLC = USART_CHSIZE_8BIT_gc;

	return port;

}

int uart_connect_port(uart_port_t *port, bool use_for_stdio) {
	_uart_buffer_t *current_buffer = _uart_get_hw_buffer(port);

	// check if the hardware is already being used, if it is, then return -1. If not then signal that we are using it now.
	if (current_buffer->current_port != 0)
		return -1;
	else
		current_buffer->current_port = port;

	// Setup the buffer;
	current_buffer->tx_buffer = port->tx_buffer;
	current_buffer->tx_buffer_size = port->tx_buffer_size;
	current_buffer->tx_buffer_start = 0;
	current_buffer->tx_buffer_end = 0;
	current_buffer->rx_buffer = port->rx_buffer;
	current_buffer->rx_buffer_size = port->rx_buffer_size;
	current_buffer->rx_buffer_start = 0;
	current_buffer->rx_buffer_end = 0;
	
	//now that the buffer has been configured, we can enable the interrupts
	port->uart_register->CTRLA = USART_RXCINTLVL_MED_gc | USART_TXCINTLVL_MED_gc;

	// If this port is to be used as the standard IO port for printf/scanf, then we need to set the _uart_stdio_port pointer
	if (use_for_stdio) {
		_uart_stdio_port = port;
		stdout = &_uart_stdio_file;
		stdin = &_uart_stdio_file;
	}

	return 0;
}

int uart_disconnect_port(uart_port_t *port) {
	_uart_buffer_t *current_buffer = _uart_get_hw_buffer(port);

	// check if this port is actually connected
	if (current_buffer->current_port == port) {
		// It it's connected, then disconnect
		current_buffer->current_port = 0;
		port->uart_register->CTRLA = 0;
		return 0;
	}
	else 
		// The port is not connected
		return -1;
}

int uart_tx_data(uart_port_t *port, void *data, uint8_t data_length) {
	if (data_length == 0)
		// This is just silly, exit
		return 0;

	_uart_buffer_t *current_buffer = _uart_get_hw_buffer(port);

	bool not_currently_transmitting = (current_buffer->tx_buffer_end == current_buffer->tx_buffer_start);

	int byte_cnt = 0; // this is defined outside the loop so we can see how many bytes we actually wrote
	for (byte_cnt = 0; byte_cnt < data_length; byte_cnt++) {
		// Check the buffer is full
		if (current_buffer->tx_buffer_end == (current_buffer->tx_buffer_start-1))
			break; // get out of the loop
		
		// if the buffer is not full, then we can copy a byte into the buffer. To make C happy, we have to first cast the void pointer to a type
		current_buffer->tx_buffer[current_buffer->tx_buffer_end] = ((uint8_t*)data)[byte_cnt];
		// increment the buffer end pointer, wrapping around when neccesary
		current_buffer->tx_buffer_end = ((current_buffer->tx_buffer_end+1)%current_buffer->tx_buffer_size);
	}

	// If we were not in the middle of transmittind data, then we should start of the transmit by putting the first byte into the output buffer
	if (not_currently_transmitting) {
		port->uart_register->DATA = current_buffer->tx_buffer[current_buffer->tx_buffer_start];
		current_buffer->tx_buffer_start = ((current_buffer->tx_buffer_start+1)%current_buffer->tx_buffer_size);
	}

	// Return the number of bytes that were actually put into the buffer
	return byte_cnt;
}

inline int uart_tx_byte(uart_port_t *port, uint8_t data) {
	return uart_tx_data(port,&data,1);
}

int uart_rx_data(uart_port_t *port, void *data, uint8_t data_length) {
	_uart_buffer_t *current_buffer = _uart_get_hw_buffer(port);
	int byte_cnt = 0;
	for (byte_cnt = 0; byte_cnt < data_length; byte_cnt++) {
		// check that there is actually more data to read
		if (current_buffer->rx_buffer_start == current_buffer->rx_buffer_end)
			// we are at the end of the buffer
			break;

		// if there is more data in the buffes, then get a byte
		((uint8_t*)data)[byte_cnt] = current_buffer->rx_buffer[current_buffer->rx_buffer_start];
		current_buffer->rx_buffer_start = ((current_buffer->rx_buffer_start+1)%current_buffer->rx_buffer_size);
	}
	return byte_cnt;
}

inline uint8_t uart_rx_byte(uart_port_t *port) {
	uint8_t rx_data = 0;
	uart_rx_data(port, &rx_data,1);
	return rx_data;
}

static _uart_buffer_t* _uart_get_hw_buffer(uart_port_t *port) {
	_uart_buffer_t *current_buffer;
	switch ((intptr_t)(port->uart_register)) {
		case (intptr_t)(&USARTC0): current_buffer = &_uart_buffer_c0; break;
		case (intptr_t)(&USARTC1): current_buffer = &_uart_buffer_c1; break;
		case (intptr_t)(&USARTD0): current_buffer = &_uart_buffer_d0; break;
		case (intptr_t)(&USARTD1): current_buffer = &_uart_buffer_d1; break;
		case (intptr_t)(&USARTE0): current_buffer = &_uart_buffer_e0; break;
		case (intptr_t)(&USARTE1): current_buffer = &_uart_buffer_e1; break;
		case (intptr_t)(&USARTF0): current_buffer = &_uart_buffer_f0; break;
		case (intptr_t)(&USARTF1): current_buffer = &_uart_buffer_f1; break;
		default: current_buffer = 0;
	}
	
	return current_buffer;
}

static int _uart_stdio_put_char(char c, FILE *stream) {
	if (_uart_stdio_port == 0)
		// No uart port has been assigned as a stdio port, return an error
		return -1;

	// Since stdio functions don't convert newlines to cr-lf, and I am lazy and like to only use new lines, check and see if we need to and a carrage return.
	if (c == '\n')
		uart_tx_byte(_uart_stdio_port,'\r');
	uart_tx_byte(_uart_stdio_port,c);
	return 0;
}

static int _uart_stdio_get_char(FILE *stream) {
	if (_uart_stdio_port == 0)
		// No uart port setup for stdio, return error
		return _FDEV_ERR;

	static uint8_t rx_data;
	if (uart_rx_data(_uart_stdio_port,&rx_data,1) == 0)
		// The buffer was empty, so return an end of file
		return _FDEV_EOF;

	// If we got this far, we have a character to return
	return rx_data;
}

