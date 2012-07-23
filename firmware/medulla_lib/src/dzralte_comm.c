#include "dzralte_comm.h"
#include "stdio.h"

uint16_t const _dzralte_crc_lookup_table[256] = {
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

dzralte_message_t* dzralte_generate_message(dzralte_message_list_t *message_list, uint8_t address, uint8_t sequence, dzralte_command_type_t command, uint8_t index, uint8_t offset, void *data_buffer, uint16_t data_length) {
	// Configure the struct
	message_list->message[sequence].sequence = sequence;	
	message_list->message[sequence].waiting_for_response = false;
	message_list->message[sequence].data_length = data_length;
	message_list->message[sequence].data_buffer = data_buffer;

	// Assemble the header data
	message_list->message[sequence].command_header[0] = _DZRALTE_SOF;
	message_list->message[sequence].command_header[1] = address;
	message_list->message[sequence].command_header[2] = (sequence & 0xF)<<2 | command;
	message_list->message[sequence].command_header[3] = index;
	message_list->message[sequence].command_header[4] = offset;
	message_list->message[sequence].command_header[5] = data_length/2;

	// Generate header CRC
	uint16_t accumulator = 0;
	for (uint8_t byte = 0; byte < 6; byte++)
		accumulator = (accumulator << 8) ^ _dzralte_crc_lookup_table[(accumulator >> 8) ^ message_list->message[sequence].command_header[byte]];

	message_list->message[sequence].command_header[6] = accumulator >> 8;
	message_list->message[sequence].command_header[7] = accumulator & 0xFF;

	// Generate data section CRC
	accumulator = 0;
	for (uint8_t byte = 0; byte < data_length; byte++)
		accumulator = (accumulator << 8) ^ _dzralte_crc_lookup_table[(accumulator >> 8) ^ message_list->message[sequence].data_buffer[byte]];
	
	message_list->message[sequence].data_CRC = accumulator;

	return &(message_list->message[sequence]);
}

void dzralte_send_message(dzralte_message_list_t *message_list, dzralte_message_t *message, uint8_t sequence, uart_port_t *port, bool regenerate_data_CRC) {
	// Check if we were passed a pointer, if we weren't, then set the pointer to the correct message
	if (message == 0)
		message = &(message_list->message[sequence]);

	// Check if we need to regenerate the CRC, if we do, then generate it
	if (regenerate_data_CRC) {
		uint16_t accumulator = 0;
		for (uint8_t byte = 0; byte < message->data_length; byte++)
			accumulator = (accumulator << 8) ^ _dzralte_crc_lookup_table[(accumulator >> 8) ^ message->data_buffer[byte]];
		message->data_CRC = accumulator;
	}

	// Now put the message into the transmit buffer
	uart_tx_data(port,message->command_header,8);
	uart_tx_data(port,message->data_buffer,message->data_length);
	uart_tx_byte(port,message->data_CRC>>8);
	uart_tx_byte(port,message->data_CRC & 0xFF);

	// Set the waiting for response variable
	message->waiting_for_response = true;
}

uint8_t dzralte_check_responses(dzralte_message_list_t *message_list, uart_port_t *port) {
	// First check that the first byte in the buffer is the SOF char, if it's not, throw out bytes until there is one
	while ((uart_rx_peek(port,0) != _DZRALTE_SOF) && uart_received_bytes(port) != 0)
		// pull the char out of the buffer
		uart_rx_byte(port);

	int message_counter = 0;
	// now read from the receive buffer until there is no more
	while (1) {	// This infinite loop might be a bad idea, but it shouldn't actually ever be infinite
		// Check that the whole message is in the buffer
		if (uart_received_bytes(port) < 8)
			// The whole header isn't even in the buffer, so just return
			return message_counter;
		
		if (uart_received_bytes(port) < (8 + (uart_rx_peek(port,5) ? uart_rx_peek(port,5)+2 : 0)))
			// The header is in the buffer, but the data section isn't, so return
			return message_counter;

		// Now that we know the message is here, get the sequence
		uint8_t sequence = uart_rx_peek(port,2)>>2;
	
		// check that the sequence is waiting for a response
		if (!message_list->message[sequence].waiting_for_response)
			sequence = 16; // we will read into this useless buffer just to get it out of the serial buffer
		else
			message_counter+=1;
	
		// Pull the header out
		uart_rx_data(port,message_list->message[sequence].response_header,8);
	
		// Now pull out the data section
		uart_rx_data(port,message_list->message[sequence].data_buffer,message_list->message[sequence].response_header[5]+2);
	
		// Signal that the message was received
		message_list->message[sequence].waiting_for_response = false;
	}

	return message_counter;
}

bool dzralte_response_received(dzralte_message_list_t *message_list, dzralte_message_t *message, uint8_t sequence) {
	// Get pointer to the message
	if (message == 0)
		message = &(message_list->message[sequence]);
	return !message->waiting_for_response;
}

