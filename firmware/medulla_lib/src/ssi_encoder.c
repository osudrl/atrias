#include "ssi_encoder.h"

#define _SSI_ENCODER_SEND_CLOCK \
encoder->spi_port.spi_port->OUTCLR = (1<<7); \
_delay_us(0.125);        /* Half of the 2 Mhz period*/ \
encoder->spi_port.spi_port->OUTSET = (1<<7); \
_delay_us(0.125);        /* Other half of the 2 Mhz period*/

ssi_encoder_t ssi_encoder_init(PORT_t *spi_port, SPI_t *spi_register, void *timestamp_timer, uint32_t *data_pointer, uint8_t data_length, uint16_t *timestamp_pointer) {

	ssi_encoder_t encoder;

	// setup the encoder struct
	encoder.spi_port = spi_init_port(spi_port, spi_register, false);
	encoder.timestamp_timer = (TC0_t*)timestamp_timer;
	encoder.data_pointer = data_pointer;
	encoder.data_length = data_length;
	encoder.timestamp_pointer = timestamp_pointer;
	encoder.input_buffer[0] = 0;
	encoder.input_buffer[1] = 0;
	encoder.input_buffer[2] = 0;
	encoder.input_buffer[3] = 0;

	// The spi driver defaults to the wrong SPI mode, so we switch it now
	spi_register->CTRL = (spi_register->CTRL & ~SPI_MODE_gm) | SPI_MODE_2_gc;

	// Then just return the encoder
	return encoder;
}

int ssi_encoder_start_reading(ssi_encoder_t *encoder) {
	// first check that we are not already reading
	if (ssi_encoder_read_complete(encoder) == false)
		return -1;

	// Check that the encoder is ready to be read from. MISO has be high
	if ((encoder->spi_port.spi_port->IN & (1<<6)) == 0)
		return -2;

	//We need to figure out how many extra bits we need to clock in at the beginning
	uint8_t extra_bits = encoder->data_length % 8;
	uint8_t data_bytes = encoder->data_length / 8;

	// clear out the data buffer
	encoder->input_buffer[0] = 0;
	encoder->input_buffer[1] = 0;
	encoder->input_buffer[2] = 0;
	encoder->input_buffer[3] = 0;


	// Now we need to manually send out the first part of the BISS packet
	// so we have to first disable the SPI driver
	encoder->spi_port.spi_register->CTRL &= ~SPI_ENABLE_bm;
	
	// First record the start time, and then send start bit
	cli();
	*(encoder->timestamp_pointer) = encoder->timestamp_timer->CNT;
	_SSI_ENCODER_SEND_CLOCK
	sei();
	
	encoder->spi_port.spi_port->OUTCLR = (1<<7); 
        _delay_us(0.0625);
	while (extra_bits != 0) {
		encoder->spi_port.spi_port->OUTCLR = (1<<7);
		// sample the bit
		encoder->input_buffer[3-data_bytes] |= (encoder->spi_port.spi_port->IN & (1<<6)) >> (7-extra_bits--);
		encoder->spi_port.spi_port->OUTSET = (1<<7);
	        _delay_us(0.0625);
	}

	encoder->spi_port.spi_register->CTRL |= SPI_ENABLE_bm;
	spi_start_receive(&(encoder->spi_port), encoder->input_buffer + 4 - (encoder->data_length/8),encoder->data_length/8);

	// At this point the clock generator will run. When an Ack is received by the pin interrupt the ISR will start the SPI transfer
	return 0;
}

void ssi_encoder_process_data(ssi_encoder_t *encoder) {
	// Fill the data pointer. Since the xMega is little-endian, we have to swap the byte order, by shiffing the data in one byte at a time.
	*(encoder->data_pointer) = ((uint32_t)encoder->input_buffer[0]) << 24 |
	                           ((uint32_t)encoder->input_buffer[1]) << 16 |
	                           ((uint32_t)encoder->input_buffer[2]) <<  8 |
	                           ((uint32_t)encoder->input_buffer[3]);

}

bool ssi_encoder_read_complete(ssi_encoder_t *encoder) {
	return !(encoder->spi_port.transaction_underway);
}


