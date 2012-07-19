#include "biss.h"

#define _BISS_SEND_CLOCK \
encoder->spi_port.spi_port->OUTCLR = (1<<7); \
_delay_us(0.125);        /* Half of the 2 Mhz period*/ \
encoder->spi_port.spi_port->OUTSET = (1<<7); \
_delay_us(0.125);        /* Other half of the 2 Mhz period*/

biss_encoder_t biss_init_encoder(PORT_t *spi_port, SPI_t *spi_register, void *clock_timer, void *timestamp_timer, uint16_t cnt_per_us, uint32_t *data_pointer, uint16_t *timestamp_pointer) {

	biss_encoder_t encoder;

	// setup the encoder struct
	encoder.spi_port = spi_init_port(spi_port, spi_register, false);
	encoder.clock_timer = (TC0_t*)clock_timer;
	encoder.timestamp_timer = (TC0_t*)timestamp_timer;
	encoder.cnt_per_us = cnt_per_us;
	encoder.data_pointer = data_pointer;
	encoder.timestamp_pointer = timestamp_pointer;

	// The spi driver defaults to the wrong SPI mode, so we switch it now
	spi_register->CTRL = (spi_register->CTRL & ~SPI_MODE_gm) | SPI_MODE_2_gc;

	// Setup the timer and pin interrupts for automatic clock generation during ack
	encoder.clock_timer->INTCTRLA = TC_OVFINTLVL_HI_gc;
	encoder.clock_timer->PER = 2;
	encoder.spi_port.spi_port->INT1MASK = (1<<6);

	// Then just return the encoder
	return encoder;
}

int biss_start_reading(biss_encoder_t *encoder) {
	// first check that we are not already reading
	if (biss_read_complete(encoder) == false)
		return -1;

	// Check that the encoder is ready to be read from. MISO has be high
	if ((encoder->spi_port.spi_port->IN & (1<<6)) == 0)
		return -2;

	// Put the encoder pointer into the appropreate pointer variable for interrupts
	switch ((intptr_t)(encoder->spi_port.spi_port)) {
		case (intptr_t)(&PORTC): _biss_encoder_PORTC = encoder; break;
		case (intptr_t)(&PORTD): _biss_encoder_PORTD = encoder; break;
		case (intptr_t)(&PORTE): _biss_encoder_PORTE = encoder; break;
		case (intptr_t)(&PORTF): _biss_encoder_PORTF = encoder;
	}

	// Now we need to manually send out the first part of the BISS packet
	// so we have to first disable the SPI driver
	encoder->spi_port.spi_register->CTRL &= ~SPI_ENABLE_bm;


	// Send the two start bits. Since we need to record the time of the first rising edge of the clock we can't use the send clock macro on the first clock.
	encoder->spi_port.spi_port->OUTCLR = (1<<7);
	_delay_us(0.125);
	*(encoder->timestamp_pointer) = encoder->timestamp_timer->CNT; // Record the time of the rising edge
	encoder->spi_port.spi_port->OUTSET = (1<<7);
	_delay_us(0.125);
	_BISS_SEND_CLOCK // Now send the second clock

	// Now we need to wait for an ack while clocking, so we start the clock generation timer
	encoder->spi_port.spi_port->INTFLAGS = 1<<1;
	encoder->spi_port.spi_register->INTCTRL = PORT_INT1LVL_MED_gc;
	encoder->clock_timer->CTRLA = TC_CLKSEL_DIV8_gc;
	
	// At this point the clock generator will run. When an Ack is received by the pin interrupt the ISR will start the SPI transfer
	return 0;
}

uint8_t biss_process_data(biss_encoder_t *encoder) {
	// Fill the data pointer. Since the xMega is little-endian, we have to swap the byte order, by shiffing the data in one byte at a time.
	*(encoder->data_pointer) = ((uint32_t)encoder->input_buffer[0]) << 24 |
	                           ((uint32_t)encoder->input_buffer[1]) << 16 |
	                           ((uint32_t)encoder->input_buffer[2]) <<  8 |
	                           ((uint32_t)encoder->input_buffer[3]);

	*(encoder->timestamp_pointer) += 4*encoder->cnt_per_us; // Add 4 microseconds to the timestamp because the actual measurement is taken 4 microseconds after the first rising edge of the clock.

	// The last byte is the status byte, so we can just return that
	return encoder->input_buffer[4];
}

bool biss_read_complete(biss_encoder_t *encoder) {
	return !(encoder->spi_port.transaction_underway || encoder->spi_port.spi_register->INTCTRL == PORT_INT1LVL_MED_gc);
}

bool biss_data_valid(biss_encoder_t *encoder) {
	return biss_read_complete(encoder) && (encoder->input_buffer[4] & (1<<7));
}


