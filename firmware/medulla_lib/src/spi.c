#include "spi.h"

// These get hard coded because they are the physical pins attached to the SPI hardware
#define SPI_CS 4
#define SPI_MOSI 5
#define SPI_MISO 6
#define SPI_CLK 7

/// Configures the buffers for a spi port
static void _spi_configure_buffers(volatile _spi_buffer_t *spi_buffer, uint8_t *tx_data, uint8_t tx_data_length, uint8_t *rx_data, uint8_t rx_data_length);

spi_port_t spi_init_port(PORT_t *spi_port, SPI_t *spi_register, bool uses_chip_select) {
	// Store values into the spi_port_t struct
	spi_port_t port;
	port.spi_port = spi_port;
	port.spi_register = spi_register;
	port.uses_chip_select = uses_chip_select;

	// Configure the IO directions for the SPI pins
	port.spi_port->DIRSET = (1<<SPI_MOSI) | (1<<SPI_CLK);
	port.spi_port->DIRCLR = 1<<SPI_MISO;
	port.spi_port->DIRSET = 1<<SPI_CS;	// CS needs to be an output, otherwise it the hardware might not transmit bytes

	if (uses_chip_select)
		// If we are using the CS line, then it should idle high
		port.spi_port->OUTSET = 1<<SPI_CS;

	port.transaction_underway = false;

	// Configure the SPI registers
	port.spi_register->CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV16_gc;
	port.spi_register->INTCTRL = SPI_INTLVL_MED_gc;
	
	return port;
}

inline int spi_start_transmit(volatile spi_port_t *spi_port, void *data, uint8_t data_length) {
	if (data_length == 0)	// Don't even bother transmitting
		return 0;
	// To start just a transmit, all we need to do is start a transmit_receive transaction with the rx_data_length set to zero.
	return spi_start_transmit_receive(spi_port, data, data_length, 0, 0);
}

inline int spi_start_receive(volatile spi_port_t *spi_port, void *data, uint8_t data_length) {
	if (data_length == 0) // Don't bother receiveing
		return 0;
	// To start just a receive, we just need to start a transmit/receive transaction with a tx_data_length set to zero.
	return spi_start_transmit_receive(spi_port, 0, 0, data, data_length);
}

int spi_start_transmit_receive(volatile spi_port_t *spi_port, void *tx_data, uint8_t tx_data_length, void *rx_data, uint8_t rx_data_length) {
	// If this spi_port already has a transfer underway, then returrn -1
	if (spi_port->transaction_underway == true)
		return -1;
	
	// If the hardware already has a transfer underway, then return -2. Otherwise, take cntrol of the hardware
	if (spi_port->spi_register == &SPIC) {
		if (_spi_buffer_c.spi_port != 0)
			return -2;
		else { 
			_spi_buffer_c.spi_port = spi_port;
			_spi_configure_buffers(&_spi_buffer_c, tx_data, tx_data_length, rx_data, rx_data_length);
		}
	}
	else if (spi_port->spi_register == &SPID) {
		if (_spi_buffer_d.spi_port != 0)
			return -2;
		else {
			_spi_buffer_d.spi_port = spi_port;
			_spi_configure_buffers(&_spi_buffer_d, tx_data, tx_data_length, rx_data, rx_data_length);
		}
	}
	else if (spi_port->spi_register == &SPIE) {
		if (_spi_buffer_e.spi_port != 0)
			return -2;
		else {
			_spi_buffer_e.spi_port = spi_port;
			_spi_configure_buffers(&_spi_buffer_e, tx_data, tx_data_length, rx_data, rx_data_length);
		}
	}
	else if (spi_port->spi_register == &SPIF) {
		if (_spi_buffer_f.spi_port != 0)
			return -2;
		else { 
			_spi_buffer_f.spi_port = spi_port;
			_spi_configure_buffers(&_spi_buffer_f, tx_data, tx_data_length, rx_data, rx_data_length);
		}
	}
	else
		// Something weird happend because spi_register no longer points to an SPI register, just exit nicely
		return -3;
	

	// Signal that a transaction is now in progress
	spi_port->transaction_underway = true;
	return 0;
}

static void _spi_configure_buffers(volatile _spi_buffer_t *spi_buffer, uint8_t *tx_data, uint8_t tx_data_length, uint8_t *rx_data, uint8_t rx_data_length) {

        // set output bufffer address and length
        spi_buffer->tx_buffer = tx_data;
        spi_buffer->tx_buffer_size = tx_data_length;
        
        // set the input buffer address and length
        spi_buffer->rx_buffer = rx_data;
        spi_buffer->rx_buffer_size = rx_data_length;

        // set buffer position to beginning
        spi_buffer->io_buffer_position = 0;
        
	// Set the data output register to start the transaction rolling. If there is no data to send, send a zero.       
        if (spi_buffer->tx_buffer_size == 0)
                spi_buffer->spi_port->spi_register->DATA = 0;
        else
                spi_buffer->spi_port->spi_register->DATA = spi_buffer->tx_buffer[0];

}

inline void spi_assert_cs(volatile spi_port_t *spi_port) {
	if (spi_port->uses_chip_select)
		spi_port->spi_port->OUTCLR = (1<<SPI_CS);
}

inline void spi_deassert_cs(volatile spi_port_t *spi_port) {
	if (spi_port->uses_chip_select)
		spi_port->spi_port->OUTSET = (1<<SPI_CS);
}

