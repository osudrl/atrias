#ifndef SPI_H
#define SPI_H

/** @file 
 *  @brief This library provides a SPI driver for the hardware SPI ports on the xMega.
 * 
 *  The SPI library is an interrupt driven interface for the SPI hardware on the
 *  xMega. The functions are non-blocking, so the user application can continue
 *  to run while SPI communications are running. Callbacks are provided on
 *  conpletion of transmit and receive.
 *
 *  NOTE: The io buffer cannot be larger than 255 bytes, so any one data transfer
 *  cannot be larger than this.
 * 
 *  NOTE 2: The slave select pin will always be set as an output. This is the
 *  only way that the SPI hardware will go into SPI master mode. 
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

// These get hard coded because they are the physical pins attached to the SPI hardware
#define SPI_CS 4
#define SPI_MOSI 5
#define SPI_MISO 6
#define SPI_CLK 7

/// This is a typedef for a function pointer to a callback function.
typedef void (*spi_callback_t)(void);

/// This struct stores all the conficuration data for an SPI port.
typedef struct {
	PORT_t *spi_port; 			/**< IO port containing the SPI pins */
	SPI_t *spi_register;			/**< SPI register address */
	bool uses_chip_select;			/**< This bool specifies if the chip select should be used */
	bool transaction_underway;		/**< If there is currently an ongoing transaction */
} spi_port_t;

// Stores pointers and sizes of an SPI port's tx and rx buffers
typedef struct {
	uint8_t *tx_buffer;			/**< Pointer to the address of the tx buffer */
	uint8_t tx_buffer_size;			/**< Length of tx buffer, length must be < 255 bytes */
	uint8_t *rx_buffer;			/**< Pointer to the address of the rx buffer */
	uint8_t rx_buffer_size;			/**< Length of rx buffer, length must be < 255 bytes */
	uint8_t io_buffer_position;		/**< Current read/write position in the rx buffer */
	spi_port_t *spi_port;
} _spi_buffer_t;

// Stores poitners to spi_port_t structs used inside interrupts
_spi_buffer_t _spi_buffer_c, _spi_buffer_d, _spi_buffer_e, _spi_buffer_f;

#define _SPI_HANDLE_INTERRUPT(spi_buffer) \
        PORTC.OUTSET = 1; \
        /* Just finished transmitting a byte, so increment buffer location */ \
        spi_buffer.io_buffer_position++; \
        /* we just read in some data, so we need to get it out of the input register */ \
        if (spi_buffer.io_buffer_position < spi_buffer.rx_buffer_size) \
                spi_buffer.rx_buffer[spi_buffer.io_buffer_position] = _SPI.DATA; \
        \
        /* If we still need to send or receive data, then start the data transfer */ \
        if (spi_buffer.io_buffer_position < spi_buffer.tx_buffer_size) { \
                /* If there is data still to be transmitted, then write it to the output buffer */ \
                _SPI.DATA = spi_buffer.tx_buffer[spi_buffer.io_buffer_position]; \
        } \
        else if (spi_buffer.io_buffer_position < spi_buffer.rx_buffer_size) { \
                /* If there is no data to be transmitted, but there is still data to be read, send zeros until all the data is in */ \
                _SPI.DATA = 0; \
        } \
        else { \
                /* we are done with the transmission, clean up */ \
                spi_buffer.spi_port->transaction_underway = false; \
                spi_buffer.spi_port = 0; \
        } \
        PORTC.OUTCLR = 1;\
        reti(); \

#ifdef SPI_USING_PORTC
ISR(SPIC_INT_vect, ISR_NAKED) {
        #define _SPI SPIC
        _SPI_HANDLE_INTERRUPT(_spi_buffer_c);
        #undef _SPI
}
#endif

#ifdef SPI_USING_PORTD
ISR(SPID_INT_vect,ISR_NAKED) {
        #define _SPI SPIF
        _SPI_HANDLE_INTERRUPT(_spi_buffer_d);
        #undef _SPI
}
#endif

#ifdef SPI_USING_PORTE
ISR(SPIE_INT_vect,ISR_NAKED) {
        #define _SPI SPIE
        _SPI_HANDLE_INTERRUPT(_spi_buffer_e);
        #undef _SPI
}
#endif

#ifdef SPI_USING_PORTF
ISR(SPIF_INT_vect,ISR_NAKED) {
        #define _SPI SPIF
        _SPI_HANDLE_INTERRUPT(_spi_buffer_f);
        #undef _SPI
}
#endif



/** @brief Initilizes a hardware SPI port on the xMega
 *
 *  This function sets up a spi_port_t struct and also initilizes the SPI
 *  hardware on the xMega. This function should always be used to generate the
 *  spi_port_t structs. If it is not used, then the hardware may not be
 *  configured correctly.
 *
 *  @param spi_port Pointer to PORT_t register struct the SPI port is on
 *  @param spi_register Pointer to the SPI_t register for the port
 *  @param uses_chip_select True if the chip select pin should be used, false, if it shouldn't
 *  @return spi_port_t for the newly configured SPI port
 */
spi_port_t spi_init_port(PORT_t *spi_port, SPI_t *spi_register, bool  uses_chip_select);

/** @brief Start a transmit SPI transaction
 *
 *  This function starts transmitting data_length bytes of data starting at
 *  *data.
 *
 *  @param spi_port Pointer to SPI port to transmit with
 *  @param data Pointer to output data buffer
 *  @param data_length Amount of data to transmit
 *  @return  0 - Sucessful
 *  @return -1 - SPI port is busy
 *  @return -2 - SPI hardware is busy
 */
int spi_start_transmit(spi_port_t *spi_port, void *data, uint8_t data_length);

/** @brief Start a receive SPI transaction
 *
 *  This function starts clocking in data_length number bytes of data and stores
 *  them at the data pointer.
 *
 *  @param spi_port Pointer to SPI port to transmit with
 *  @param data Pointer to input data buffer
 *  @param data_length Amount of data to clock in
 *  @return  0 - Sucessful
 *  @return -1 - SPI port is busy
 *  @return -2 - SPI hardware is busy
 */
int spi_start_receive(spi_port_t *spi_port, void *data, uint8_t data_length);

/** @brief Start a transmit/receive SPI transaction
 *
 *  This function starts trasnsmitting the data stored at tx_data and reading
 *  data into rx_data. Enough clock pulses will be generated to either transmit
 *  tx_data_length number of bytes or read in rx_data_length number of bytes,
 *  whichever is larger.
 *
 *  @param spi_port Pointer to SPI port to transmit with
 *  @param tx_data Pointer to output data buffer
 *  @param tx_data_length Amount of data to transmit
 *  @param rx_data Pointer to input data buffer
 *  @param rx_data_length Amount of data to clock in
 *  @return  0 - Sucessful
 *  @return -1 - SPI port is busy
 *  @return -2 - SPI hardware is busy
 */
int spi_start_transmit_receive(spi_port_t *spi_port, void *tx_data, uint8_t tx_data_length, void *rx_data, uint8_t rx_data_length); 

#endif //SPI_H
