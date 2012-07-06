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
 *  @note The io buffer cannot be larger than 255 bytes, so any one data transfer
 *  cannot be larger than this.
 *
 *  @par
 *  @note The slave select pin will always be set as an output. This is the
 *  only way that the SPI hardware will go into SPI master mode. 
 *
 *  @par
 *  @note Because the spi_port_t struct can be modified inside the intrrupt
 *  context it is important to define all spi_port_t variables as volatile. If
 *  this is not done, gcc not see that the variable can change. This can cause
 *  problems in particular while waiting for the transaction_underway flag.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

/** @brief Stores all the information neccesary to use an SPI port
 *
 *  This struct is used by the SPI library to keep track of a port's
 *  configuration. It is possible to have two port structs defined for the same
 *  xMega SPI hardware. It is not possible to transmit or receive with both
 *  ports at the same time, using two different structs on the same hardware
 *  will work.
 */
typedef struct {
	PORT_t *spi_port; 			/**< IO port containing the SPI pins */
	SPI_t *spi_register;			/**< SPI register address */
	bool uses_chip_select;			/**< This bool specifies if the chip select should be used */
	bool transaction_underway;		/**< If there is currently an ongoing transaction */
} spi_port_t;

/** @brief Stores pointers and sizes of an SPI port's tx and rx buffers
 *
 *  When a SPI transaction is started, the information about the data buffers
 *  are stored in structs of this type. There is a global _spi_buffer_t struct
 *  defined for each hardware SPI port, which is connected to the interrupts for
 *  that hardware. This not only speeds up ISRs, but it also makes ensures that
 *  multiple SPI transactions cannot be started on the same hardware.
 */
typedef struct {
	uint8_t *tx_buffer;			/**< Pointer to the address of the tx buffer */
	uint8_t tx_buffer_size;			/**< Length of tx buffer, length must be < 255 bytes */
	uint8_t *rx_buffer;			/**< Pointer to the address of the rx buffer */
	uint8_t rx_buffer_size;			/**< Length of rx buffer, length must be < 255 bytes */
	uint8_t io_buffer_position;		/**< Current read/write position in the rx buffer */
	volatile spi_port_t *spi_port;
} _spi_buffer_t;

_spi_buffer_t _spi_buffer_c,	/**< @brief Struct for storing buffer information for SPIC */
              _spi_buffer_d,	/**< @brief Struct for storing buffer informatino for SPID */
              _spi_buffer_e,	/**< @brief Struct for storing buffer informatino for SPIE */
              _spi_buffer_f;	/**< @brief Struct for storing buffer informatino for SPIF */

/** @brief Macro that defines the SPI interrupt service routine
 *
 *  The interrupt handlers for all of the SPI interrupts are the same except for
 *  which hardware and _spi_buffer_t structs they reference. Because we want the
 *  the ISR to run as efficiently as possible the generic interupt handler is
 *  defined as macro. This removes function call overhead.
 *
 *  @param _SPI_REGISTER SPI_t register for the SPI port to use
 *  @param _SPI_BUFFER Globally defined _spi_buffer_t from which to get buffer
 *  information.
 */
#define _SPI_HANDLE_INTERRUPT(_SPI_REGISTER, _SPI_BUFFER) \
	/* we just send a bunch of clock pulses, if we were reading data, collect it into the buffer before incrementing*/ \
	if (_SPI_BUFFER.io_buffer_position < _SPI_BUFFER.rx_buffer_size) \
		_SPI_BUFFER.rx_buffer[_SPI_BUFFER.io_buffer_position] = _SPI_REGISTER.DATA; \
	\
	/* Just finished transmitting or receiving a byte, so increment buffer location */ \
	_SPI_BUFFER.io_buffer_position++; \
	\
	/* If we still need to send or receive data, then start the data transfer */ \
	if (_SPI_BUFFER.io_buffer_position < _SPI_BUFFER.tx_buffer_size) { \
		/* If there is data still to be transmitted, then write it to the output buffer */ \
		_SPI_REGISTER.DATA = _SPI_BUFFER.tx_buffer[_SPI_BUFFER.io_buffer_position]; \
	} \
	else if (_SPI_BUFFER.io_buffer_position < _SPI_BUFFER.rx_buffer_size) { \
		/* If there is no data to be transmitted, but there is still data to be read, send zeros until all the data is in */ \
		_SPI_REGISTER.DATA = 0; \
	} \
	else { \
		/* we are done with the transmission, clean up */ \
		_SPI_BUFFER.spi_port->transaction_underway = false; \
		_SPI_BUFFER.spi_port = 0; \
	} \
	

#ifdef SPI_USES_PORTC
ISR(SPIC_INT_vect) {
	_SPI_HANDLE_INTERRUPT(SPIC,_spi_buffer_c);
}
#endif

#ifdef SPI_USES_PORTD
ISR(SPID_INT_vect) {
	_SPI_HANDLE_INTERRUPT(SPID,_spi_buffer_d);
}
#endif

#ifdef SPI_USES_PORTE
ISR(SPIE_INT_vect) {
	_SPI_HANDLE_INTERRUPT(SPIE,_spi_buffer_e);
}
#endif

#ifdef SPI_USES_PORTF
ISR(SPIF_INT_vect) {
	_SPI_HANDLE_INTERRUPT(SPIF,_spi_buffer_f);
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
int spi_start_transmit(volatile spi_port_t *spi_port, void *data, uint8_t data_length);

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
int spi_start_receive(volatile spi_port_t *spi_port, void *data, uint8_t data_length);

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
int spi_start_transmit_receive(volatile spi_port_t *spi_port, void *tx_data, uint8_t tx_data_length, void *rx_data, uint8_t rx_data_length); 

/** @brief Asserts the Chip Select (Slave Select) pin if enabled
 *
 *  @param spi_port SPI port to assert CS pin on
 */
void spi_assert_cs(volatile spi_port_t *spi_port);

/** @brief Releases the Chip Select (Slave Select) pin if it is enabled
 *
 *  @param spi_port SPI port to deassert CS pin on
 */
void spi_deassert_cs(volatile spi_port_t *spi_port);

#endif //SPI_H
