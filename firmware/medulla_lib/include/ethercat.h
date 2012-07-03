#ifndef ETHERCAT_H
#define ETHERCAT_H


/// Driver for using the EtherCAT slave chip on the medulla board
/** The ET1100 EtherCAT slave chip on the medulla board interfaces with the
 *  medulla over an SPI interface. This driver handles the SPI communication
 *  with the slave. It also handles mapping memory locations in xMega memory to
 *  the memory locations of PDOs on the EtherCAT slave chip.
 */

#include <avr/io.h>
#include "io_pin.h"

/// This struct describes the hardware connections to the ethercat chip

typedef struct {
	PORT_t *port_reg;
	SPI_t *spi_reg;
	io_pin_t eeprom_pin;
	io_pin_t irq_pin
} ec_HWConfig_t

#endif // ETHERCAT_H
