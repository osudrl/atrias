#ifndef ETHERCAT_H
#define ETHERCAT_H

/** @file
 *  @brief Driver for using the EtherCAT slave chip on the medulla board
 *
 *  The ET1100 EtherCAT slave chip on the medulla board interfaces with the
 *  medulla over an SPI interface. This driver handles the SPI communication
 *  with the slave. It also handles mapping memory locations in xMega memory to
 *  the memory locations of PDOs on the EtherCAT slave chip.
 */

#include <avr/io.h>
#include "io_pin.h"
#include "spi.h"

// Base addresses of the tx and rx sync managers
#define _ECAT_RX_SM_BASE 0x0800+16
#define _ECAT_TX_SM_BASE 0x0800+24

// Offsets from SM base addresses for registers
#define _ECAT_SM_ADDR_OFFSET 0x0
#define _ECAT_SM_LENGTH_OFFSET 0x02

// Ethercat commands
#define _ECAT_READ_CMD 3
#define _ECAT_WRITE_CMD 4
#define _ECAT_ADDREX_CMD 6

/** @brief Struct that defines the slave configuration.
 *  
 *  This struct keeps tract of the ethercat slave's configration. The rx and tx
 *  sync manager buffers must be allocated by the user and must be of the
 *  appropreate size. ecat_slave_init() checks for this, but after initilization
 *  the values are assumed to be correct. If the sync manager size is smaller
 *  than the sync manager size in the ESC, the ESC will throw out any data
 *  wrietten to the SM.
 */
typedef struct {
	volatile spi_port_t spi_port;		/**< Struct defining the SPI port used to communicate with ESC */
	io_pin_t eeprom_pin;		/**< Pin connected to EEPROM loaded output from ESC */
	io_pin_t irq_pin;		/**< Pin connected to the interrput request output on ESC */
	uint8_t * rx_sm_buffer;		/**< Pointer to buffer storing rx sync manager memory */
	uint16_t rx_sm_size;		/**< Size of the the rx sync manager */
	uint16_t rx_sm_phy_addr;	/**< Physical address of rx sync manager memory in ESC */
	uint8_t * tx_sm_buffer;		/**< Pointer to buffer storing tx sync manager memory */
	uint16_t tx_sm_size;		/**< Size of tx sync manager */
	uint16_t tx_sm_phy_addr;	/**< Physical address of tx sync manager memory in ESC */
} ecat_slave_t;

/** @brief Struct used to transmit information about PDO entries
 *
 *  This struct is used to pass information about the PDO entries into the
 *  ecat_config_pdos() function. The pdo_entry_pointer_address is stored as a
 *  pointer to a pointer so that the users can keep it's PDO entry addresses in
 *  it's own variables.
 */
typedef struct {
	void **pdo_entry_pointer_address;	/**< Pointer to a pointer that stores the address of the the PDO entry */
	uint8_t entry_size;			/**< Size of the pdo entry in bytes */
} ecat_pdo_entry_t;

/** @brief Initilizes ethercat slave
 *
 *  This function sets up an ecat_slate_t struct and configures the hardware for
 *  communications with the ESC chip. Before the slave can be used the sync
 *  managers need to be configured by calling ecat_configure_sync_managers().
 *
 *  @note
 *  If the ESC's eeprom has not been loaded, this function will block until the
 *  EEPROM has been loaded.
 *  
 *  @note
 *  The ecat_slave_t struct returned from this function should be put into a
 *  volatile variable. If it is not, then the ethercat code may not work. 
 */
ecat_slave_t ecat_init_slave(PORT_t *spi_port, SPI_t *spi_register, io_pin_t eeprom_pin, io_pin_t irq_pin);

/** @brief Initilize the sync manager buffers
 *
 *  This function configures the sync managers for the a given ecat_slave_t
 *  struct. If the buffers are not large enough to store the sync managers, then
 *  an error is returned.
 */
void ecat_init_sync_managers(ecat_slave_t *ecat_slave, void *rx_sm_buffer, uint16_t rx_sm_size, uint16_t rx_sm_phy_address, void * tx_sm_buffer, uint16_t tx_sm_size, uint16_t tx_sm_phy_address);

/** @brief Configure the pdo entry pointers
 *
 *  This function is passed an array of PDO entry structs. For each PDO entry,
 *  the PDO entry pointer is pointed at the proper address in the sync manager
 *  buffer. This function assumes that the number and order of pdo entries is
 *  the same as how the ESC is configured (either by the master or by the
 *  EEPROM). 
 */
void ecat_configure_pdo_entries(ecat_slave_t *ecat_slave, ecat_pdo_entry_t rx_pdo_entries[], uint8_t rx_entry_count, ecat_pdo_entry_t tx_pdo_entries[], uint8_t tx_entry_count);

/** @brief Read the current rx sync manager and PDO data therein
 */
void ecat_read_rx_sm(ecat_slave_t *ecat_slave);

/** @brief Write the tx sync manager PDOs
 */
void ecat_write_tx_sm(ecat_slave_t *ecat_slave);

void ecat_update_status(ecat_slave_t *ecat_slave);

/** @brief Read memory address in ESC
 *
 *  @param ecat_slave pointer to ethercat struct
 *  @param reg_addr address of register to read
 *  @param buffer pointer to where the register value should be stored
 *  @param length Amount of data to read
 *  @return Status returned by ethercat slave
 */
uint16_t _ecat_read_register(ecat_slave_t *ecat_slave, uint16_t reg_addr, void *buffer, uint8_t length);

/** @brief Write memory address in ESC
 *
 *  @param ecat_slave Pointer to ethecat slave struct
 *  @param reg_addr Address of register to write
 *  @param buffer Pointer to data to write to register
 *  @param length Amount of data to write to register
 *  @return Status returned by ethercat slave
 */
uint16_t _ecat_write_register(ecat_slave_t *ecat_slave, uint16_t reg_addr, void *buffer, uint8_t length);

/** @brief Sends an address to the ethercat slave, handling extended addresses
 */
uint16_t _ecat_write_address(ecat_slave_t *ecat_slave, uint16_t address, uint8_t command);

#endif // ETHERCAT_H
