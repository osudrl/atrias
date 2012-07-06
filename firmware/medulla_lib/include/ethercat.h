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

// Useful register addresses in ESC
#define _ECAT_AL_CONTROL 0x0120		/**< Address of AL Control register in ESC */
#define _ECAT_AL_STATUS 0x0130		/**< Addres of AL Status register in ESC */

// SPI protocol commands
#define _ECAT_READ_CMD 3		/**< Command to read from ESC */
#define _ECAT_WRITE_CMD 4		/**< Command to write to ESC */
#define _ECAT_ADDREX_CMD 6		/**< Command that tells the ECS that there is one more address byte. */

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
 *  @param spi_port Port that the EtherCAT slave chip is connected to
 *  @param spi_register SPI hardware that the EtherCAT slave chip is connected to
 *  @param eeprom_pin Pin that the EEPROM loaded pin is connected to
 *  @param irq_pin Pin that the interrupt request pin is connected to
 *  @return Returns a configured ecat_slave_t struct for the slave chip
 */
ecat_slave_t ecat_init_slave(PORT_t *spi_port, SPI_t *spi_register, io_pin_t eeprom_pin, io_pin_t irq_pin);

/** @brief Initilize the sync manager buffers
 *
 *  This function configures the sync managers for the a given ecat_slave_t
 *  struct. This function does not check the size of physical address of either
 *  of the sync managers. It is important that both of these are set correctly,
 *  otherwise the sync managers, and thus the PDOs cannot be read. Evem if only
 *  some of the PDOs are being used, the sync manager sizes must be equal to the
 *  configured sizes of the sync managers. If this is not correct, then the
 *  ethercat slave will throw out any PDO data sent to it.
 *
 *  @param ecat_slave Pointer to the EtherCAT slave struct to configure
 *  @param rx_sm_buffer Pointer to a buffer large enough to store the tx sync
 *  manager.
 *  @param rx_sm_size Size of the rx sync manager
 *  @param rx_sm_phy_address Address of the rx sync manager in the ESC's memory
 *  @param tx_sm_buffer Pointer to a buffer large enough to store the tx sync
 *  manager.
 *  @param tx_sm_size Size of the tx sync manager
 *  @param tx_sm_phy_address Address of the tx sync manager in the ECS's memory
 */
void ecat_init_sync_managers(ecat_slave_t *ecat_slave, void *rx_sm_buffer, uint16_t rx_sm_size, uint16_t rx_sm_phy_address, void * tx_sm_buffer, uint16_t tx_sm_size, uint16_t tx_sm_phy_address);

/** @brief Configure the pdo entry pointers
 *
 *  This function is passed an array of PDO entry structs. For each PDO entry,
 *  the PDO entry pointer is pointed at the proper address in the sync manager
 *  buffer. This function assumes that the number and order of pdo entries is
 *  the same as how the ESC is configured (either by the master or by the
 *  EEPROM). 
 *
 *  @param ecat_slave Pointer to EtherCAT slave struct to configure PDOs for
 *  @param rx_pdo_entries An array of PDO entry structs that are located on the
 *  rx sync manager to configure.
 *  @param rx_entry_count Number of PDO entries in the rx_pdo_entries array
 *  @param tx_pdo_entries Array of PDO entries to configure on the tx sync
 *  manager.
 *  @param tx_entry_count Number of PDO entries in the tx_pdo_entries array
 */
void ecat_configure_pdo_entries(ecat_slave_t *ecat_slave, ecat_pdo_entry_t rx_pdo_entries[], uint8_t rx_entry_count, ecat_pdo_entry_t tx_pdo_entries[], uint8_t tx_entry_count);

/** @brief Read the current rx sync manager and PDO data therein
 *
 *  Use this function to read all of current values for the RX sync manager into
 *  the xMega's memory, This will update all the values stored at the PDO
 *  pointers.
 *
 *  @param ecat_slave Pointer to EtherCAT slave struct to use
 */
void ecat_read_rx_sm(ecat_slave_t *ecat_slave);

/** @brief Write the tx sync manager data to the ESC.
 *
 *  This function will write the data currently stored in the xMega's tx sync
 *  manager buffer to the EtherCAT slave controller. This will only transfer the
 *  data to the slave controller, the EtherCAT master still has to send an
 *  EtherCAT packet before it receives the data.
 *
 *  @param ecat_slave Pointer to the EtherCAT slave struct to write TX SM of.
 */
void ecat_write_tx_sm(ecat_slave_t *ecat_slave);

/** @brief Copies value from AL Control register to AL Status register
 *
 *  If the ET1100 is not in Device Emulation mode, the xMega is responsible for
 *  updating the AL Status register. When the EtherCAT master requests a new
 *  slave state by writing to the AL Control register it expects the slave to
 *  respond by writting the value for this state to the AL Status register. If
 *  the xMega doesn't do this the ESC cannot go into the requested state, and it
 *  will go into an error state.
 *
 *  This function updates the AL Status register basted upon the value of the AL
 *  Control register. It reads AL Control and writes it to AL Status. This
 *  function needs to get called on a regular basis to keep the slave controller
 *  from going into an error state when the master requests a state change.
 *
 *  @param ecat_slave Pointer to EtherCAT slave struct to use
 */
void ecat_update_status(ecat_slave_t *ecat_slave);

/** @brief Read register address in EtherCAT slave
 *
 *  This function starts reading from the EtherCAT slave controller starting at
 *  reg_addr and continuing for length bytes. The data is written into memory
 *  staring at buffer.
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
 *  This function transferes length number of bytes from buffer into the
 *  EtherCAT slave registers. Writing start at reg_addr.
 *
 *  @param ecat_slave Pointer to ethecat slave struct
 *  @param reg_addr Address of register to write
 *  @param buffer Pointer to data to write to register
 *  @param length Amount of data to write to register
 *  @return Status returned by ethercat slave
 */
uint16_t _ecat_write_register(ecat_slave_t *ecat_slave, uint16_t reg_addr, void *buffer, uint8_t length);

/** @brief Sends an address to the ethercat slave, handling extended addresses
 *
 *  The ET1100 has a sligtly complex addressing system. This function manages
 *  sending different lengths of addresses depending on what the ESC is
 *  expecting. This function also sends an extra byte of 0xFF after a write
 *  command to prepare the slave for reading. This function is blocking.
 *
 *  @param ecat_slave Pointer to EtherCAT slave struct to address
 *  @param address EtherCAT slave register address to send address for
 *  @param command Command to send along with address
 *  @return Returns the 16 bit status value returned by the EtherCAT slave when
 *  writing the address.
 */
uint16_t _ecat_write_address(ecat_slave_t *ecat_slave, uint16_t address, uint8_t command);

#endif // ETHERCAT_H
