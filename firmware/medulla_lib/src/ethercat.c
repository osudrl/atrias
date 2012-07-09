#include "ethercat.h"

ecat_slave_t ecat_init_slave(PORT_t *spi_port, SPI_t *spi_register, io_pin_t eeprom_pin, io_pin_t irq_pin) {
	ecat_slave_t slave;
	
	// Initilize the SPI port
	slave.spi_port = spi_init_port(spi_port, spi_register, true);

	// Initilize IO pins
	io_set_direction(eeprom_pin, io_input);
	io_set_direction(irq_pin, io_input);

	// Wait for eeprom to be loaded
	while(io_get_input(eeprom_pin) == io_low);

	return slave;
}

void ecat_init_sync_managers(ecat_slave_t *ecat_slave, void *rx_sm_buffer, uint16_t rx_sm_size, uint16_t rx_sm_phy_address, void * tx_sm_buffer, uint16_t tx_sm_size, uint16_t tx_sm_phy_address) {

	ecat_slave->rx_sm_buffer = rx_sm_buffer;
	ecat_slave->tx_sm_buffer = tx_sm_buffer;

	// If the sync managers fit, then we can set the sync manager buffer sizes to the sm sizes
	ecat_slave->rx_sm_size = rx_sm_size;
	ecat_slave->tx_sm_size = tx_sm_size;

	// Now get the sync manager addresses
	ecat_slave->rx_sm_phy_addr = rx_sm_phy_address;
	ecat_slave->tx_sm_phy_addr = tx_sm_phy_address;
}

void ecat_configure_pdo_entries(ecat_slave_t *ecat_slave, ecat_pdo_entry_t rx_pdo_entries[], uint8_t rx_entry_count, ecat_pdo_entry_t tx_pdo_entries[], uint8_t tx_entry_count) {

	uint8_t *current_pointer = ecat_slave->rx_sm_buffer;
	for (int pdo_cnt = 0; pdo_cnt < rx_entry_count; pdo_cnt++) {
		*(rx_pdo_entries[pdo_cnt].pdo_entry_pointer_address) = current_pointer;
		current_pointer += rx_pdo_entries[pdo_cnt].entry_size;
	}

	current_pointer = ecat_slave->tx_sm_buffer;
	for (int pdo_cnt = 0; pdo_cnt < tx_entry_count; pdo_cnt++) {
		*(tx_pdo_entries[pdo_cnt].pdo_entry_pointer_address) = current_pointer;
		current_pointer += tx_pdo_entries[pdo_cnt].entry_size;
	}
}



void ecat_read_rx_sm(ecat_slave_t *ecat_slave) {
	_ecat_read_register(ecat_slave, ecat_slave->rx_sm_phy_addr,ecat_slave->rx_sm_buffer, ecat_slave->rx_sm_size);
}

void ecat_write_tx_sm(ecat_slave_t *ecat_slave) {
	_ecat_write_register(ecat_slave, ecat_slave->tx_sm_phy_addr, ecat_slave->tx_sm_buffer, ecat_slave->tx_sm_size);
}

void ecat_update_status(ecat_slave_t *ecat_slave) {
	uint8_t status;
	_ecat_read_register(ecat_slave,_ECAT_AL_CONTROL,&status,1);
	_ecat_write_register(ecat_slave,_ECAT_AL_STATUS,&status,1);
}

uint16_t _ecat_read_register(ecat_slave_t *ecat_slave, uint16_t reg_addr, void *buffer, uint8_t length) {
	uint16_t ecat_status;
	static uint8_t end_byte = 0xFF;

	spi_assert_cs(&(ecat_slave->spi_port));

	// First, write out the 16 bit address and read
	ecat_status = _ecat_write_address(ecat_slave,reg_addr,_ECAT_READ_CMD);

	// Start reading all the data except the last byte
	spi_start_receive(&(ecat_slave->spi_port),buffer,length-1);

	// wait while there is still a spi_transaction is underway
	while (ecat_slave->spi_port.transaction_underway);
	
	spi_start_transmit_receive(&(ecat_slave->spi_port),&end_byte,1,buffer+length-1,1);	
	while (ecat_slave->spi_port.transaction_underway);

	spi_deassert_cs(&(ecat_slave->spi_port));
	
	// return the status
	return ecat_status;
}

uint16_t  _ecat_write_register(ecat_slave_t *ecat_slave, uint16_t reg_addr, void *buffer, uint8_t length) {
	uint16_t ecat_status; 

	spi_assert_cs(&(ecat_slave->spi_port));
	
	// First, write out the 16 bit address and read
	ecat_status = _ecat_write_address(ecat_slave,reg_addr,_ECAT_WRITE_CMD);

	// Now strt send in the data
	spi_start_transmit(&(ecat_slave->spi_port),buffer,length);

	// If we are blocking and there is still an spi_transaction is underway
	while (ecat_slave->spi_port.transaction_underway);

	spi_deassert_cs(&(ecat_slave->spi_port));

	// return the status
	return ecat_status;
}

uint16_t _ecat_write_address(ecat_slave_t *ecat_slave, uint16_t address, uint8_t command) {
	uint8_t out_data[4] = {0,0,0xFF,0xFF};
	uint8_t addr_data_length;
	uint16_t ecat_status;
	
	out_data[0] = (uint8_t)(address>>5);

	if (address>0xFFF) {
		out_data[1] = ((uint8_t)(address&0x1F)<<3) | _ECAT_ADDREX_CMD;
		out_data[2] = ((uint8_t)(address>>8)&0xE0) | (command<<2);
		addr_data_length = 3;
	}
	else {
		out_data[1] = ((uint8_t)(address&0x1F)<<3) | command;
		addr_data_length = 2;
	}

	// If we are transmitting, we need to send an extra byte to wait for the ESC to ge ready
	if (command == _ECAT_READ_CMD) {
		out_data[addr_data_length] = 0xFF;
		addr_data_length += 1;
	}

	spi_start_transmit_receive(&(ecat_slave->spi_port),out_data,addr_data_length,&ecat_status,2);
	while (ecat_slave->spi_port.transaction_underway);

	// For some reason we need to flip the byte order
	return (ecat_status<<8) | (ecat_status>>8);
}

