#include <medulla_imu.h>
#include <crc.h>

//--- Define interrupt functions ---//

//--- Define ethercat PDO entries ---//

// RxPDO entries
medulla_state_t *imu_command_state_pdo;
uint16_t *imu_counter_pdo;

// TxPDO entries
uint8_t *imu_medulla_id_pdo;
medulla_state_t *imu_current_state_pdo;
uint8_t *imu_medulla_counter_pdo;
uint8_t *imu_error_flags_pdo;
uint32_t *XAngDelta_pdo;
uint32_t *YAngDelta_pdo;
uint32_t *ZAngDelta_pdo;
uint32_t *XAccel_pdo;
uint32_t *YAccel_pdo;
uint32_t *ZAccel_pdo;
uint8_t  *Status_pdo;
uint8_t  *Seq_pdo;
int16_t  *Temp_pdo;
uint32_t  *CRC_pdo;

ecat_pdo_entry_t imu_rx_pdos[] = {
	{((void**)(&imu_command_state_pdo)),1},
	{((void**)(&imu_counter_pdo)),2}
};

ecat_pdo_entry_t imu_tx_pdos[] = {
	{((void**)(&imu_medulla_id_pdo)),1},
	{((void**)(&imu_current_state_pdo)),1},
	{((void**)(&imu_medulla_counter_pdo)),1},
	{((void**)(&imu_error_flags_pdo)),1},
	{((void**)(&XAngDelta_pdo)),4},
	{((void**)(&YAngDelta_pdo)),4},
	{((void**)(&ZAngDelta_pdo)),4},
	{((void**)(&XAccel_pdo)),4},
	{((void**)(&YAccel_pdo)),4},
	{((void**)(&ZAccel_pdo)),4},
	{((void**)(&Status_pdo)),1},
	{((void**)(&Seq_pdo)),1},
	{((void**)(&Temp_pdo)),2},
	{((void**)(&CRC_pdo)),4}
};

void imu_initialize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter,TC0_t *timestamp_timer, uint16_t **master_watchdog) {
	#if defined DEBUG_LOW || defined DEBUG_HIGH
	printf("[Medulla IMU] Initializing IMU with ID: %04x\n",id);
	#endif

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initializing sync managers\n");
	#endif // DEBUG_HIGH
	ecat_init_sync_managers(ecat_slave, rx_sm_buffer, MEDULLA_IMU_OUTPUTS_SIZE, 0x1000, tx_sm_buffer, MEDULLA_IMU_INPUTS_SIZE, 0x2000);

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initializing PDO entries\n");
	#endif // DEBUG_HIGH
	ecat_configure_pdo_entries(ecat_slave, imu_rx_pdos, MEDULLA_IMU_RX_PDO_COUNT, imu_tx_pdos, MEDULLA_IMU_TX_PDO_COUNT);

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initializing UART\n");
	#endif
	imu_port = uart_init_port(&PORTD, &USARTD0, uart_baud_921600, imu_tx_buffer, IMU_TX_BUF_SZ, imu_rx_buffer, IMU_RX_BUF_SZ); // IMU communication over amplifier port
	uart_connect_port(&imu_port, false);

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initializing Master Sync pin\n");
	#endif
	io_init_pin(&PORTF, 1);         // This is the clock pin for the strain gauge connector (currently wired to the IMU master sync pin)
	PORTF.DIR = PORTF.DIR | (1<<1); // TODO: Fix GPIO library and use io_set_direction().

	*master_watchdog     = imu_counter_pdo;
	*packet_counter      = imu_medulla_counter_pdo;
	*imu_medulla_id_pdo  = id;
	*commanded_state     = imu_command_state_pdo;
	*current_state       = imu_current_state_pdo;

	crc_generate_table();
	#ifdef DEBUG_HIGH
	crc_debug_print_table();
	crc_debug_check_crc();
	#endif
}

void imu_enable_outputs(void) {}

void imu_disable_outputs(void) {}

void imu_process_data(void) {
	// First verify that the header is intact. If not, then the data was bad and we should leave that data as-is.
	// This will be caught on the master because the sequence value will stay the same
	if (imu_packet[0] != 0xFE || imu_packet[1] != 0x81 || imu_packet[2] != 0xFF || imu_packet[3] != 0x55) {
		*imu_error_flags_pdo |= ERROR_FLAG_HEADER;
		return;
	}

	// Also check if the CRC matches the expected value
	populate_byte_to_data(&(imu_packet[32]), CRC_pdo);

	if (!is_packet_good(crc_calc(imu_packet, CRC_PAYLD_SZ), *CRC_pdo)) {
		*imu_error_flags_pdo |= ERROR_FLAG_CRC;
		return;
	}

	// The data seems good. Populate the PDOs
	populate_byte_to_data(&(imu_packet[4]),  XAngDelta_pdo);                  // XAngDelta
	populate_byte_to_data(&(imu_packet[8]),  YAngDelta_pdo);                  // YAngDelta
	populate_byte_to_data(&(imu_packet[12]), ZAngDelta_pdo);                  // ZAngDelta
	populate_byte_to_data(&(imu_packet[16]), XAccel_pdo);                     // XAccel
	populate_byte_to_data(&(imu_packet[20]), YAccel_pdo);                     // YAccel
	populate_byte_to_data(&(imu_packet[24]), ZAccel_pdo);                     // ZAccel
	*Status_pdo = imu_packet[28];                                             // Status
	*Seq_pdo    = imu_packet[29];                                             // Seq
	*Temp_pdo   = ((int16_t)imu_packet[30])<<8 | ((int16_t)imu_packet[31]);   // Temp
	*Status_pdo = 38;
}

void imu_update_inputs(uint8_t id) {
	// Reset the error flags so they can be easily updated this iteration.
	*imu_error_flags_pdo = 0;

	// Receive the data sent during the last iteration.
	// Check that we've received the correct amount of data.
	if (uart_rx_data(&imu_port, imu_packet, KVH_MSG_SIZE) == KVH_MSG_SIZE) {
		// We have received (at least) the right amount of data, go ahead and process it.
		imu_process_data();
	} else {
		*imu_error_flags_pdo |= ERROR_FLAG_PAYLD_SZ;
	}
}

void imu_post_ecat(void) {
	// Trigger Master Sync. This will cause the IMU to output data for the next iteration
	PORTF.OUT |= (1<<1);  // TODO: Fix GPIO library so we can use io_set_output.
	_delay_us(40);        // The KVH manual requires at least 30 microseconds. I'll do 40 here just to be safe.
	PORTF.OUT &= ~(1<<1); // TODO: Fix GPIO library.
}

bool imu_run_halt(uint8_t id)
{
	return true;
}

void imu_update_outputs(uint8_t id)
{
}

inline void imu_estop(void) {
	*imu_error_flags_pdo |= medulla_error_estop;
}

bool imu_check_error(uint8_t id) {
	return false;
}

bool imu_check_halt(uint8_t id) {
	return false;
}

void imu_reset_error(void) {
	*imu_error_flags_pdo = 0;
}

/* NOTE this obviously assumes 4-byte block */
void populate_byte_to_data(const uint8_t* data_byte, uint32_t* data) {
	*(((uint8_t*)data)+3) = *(data_byte++);
	*(((uint8_t*)data)+2) = *(data_byte++);
	*(((uint8_t*)data)+1) = *(data_byte++);
	*(((uint8_t*)data)+0) = *(data_byte);
}

