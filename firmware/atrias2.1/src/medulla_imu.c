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
	*imu_error_flags_pdo = 0;

	#if defined DEBUG_LOW || defined DEBUG_HIGH
	printf("[Medulla IMU] Initilizing IMU with ID: %04x\n",id);
	#endif

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initilizing sync managers\n");
	#endif // DEBUG_HIGH
	ecat_init_sync_managers(ecat_slave, rx_sm_buffer, MEDULLA_IMU_OUTPUTS_SIZE, 0x1000, tx_sm_buffer, MEDULLA_IMU_INPUTS_SIZE, 0x2000);

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initilizing PDO entries\n");
	#endif // DEBUG_HIGH
	ecat_configure_pdo_entries(ecat_slave, imu_rx_pdos, MEDULLA_IMU_RX_PDO_COUNT, imu_tx_pdos, MEDULLA_IMU_TX_PDO_COUNT);

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initilizing UART\n");
	#endif
	imu_port = uart_init_port(&PORTF, &USARTF0, uart_baud_921600, imu_tx_buffer, KVH_TX_BUFFER_LENGTH, imu_rx_buffer, KVH_RX_BUFFER_LENGTH);
	uart_connect_port(&imu_port, false);

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initilizing Master Sync pin\n");
	#endif
	msync_pin = io_init_pin(&PORTF, 1);
	PORTF.DIR = PORTF.DIR | (1<<1);   // TODO: Fix GPIO library and use io_set_direction().

	*master_watchdog = imu_counter_pdo;
	*packet_counter = imu_medulla_counter_pdo;
	*imu_medulla_id_pdo = id;
	*commanded_state = imu_command_state_pdo;
	*current_state = imu_current_state_pdo;

	crc_generate_table();
	#ifdef DEBUG_HIGH
	uint8_t i;
	for (i=0; i<20; i++) {
		printf("CRC table entry %02d: %08lx\n", i, crc_table[i]);
	}

	// Test CRC with packet from IMU.
	uint8_t str[] = {0xfe,0x81,0xff,0x55,0xb6,0x06,0xd8,0x1e,0xb6,0xc4,0x94,0x0f,0x36,0xd9,0x5a,0x83,0x3c,0x61,0xad,0x86,0x3b,0xe5,0xd0,0x86,0x3f,0x7f,0xdc,0xd4,0x77,0x72,0x00,0x26};
	uint32_t crc = 0xc0f2d540;
	uint8_t len = 32;

	printf("CRC %08lx compare result: %1x\n", crc_calc(str, len), is_packet_good(crc_calc(str, len), crc));
	#endif
}

void imu_enable_outputs(void) {}

void imu_disable_outputs(void) {}

void imu_update_inputs(uint8_t id) {
	static uint8_t last_seq = 0;

	// Flush buffer.
	uart_rx_data(&imu_port, imu_packet, uart_received_bytes(&imu_port));

	// Trigger Master Sync. IMU will assert TOV_Out 300 ns after this.
	PORTF.OUT |= (1<<1);   // TODO: Fix GPIO library so we can use io_set_output.
	_delay_us(60);   // This should be at least 30 us. I think the Medulla has trouble actually waiting 30 us, so here's 60 instead.
	PORTF.OUT &= ~(1<<1);   // TODO: Fix GPIO library.

	// TODO(yoos): Waiting for the buffer to fill up with 36 bytes would be the
	// right way to do this, but this doesn't work right now.
	//while (uart_received_bytes(&imu_port) < 36);   // Wait for entire packet.

	// Instead, we can just wait for the worst case delay between TOV_Out
	// assertion and the beginning of IMU packet transmission (around 80 us).
	// Not waiting here long enough will cause packet corruption.
	_delay_us(60);
	uart_rx_data(&imu_port, imu_packet, uart_received_bytes(&imu_port));

	// Populate data from IMU. Refer to p. 10 in manual for data locations.
	populate_byte_to_data(&(imu_packet[4]), XAngDelta_pdo);   // XAngDelta
	populate_byte_to_data(&(imu_packet[8]), YAngDelta_pdo);   // YAngDelta
	populate_byte_to_data(&(imu_packet[12]), ZAngDelta_pdo);   // ZAngDelta
	populate_byte_to_data(&(imu_packet[16]), XAccel_pdo);   // XAccel
	populate_byte_to_data(&(imu_packet[20]), YAccel_pdo);   // YAccel
	populate_byte_to_data(&(imu_packet[24]), ZAccel_pdo);   // ZAccel
	*Status_pdo = imu_packet[28];   // Status
	*Seq_pdo = imu_packet[29];   // Seq
	*Temp_pdo = ((int16_t)imu_packet[30])<<8 | ((int16_t)imu_packet[31]);   // Temp
	populate_byte_to_data(&(imu_packet[32]), CRC_pdo);   // CRC

	// Check for duplicate packet by comparing sequence number.
	if (*Seq_pdo == last_seq) {
		#ifdef DEBUG_HIGH
		printf(".");
		#endif
		*imu_error_flags_pdo |= (1<<ERROR_FLAG_DUP_PACKET);
	}
	else {
		*imu_error_flags_pdo &= ~(1<<ERROR_FLAG_DUP_PACKET);
	}
	last_seq = *Seq_pdo;

	// Check CRC.
	if (!is_packet_good(crc_calc(imu_packet, 32), *CRC_pdo)) {
		*imu_error_flags_pdo |= (1<<ERROR_FLAG_BAD_CRC);
	}
	else {
		*imu_error_flags_pdo &= ~(1<<ERROR_FLAG_BAD_CRC);
	}

	// Check that we see the expected header.
	if ((uint32_t) imu_packet[0] != 0xfe81ff55) {
		#if defined(DEBUG_LOW) || defined(DEBUG_HIGH)
		printf("[Medulla IMU] Malformed packet header error.\n");
		#endif
		*imu_error_flags_pdo |= (1<<ERROR_FLAG_MALFORMED_HEADER);
	}
	else {
		*imu_error_flags_pdo &= ~(1<<ERROR_FLAG_MALFORMED_HEADER);
	}

	// Realtime-killing debug
	#ifdef DEBUG_HIGH
	// NOTE this will corrupt IMU packets at 1 kHz! You know, real-time and
	// yadda yadda.
	static uint8_t counter = 0;
	if (counter == -1) {
		printf("[Medulla IMU] Seq: %3u   Gyro: %08lx %08lx %08lx   Acc: %08lx %08lx %08lx  Status: %2x  Temp: %2d  CRC: %08lx\n",
				*Seq_pdo,
				*XAngDelta_pdo,
				*YAngDelta_pdo,
				*ZAngDelta_pdo,
				*XAccel_pdo,
				*YAccel_pdo,
				*ZAccel_pdo,
				*Status_pdo,
				*Temp_pdo,
				*CRC_pdo
				);
	}
	counter = (counter+1) % 10;
	#endif // DEBUG_HIGH
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
	static uint8_t bad_seq_counter = 0;

	// Count duplicate packets and complain if we're duplicating more than
	// half.
	if ((*imu_error_flags_pdo >> ERROR_FLAG_DUP_PACKET) & 1) {
		bad_seq_counter++;
		if (bad_seq_counter == 100) {
			#if defined(DEBUG_LOW) || defined(DEBUG_HIGH)
			printf("[Medulla IMU] Duplicate packet error.\n");
			#endif
			return true;
		}
	}
	else if (bad_seq_counter > 0) {
		bad_seq_counter--;
	}

	// On bad CRC, just set the error flag and don't complain here, as it will
	// very likely coincide with a bad sequence number, for which we already
	// have a counter.
	if ((*imu_error_flags_pdo >> ERROR_FLAG_BAD_CRC) & 1) {
	}

	// Check that we see the expected header. Because we clear the UART buffer
	// before triggering MSync, I can't imagine how we would ever encounter
	// this failure mode, so if we do, something really bad must be going on,
	// so complain.
	if ((*imu_error_flags_pdo >> ERROR_FLAG_MALFORMED_HEADER) & 1) {
		return true;
	}

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

