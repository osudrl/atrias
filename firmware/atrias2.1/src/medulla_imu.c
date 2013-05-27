#include <medulla.h>
#include <medulla_imu.h>

//--- Define interrupt functions ---//
UART_USES_PORT(USARTF0)   // KVH 1750

//--- Define ethercat PDO entries ---//

#define SHIFT_1BYTE(X)	(X<<8)
#define SHIFT_2BYTE(X)	(X<<16)
#define SHIFT_3BYTE(X)	(X<<24)

uint32_t buffer[1000];
uint32_t counter = 0;

// RxPDO entries
medulla_state_t *imu_command_state_pdo;
uint16_t *imu_counter_pdo;

// TxPDO entries
uint8_t *imu_medulla_id_pdo;
medulla_state_t *imu_current_state_pdo;
uint8_t *imu_medulla_counter_pdo;
uint8_t *imu_error_flags_pdo;
uint32_t *XAngDelta_pdo;
uint32_t *XAccel_pdo;
uint32_t *YAngDelta_pdo;
uint32_t *YAccel_pdo;
uint32_t *ZAngDelta_pdo;
uint32_t *ZAccel_pdo;
uint8_t  *Status_pdo;
uint8_t  *Seq_pdo;
int16_t  *Temp_pdo;

ecat_pdo_entry_t imu_rx_pdos[] = {{((void**)(&imu_command_state_pdo)),1},
	{((void**)(&imu_counter_pdo)),2}};

ecat_pdo_entry_t imu_tx_pdos[] = {{((void**)(&imu_medulla_id_pdo)),1},
	{((void**)(&imu_current_state_pdo)),1},
	{((void**)(&imu_medulla_counter_pdo)),1},
	{((void**)(&imu_error_flags_pdo)),1},
	{((void**)(&XAngDelta_pdo)),4},
	{((void**)(&XAccel_pdo)),4},
	{((void**)(&YAngDelta_pdo)),4},
	{((void**)(&YAccel_pdo)),4},
	{((void**)(&ZAngDelta_pdo)),4},
	{((void**)(&ZAccel_pdo)),4},
	{((void**)(&Status_pdo)),1},
	{((void**)(&Seq_pdo)),1},
	{((void**)(&Temp_pdo)),2}};

void imu_initilize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter,TC0_t *timestamp_timer, uint16_t **master_watchdog) {
	*imu_error_flags_pdo = 0;

	#if defined DEBUG_LOW || defined DEBUG_HIGH
	printf("[Medulla IMU] Initilizing IMU with ID: %04x\n",id);
	#endif

	#ifdef ENABLE_ECAT

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initilizing sync managers\n");
	#endif // DEBUG_HIGH
	ecat_init_sync_managers(ecat_slave, rx_sm_buffer, MEDULLA_IMU_OUTPUTS_SIZE, 0x1000, tx_sm_buffer, MEDULLA_IMU_INPUTS_SIZE, 0x2000);

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initilizing PDO entries\n");
	#endif // DEBUG_HIGH
	ecat_configure_pdo_entries(ecat_slave, imu_rx_pdos, MEDULLA_IMU_RX_PDO_COUNT, imu_tx_pdos, MEDULLA_IMU_TX_PDO_COUNT);

	#else

	// TODO: I want to enable the below debug printf, but the Medulla is silly
	// and will freeze up if I print too much.
	//#ifdef DEBUG_HIGH
	//printf("[Medulla IMU] Initilizing dummy PDO entries\n");
	//#endif // DEBUG_HIGH
	XAngDelta_pdo  = dummy_pdo+4;
	XAccel_pdo = dummy_pdo+8;
	YAngDelta_pdo = dummy_pdo+12;
	YAccel_pdo = dummy_pdo+16;
	ZAngDelta_pdo = dummy_pdo+20;
	ZAccel_pdo = dummy_pdo+24;
	Status_pdo = dummy_pdo+28;
	Seq_pdo = dummy_pdo+29;
	Temp_pdo = dummy_pdo+30;

	#endif // ENABLE_ECAT

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initilizing UART\n");
	#endif
	imu_port = uart_init_port(&PORTF, &USARTF0, uart_baud_921600, imu_tx_buffer, KVH_TX_BUFFER_LENGTH, imu_rx_buffer, KVH_RX_BUFFER_LENGTH);
	uart_connect_port(&imu_port, false);

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initilizing Master Sync pin\n");
	#endif
	msync_pin = io_init_pin(&PORTF, 1);
	//io_set_direction(msync_pin, io_output);
	PORTF.DIR = PORTF.DIR | (1<<1);   // TODO: Why doesn't the above (commented) line work?

	*master_watchdog = imu_counter_pdo;
	*packet_counter = imu_medulla_counter_pdo;
	*imu_medulla_id_pdo = id;
	*commanded_state = imu_command_state_pdo;
	*current_state = imu_current_state_pdo;
}

void imu_update_inputs(uint8_t id) {
	// Trigger Master Sync
	//io_set_output(msync_pin, io_high);
	PORTF.OUT = PORTF.OUT | (1<<1);   // TODO: Why doen't the above (commented) line work?
	_delay_us(30);   // This should be at least 30 us.
	io_set_output(msync_pin, io_low);

	while (uart_received_bytes(&imu_port) < 36);   // Wait for entire packet.
	uart_rx_data(&imu_port, imu_packet, uart_received_bytes(&imu_port));

	(*imu_medulla_id_pdo) = 0;
	(*imu_current_state_pdo)= 0;
	(*imu_medulla_counter_pdo)= 0;
	(*imu_error_flags_pdo)= 0;
	(*XAngDelta_pdo) = 0;
	(*XAccel_pdo)= 0;
	(*YAngDelta_pdo)= 0;
	(*YAccel_pdo)= 0;
	(*ZAngDelta_pdo)= 0;
	(*ZAccel_pdo)= 0;
	(*Status_pdo)= 0;
	(*Seq_pdo)= 0;
	(*Temp_pdo)= 0;

	//TxPDO entries
	(*imu_medulla_id_pdo) = 1;
	(*imu_current_state_pdo) = 2;
	(*imu_medulla_counter_pdo) = 3;
	(*imu_error_flags_pdo) = 4;

	// Populate data from IMU. Refer to p. 10 in manual for data locations.
	populate_byte_to_data(imu_packet[4], XAngDelta_pdo);   // XAngDelta
	populate_byte_to_data(imu_packet[8], YAngDelta_pdo);   // YAngDelta
	populate_byte_to_data(imu_packet[12], ZAngDelta_pdo);   // ZAngDelta
	populate_byte_to_data(imu_packet[16], XAccel_pdo);   // XAccel
	populate_byte_to_data(imu_packet[20], YAccel_pdo);   // YAccel
	populate_byte_to_data(imu_packet[24], ZAccel_pdo);   // ZAccel
	*Status_pdo = imu_packet[28];   // Status
	*Seq_pdo = imu_packet[29];   // Seq
	*Temp_pdo = SHIFT_1BYTE((int16_t)imu_packet[30]) + ((int16_t)imu_packet[31]);   // Temp

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Seq: %u\n", *Seq_pdo);
	#endif // DEBUG_HIGH
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

bool imu_calculating_checksum(uint8_t *rx_buffer,uint8_t rx_buffer_length) {
	bool tGoodResponse;
	uint8_t tix;
	uint16_t tChksum;
	uint16_t tResponseChksum;

	tChksum = 0;
	for (tix = 0;tix < (rx_buffer_length-2);tix++) {
		tChksum += (uint16_t)(rx_buffer[tix]);
	}

	tResponseChksum = 0;
	tResponseChksum = rx_buffer[rx_buffer_length-2] << 8;
	tResponseChksum += rx_buffer[rx_buffer_length-1];

	if (tChksum==tResponseChksum) {
		tGoodResponse = true;
	}
	else {
		tGoodResponse = false;
	}

	return tGoodResponse;
}

uint8_t imu_read_euler_angles(uint32_t *ptr_roll, uint32_t *ptr_pitch, uint32_t *ptr_yaw) {
	uint8_t tx_buffer[1];
	uint8_t rx_buffer[19];

	tx_buffer[0] = 0xCE;

	// read euler angles
	(*ptr_roll) = SHIFT_3BYTE(rx_buffer[1]);
	(*ptr_roll) += SHIFT_2BYTE(rx_buffer[2]);
	(*ptr_roll) += SHIFT_1BYTE(rx_buffer[3]);
	(*ptr_roll) += rx_buffer[4];

	(*ptr_pitch) = SHIFT_3BYTE(rx_buffer[5]);
	(*ptr_pitch) += SHIFT_2BYTE(rx_buffer[6]);
	(*ptr_pitch) += SHIFT_1BYTE(rx_buffer[7]);
	(*ptr_pitch) += rx_buffer[8];

	(*ptr_yaw) = SHIFT_3BYTE(rx_buffer[9]);
	(*ptr_yaw) += SHIFT_2BYTE(rx_buffer[10]);
	(*ptr_yaw) += SHIFT_1BYTE(rx_buffer[11]);
	(*ptr_yaw) += rx_buffer[12];

	return rx_buffer[0];
}

void populate_byte_to_data(const uint8_t* data_byte, uint32_t* data) {
	(*data) = SHIFT_3BYTE((uint32_t)(*data_byte));
	++data_byte;
	(*data) += SHIFT_2BYTE((uint32_t)(*data_byte));
	++data_byte;
	(*data) += SHIFT_1BYTE((uint32_t)(*data_byte));
	++data_byte;
	(*data) += (uint32_t)(*data_byte);
}

