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
int32_t *imu_command_pdo;

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
	{((void**)(&imu_counter_pdo)),2},
	{((void**)(&imu_command_pdo)),2}};

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
	#endif
	ecat_init_sync_managers(ecat_slave, rx_sm_buffer, MEDULLA_IMU_OUTPUTS_SIZE, 0x1000, tx_sm_buffer, MEDULLA_IMU_INPUTS_SIZE, 0x2000);

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initilizing PDO entries\n");
	#endif
	ecat_configure_pdo_entries(ecat_slave, imu_rx_pdos, MEDULLA_IMU_RX_PDO_COUNT, imu_tx_pdos, MEDULLA_IMU_TX_PDO_COUNT);
	#endif // ENABLE_ECAT

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Initilizing UART\n");
	#endif
	imu_port = uart_init_port(&PORTF, &USARTF0, uart_baud_921600, imu_tx_buffer, KVH_TX_BUFFER_LENGTH, imu_rx_buffer, KVH_RX_BUFFER_LENGTH);
	uart_connect_port(&imu_port, false);

	#ifdef DEBUG_HIGH
	printf("[Medulla IMU] Configuring IMU\n");
	#endif
	// TODO: configure IMU.

	*master_watchdog = imu_counter_pdo;
	*packet_counter = imu_medulla_counter_pdo;
	*imu_medulla_id_pdo = id;
	*commanded_state = imu_command_state_pdo;
	*current_state = imu_current_state_pdo;
}

void imu_update_inputs(uint8_t id) {
	uint8_t response;
	uint32_t pResponse[36];
	uint8_t u8_pResponse[36];

	uint8_t led_cnt = 0;

	// Blinking means IMU is performing fine.
	if (led_cnt/((uint32_t)(100))) {
		LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_GREEN;
	}
	else {
		LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_BLUE;
	}
	++led_cnt;

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

	for (uint8_t i=0;i<36;i++) {
		pResponse[i] = u8_pResponse[i];
	}

	//TxPDO entries
	(*imu_medulla_id_pdo) = 1;
	(*imu_current_state_pdo) = 2;
	(*imu_medulla_counter_pdo) = 3;
	(*imu_error_flags_pdo) = 4;

	// Populate data from IMU. Refer to p. 10 in manual for
	// data locations.
	uint8_t *ptr;

	// XAngDelta
	ptr = u8_pResponse+4;
	populate_byte_to_data(ptr,XAngDelta_pdo);

	// YAngDelta
	ptr = u8_pResponse+8;
	populate_byte_to_data(ptr,YAngDelta_pdo);

	// ZAngDelta
	ptr = u8_pResponse+12;
	populate_byte_to_data(ptr,ZAngDelta_pdo);

	// XAccel
	ptr = u8_pResponse+16;
	populate_byte_to_data(ptr,XAccel_pdo);

	// YAccel
	ptr = u8_pResponse+20;
	populate_byte_to_data(ptr,YAccel_pdo);

	// ZAccel
	ptr = u8_pResponse+24;
	populate_byte_to_data(ptr,ZAccel_pdo);

	// Status
	ptr = u8_pResponse+28;
	*Status_pdo = *ptr;

	// Seq
	ptr = u8_pResponse+29;
	*Seq_pdo = *ptr;

	// Temp
	ptr = u8_pResponse+30;
	*Temp_pdo = SHIFT_1BYTE((int16_t)(*ptr++));
	*Temp_pdo += (int16_t)(*ptr);
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

uint8_t imu_set_active_mode(void) {
	uint8_t imu_tx_buffer[4];
	uint8_t imu_rx_buffer[4];

	imu_tx_buffer[0] = 0xD4;
	imu_tx_buffer[1] = 0xA3;
	imu_tx_buffer[2] = 0x47;
	imu_tx_buffer[3] = 0x01;

	while (1) {
		uart_tx_data(&imu_port, imu_tx_buffer, 4);
		uart_rx_data(&imu_port, imu_rx_buffer, 4);
		_delay_ms(5);
	}
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

