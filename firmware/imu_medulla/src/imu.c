#include "imu.h"

// Make vectors of command bytes and response lengths for single-byte commands
uint8_t imu_cmd_single_byte[] = {IMU_CMD_RAW_ACCEL_AND_RATE,
								 IMU_CMD_ACCEL_AND_RATE,
								 IMU_CMD_DELTA_ANG_AND_VEL,
								 IMU_CMD_ORIENTATION_MATRIX,
								 IMU_CMD_ORIENTATION_UPDATE,
								 IMU_CMD_MAGNETOMETER_VECTOR,
								 IMU_CMD_ACCEL_RATE_AND_ORIENTATION,
								 IMU_CMD_ACCEL_RATE_AND_MAGNETOMETER,
								 IMU_CMD_ACCEL_RATE_MAGNETOMETER_AND_ORIENTATION,
								 IMU_CMD_EULER_ANGLES,
								 IMU_CMD_EULER_ANGLES_AND_RATES,
								 IMU_CMD_TEMPERATURES,
								 IMU_CMD_GYRO_STABILIZED_ACCEL_RATE_AND_MAGNETOMETER,
								 IMU_CMD_DELTA_ANG_AND_VEL_AND_MAGNETOMETER,
								 IMU_CMD_STATIONARY_TEST,
								 IMU_CMD_QUATERNION,
								 IMU_CMD_READ_FIRMWARE_REV};
								 
uint8_t imu_ack_single_byte[] = {IMU_ACK_RAW_ACCEL_AND_RATE,
								 IMU_ACK_ACCEL_AND_RATE,
								 IMU_ACK_DELTA_ANG_AND_VEL,
								 IMU_ACK_ORIENTATION_MATRIX,
								 IMU_ACK_ORIENTATION_UPDATE,
								 IMU_ACK_MAGNETOMETER_VECTOR,
								 IMU_ACK_ACCEL_RATE_AND_ORIENTATION,
								 IMU_ACK_ACCEL_RATE_AND_MAGNETOMETER,
								 IMU_ACK_ACCEL_RATE_MAGNETOMETER_AND_ORIENTATION,
								 IMU_ACK_EULER_ANGLES,
								 IMU_ACK_EULER_ANGLES_AND_RATES,
								 IMU_ACK_TEMPERATURES,
								 IMU_ACK_GYRO_STABILIZED_ACCEL_RATE_AND_MAGNETOMETER,
								 IMU_ACK_DELTA_ANG_AND_VEL_AND_MAGNETOMETER,
								 IMU_ACK_STATIONARY_TEST,
								 IMU_ACK_QUATERNION,
								 IMU_ACK_READ_FIRMWARE_REV};

// RxPDO entries
medulla_state_t *imu_command_state_pdo;
uint16_t *imu_counter_pdo;
uint16_t *imu_command_pdo;

// TxPDO entries
uint8_t *imu_medulla_id_pdo;
medulla_state_t *imu_current_state_pdo;
uint8_t *imu_medulla_counter_pdo;
uint8_t *imu_error_flags_pdo;

uint32_t *XAngRate_pdo;
uint32_t *XAccel_pdo;
uint32_t *YAngRate_pdo;
uint32_t *YAccel_pdo;
uint32_t *ZAngRate_pdo;
uint32_t *ZAccel_pdo;
uint32_t *M11_pdo;
uint32_t *M12_pdo;
uint32_t *M13_pdo;
uint32_t *M21_pdo;
uint32_t *M22_pdo;
uint32_t *M23_pdo;
uint32_t *M31_pdo;
uint32_t *M32_pdo;
uint32_t *M33_pdo;

// Allocate UART buffers for use by ISRs
uint8_t imu_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t imu_tx_buffer[UART_TX_BUFFER_SIZE];

// UART
uart_port_t imu_port;


/** State machine variables **/
uint8_t packet_buffer[PACKET_BUFFER_SIZE];
uint8_t packet_buffer_length = 0;
uint8_t sync_state = 0;
uint8_t imu_current_state = 0;
uint8_t imu_command = 0;
uint8_t led_cnt = 0;
uint32_t cmd_cnt = 0;
imu_sampling_settings_t current_settings;
									

void imu_init_uart(void) {
	imu_port = uart_init_port(&PORTE, &USARTE0, uart_baud_460800, imu_tx_buffer, UART_TX_BUFFER_SIZE, imu_rx_buffer, UART_RX_BUFFER_SIZE);
	uart_connect_port(&imu_port, true);
}

void imu_init_pdos(void) {
	// Init PDOs
	(*imu_medulla_id_pdo) = 0;
	(*imu_current_state_pdo)= 0;
	(*imu_medulla_counter_pdo)= 0;
	(*imu_error_flags_pdo)= 0;
	(*XAngRate_pdo) = 0;
	(*XAccel_pdo)= 0;
	(*YAngRate_pdo)= 0;
	(*YAccel_pdo)= 0;
	(*ZAngRate_pdo)= 0;
	(*ZAccel_pdo)= 0;
	(*M11_pdo)= 0;
	(*M12_pdo)= 0;
	(*M13_pdo)= 0;
	(*M21_pdo)= 0;
	(*M22_pdo)= 0;
	(*M23_pdo)= 0;
	(*M31_pdo)= 0;
	(*M32_pdo)= 0;
	(*M33_pdo)= 0;
}

void imu_run_state_machine(void) {
	
	uint8_t response;
	uint8_t second_header_byte = 0;
	bool valid_checksum = false;
	uint8_t i;
	uint8_t numreq;
	
	
	// Blinking means IMU is performing fine.
	if (led_cnt & 0x80)
	{
		LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_VIOLATE;
	}
	else
	{
		LED_PORT.OUT = (LED_PORT.OUT & ~LED_MASK) | LED_BLUE;
	}		 
	
	++led_cnt;
	++cmd_cnt;
	
	// Enable/disable continuous mode
	if ((*imu_command_pdo) != imu_command) {
		if ((*imu_command_pdo)>=2 && (*imu_command_pdo)<=1000) {
			imu_get_sampling_settings(&current_settings);
			current_settings.up_compensation_time_constant	= (*imu_command_pdo);
			imu_set_sampling_settings(&current_settings);
			imu_get_sampling_settings(&current_settings);
			imu_command = (uint8_t) current_settings.up_compensation_time_constant;
		}
	}
		
	if (imu_current_state==1)
	{
		switch (sync_state) {
		case 0:
			// Read data until we see a header packet
			packet_buffer[0] = 0;
			while (uart_received_bytes(&imu_port)>0) {
				uart_rx_data(&imu_port,packet_buffer,1);
				if (packet_buffer[0]==IMU_CMD_ORIENTATION_MATRIX) {
					#ifdef MATLAB_MEX_FILE
					mexPrintf("FOUND HEADER.\n");
					#endif
					packet_buffer_length = 1;
					sync_state = 1;
					break;
				}
			}
			break;
			
		case 1:
			// Read one packet length of data
			// We assume that (packet_buffer_length) bytes are already in place
			//read_bytes(packet_buffer+packet_buffer_length, IMU_ACK_ORIENTATION_MATRIX-packet_buffer_length);
			packet_buffer_length += uart_rx_data(&imu_port,packet_buffer+packet_buffer_length, IMU_ACK_ORIENTATION_MATRIX-packet_buffer_length);
			if (packet_buffer_length==IMU_ACK_ORIENTATION_MATRIX) {
				//Verify checksum
				valid_checksum = imu_verify_checksum(packet_buffer,IMU_ACK_ORIENTATION_MATRIX);
				if (valid_checksum) {
					parse_OriMatrix(packet_buffer+1);
					sync_state = 0;
				}
				else {
					// See if there was another header byte in the last "packet"
					// If so, shift the data and try reading enough bytes to complete the packet
					second_header_byte = 0;
					sync_state = 0;
					for (i=1; i<IMU_ACK_ORIENTATION_MATRIX; i++) {
						if (second_header_byte>0) {
							packet_buffer[i-second_header_byte] = packet_buffer[i];
						}
						else if (packet_buffer[i]==IMU_CMD_ORIENTATION_MATRIX) {
							second_header_byte = i;
							packet_buffer[i-second_header_byte] = packet_buffer[i]; // this should be redundant...
							packet_buffer_length = IMU_ACK_ORIENTATION_MATRIX-i;
							sync_state = 1;
						}
					}
				}
			}
			else {
				valid_checksum = false;
			}
			break;
			
		default:
			sync_state = 0;
		}
		numreq = 0;
	}
	else {
		numreq = imu_poll_single_byte_command(imu_cmd_orientation_matrix, packet_buffer);
		parse_OriMatrix(packet_buffer+1);
		
		(*YAngRate_pdo) = imu_command;
		(*ZAngRate_pdo) = *imu_command_pdo;
		(*XAccel_pdo) = current_settings.up_compensation_time_constant;
		(*YAccel_pdo) = cmd_cnt;
		(*ZAccel_pdo) = current_settings.enable_up_compensation;
	}
	
	//TxPDO entries
	(*imu_medulla_id_pdo) = imu_command;
	(*imu_current_state_pdo) = imu_current_state;
	(*imu_medulla_counter_pdo) = (uint8_t)(*imu_command_pdo);
	(*imu_error_flags_pdo) = 1;
}

void imu_get_state_machine_ouputs(uint8_t *y0, uint8_t *y1, uint8_t *y2, uint8_t *y3, uint8_t *y4, uint8_t *y5, uint32_t *y6) {
	uint8_t i;
	
	for (i=0; i<PACKET_BUFFER_SIZE; i++)
		y0[i] = packet_buffer[i];
	
	(*y1) = packet_buffer_length;
	(*y2) = sync_state;
	(*y3) = imu_current_state;
	(*y4) = imu_command;
	(*y5) = led_cnt;
	(*y6) = cmd_cnt;
}

uint8_t get_packet_buffer_length(void) {
	return packet_buffer_length;
}

uint8_t imu_poll_single_byte_command(imu_command_t cmd, uint8_t *response) {
	uint8_t numreq = 1;
	
	imu_flush();
	
	while (!imu_send_command_single_byte(cmd, response))
		numreq++;
		
	return numreq;
}

uint8_t imu_get_sampling_settings(imu_sampling_settings_t *current_settings) {
	uint8_t cmd[20] = {0xDB, 0xA8, 0xB9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t response[19];
	uint8_t numreq = 1;
	
	while (!imu_send_command(cmd, 20, response, 19))
		numreq++;
	
	parse_sampling_settings(current_settings, response);
	return numreq;
}

uint8_t imu_set_sampling_settings(imu_sampling_settings_t *current_settings) {
	uint8_t cmd[20] = {0xDB, 0xA8, 0xB9, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t response[19];
	uint8_t numreq = 1;
	
	encode_sampling_settings(current_settings, cmd);
	
	while (!imu_send_command(cmd, 20, response, 19))
		numreq++;
	
	return numreq;
}


uint8_t imu_capture_gyro_bias(uint16_t sampling_time) {
	uint8_t cmd[5] = {0xCD, 0xC1, 0x29, 0x00, 0x00};
	uint8_t response[7];
	uint8_t numreq = 1;

	convert_uint16_to_bytes(sampling_time, cmd+3);
	
	while (!imu_send_command(cmd, 5, response, 19))
		numreq++;

	return numreq;
}

uint8_t imu_realign_up_and_north(uint8_t up_realign_time, uint8_t north_realign_time) {
	uint8_t cmd[10] = {0xDD, 0x54, 0x4C, 0x00, up_realign_time, north_realign_time, 0x00, 0x00, 0x00, 0x00};
	uint8_t response[7];
	uint8_t numreq = 1;
	
	while (!imu_send_command(cmd, 10, response, 7))
		numreq++;
	
	return numreq;
}

uint8_t imu_set_continuous_mode(uint8_t continuous_command){
	uint8_t cmd[4];
	uint8_t response[1];
	uint8_t numreq = 1;
	
	cmd[0] = IMU_CMD_SET_CONTINUOUS_MODE_B0;
	cmd[1] = IMU_CMD_SET_CONTINUOUS_MODE_B1;
	cmd[2] = IMU_CMD_SET_CONTINUOUS_MODE_B2;
	cmd[3] = continuous_command;
	
	while (!imu_send_command(cmd, 4, response, 0) || (response[0]!=continuous_command))
		numreq++;
	
	return numreq;
}

uint8_t imu_unset_continuous_mode(void) {
	uint8_t cmd[3];
	uint8_t numreq = 1;
	
	cmd[0] = IMU_CMD_UNSET_CONTINUOUS_MODE_B0;
	cmd[1] = IMU_CMD_UNSET_CONTINUOUS_MODE_B1;
	cmd[2] = IMU_CMD_UNSET_CONTINUOUS_MODE_B2;
	
	while (!imu_send_command(cmd, 3, NULL, 0))
		numreq++;
	
	return numreq;
}

uint8_t imu_set_active_mode(void){
	uint8_t tx_buffer[4];
	uint8_t rx_buffer[4];
	
	tx_buffer[0] = 0xD4;
	tx_buffer[1] = 0xA3;
	tx_buffer[2] = 0x47;
	tx_buffer[3] = 0x01;
	
	while (1)
	{
		uart_tx_data(&imu_port,tx_buffer, 4);
		uart_rx_data(&imu_port,rx_buffer,4);
		_delay_ms(5);
	}
}

uint8_t imu_soft_reset(void) {
	uint8_t cmd[3];
	uint8_t numreq = 1;
	
	cmd[0] = IMU_CMD_SOFT_RESET_B0;
	cmd[1] = IMU_CMD_SOFT_RESET_B1;
	cmd[2] = IMU_CMD_SOFT_RESET_B2;
	
	while (!imu_send_command(cmd, 3, NULL, 0))
		numreq++;
	
	return numreq;
}

bool imu_send_command_single_byte(imu_command_t cmd, uint8_t *response) {
	return imu_send_command(&imu_cmd_single_byte[cmd], 1, response, imu_ack_single_byte[cmd]);
}

bool imu_send_command(uint8_t *cmd, uint8_t cmd_length, uint8_t *response, uint8_t response_length) {
	
	uint16_t tx_buffer_length;
	uint16_t rx_buffer_length;
		
	// Send command
	tx_buffer_length = uart_tx_data(&imu_port, cmd, cmd_length);

	if (tx_buffer_length<cmd_length) {
		return false;
	}	
	
	// Receive data
	rx_buffer_length = 0;
	while (rx_buffer_length<response_length) {
		rx_buffer_length += uart_rx_data(&imu_port,response+rx_buffer_length,response_length-rx_buffer_length);
	}
	
	// Verify checksum
	if (response_length>0) {
		return (rx_buffer_length==response_length) && (response[0]==cmd[0]) && imu_verify_checksum(response,response_length); 
	}	
	else {
		return true;
	}	
}

bool imu_verify_checksum(uint8_t *rx_buffer,uint8_t rx_buffer_length){
	bool tGoodResponse;
	uint8_t tix;
	uint16_t tChksum;
	uint16_t tResponseChksum;
	
	tChksum = 0;
	for (tix = 0;tix < (rx_buffer_length-2);tix++)
	{
		tChksum += (uint16_t)(rx_buffer[tix]);
	}
	
	tResponseChksum = 0;
	tResponseChksum = rx_buffer[rx_buffer_length-2] << 8;
	tResponseChksum += rx_buffer[rx_buffer_length-1];
	
	if (tChksum==tResponseChksum)
	{
		tGoodResponse = true;
	}
	else
	{
		tGoodResponse = false;
	}
	
	return tGoodResponse;
}

void convert_bytes_to_uint32(const uint8_t* data_byte, uint32_t* data) {
	(*data) = SHIFT_3BYTE((uint32_t)(*data_byte));
	++data_byte;
	(*data) += SHIFT_2BYTE((uint32_t)(*data_byte));
	++data_byte;
	(*data) += SHIFT_1BYTE((uint32_t)(*data_byte));
	++data_byte;
	(*data) += (uint32_t)(*data_byte);
}

void convert_bytes_to_uint16(const uint8_t* data_byte, uint16_t* data) {
	(*data) = SHIFT_1BYTE((uint16_t)(*data_byte));
	++data_byte;
	(*data) += (uint16_t)(*data_byte);
}

void convert_uint16_to_bytes(const uint16_t data, uint8_t* data_byte) {
	data_byte[0] = (uint8_t)(data >> 8);
	data_byte[1] = (uint8_t)(data);
}

uint16_t read_bytes(uint8_t *rx_buffer, uint16_t numbytes, uint16_t max_attempts) {
	uint16_t rx_buffer_length = 0;
	uint16_t offset = 0;
	uint16_t num_attempts = 0;

	while (rx_buffer_length<numbytes && num_attempts<max_attempts)
	{
		offset = uart_rx_data(&imu_port,rx_buffer+rx_buffer_length,numbytes-rx_buffer_length);
		rx_buffer_length = rx_buffer_length + offset;
		num_attempts++;
	}
	return rx_buffer_length;
}

void imu_flush(void) {
	while (uart_received_bytes(&imu_port)>0)
		uart_rx_byte(&imu_port);
}

uint8_t parse_OriMatrix(uint8_t *pResponse) {
	uint8_t byte = 0;
	
	convert_bytes_to_uint32(pResponse+byte,M11_pdo);
	byte += 4;
	convert_bytes_to_uint32(pResponse+byte,M12_pdo);
	byte += 4;
	convert_bytes_to_uint32(pResponse+byte,M13_pdo);
	byte += 4;
	convert_bytes_to_uint32(pResponse+byte,M21_pdo);
	byte += 4;
	convert_bytes_to_uint32(pResponse+byte,M22_pdo);
	byte += 4;
	convert_bytes_to_uint32(pResponse+byte,M23_pdo);
	byte += 4;
	convert_bytes_to_uint32(pResponse+byte,M31_pdo);
	byte += 4;
	convert_bytes_to_uint32(pResponse+byte,M32_pdo);
	byte += 4;
	convert_bytes_to_uint32(pResponse+byte,M33_pdo);
	byte += 4;
	
	return byte;
}

uint8_t parse_accel(uint8_t *pResponse) {
	uint8_t byte = 0;
	
	convert_bytes_to_uint32(pResponse+byte,XAccel_pdo);
	byte += 4;
	convert_bytes_to_uint32(pResponse+byte,YAccel_pdo);
	byte += 4;
	convert_bytes_to_uint32(pResponse+byte,ZAccel_pdo);
	byte += 4;
	
	return byte;
}

uint8_t parse_angRates(uint8_t *pResponse) {
	uint8_t byte = 0;
	
	convert_bytes_to_uint32(pResponse+byte,XAngRate_pdo);
	byte += 4;
	convert_bytes_to_uint32(pResponse+byte,YAngRate_pdo);
	byte += 4;
	convert_bytes_to_uint32(pResponse+byte,ZAngRate_pdo);
	byte += 4;
	
	return byte;
}

void parse_sampling_settings(imu_sampling_settings_t *current_settings, uint8_t *response) {
	uint16_t data_conditioning;
	
	convert_bytes_to_uint16(response+1, &(current_settings->decimation));
	convert_bytes_to_uint16(response+3, &data_conditioning);
	current_settings->gyro_accel_filter_length	 = response[5];
	current_settings->magnetometer_filter_length = response[6];
	convert_bytes_to_uint16(response+7, &(current_settings->up_compensation_time_constant));
	convert_bytes_to_uint16(response+9, &(current_settings->north_compensation_time_constant));
	
	current_settings->enable_orientation		= data_conditioning & (1 << 0);
	current_settings->enable_coning_sculling	= data_conditioning & (1 << 1);
	current_settings->enable_little_endian		= data_conditioning & (1 << 4);
	current_settings->suppress_nan				= data_conditioning & (1 << 5);
	current_settings->enable_fin_size_corr		= data_conditioning & (1 << 6);
	current_settings->enable_magnetometer		= !(data_conditioning & (1 << 8));
	current_settings->enable_north_compensation = !(data_conditioning & (1 << 10));
	current_settings->enable_up_compensation	= !(data_conditioning & (1 << 11));
	current_settings->enable_quaternion			= data_conditioning & (1 << 12);
}

void encode_sampling_settings(const imu_sampling_settings_t *current_settings, uint8_t *cmd) {
	uint16_t data_conditioning = 0;
	
	data_conditioning |= (current_settings->enable_orientation		   ? (1 << 0) : 0);
	data_conditioning |= (current_settings->enable_coning_sculling	   ? (1 << 1) : 0);
	data_conditioning |= (current_settings->enable_little_endian	   ? (1 << 4) : 0);
	data_conditioning |= (current_settings->suppress_nan			   ? (1 << 5) : 0);
	data_conditioning |= (current_settings->enable_fin_size_corr	   ? (1 << 6) : 0);
	data_conditioning |= (!current_settings->enable_magnetometer	   ? (1 << 8) : 0);
	data_conditioning |= (!current_settings->enable_north_compensation ? (1 << 10) : 0);
	data_conditioning |= (!current_settings->enable_up_compensation    ? (1 << 11) : 0);
	data_conditioning |= (current_settings->enable_quaternion		   ? (1 << 12) : 0);
	
	convert_uint16_to_bytes(current_settings->decimation, cmd+4);
	convert_uint16_to_bytes(data_conditioning, cmd+6);
	cmd[8] = current_settings->gyro_accel_filter_length;
	cmd[9] = current_settings->magnetometer_filter_length;
	convert_uint16_to_bytes(current_settings->up_compensation_time_constant, cmd+10);
	convert_uint16_to_bytes(current_settings->north_compensation_time_constant, cmd+12);
}
