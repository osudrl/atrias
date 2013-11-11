#ifndef IMU_H_
#define IMU_H_

#include "medulla_imu.h"

typedef struct {
	uint16_t decimation;
	bool enable_orientation;
	bool enable_coning_sculling;
	bool enable_little_endian;
	bool suppress_nan;
	bool enable_fin_size_corr;
	bool enable_magnetometer;
	bool enable_north_compensation;
	bool enable_up_compensation;
	bool enable_quaternion;
	uint8_t gyro_accel_filter_length;
	uint8_t magnetometer_filter_length;
	uint16_t up_compensation_time_constant;
	uint16_t north_compensation_time_constant;
} imu_sampling_settings_t;

typedef enum {
	imu_cmd_raw_accel_and_rate									= 0,
	imu_cmd_accel_and_rate										= 1,
	imu_cmd_delta_ang_and_vel									= 2,
	imu_cmd_orientation_matrix									= 3,
	imu_cmd_orientation_update									= 4,
	imu_cmd_magnetometer_vector									= 5,
	imu_cmd_accel_rate_and_orientation							= 6,
	imu_cmd_accel_rate_and_magnetometer							= 7,
	imu_cmd_accel_rate_magnetometer_and_orientation				= 8,
	imu_cmd_euler_angles										= 9,
	imu_cmd_euler_angles_and_rates								= 10,
	imu_cmd_temperatures										= 11,
	imu_cmd_gyro_stabilized_accel_rate_and_magnetometer			= 12,
	imu_cmd_delta_ang_and_vel_and_magnetometer					= 13,
	imu_cmd_stationary_test										= 14,
	imu_cmd_quaternion											= 15,
	imu_cmd_read_firmware_rev									= 16
} imu_command_t;

#define IMU_CMD_RAW_ACCEL_AND_RATE								0xC1
#define IMU_CMD_ACCEL_AND_RATE									0xC2
#define IMU_CMD_DELTA_ANG_AND_VEL   							0xC3
#define IMU_CMD_ORIENTATION_MATRIX   							0xC5
#define IMU_CMD_ORIENTATION_UPDATE    							0xC6
#define IMU_CMD_MAGNETOMETER_VECTOR   							0xC7
#define IMU_CMD_ACCEL_RATE_AND_ORIENTATION  					0xC8
#define IMU_CMD_ACCEL_RATE_AND_MAGNETOMETER 					0xCB
#define IMU_CMD_ACCEL_RATE_MAGNETOMETER_AND_ORIENTATION 		0xCC
#define IMU_CMD_EULER_ANGLES           							0xCE
#define IMU_CMD_EULER_ANGLES_AND_RATES 							0xCF
#define IMU_CMD_TEMPERATURES           							0xD1
#define IMU_CMD_GYRO_STABILIZED_ACCEL_RATE_AND_MAGNETOMETER		0xD2
#define IMU_CMD_DELTA_ANG_AND_VEL_AND_MAGNETOMETER 				0xD3
#define IMU_CMD_STATIONARY_TEST 								0xDA
#define IMU_CMD_QUATERNION 										0xDF
#define IMU_CMD_READ_FIRMWARE_REV 								0xE9
	
#define IMU_ACK_RAW_ACCEL_AND_RATE								31
#define IMU_ACK_ACCEL_AND_RATE									31
#define IMU_ACK_DELTA_ANG_AND_VEL   							31
#define IMU_ACK_ORIENTATION_MATRIX   							43
#define IMU_ACK_ORIENTATION_UPDATE    							43
#define IMU_ACK_MAGNETOMETER_VECTOR   							19
#define IMU_ACK_ACCEL_RATE_AND_ORIENTATION  					67
#define IMU_ACK_ACCEL_RATE_AND_MAGNETOMETER 					43
#define IMU_ACK_ACCEL_RATE_MAGNETOMETER_AND_ORIENTATION 		79
#define IMU_ACK_EULER_ANGLES           							19
#define IMU_ACK_EULER_ANGLES_AND_RATES 							31
#define IMU_ACK_TEMPERATURES           							15
#define IMU_ACK_GYRO_STABILIZED_ACCEL_RATE_AND_MAGNETOMETER		43
#define IMU_ACK_DELTA_ANG_AND_VEL_AND_MAGNETOMETER 				43
#define IMU_ACK_STATIONARY_TEST 								7
#define IMU_ACK_QUATERNION 										23
#define IMU_ACK_READ_FIRMWARE_REV 								7
	
#define SHIFT_1BYTE(X)	(X<<8)
#define SHIFT_2BYTE(X)	(X<<16)
#define SHIFT_3BYTE(X)	(X<<24)

#define UART_RX_BUFFER_SIZE 100
#define UART_TX_BUFFER_SIZE 100
#define PACKET_BUFFER_SIZE 100

#define IMU_CMD_UNSET_CONTINUOUS_MODE_B0 0xFA
#define IMU_CMD_UNSET_CONTINUOUS_MODE_B1 0x75
#define IMU_CMD_UNSET_CONTINUOUS_MODE_B2 0xB4

#define IMU_CMD_SET_CONTINUOUS_MODE_B0 0xC4
#define IMU_CMD_SET_CONTINUOUS_MODE_B1 0xC1
#define IMU_CMD_SET_CONTINUOUS_MODE_B2 0x29

#define IMU_CMD_SOFT_RESET_B0 0xFE
#define IMU_CMD_SOFT_RESET_B1 0x9E
#define IMU_CMD_SOFT_RESET_B2 0x3A

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

void imu_init_uart(void);
void imu_init_pdos(void);
void imu_run_state_machine(void);
void imu_get_state_machine_ouputs(uint8_t *y0, uint8_t *y1, uint8_t *y2, uint8_t *y3, uint8_t *y4, uint8_t *y5, uint32_t *y6);
uint8_t get_packet_buffer_length(void);

uint8_t imu_poll_single_byte_command(imu_command_t cmd, uint8_t *response);
uint8_t imu_set_continuous_mode(uint8_t continuous_command);
uint8_t imu_unset_continuous_mode(void);
uint8_t imu_set_active_mode(void);
uint8_t imu_soft_reset(void);
uint8_t imu_get_sampling_settings(imu_sampling_settings_t *current_settings);
uint8_t imu_set_sampling_settings(imu_sampling_settings_t *current_settings);
uint8_t imu_capture_gyro_bias(uint16_t sampling_time);
uint8_t imu_realign_up_and_north(uint8_t up_realign_time, uint8_t north_realign_time);

bool imu_send_command_single_byte(imu_command_t cmd, uint8_t *response);
bool imu_send_command(uint8_t *cmd, uint8_t cmd_length, uint8_t *response, uint8_t response_length);
bool imu_verify_checksum(uint8_t *rx_buffer,uint8_t rx_buffer_length);

void convert_bytes_to_uint32(const uint8_t* data_byte,uint32_t* data);
void convert_bytes_to_uint16(const uint8_t* data_byte, uint16_t* data);
void convert_uint16_to_bytes(const uint16_t data, uint8_t* data_byte);

uint16_t read_bytes(uint8_t *rx_buffer, uint16_t numbytes, uint16_t max_attempts);
void imu_flush(void);

uint8_t parse_OriMatrix(uint8_t *pResponse);
uint8_t parse_accel(uint8_t *pResponse);
uint8_t parse_angRates(uint8_t *pResponse);
void parse_sampling_settings(imu_sampling_settings_t *current_settings, uint8_t *response);
void encode_sampling_settings(const imu_sampling_settings_t *current_settings, uint8_t *cmd);

#endif /* IMU_H_ */
