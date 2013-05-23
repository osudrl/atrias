#ifndef MEDULLA_IMU_H
#define MEDULLA_IMU_H

#include "/usr/avr/include/stdio.h"
#include <string.h>

// Include the robot definitions
#include "robot_invariant_defs.h"
#include "robot_variant_defs.h"

// Include medulla_lib headers
#include "ethercat.h"
#include "adc.h"

// KVH stuff. TODO: Move this to own header file.
typedef struct {
	uint8_t tx_buffer[KVH_TX_BUFFER_LENGTH];
	uint8_t rx_buffer[KVH_RX_BUFFER_LENGTH];
	uart_port_t uart_port;
	uint8_t *data_buffer;
} kvh_data_t;

kvh_init(PORT_t *port, USART_t *uart, kvh_data_t *data, uint8_t *data_buffer);

// Medulla stuff.
void imu_initilize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter, TC0_t *timestamp_timer, uint16_t **master_watchdog);

void imu_read_data(ecat_slave_t ecat_port);
bool imu_calculating_checksum(uint8+_t *rx_buffer, uint8_t rx_buffer_length);
uint8_t imu_set_continuous_mode(void);
uint8_t imu_unset_continuous_mode(void);
uint8_t imu_set_active_mode(void);
uint8_t imu_read_euler_angles(uint32_t *ptr_roll, uint32_t *ptr_pitch, uint32_t *ptr_yaw);
uint8_t imu_read_accel_angRate_OriMatrix(uint8_t *ptr_response);
uint8_t imu_pull_accel_angRate_OriMatrix(uint8_t *ptr_response);
uint8_t imu_soft_reset(void);
uint8_t imu_reset_update_mode(void);

#endif // MEDULLA_IMU_H

