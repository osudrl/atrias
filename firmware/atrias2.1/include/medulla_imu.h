#ifndef MEDULLA_IMU_H
#define MEDULLA_IMU_H

#include <stdio.h>
#include <string.h>
#include <util/delay.h>

// Include the robot definitions
#include "robot_invariant_defs.h"
#include "robot_variant_defs.h"

// Include medulla_lib headers
#include "ethercat.h"
#include "adc.h"
#include "io_pin.h"
#include "uart.h"

#define ERROR_FLAG_PAYLD_SZ (1<<0)
#define ERROR_FLAG_HEADER   (1<<1)
#define ERROR_FLAG_CRC      (1<<2)

// Message and buffer sizes. Note that the UART driver ignores 1 byte of the buffer (bug?), so we'll add 1 to the buffer size to compensate.
#define KVH_MSG_SIZE 36
#define IMU_TX_BUF_SZ (KVH_MSG_SIZE+1)
#define IMU_RX_BUF_SZ (KVH_MSG_SIZE+1)

// We also need to (separately) know the size of the data to be CRC'd
#define CRC_PAYLD_SZ (KVH_MSG_SIZE-4)

// KVH stuff.
uint8_t imu_tx_buffer[IMU_TX_BUF_SZ];
uint8_t imu_rx_buffer[IMU_RX_BUF_SZ];
uint8_t imu_packet[KVH_MSG_SIZE];
uart_port_t imu_port;
io_pin_t msync_pin;

void populate_byte_to_data(const uint8_t* data_byte, uint32_t* data);

// IMU decoding functions
void imu_process_data(void);

// Medulla stuff.
void imu_initialize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter, TC0_t *timestamp_timer, uint16_t **master_watchdog);

void imu_enable_outputs(void);
void imu_disable_outputs(void);
void imu_update_inputs(uint8_t id);
bool imu_run_halt(uint8_t id);
void imu_update_outputs(uint8_t id);
void imu_post_ecat(void);
void imu_estop(void);
bool imu_check_error(uint8_t id);
bool imu_check_halt(uint8_t id);
void imu_reset_error(void);

#endif // MEDULLA_IMU_H

