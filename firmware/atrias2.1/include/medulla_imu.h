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

#define ERROR_FLAG_KVH_SEQ 0
#define ERROR_FLAG_KVH_CRC 1
#define ERROR_FLAG_KVH_HEADER 2
//#define ERROR_FLAG_UNDEFINED 3
//#define ERROR_FLAG_UNDEFINED 4
//#define ERROR_FLAG_UNDEFINED 5
//#define ERROR_FLAG_UNDEFINED 6
//#define ERROR_FLAG_UNDEFINED 7

#define KVH_TX_BUFFER_LENGTH 250
#define KVH_RX_BUFFER_LENGTH 250

// KVH stuff. TODO: Move this to own header file.
uint8_t imu_tx_buffer[KVH_TX_BUFFER_LENGTH];
uint8_t imu_rx_buffer[KVH_RX_BUFFER_LENGTH];
uint8_t imu_packet[36];
uart_port_t imu_port;
io_pin_t msync_pin;

void imu_clear_buffer(void);
void populate_byte_to_data(const uint8_t* data_byte, uint32_t* data);

// Medulla stuff.
void imu_initialize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter, TC0_t *timestamp_timer, uint16_t **master_watchdog);

void imu_enable_outputs(void);
void imu_disable_outputs(void);
void imu_update_inputs(uint8_t id);
bool imu_run_halt(uint8_t id);
void imu_update_outputs(uint8_t id);
void imu_estop(void);
bool imu_check_error(uint8_t id);
bool imu_check_halt(uint8_t id);
void imu_reset_error(void);

#endif // MEDULLA_IMU_H

