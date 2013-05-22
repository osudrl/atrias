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

void imu_initilize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter, TC0_t *timestamp_timer, uint16_t **master_watchdog);

void imu_enable_outputs(void);
void imu_disable_outputs(void);
void imu_update_inputs(uint8_t id);  /**< Function called to read all the sensors */
bool imu_run_halt(uint8_t id);       /**< Runs the halt controller. Returns true if the controller wants to continue running */ 
void imu_update_outputs(uint8_t id); /**< Called to update motor outputs */
void imu_estop(void);                /**< Called to when estop happens, should send 0 torques to motor */
bool imu_check_error(uint8_t id);    /**< Checks sensor readings to decide if estop should happen, returns true if there should be an error */
bool imu_check_halt(uint8_t id);     /**< Checks sensors to determine if medulla should go into halt state, returns true if medulla should go into halt */
void imu_reset_error();
void imu_wait_loop();

#endif // MEDULLA_IMU_H

