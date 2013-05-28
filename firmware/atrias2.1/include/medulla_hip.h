#ifndef MEDULLA_HIP_H
#define MEDULLA_HIP_H

#include <stdio.h>
#include <string.h>

// Include the robot definitions
#include "robot_invariant_defs.h"
#include "robot_variant_defs.h"

// Include medulla_lib headers
#include "ethercat.h"
#include "limit_switch.h"
#include "adc.h"
#include "renishaw_ssi_encoder.h"
#include "quadrature_encoder.h"
#include "imu.h"

#include "amplifier.h"

// Defines for amplifier error output
#define AMP_DIO_PORT PORTH
#define AMP1_ERROR_bm (1<<0)
#define AMP1_ENABLED_bm (1<<1)

// Defines for which systems to check for error state
//#define ERROR_CHECK_LIMIT_SWITCH
//#define ERROR_CHECK_THERMISTORS
//#define ERROR_CHECK_MOTOR_VOLTAGE
//#define ERROR_CHECK_LOGIC_VOLTAGE
//#define ERROR_CHECK_AMP

void hip_initilize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter, TC0_t *timestamp_timer, uint16_t **master_watchdog);

void hip_enable_outputs(void);
void hip_disable_outputs(void);
void hip_update_inputs(uint8_t id);  /**< Function called to read all the sensors */
bool hip_run_halt(uint8_t id);       /**< Runs the halt controller. Returns true if the controller wants to continue running */ 
void hip_update_outputs(uint8_t id); /**< Called to update motor outputs */
void hip_estop(void);                /**< Called to when estop happens, should send 0 torques to motor */
bool hip_check_error(uint8_t id);    /**< Checks sensor readings to decide if estop should happen, returns true if there should be an error */
bool hip_check_halt(uint8_t id);     /**< Checks sensors to determine if medulla should go into halt state, returns true if medulla should go into halt */
void hip_reset_error();
void hip_wait_loop();

#endif //MEDULLA_LEG_H
